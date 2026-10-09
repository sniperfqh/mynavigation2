#!/usr/bin/env python3
"""Explore a live SLAM occupancy grid; no world geometry or truth input."""
import argparse
import gzip
import heapq
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
from scipy import ndimage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def shortest_paths(free, start):
    distances = np.full(free.shape, np.inf)
    parents = {}
    if not free[start]:
        return distances, parents
    distances[start] = 0.
    queue = [(0., start)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost != distances[cell]:
            continue
        y, x = cell
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ny, nx = y + dy, x + dx
            if not (0 <= ny < free.shape[0] and 0 <= nx < free.shape[1]) or not free[ny, nx]:
                continue
            if dy and dx and (not free[y, nx] or not free[ny, x]):
                continue
            value = cost + math.hypot(dx, dy)
            if value < distances[ny, nx]:
                distances[ny, nx] = value
                parents[(ny, nx)] = cell
                heapq.heappush(queue, (value, (ny, nx)))
    return distances, parents


class Explorer(Node):
    def __init__(self, output, validation_points=None):
        super().__init__('warehouse_explorer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.grid = None
        self.scan = None
        self.odom = None
        self.scan_time = self.odom_time = 0.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.inputs = [
            self.create_subscription(OccupancyGrid, '/map', self.receive_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)),
            self.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)]
        self.records = []
        self.truth_records = []
        self.inputs.append(self.create_subscription(
            TFMessage, '/mapping_truth', self.record_truth, qos_profile_sensor_data))
        self.trajectory = []
        self.visited = []
        self.blocked = []
        self.path = []
        self.target = None
        self.start_pose = None
        self.last_scan_record = 0.
        self.last_plan = 0.
        self.target_start = 0.
        self.last_progress = time.monotonic()
        self.best_distance = math.inf
        self.returning = False
        self.no_frontier_since = None
        self.complete = False
        self.events = []
        self.initial_rotation = 0.
        self.previous_odom_yaw = None
        self.validation = validation_points is not None
        self.validation_points = list(validation_points or [])
        self.survey = None
        self.last_command = [0., 0.]

    def receive_map(self, msg):
        self.grid = msg

    def record_truth(self, msg):
        # Diagnostic recording only. Planning and control never read this list.
        for transform in msg.transforms:
            if transform.child_frame_id == 'diffbot':
                p, q = transform.transform.translation, transform.transform.rotation
                self.truth_records.append([time.monotonic(), p.x, p.y, p.z,
                                           q.x, q.y, q.z, q.w])

    def receive_scan(self, msg):
        self.scan, self.scan_time = msg, time.monotonic()

    def receive_odom(self, msg):
        self.odom, self.odom_time = msg, time.monotonic()

    def pose(self):
        t = self.buffer.lookup_transform('map', 'base_footprint', Time())
        if self.get_clock().now().nanoseconds / 1e9 - (t.header.stamp.sec + t.header.stamp.nanosec / 1e9) > .5:
            raise RuntimeError('stale map TF')
        p, q = t.transform.translation, t.transform.rotation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def command(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.publisher.publish(msg)
        self.last_command = [float(v), float(w)]

    def plan(self, pose):
        msg = self.grid
        scale = max(1, round(.1 / msg.info.resolution))
        h, w = msg.info.height // scale, msg.info.width // scale
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)[:h * scale, :w * scale].reshape(h, scale, w, scale)
        occupied = (data >= 50).any(axis=(1, 3))
        known_free = ((data >= 0) & (data < 50)).all(axis=(1, 3))
        step = scale * msg.info.resolution
        clearance = ndimage.distance_transform_edt(~occupied) * step
        unknown_distance = ndimage.distance_transform_edt(known_free) * step
        free = known_free & (clearance > .45) & (unknown_distance > .15)
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        cell = (int((pose[1] - oy) / step), int((pose[0] - ox) / step))
        if not (0 <= cell[0] < h and 0 <= cell[1] < w):
            raise RuntimeError('robot outside map')
        if not free[cell]:
            # Permit only the current cell to exit a discretization boundary.
            # Every following cell still requires the full planning margin.
            if not occupied[cell] and clearance[cell] > .35 and unknown_distance[cell] > .15:
                free[cell] = True
            else:
                raise RuntimeError(f'robot footprint has no safe known-free cell: '
                                   f'occupied={occupied[cell]}, clearance={clearance[cell]:.3f}, '
                                   f'unknown_distance={unknown_distance[cell]:.3f}')
        distance, parents = shortest_paths(free, cell)
        yy, xx = np.indices(free.shape)
        wx, wy = ox + (xx + .5) * step, oy + (yy + .5) * step
        if self.returning:
            goal = (int((self.start_pose[1] - oy) / step), int((self.start_pose[0] - ox) / step))
            if not (0 <= goal[0] < h and 0 <= goal[1] < w) or not np.isfinite(distance[goal]):
                raise RuntimeError('no safe return route')
        elif self.target is not None:
            goal = (int((self.target[1] - oy) / step), int((self.target[0] - ox) / step))
            if not (0 <= goal[0] < h and 0 <= goal[1] < w) or not np.isfinite(distance[goal]):
                self.events.append({'event': 'route_invalidated', 'point': self.target})
                self.target, self.path = None, []
                return
        else:
            if self.validation:
                if not self.validation_points:
                    self.complete = True
                    self.path = []
                    return
                point = self.validation_points[0]
                goal = (int((point[1] - oy) / step), int((point[0] - ox) / step))
                if not (0 <= goal[0] < h and 0 <= goal[1] < w) or not np.isfinite(distance[goal]):
                    raise RuntimeError('validation point is not safely reachable')
            else:
                goal = self.frontier_goal(free, distance, known_free, occupied, unknown_distance,
                                          wx, wy, step)
                if goal is None:
                    self.path = []
                    return
        cells = [goal]
        while cells[-1] != cell:
            cells.append(parents[cells[-1]])
        self.path = [(ox + (x + .5) * step, oy + (y + .5) * step) for y, x in reversed(cells)]
        new_target = self.path[-1]
        if self.target is None or math.dist(new_target, self.target) > .3:
            self.target_start = self.last_progress = time.monotonic()
            self.best_distance = math.inf
            self.events.append({'event': 'target', 'point': new_target, 'returning': self.returning, 'time': self.target_start})
            print('TARGET', new_target, 'returning', self.returning, flush=True)
        self.target = new_target

    def frontier_goal(self, free, distance, known_free, occupied, unknown_distance, wx, wy, step):
        frontier = free & np.isfinite(distance) & (unknown_distance < .65) & (distance * step > .6)
        # Isolated unknown pixels do not justify crossing the warehouse.
        nearby_unknown = ndimage.uniform_filter((~known_free & ~occupied).astype(float), size=11)
        frontier &= nearby_unknown > .03
        for x, y in self.visited + self.blocked:
            frontier &= np.hypot(wx - x, wy - y) > .7
        candidates = np.argwhere(frontier)
        if not len(candidates):
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            if time.monotonic() - self.no_frontier_since > 15:
                self.returning = True
                self.events.append({'event': 'return_to_start', 'time': time.monotonic()})
            return None
        self.no_frontier_since = None
        score = nearby_unknown[frontier] / (1 + distance[frontier] * step * .08)
        return tuple(candidates[int(np.argmax(score))])


    def tick(self):
        now = time.monotonic()
        if self.grid is None or self.scan is None or self.odom is None:
            self.command()
            return
        if now - self.scan_time > .5 or now - self.odom_time > .5:
            self.command()
            raise RuntimeError('stale lidar or odometry')
        if self.count_publishers('/cmd_vel') != 1:
            self.command()
            raise RuntimeError('conflicting velocity publisher')
        pose = self.pose()
        if self.start_pose is None:
            self.start_pose = pose
            self.events.append({'event': 'initial_pose', 'pose': pose})
        self.trajectory.append([now, self.get_clock().now().nanoseconds / 1e9, *pose])
        if now - self.last_scan_record >= 1:
            p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
            scan = self.scan
            self.records.append({'wall': now, 'stamp': scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9, 'pose': pose, 'odom': [p.x, p.y, q.x, q.y, q.z, q.w], 'velocity': [self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z], 'command': self.last_command, 'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment, 'range_min': scan.range_min, 'range_max': scan.range_max, 'ranges': [float(r) if math.isfinite(r) else None for r in scan.ranges]})
            self.last_scan_record = now
        q = self.odom.pose.pose.orientation
        odom_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        if self.initial_rotation < 2 * math.pi + .1:
            if self.previous_odom_yaw is not None:
                delta = odom_yaw - self.previous_odom_yaw
                self.initial_rotation += math.atan2(math.sin(delta), math.cos(delta))
            self.previous_odom_yaw = odom_yaw
            finite = [r for r in self.scan.ranges if math.isfinite(r)]
            if not finite or min(finite) < .6:
                raise RuntimeError('initial scan rotation lacks footprint clearance')
            self.command(0., .3)
            return
        if now - self.last_plan >= 3:
            self.plan(pose)
            self.last_plan = now
        if not self.path or self.target is None:
            self.command()
            return
        remaining = math.dist(pose[:2], self.target)
        if self.survey is not None:
            delta = odom_yaw - self.survey['yaw']
            self.survey['angle'] += math.atan2(math.sin(delta), math.cos(delta))
            self.survey['yaw'] = odom_yaw
            if self.survey['angle'] >= self.survey['stage'] * 2 * math.pi / 3:
                self.command()
                if self.survey['wait'] is None:
                    self.survey['wait'] = now
                    self.events.append({'event': 'validation_heading', 'point': self.target,
                                        'pose': pose, 'wall': now, 'stage': self.survey['stage']})
                if now - self.survey['wait'] > 3:
                    self.survey['stage'] += 1
                    self.survey['wait'] = None
                    if self.survey['stage'] == 3:
                        self.visited.append(self.target)
                        self.validation_points.pop(0)
                        self.survey = None
                        self.target, self.path = None, []
                        self.last_plan = 0.
                return
            finite = [r for r in self.scan.ranges if math.isfinite(r)]
            if not finite or min(finite) < .4:
                raise RuntimeError('validation rotation lacks clearance')
            self.command(0., .3)
            return
        if remaining < self.best_distance - .05:
            self.best_distance, self.last_progress = remaining, now
        if remaining < .2:
            self.command()
            if self.validation:
                self.survey = {'yaw': odom_yaw, 'angle': 0., 'stage': 0, 'wait': None}
                return
            self.visited.append(self.target)
            if self.returning:
                self.complete = True
            self.target = None
            self.path = []
            self.last_plan = 0.
            return
        if now - self.last_progress > 40:
            if self.validation:
                raise RuntimeError('validation target stalled: ' + str(self.target))
            self.events.append({'event': 'blocked', 'point': self.target})
            self.blocked.append(self.target)
            self.target, self.path = None, []
            self.last_plan = 0.
            self.command()
            return
        nearest = min(range(len(self.path)), key=lambda i: math.dist(pose[:2], self.path[i]))
        carrot = self.path[-1]
        for point in self.path[nearest:]:
            if math.dist(pose[:2], point) >= .3:
                carrot = point
                break
        heading = math.atan2(carrot[1] - pose[1], carrot[0] - pose[0])
        error = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        ranges = [r for i, r in enumerate(self.scan.ranges) if math.isfinite(r) and abs(self.scan.angle_min + i * self.scan.angle_increment) < .65]
        v = min(.2, remaining * .5) * max(0., math.cos(error)) if abs(error) < .4 else 0.
        if not ranges or min(ranges) < .35:
            v = 0.
        self.command(v, max(-.3, min(.3, 1.5 * error)))

    def save(self, status, error=None):
        if self.grid is not None:
            msg = self.grid
            np.savez_compressed(self.output / 'grid.npz',
                                data=np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width),
                                origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                                resolution=msg.info.resolution)
        progress = {'status': status, 'error': error, 'start_pose': self.start_pose, 'visited': self.visited, 'blocked': self.blocked, 'events': self.events, 'trajectory_samples': len(self.trajectory), 'scan_samples': len(self.records)}
        (self.output / 'progress.json').write_text(json.dumps(progress, indent=2, allow_nan=False))
        for name, data in [('scans', self.records), ('trajectory', self.trajectory),
                           ('truth', self.truth_records)]:
            with gzip.open(self.output / (name + '.json.gz'), 'wt') as stream:
                json.dump(data, stream, allow_nan=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=5400.)
    parser.add_argument('--launch-simulation', action='store_true',
                        help='Run all ROS processes in one isolated process tree')
    parser.add_argument('--validation-points', type=Path)
    parser.add_argument('--map', type=Path)
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--domain', type=int, default=232,
                        help='Private ROS domain for the simulation process tree')
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('domain must be between 0 and 232')
    args.output.mkdir(parents=True, exist_ok=False)
    processes, logs = [], []
    if args.launch_simulation:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
        os.environ['SPDLOG_WRAPPER_LOG_DIR'] = str(args.output / 'spdlog')
        os.environ['SPDLOG_WRAPPER_FILE_LEVEL'] = 'info'
        partition = 'warehouse_mapping_' + str(os.getpid())
        os.environ['IGN_PARTITION'] = os.environ['GZ_PARTITION'] = partition
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        context = Context()
        rclpy.init(context=context)
        probe = Node('mapping_domain_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, executor=executor, timeout_sec=.1)
        used = any(probe.count_publishers(topic) for topic in ('/scan', '/odom', '/clock', '/cmd_vel'))
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
        if used:
            parser.error('private ROS domain is occupied; simulation not launched')
        commands = [
            ['ros2', 'launch', 'myworld_bringup', 'mapping.launch.py', 'ign_partition:=' + partition],
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/world/myworld2/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
             '--ros-args', '-r', '/world/myworld2/dynamic_pose/info:=/mapping_truth']]
        if args.validation_points:
            if args.map is None or args.params_file is None:
                parser.error('validation requires --map and --params-file')
            commands[0] += ['mapping:=false', 'map:=' + str(args.map.resolve()),
                            'params_file:=' + str(args.params_file.resolve())]
        for index, command in enumerate(commands):
            log = (args.output / f'process_{index}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
    rclpy.init()
    node = Explorer(args.output, json.loads(args.validation_points.read_text())
                    if args.validation_points else None)
    begin = last_save = time.monotonic()
    failure_since = None
    status, error = 'FAILED', None
    try:
        while time.monotonic() - begin < args.max_seconds and not node.complete:
            # Drain high-rate clock/odom/TF before evaluating freshness.
            drain_until = time.monotonic() + .025
            while time.monotonic() < drain_until:
                rclpy.spin_once(node, timeout_sec=0.)
            if time.monotonic() - begin > 90 and node.start_pose is None:
                raise RuntimeError('map/localization startup timeout')
            try:
                node.tick()
                failure_since = None
            except Exception as exc:
                node.command()
                if failure_since is None:
                    failure_since = time.monotonic()
                    node.events.append({'event': 'safety_wait', 'reason': str(exc),
                                        'wall': failure_since})
                if time.monotonic() - begin > 90 and time.monotonic() - failure_since > 5:
                    raise
            time.sleep(.02)
            if time.monotonic() - last_save > 15:
                node.save('RUNNING')
                last_save = time.monotonic()
        if not node.complete:
            raise RuntimeError('mapping timeout; map not qualified for promotion')
        if not node.validation:
            from nav_msgs.srv import GetMap
            from slam_toolbox.srv import Pause
            from export_slam_map import export_grid

            def call_snapshot_service(service, type_, request):
                client = node.create_client(type_, service)
                if not client.wait_for_service(timeout_sec=10.):
                    raise RuntimeError('snapshot service unavailable: ' + service)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
                if not future.done() or future.result() is None:
                    raise RuntimeError('snapshot service failed: ' + service)
                response = future.result()
                node.destroy_client(client)
                return response

            node.command()
            call_snapshot_service('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
            snapshot = call_snapshot_service('/slam_toolbox/dynamic_map', GetMap, GetMap.Request()).map
            if not snapshot.info.width or not snapshot.info.height:
                raise RuntimeError('empty final SLAM snapshot')
            export_grid(snapshot, args.output)
            subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                        'slam_toolbox/srv/SerializePoseGraph',
                        '{filename: "' + str(args.output / 'myworld3') + '"}'],
                       check=True, timeout=30., stdout=(args.output / 'serialize.log').open('w'))
        status = 'VALIDATED' if node.validation else 'EXPLORED'
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc)
        print('FAILED', error, flush=True)
    finally:
        for _ in range(20):
            node.command()
            rclpy.spin_once(node, timeout_sec=.05)
        node.save(status, error)
        if not node.validation and status == 'FAILED' and node.grid is not None:
            # Retain an explicitly unqualified candidate for diagnosis/resume.
            try:
                subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                '-f', str(args.output / 'candidate'), '--ros-args',
                                '-p', 'save_map_timeout:=20.0'], check=True, timeout=30.)
                subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                                'slam_toolbox/srv/SerializePoseGraph',
                                '{filename: "' + str(args.output / 'candidate') + '"}'],
                               check=True, timeout=30.)
            except (subprocess.SubprocessError, OSError) as save_error:
                print('Candidate save failed:', save_error, flush=True)
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10.)
        for log in logs:
            log.close()
    return 0 if status in ('EXPLORED', 'VALIDATED') else 1


if __name__ == '__main__':
    raise SystemExit(main())
