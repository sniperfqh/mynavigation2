#!/usr/bin/env python3
"""Explicit simulation-only acceptance probe; never run automatically by CTest."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath, OccupancyGrid
from byd_custom_msgs.msg import MotionState
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--run-simulation-goals', action='store_true')
    parser.add_argument('--directions', default='1,-1', help='One or two directions, each 1 or -1')
    args = parser.parse_args()
    directions = [int(value) for value in args.directions.split(',')]
    if not 1 <= len(directions) <= 2 or any(value not in (-1, 1) for value in directions):
        parser.error('directions must contain one or two values from 1,-1')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    report = {'kind': 'Gazebo with external Cartographer; localization-relative measurements', 'cases': []}
    rclpy.init()
    node = rclpy.create_node('autonomous_fixed_acceptance_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest, events, subscriptions = {}, [], []
    planner_ids = set()
    current = None
    handle = None
    trace = (output / 'trace.jsonl').open('w', buffering=1)

    def receive(topic, message):
        latest[topic] = (time.monotonic(), message)
        if isinstance(message, NavPath):
            data = [[p.pose.position.x, p.pose.position.y, p.pose.orientation.x,
                     p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in message.poses]
            event = {'topic': topic, 'hash': hashlib.sha256(json.dumps(data).encode()).hexdigest(),
                     'poses': len(data), 'data': data, 'case': current}
            events.append(event)
            trace.write(json.dumps(event) + '\n')
        elif isinstance(message, String):
            trace.write(json.dumps({'topic': topic, 'case': current, 'data': message.data}) + '\n')
        elif isinstance(message, Twist) and current is not None:
            trace.write(json.dumps({'topic': topic, 'case': current, 'vx': message.linear.x,
                                    'wz': message.angular.z, 'wall': time.time()}) + '\n')
        elif isinstance(message, GoalStatusArray):
            planner_ids.update(bytes(s.goal_info.goal_id.uuid).hex() for s in message.status_list)

    for topic, msg_type, qos in [('/motion_state', MotionState, qos_profile_sensor_data),
                                 ('/cmd_vel_nav', Twist, 10), ('/cmd_vel', Twist, 10),
                                 ('/plan', NavPath, 10), ('/autonomous_fixed/plan', NavPath, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                                 ('/autonomous_fixed/execution_status', String, 10),
                                 ('/map', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                                 ('/compute_path_to_pose/_action/status', GoalStatusArray, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
        subscriptions.append(node.create_subscription(msg_type, topic, lambda m, t=topic: receive(t, m), qos))

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)

    def future_result(future, timeout):
        until = time.monotonic() + timeout
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def parameters(server, names):
        client = node.create_client(GetParameters, server + '/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Parameter service unavailable: ' + server)
        request = GetParameters.Request()
        request.names = names
        response = future_result(client.call_async(request), 5)
        return {name: value.string_array_value if value.type == 9 else value.string_value if value.type == 4 else value.double_value for name, value in zip(names, response.values)}

    def pose():
        transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        age = (node.get_clock().now() - rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
        return transform.transform.translation.x, transform.transform.translation.y, yaw, age

    def stopped():
        sample = latest.get('/motion_state')
        if sample is None or time.monotonic() - sample[0] > 0.2:
            return False
        velocity = sample[1]
        return abs(velocity.v_car) <= 0.01 and abs(velocity.w_car) <= 0.05

    def corridor_free(start, target):
        sample = latest.get('/map')
        if sample is None:
            return False
        grid = sample[1]
        origin = grid.info.origin.position
        # Conservative swept rectangle for the configured 1.2m by 0.8m footprint.
        for step in range(13):
            cx = start[0] + (target[0] - start[0]) * step / 12
            cy = start[1] + (target[1] - start[1]) * step / 12
            for longitudinal in [-0.65, -0.4, 0.0, 0.4, 0.65]:
                for lateral in [-0.45, -0.2, 0.0, 0.2, 0.45]:
                    x = cx + longitudinal * math.cos(start[2]) - lateral * math.sin(start[2])
                    y = cy + longitudinal * math.sin(start[2]) + lateral * math.cos(start[2])
                    ix = int(math.floor((x - origin.x) / grid.info.resolution))
                    iy = int(math.floor((y - origin.y) / grid.info.resolution))
                    if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                        return False
                    value = grid.data[iy * grid.info.width + ix]
                    if value < 0 or value >= 65:
                        return False
        return True

    try:
        spin(4)
        report['nodes'] = node.get_node_names()
        report['navigator'] = parameters('/regulated_navigator', ['operation_mode', 'planner_id'])
        report['planner'] = parameters('/planner_server', ['selected_planner', 'SmacHybrid.motion_model_for_search'])
        report['controller'] = parameters('/controller_server', ['controller_plugins', 'controller_frequency'])
        if report['navigator']['operation_mode'] != 'autonomous_fixed' or report['controller']['controller_plugins'] != ['AutonomousFixedPathController']:
            raise RuntimeError('New mode/controller is not loaded')
        report['initial_pose'] = pose()
        report['preflight_stopped'] = stopped()
        if not args.run_simulation_goals:
            return
        if 'simulation_sensor_bridge' not in report['nodes']:
            raise RuntimeError('Simulation bridge absent; refusing to issue a movement goal')
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('NavigateToPose server unavailable')
        # 错误接口请求应在接收阶段拒绝，不消耗一次规划机会。
        multi = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
        if not multi.wait_for_server(timeout_sec=5):
            raise RuntimeError('NavigateThroughPoses rejection endpoint unavailable')
        sample = NavigateToPose.Goal()
        sample.pose.header.frame_id = 'map'
        sample.pose.pose.orientation.w = 1.0
        sample.behavior_tree = 'unsupported.xml'
        rejected_bt = not future_result(client.send_goal_async(sample), 5).accepted
        multiple = NavigateThroughPoses.Goal()
        multiple.poses = [sample.pose]
        rejected_multi = not future_result(multi.send_goal_async(multiple), 5).accepted
        report['entry_rejections'] = {'custom_behavior_tree': rejected_bt, 'through_poses': rejected_multi}
        if not (rejected_bt and rejected_multi):
            raise RuntimeError('Fourth-mode entry restrictions were not enforced')
        for index, direction in enumerate(directions, 1):
            spin(1)
            start = pose()
            if not (-0.05 <= start[3] <= 0.3) or not stopped():
                raise RuntimeError('Localization or stopped feedback not ready before goal')
            target = (start[0] + direction * 0.6 * math.cos(start[2]),
                      start[1] + direction * 0.6 * math.sin(start[2]), start[2])
            if not corridor_free(start, target):
                report['cases'].append({'index': index, 'status': 'SKIPPED_UNCLEAR_CORRIDOR'})
                continue
            current = index
            before = set(planner_ids)
            first_event = len(events)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
            goal.pose.pose.orientation.z = math.sin(target[2] * 0.5)
            goal.pose.pose.orientation.w = math.cos(target[2] * 0.5)
            started = time.monotonic()
            handle = future_result(client.send_goal_async(goal), 5)
            if not handle.accepted:
                raise RuntimeError('Goal rejected')
            result_future = handle.get_result_async()
            deadline = time.monotonic() + 45
            while not result_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            canceled = not result_future.done()
            if canceled:
                future_result(handle.cancel_goal_async(), 5)
            result = future_result(result_future, 8)
            handle = None
            spin(1)
            end = pose()
            plans = [e for e in events[first_event:] if e['topic'] == '/autonomous_fixed/plan']
            report['cases'].append({'index': index, 'direction': direction, 'target': target,
                                    'status': result.status, 'observer_canceled': canceled,
                                    'duration': time.monotonic() - started,
                                    'planning_requests_observed': len(planner_ids - before),
                                    'frozen_path_publications': len(plans),
                                    'unique_frozen_paths': len({e['hash'] for e in plans}),
                                    'xy_error': math.hypot(end[0] - target[0], end[1] - target[1]),
                                    'yaw_error': abs(math.atan2(math.sin(end[2] - target[2]), math.cos(end[2] - target[2]))),
                                    'stopped': stopped()})
            print(json.dumps(report['cases'][-1]), flush=True)
            current = None
            if result.status != 4:
                break
    except Exception as error:
        report['error'] = str(error)
    finally:
        if handle is not None and handle.accepted:
            try:
                future_result(handle.cancel_goal_async(), 5)
                spin(1)
            except Exception as error:
                report['cleanup_error'] = str(error)
        report['final_stopped'] = stopped()
        (output / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'cases'}, ensure_ascii=False), flush=True)
        trace.close()
        node.destroy_node()
        rclpy.shutdown()
    return 1 if 'error' in report or any(case.get('status') not in (4, 'SKIPPED_UNCLEAR_CORRIDOR') for case in report['cases']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
