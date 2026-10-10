#!/usr/bin/env python3
"""Isolated fixed-path speed/stop experiment; Gazebo truth remains diagnostic only."""
import argparse
import hashlib
import gzip
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from speed_sweep_metrics import evaluate, first_window, first_stopped_window, motion_metrics, observed_position_valid, valid_path, command_evidence, LocalizationCorrectionGuard, stopped_window


class RequestTimeout(RuntimeError):
    pass


class GoalResultTimeout(RuntimeError):
    pass


class LocalizationAnomaly(RuntimeError):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--margin', type=float, default=.1)
    parser.add_argument('--creep', type=float, default=.01)
    parser.add_argument('--deceleration', type=float, default=.25)
    parser.add_argument('--tolerance', type=float, default=.01)
    parser.add_argument('--stop-entry', type=float, default=0.)
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--length', type=float, default=2.)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--stationary-seconds', type=float, default=60.)
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--world-file', type=Path)
    parser.add_argument('--map-file', type=Path)
    parser.add_argument('--start-x', type=float, default=-3.5)
    parser.add_argument('--start-y', type=float, default=-7.3)
    parser.add_argument('--direction', choices=['both', 'forward', 'reverse'], default='both')
    parser.add_argument('--goal-timeout', type=float, default=180.)
    parser.add_argument('--terminal-lookahead', type=float)
    parser.add_argument('--scan-every', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=3407)
    args = parser.parse_args()
    tool_snapshot = Path(__file__).read_bytes()
    if not (0 < args.margin <= .1 and 0 < args.creep <= .03
            and .25 <= args.deceleration <= 1.0 and 0 < args.tolerance <= .01 and 0 <= args.stop_entry <= args.tolerance
            and 0 < args.speed <= 1.5 and 0 < args.length <= 7
            and args.repeats > 0 and args.stationary_seconds >= 0 and 0 < args.goal_timeout <= 180.
            and all(math.isfinite(x) for x in [args.start_x, args.start_y, args.stationary_seconds])
            and .1 <= args.scan_every <= 10.
            and (args.terminal_lookahead is None or .01 <= args.terminal_lookahead <= .25)):
        parser.error('parameters outside the approved screening bounds')
    hold_file = args.output.parent / '.hold-next-launch'
    if hold_file.exists():
        print('WAITING_FOR_EXPERIMENT_BUILD', str(hold_file), flush=True)
    while hold_file.exists():
        time.sleep(.2)
    # A random private domain avoids sending goals into the default hardware domain.
    domain = 190 + os.getpid() % 30
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output.resolve() / 'ros_logs')
    os.environ['SPDLOG_WRAPPER_LOG_DIR'] = str(args.output.resolve() / 'spdlog')
    os.environ['SPDLOG_WRAPPER_FILE_LEVEL'] = 'info'
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from nav_msgs.msg import Odometry, Path as ROSPath
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage
    from byd_custom_msgs.action import NavigationService
    from byd_custom_msgs.msg import NaviSegment, MotionState, ControlRes
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters

    probe_context = Context()
    rclpy.init(context=probe_context)
    probe = rclpy.create_node('arrival_domain_probe', context=probe_context)
    probe_executor = SingleThreadedExecutor(context=probe_context)
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, executor=probe_executor, timeout_sec=.1)
    existing = [name for name, namespace in probe.get_node_names_and_namespaces()
                if name != 'arrival_domain_probe']
    probe.destroy_node()
    probe_executor.shutdown()
    rclpy.shutdown(context=probe_context)
    if existing:
        parser.error(f'domain {domain} already in use: {existing}; simulation not launched')

    # Keep the original map/world and all safety limits. Only spawn pose changes.
    import yaml
    from ament_index_python.packages import get_package_share_directory
    import xml.etree.ElementTree as ET
    share = Path(get_package_share_directory('myworld_bringup'))
    params = yaml.safe_load((args.params_file or share / 'params/myworld2.yaml').read_text())
    # Fixed-path never requests a planner. Keep the server available without
    # initializing unused high-resolution lattice lookup tables.
    params['planner_server']['ros__parameters']['planner_plugins'] = ['GridBasedAstar']
    params['planner_server']['ros__parameters']['selected_planner'] = 'GridBasedAstar'
    params['controller_server']['ros__parameters']['FixedPathController']['goal_stop_entry_tolerance'] = args.stop_entry
    params['amcl']['ros__parameters']['random_seed'] = args.seed
    params['amcl']['ros__parameters']['initial_pose'] = {'x': args.start_x, 'y': args.start_y, 'z': 0., 'yaw': math.pi / 2}
    experiment_params = args.output / 'params.yaml'
    experiment_params.write_text(yaml.safe_dump(params, sort_keys=False))
    world = ET.parse(args.world_file or share / 'models/myworld2/world_only.sdf')
    robot = next(i for i in world.getroot().find('world').findall('include') if i.findtext('name') == 'diffbot')
    robot.find('pose').text = f'{args.start_x} {args.start_y} 0.01 0 0 {math.pi / 2}'
    experiment_world = args.output / 'world.sdf'
    world.write(experiment_world, encoding='unicode')
    log = (args.output / 'launch.log').open('w')
    partition = 'arrival_' + uuid.uuid4().hex
    command = ['ros2', 'launch', 'myworld_bringup', 'entry.launch.py',
               'operation_mode:=fixed_path', 'headless:=true', 'use_rviz:=false',
               'use_collision_monitor:=true', 'use_collision_visualization:=false',
               f'ign_partition:={partition}',
               f'fixed_path_goal_braking_distance_margin:={args.margin}',
               f'fixed_path_goal_final_approach_velocity:={args.creep}',
               f'fixed_path_goal_linear_deceleration:={args.deceleration}',
               f'fixed_path_xy_goal_tolerance:={args.tolerance}']
    if args.terminal_lookahead is not None:
        command.extend([f'fixed_path_goal_terminal_lookahead_dist:={args.terminal_lookahead}', f'fixed_path_goal_terminal_lookahead_min_dist:={args.terminal_lookahead}', f'fixed_path_goal_terminal_lookahead_max_dist:={args.terminal_lookahead}'])
    command.extend([f'params_file:={experiment_params.resolve()}', f'world_file:={experiment_world.resolve()}'])
    if args.map_file:
        command.append(f'map:={args.map_file.resolve()}')
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    truth_log = (args.output / 'truth_bridge.log').open('w')
    truth_env = os.environ.copy()
    truth_env['IGN_PARTITION'] = partition
    truth_env['GZ_PARTITION'] = partition
    truth_proc = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                  '/world/myworld2/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                                  '--ros-args', '-r', '__node:=arrival_truth_bridge',
                                  '-r', '/world/myworld2/dynamic_pose/info:=/arrival_ground_truth'],
                                 env=truth_env, stdout=truth_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('arrival_screen', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    trace = []
    pose_trace = []
    localization_trace = []
    path_trace = []
    truth_trace = []
    scan_trace = []
    truth_frames = set()
    subscriptions = []
    report = {'configuration': vars(args).copy(), 'domain': domain, 'launch': command,
              'measurement': 'estimated map/base_footprint TF; not physical accuracy', 'goals': []}
    report['tool_sha256'] = hashlib.sha256(tool_snapshot).hexdigest()
    (args.output / 'runner_snapshot.py').write_bytes(tool_snapshot)
    (args.output / 'metrics_snapshot.py').write_bytes((Path(__file__).parent / 'speed_sweep_metrics.py').read_bytes())
    report['schema_version'] = 5
    report['source_commit'] = subprocess.check_output(['git', '-C', str(Path(__file__).resolve().parents[2]), 'rev-parse', 'HEAD'], text=True).strip()
    report['ign_partition'] = partition
    report['scan_sampling_interval_s'] = args.scan_every
    diff = subprocess.check_output(['git', '-C', str(Path(__file__).resolve().parents[2]), 'diff', '--binary'], text=True)
    report['source_diff_sha256'] = hashlib.sha256(diff.encode()).hexdigest()
    (args.output / 'source_diff.patch').write_text(diff)
    report['deployed_library_sha256'] = {}
    for package, library in [('nav2_amcl', 'libmotions_lib.so'), ('nav2_amcl', 'libpf_lib.so'), ('nav2_amcl', 'libamcl_core.so'), ('nav2_amcl', 'libmap_lib.so'), ('nav2_amcl', 'libsensors_lib.so'), ('nav2_regulated_modules', 'libnav2_regulated_modules_fixed_path_controller.so')]:
        deployed = Path(get_package_share_directory(package)).parents[1] / 'lib' / library
        report['deployed_library_sha256'][str(deployed)] = hashlib.sha256(deployed.read_bytes()).hexdigest()
    report['configuration']['output'] = str(args.output)
    report['configuration']['params_file'] = str(args.params_file) if args.params_file else None
    report['configuration']['world_file'] = str(args.world_file) if args.world_file else None
    report['configuration']['map_file'] = str(args.map_file) if args.map_file else None

    def receive(topic, msg):
        latest[topic] = (time.monotonic(), msg)
        if topic in ('/odom', '/odometry'):
            velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        elif topic == '/motion_state':
            velocity = (msg.v_car, msg.w_car)
        elif topic == '/control_to_uart':
            velocity = (msg.v, msg.w)
        else:
            velocity = (msg.linear.x, msg.angular.z)
        stamp = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                 if hasattr(msg, 'header') else node.get_clock().now().nanoseconds * 1e-9)
        trace.append([time.monotonic(), stamp, topic, *velocity])

    for topic, typ in [('/odom', Odometry), ('/odometry', Odometry),
                       ('/motion_state', MotionState), ('/control_to_uart', ControlRes),
                       ('/cmd_vel_nav', Twist), ('/cmd_vel_collision_in', Twist), ('/cmd_vel', Twist)]:
        subscriptions.append(node.create_subscription(typ, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))

    def amcl_receive(msg):
        p = msg.pose.pose.position
        localization_trace.append([time.monotonic(), 'amcl', msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                   p.x, p.y, msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35]])

    def tf_receive(msg):
        for t in msg.transforms:
            if t.header.frame_id not in ('map', 'odom'):
                continue
            p, q = t.transform.translation, t.transform.rotation
            localization_trace.append([time.monotonic(), 'tf', t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                                       t.header.frame_id, t.child_frame_id, p.x, p.y, q.x, q.y, q.z, q.w,
                                       node.get_clock().now().nanoseconds * 1e-9])

    subscriptions.append(node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_receive, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage, '/tf', tf_receive, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(ROSPath, '/fixed_path_plan', lambda msg: path_trace.append([time.monotonic(), msg.header.frame_id, [[p.pose.position.x, p.pose.position.y] for p in msg.poses]]), qos_profile_sensor_data))

    def truth_receive(msg):
        for t in msg.transforms:
            truth_frames.add(t.child_frame_id)
            if t.child_frame_id != 'diffbot':
                continue
            p, q = t.transform.translation, t.transform.rotation
            truth_trace.append([time.monotonic(), t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                                t.header.frame_id, t.child_frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                node.get_clock().now().nanoseconds * 1e-9])

    from sensor_msgs.msg import LaserScan

    def scan_receive(msg):
        if scan_trace and time.monotonic() - scan_trace[-1][0] < args.scan_every:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        scan_trace.append([time.monotonic(), stamp, msg.header.frame_id,
                           msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max,
                           [r if math.isfinite(r) else None for r in msg.ranges]])

    subscriptions.append(node.create_subscription(TFMessage, '/arrival_ground_truth', truth_receive, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(LaserScan, '/scan', scan_receive, qos_profile_sensor_data))
    guard = None
    parked_since = None

    def wait(future, timeout=10., measure=False):
        nonlocal parked_since
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if measure and (not pose_trace or time.monotonic() - pose_trace[-1][0] >= .02):
                pose_trace.append(sample())
                if guard is not None and guard.observe(pose_trace[-1]):
                    raise LocalizationAnomaly('map/odom correction exceeded 0.20 m for two samples')
                current = pose_trace[-1]
                if active_case is not None and math.dist(current[1:3], active_case['target']) < .05 and abs(current[3]) < .001 and abs(current[4]) < .001:
                    if parked_since is None:
                        parked_since = current[0]
                    elif current[0] - parked_since > 3.:
                        raise RuntimeError('test watchdog: parked near goal for 3s without action completion')
                else:
                    parked_since = None
        if not future.done():
            raise (GoalResultTimeout('goal result deadline exceeded') if measure else RequestTimeout('ROS request timeout'))
        return future.result()

    def sample():
        recovery_started = time.monotonic()
        received, odom = latest['/odom']
        if recovery_started - received > .05:
            deadline = recovery_started + .5
            while time.monotonic() < deadline and time.monotonic() - latest['/odom'][0] > .05:
                rclpy.spin_once(node, timeout_sec=.005)
            report.setdefault('sampling_recoveries', []).append({'wall': recovery_started, 'duration_s': time.monotonic() - recovery_started, 'previous_odom_age_s': recovery_started - received})
        transform = buffer.lookup_transform('map', 'base_footprint', Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        # AMCL postdates map->odom by transform_tolerance; use latest composed
        # transform age (typically limited by odom->base), allowing clock epsilon.
        age = max(0., node.get_clock().now().nanoseconds * 1e-9 - stamp)
        received, odom = latest['/odom']
        if time.monotonic() - received > .2:
            raise RuntimeError('stale odometry')
        p = transform.transform.translation
        odom_transform = buffer.lookup_transform('odom', 'base_footprint', Time.from_msg(transform.header.stamp))
        op = odom_transform.transform.translation
        v = odom.twist.twist
        return (time.monotonic(), p.x, p.y, v.linear.x, v.angular.z, age, op.x, op.y, stamp)

    def collect(seconds):
        data = []
        begin = time.monotonic()
        while time.monotonic() - begin < seconds + .03:
            rclpy.spin_once(node, timeout_sec=.005)
            if not data or time.monotonic() - data[-1][0] >= .02:
                data.append(sample())
        return data

    handle = None
    goal_result_future = None
    active_case = None
    try:
        client = ActionClient(node, NavigationService, '/navigation_service')
        state = node.create_client(GetState, '/regulated_navigator/get_state')
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if client.server_is_ready() and '/odom' in latest and state.service_is_ready():
                if wait(state.call_async(GetState.Request())).current_state.id == 3:
                    try:
                        sample()
                        break
                    except (TransformException, KeyError):
                        pass
        else:
            raise RuntimeError('simulation not ready')
        report['lifecycle_states'] = {}
        for lifecycle_name in ['amcl', 'map_server', 'controller_server', 'velocity_smoother', 'collision_monitor', 'regulated_navigator']:
            lc = node.create_client(GetState, '/' + lifecycle_name + '/get_state')
            if not lc.wait_for_service(timeout_sec=10):
                raise RuntimeError('lifecycle service missing: ' + lifecycle_name)
            deadline = time.monotonic() + 45
            while wait(lc.call_async(GetState.Request())).current_state.id != 3:
                if time.monotonic() > deadline:
                    raise RuntimeError('lifecycle inactive: ' + lifecycle_name)
                rclpy.spin_once(node, timeout_sec=.1)
            report['lifecycle_states'][lifecycle_name] = 'active'
        param = node.create_client(GetParameters, '/controller_server/get_parameters')
        if not param.wait_for_service(timeout_sec=10):
            raise RuntimeError('controller parameters missing')
        req = GetParameters.Request()
        req.names = ['controller_frequency'] + ['FixedPathController.' + key for key in
                    ['goal_braking_distance_margin', 'goal_final_approach_velocity', 'goal_linear_deceleration']]
        req.names += ['fixed_path_goal_checker.xy_goal_tolerance',
                      'FixedPathController.goal_braking_reaction_time',
                      'FixedPathController.adaptive_goal_braking_enabled']
        values = wait(param.call_async(req)).values
        report['active_parameters'] = {key: (value.bool_value if value.type == 1 else value.double_value) for key, value in zip(req.names, values)}
        expected = [100., args.margin, args.creep, args.deceleration, args.tolerance, .1]
        if any(v.type != 3 or abs(v.double_value - e) > 1e-9 for v, e in zip(values[:6], expected)) or values[6].type != 1 or values[6].bool_value:
            raise RuntimeError('launch overrides did not reach controller')
        smoother = node.create_client(GetParameters, '/velocity_smoother/get_parameters')
        if not smoother.wait_for_service(timeout_sec=10):
            raise RuntimeError('smoother parameters missing')
        req.names = ['smoothing_frequency', 'feedback', 'feedback_correction_time']
        sv = wait(smoother.call_async(req)).values
        report['smoother_parameters'] = {'smoothing_frequency': sv[0].double_value,
                                        'feedback': sv[1].string_value,
                                        'feedback_correction_time': sv[2].double_value}
        if sv[0].double_value != 100. or sv[1].string_value != 'CLOSED_LOOP' or abs(sv[2].double_value - .1) > 1e-9:
            raise RuntimeError('100 Hz closed-loop baseline changed')
        req.names = ['odom_topic', 'immediate_stop_on_zero_command', 'max_velocity', 'max_accel', 'max_decel']
        extra = wait(smoother.call_async(req)).values
        report['smoother_parameters'].update(odom_topic=extra[0].string_value, immediate_stop_on_zero_command=extra[1].bool_value, max_velocity=list(extra[2].double_array_value), max_accel=list(extra[3].double_array_value), max_decel=list(extra[4].double_array_value))
        if extra[0].string_value != '/odom' or not extra[1].bool_value:
            raise RuntimeError('simulation feedback or zero-stop routing differs from baseline')
        req.names = ['FixedPathController.goal_stop_entry_tolerance']
        ev = wait(param.call_async(req)).values[0]
        report['active_parameters'][req.names[0]] = ev.double_value
        if ev.type != 3 or abs(ev.double_value - args.stop_entry) > 1e-9:
            raise RuntimeError('stop entry tolerance did not reach controller')
        amcl = node.create_client(GetParameters, '/amcl/get_parameters')
        if not amcl.wait_for_service(timeout_sec=10):
            raise RuntimeError('AMCL parameters missing')
        req.names = ['update_min_d', 'update_min_a', 'transform_tolerance', 'random_seed']
        av = wait(amcl.call_async(req)).values
        report['amcl_parameters'] = {key: (value.integer_value if value.type == 2 else value.double_value) for key, value in zip(req.names, av)}
        if av[-1].integer_value != args.seed or av[-1].type != 2:
            raise RuntimeError('fixed AMCL seed not active')
        laser = buffer.lookup_transform('base_footprint', 'base_scan', Time()).transform
        report['laser_extrinsic'] = {'x': laser.translation.x, 'y': laser.translation.y, 'z': laser.translation.z,
                                     'qx': laser.rotation.x, 'qy': laser.rotation.y, 'qz': laser.rotation.z, 'qw': laser.rotation.w}
        print('READY', report['active_parameters'], flush=True)
        truth_deadline = time.monotonic() + 15.
        while not truth_trace and time.monotonic() < truth_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not truth_trace:
            raise RuntimeError('Gazebo truth bridge missing diffbot; frames=' + str(sorted(truth_frames)))
        if args.stationary_seconds:
            stationary = collect(args.stationary_seconds)
            report['stationary'] = evaluate(stationary, stationary[0][1:3], (1., 0.), args.stationary_seconds)
            (args.output / 'stationary.json').write_text(json.dumps(stationary))
            if report['stationary']['max_xy_error_m'] > .01:
                raise RuntimeError('stationary localization drift exceeds 10 mm; precision screening blocked')
        initial_publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        directions = ['forward', 'reverse'] if args.direction == 'both' else [args.direction]
        for index in range(args.repeats * len(directions)):
            direction = directions[index % len(directions)]
            reverse = direction == 'reverse'
            sign = -1 if reverse else 1
            start = (args.start_x, args.start_y + (args.length if reverse else 0.))
            target = (args.start_x, args.start_y + (0. if reverse else args.length))
            tangent = (0., float(sign))
            reset_env = os.environ.copy()
            reset_env['IGN_PARTITION'] = partition
            # Teleport only our owned simulator; do not reset the simulation clock.
            reset_req = f'name: "diffbot", position: {{x: {start[0]}, y: {start[1]}, z: 0.01}}, orientation: {{z: {math.sin(math.pi / 4)}, w: {math.cos(math.pi / 4)}}}'
            reset = subprocess.run(['ign', 'service', '-s', '/world/myworld2/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', reset_req], env=reset_env, capture_output=True, text=True, timeout=8)
            if reset.returncode or 'true' not in reset.stdout:
                raise RuntimeError('simulator pose reset failed: ' + reset.stdout + reset.stderr)
            reset_deadline = time.monotonic() + 5.
            reset_returned = time.monotonic()
            while time.monotonic() < reset_deadline:
                rclpy.spin_once(node, timeout_sec=.005)
                if latest.get('/odom', (0.,))[0] > reset_returned + .2:
                    break
            else:
                raise RuntimeError('odometry did not resume after simulator reset')
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.header.stamp = buffer.lookup_transform('odom', 'base_footprint', Time()).header.stamp
            initial.pose.pose.position.x, initial.pose.pose.position.y = start
            initial.pose.pose.orientation.z = math.sin(math.pi / 4)
            initial.pose.pose.orientation.w = math.cos(math.pi / 4)
            initial.pose.covariance[0] = initial.pose.covariance[7] = .0001
            initial.pose.covariance[35] = .0001
            initial_publisher.publish(initial)
            reset_samples = collect(5.)
            if not stopped_window(reset_samples) or math.dist(reset_samples[-1][1:3], start) > .10:
                raise RuntimeError('initial pose not ready after reset')
            p = sample()
            goal = NavigationService.Goal()
            goal.task_id = f'arrival_{index}'
            seg = NaviSegment()
            seg.segment_type = 1
            seg.segment_id = str(index)
            seg.segment_name = 'arrival_screen'
            seg.node1.x, seg.node1.y = start
            seg.node2.x, seg.node2.y = target
            seg.motion_direction = 2 if reverse else 1
            seg.max_speed = args.speed
            seg.max_load_speed = args.speed
            goal.navi_segment = [seg]
            parked_since = None
            begin = time.monotonic()
            active_case = {'index': index, 'direction': direction,
                           'begin_wall': begin, 'start': p[1:3], 'target': target, 'status': None,
                           'finish': False, 'accepted': False, 'goal_accepted': False, 'outcome': 'PENDING',
                           'path_geometry_valid': False}
            report['goals'].append(active_case)
            (args.output / 'results.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            handle = wait(client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('goal rejected')
            active_case['goal_accepted'] = True
            (args.output / 'results.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            t = buffer.lookup_transform('map', 'odom', Time(nanoseconds=int(p[8] * 1e9)))
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            guard = LocalizationCorrectionGuard(p, yaw)
            active_case['guard_reference_yaw'] = yaw
            goal_result_future = handle.get_result_async()
            result = wait(goal_result_future, args.goal_timeout, measure=True)
            end = time.monotonic()
            active_case['maximum_localization_correction_m'] = guard.maximum
            guard = None
            active_case.update(status=result.status, finish=result.result.finish,
                               outcome={4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(result.status, 'UNKNOWN'))
            post = collect(3.)
            window = first_stopped_window(post)
            if not window:
                raise RuntimeError('no complete stopped window after action result')
            metrics = evaluate(window, target, tangent)
            metrics['observed_post_position_valid'] = observed_position_valid(post, target, window[-1][0])
            metrics['passed'] = metrics['passed'] and metrics['observed_post_position_valid']
            metrics['odom_drift_m'] = max(math.hypot(s[6] - window[0][6], s[7] - window[0][7]) for s in window)
            zero, peak = command_evidence(trace, begin, window[0][0], window[-1][0])
            metrics['zero_output'] = zero
            metrics['post_peak_command_w'] = peak
            metrics['passed'] = metrics['passed'] and zero and peak is not None and peak <= .05
            nav = [s for s in trace if begin <= s[0] <= end and s[2] == '/cmd_vel_nav']
            crawl = sum(b[0] - a[0] for a, b in zip(nav, nav[1:]) if .005 <= abs(a[3]) <= args.creep * 1.05)
            measured = [s for s in trace if begin <= s[0] <= end and s[2] == '/odom']
            desired_sign = sign
            unexpected_reversal = any(desired_sign * s[3] < -.01 for s in measured)
            paths = [s for s in path_trace if begin <= s[0] <= end]
            geometry_valid = bool(paths) and all(s[1] == 'map' and valid_path(s[2], target) for s in paths)
            case = active_case
            case.update(motion_metrics(pose_trace, begin, end, target, args.speed, args.creep))
            case['acceptance_wall'] = window[-1][0]
            case['terminal_duration_s'] = window[-1][0] - case['terminal_entry_wall'] if case['terminal_entry_wall'] is not None else None
            case['total_duration_s'] = window[-1][0] - begin
            case['terminal_duration_sim_s'] = window[-1][8] - case['terminal_entry_sim'] if case['terminal_entry_sim'] is not None else None
            case['observation_sim_s'] = window[-1][8] - window[0][8]
            case.update({'index': index, 'direction': direction,
                    'begin_wall': begin, 'result_wall': end, 'start': p[1:3],
                    'status': result.status, 'finish': result.result.finish,
                    'duration_s': end - begin, 'raw_crawl_s': crawl, 'target': target,
                    'unexpected_reversal': unexpected_reversal,
                    'path_geometry_valid': geometry_valid,
                    'peak_measured_speed': max((abs(s[3]) for s in measured), default=0.),
                    'settling_diagnostic_3s': evaluate(post, target, tangent, duration=3.),
                    'post_stop': metrics, 'accepted': result.status == 4 and result.result.finish and metrics['passed'] and not unexpected_reversal and geometry_valid and case['speed_reached']})
            (args.output / f'post_{index}.json').write_text(json.dumps(post))
            (args.output / 'results.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(case), flush=True)
            handle = None
            if result.status != 4 or not result.result.finish:
                raise RuntimeError('action failed; stop alternating route to avoid leaving test corridor')
            if not case['accepted']:
                raise RuntimeError('acceptance failed; stop precision screening')
            active_case = None
    except (Exception, KeyboardInterrupt) as error:
        if guard is not None and active_case is not None:
            active_case['maximum_localization_correction_m'] = guard.maximum
        guard = None
        report['error'] = str(error)
        report['error_type'] = {RequestTimeout: 'ROS_REQUEST_TIMEOUT', GoalResultTimeout: 'GOAL_RESULT_TIMEOUT',
                                LocalizationAnomaly: 'LOCALIZATION_ANOMALY'}.get(type(error), 'TEST_FAILURE')
        if active_case is not None:
            active_case['error_type'] = report['error_type']
            if active_case['outcome'] == 'PENDING':
                active_case['outcome'] = report['error_type']
            active_case['accepted'] = False
            active_case['duration_s'] = time.monotonic() - active_case['begin_wall']
            paths = [s for s in path_trace if s[0] >= active_case['begin_wall']]
            active_case['path_geometry_valid'] = bool(paths) and all(s[1] == 'map' and valid_path(s[2], active_case['target']) for s in paths)
        print('SCREEN ERROR', error, flush=True)
        if handle is not None and handle.accepted:
            try:
                response = wait(handle.cancel_goal_async(), 5)
                report['cancel_acknowledged'] = bool(response.goals_canceling)
            except Exception as cancel_error:
                report['cancel_acknowledged'] = False
                report['cancel_error'] = str(cancel_error)
        try:
            stopped = collect(3.)
            report['stop_confirmed'] = stopped_window(stopped)
            (args.output / 'failure_stop.json').write_text(json.dumps(stopped, allow_nan=False))
            if goal_result_future is not None and goal_result_future.done() and active_case is not None:
                final_result = goal_result_future.result()
                active_case['cancel_result_status'] = final_result.status
        except Exception as stop_error:
            report['stop_confirmed'] = False
            report['stop_error'] = str(stop_error)
    finally:
        report['truth_frames'] = sorted(truth_frames)
        report['truth_time_basis'] = 'receipt_wall_time; Pose_V/TF bridge source stamp may be zero'
        (args.output / 'results.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        with gzip.open(args.output / 'velocity_trace.json.gz', 'wt') as raw:
            json.dump(trace, raw, allow_nan=False)
        with gzip.open(args.output / 'pose_trace.json.gz', 'wt') as raw:
            json.dump(pose_trace, raw, allow_nan=False)
        with gzip.open(args.output / 'localization_trace.json.gz', 'wt') as raw:
            json.dump(localization_trace, raw, allow_nan=False)
        with gzip.open(args.output / 'path_trace.json.gz', 'wt') as raw:
            json.dump(path_trace, raw, allow_nan=False)
        with gzip.open(args.output / 'ground_truth.json.gz', 'wt') as raw:
            json.dump(truth_trace, raw, allow_nan=False)
        with gzip.open(args.output / 'scan_trace.json.gz', 'wt') as raw:
            json.dump(scan_trace, raw, allow_nan=False)
        for owned in [truth_proc, proc]:
            if owned.poll() is not None:
                continue
            os.killpg(owned.pid, signal.SIGINT)
            try:
                owned.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(owned.pid, signal.SIGTERM)
                try:
                    owned.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(owned.pid, signal.SIGKILL)
                    owned.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        truth_log.close()
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())
