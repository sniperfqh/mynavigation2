# 唯一规控主入口，装配常驻遥控、fixed_path_exec 和 lattice_exec 两条执行通道，并分别管理生命周期。导航任务由 Action Goal 触发。

import os
import yaml
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


class NavigationRewrittenYaml(RewrittenYaml):
    """从共享参数基线生成固定路径与 Lattice 执行配置，保留每条通道自己的控制与限速参数。"""

    def __init__(self, **kwargs):
        self.navigation_kwargs = kwargs
        super().__init__(**kwargs)

    def loaded_fixed_path_parameters(self, context):
        """构造固定路径控制和限速配置，保留共享基线中更严格的纵向约束。"""
        import tempfile
        baseline = super().perform(context)
        with open(baseline, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs.get('root_key') or ''))
        config = document[namespace] if namespace else document
        smoother = config['velocity_smoother']['ros__parameters']
        # Preserve lateral/angular limits and controller braking geometry; only the x axis changes.
        limits = {
            'max_accel': (0.3, [2.0, 0.0, 3.2]),
            'max_decel': (-0.3, [-2.0, 0.0, -3.2]),
            'max_accel_jerk': (1.0, [6.0, 0.0, 4.0]),
            'max_decel_jerk': (1.0, [8.0, 0.0, 6.0]),
        }
        for name, (longitudinal, default) in limits.items():
            values = list(smoother.get(name, default))
            if len(values) != 3:
                raise ValueError(f'{name} must contain x, y and angular limits')
            current = float(values[0])
            if not math.isfinite(current) or current * longitudinal <= 0.0:
                raise ValueError(f'{name} must have a finite x limit with the correct sign')
            values[0] = max(current, longitudinal) if longitudinal < 0.0 else min(current, longitudinal)
            smoother[name] = values
        # The existing S-curve uses the smaller acceleration/deceleration magnitude for both.
        effective_deceleration = min(smoother['max_accel'][0], -smoother['max_decel'][0])
        planned_deceleration = float(config['controller_server']['ros__parameters']['FixedPathController'].get('goal_linear_deceleration', 0.25))
        if not math.isfinite(planned_deceleration) or planned_deceleration <= 0.0 or planned_deceleration > effective_deceleration:
            raise ValueError('fixed_path goal_linear_deceleration must be positive and no greater than the effective longitudinal S-curve limit')
        output = tempfile.NamedTemporaryFile(mode='w', delete=False)
        yaml.safe_dump(document, output)
        output.close()
        return output.name

    def fixed_parameters(self, context):
        """生成单次 Lattice 半自主通道配置，选择包装规划器、冻结路径控制器和分段到点检查器。"""
        import tempfile
        baseline = super().perform(context)
        with open(baseline, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs.get('root_key') or ''))
        config = document[namespace] if namespace else document
        # 此基线块提供障碍检查开关，当前半自主规划器由下方固定选择为 Lattice。
        profile = (config.get('autonomous_fixed_profile') or {}).get('ros__parameters') or {}
        obstacle_check_enabled = bool(profile.get('obstacle_check_enabled', True))
        # 当前 diff 基元含 16 个朝向，间隔 22.5°；最终航向容差约 9°，由包装器和协调器分别复核。
        lattice_yaw = 0.15726646259971647
        lattice_file = os.path.join(
            get_package_share_directory('nav2_smac_planner'),
            'sample_primitives', '5cm_resolution', '0.5m_turning_radius', 'diff', 'output.json')
        # 为 Lattice 通道生成矩形足迹和独立膨胀代价参数。
        footprint = '[[0.6, 0.45], [0.6, -0.45], [-0.6, -0.45], [-0.6, 0.45]]'
        inflation = 0.90
        for name in ['global_costmap', 'local_costmap']:
            costmap = config[name][name]['ros__parameters']
            costmap.update(footprint=footprint, footprint_padding=0.0)
            costmap.pop('robot_radius', None)
            costmap['inflation_layer'].update(inflation_radius=inflation, cost_scaling_factor=2.5)
        navigator = config['regulated_navigator']['ros__parameters']
        # Budget the error against the original action goal, including planner quantization.
        final_xy = float(navigator.get('autonomous_fixed_xy_tolerance', 0.01))
        if not 0.0 < final_xy < float('inf'):
            raise ValueError('autonomous_fixed_xy_tolerance must be finite and positive')
        final_xy = min(final_xy, 0.01)
        endpoint_budget = min(0.003, final_xy * 0.3)
        speed = float(navigator.get('autonomous_fixed_max_speed', 0.5))
        short_pause_timeout = float(navigator.get('autonomous_fixed_short_pause_timeout', 0.5))
        resume_stable_duration = float(navigator.get('autonomous_fixed_resume_stable_duration', 0.15))
        localization_timeout = float(navigator.get('localization_timeout', 0.3))
        tracking_localization_timeout = float(navigator.get('autonomous_fixed_tracking_localization_timeout', max(0.6, localization_timeout)))
        terminal_localization_distance = float(navigator.get('autonomous_fixed_terminal_localization_distance', 0.5))
        if (not all(math.isfinite(value) and value > 0.0 for value in
                    [localization_timeout, tracking_localization_timeout, terminal_localization_distance])
                or tracking_localization_timeout < localization_timeout):
            raise ValueError('autonomous_fixed localization timeouts and terminal distance must be positive; tracking timeout must cover the strict timeout')
        navigator.update(
                         planner_id='SmacLattice',
                         controller_id='AutonomousFixedPathController',
                         goal_checker_id='autonomous_fixed_goal_checker', use_smoother=False,
                         progress_timeout=10.0, replan_frequency=1.0,
                         autonomous_fixed_xy_tolerance=final_xy,
                         # 中间段复核容差与 cusp goal checker 对齐；状态格模式的终点旋转会带来厘米级滑移。
                         autonomous_fixed_cusp_xy_tolerance=0.02,
                         # 分段判据必须与控制器一致，否则 navigator 认为的单段会被控制器再次切分。
                         autonomous_fixed_rotation_distance=0.025,
                         # 旋转段位置验收半径，与控制器 rotation_position_tolerance 保持一致。
                         autonomous_fixed_rotation_position_tolerance=0.05,
                         autonomous_fixed_short_pause_timeout=short_pause_timeout,
                         autonomous_fixed_resume_stable_duration=resume_stable_duration,
                         autonomous_fixed_tracking_localization_timeout=tracking_localization_timeout,
                         autonomous_fixed_terminal_localization_distance=terminal_localization_distance)
        planner = config['planner_server']['ros__parameters']
        # 基元允许倒车：冻结路径按换向切段，段间短拼接残差按旋转段处理。
        # 内部平滑保持关闭，确保每个采样点保留自身状态格朝向。
        planner['planner_plugins'] = ['SmacLattice']
        planner['selected_planner'] = 'SmacLattice'
        planner.setdefault('SmacLattice', {}).update(
            plugin='nav2_regulated_modules::AutonomousLatticePlanner',
            lattice_filepath=lattice_file, tolerance=endpoint_budget,
            allow_reverse_expansion=True, max_planning_time=3.5, smooth_path=False)
        controller = config['controller_server']['ros__parameters']
        controller.update(controller_frequency=100.0,
                          controller_plugins=['AutonomousFixedPathController'],
                          goal_checker_plugins=['autonomous_fixed_goal_checker', 'autonomous_fixed_cusp_checker'])
        tracker = dict(controller['FixedPathController'])
        tracker.pop('plugin', None)
        tracker.update(desired_linear_vel=speed, rotate_to_heading_angular_vel=0.4,
                       max_angular_accel=0.8, terminal_convergence_enabled=True,
                       goal_final_approach_velocity=0.01,
                       goal_stop_entry_tolerance=final_xy * 0.5,
                       goal_terminal_lookahead_dist=0.08,
                       goal_terminal_lookahead_min_dist=0.05,
                       goal_terminal_lookahead_max_dist=0.12,
                       goal_terminal_lookahead_reference_speed=0.1,
                       goal_terminal_lookahead_speed_gain=0.2)
        controller['AutonomousFixedPathController'] = {
            'plugin': 'nav2_regulated_modules::AutonomousFixedPathController',
            'tracker': tracker, 'settle_duration': 0.5,
            'brake_stable_duration': 0.2, 'terminal_review_timeout': 2.0,
            'terminal_capture_margin': 0.02,
            'terminal_trim_max_speed': 0.02, 'terminal_trim_max_step': 0.02,
            'terminal_trim_max_distance': 0.03, 'terminal_trim_timeout': 5.0,
            'terminal_trim_max_attempts': 2, 'terminal_trim_confirm_duration': 0.2,
            'max_angular_velocity': 0.4, 'max_angular_acceleration': 0.8,
            # 状态格网格上的纯旋转基元可能残留几毫米位移，按 2.5 cm 归为旋转段；
            # 旋转段无位移可消除误差，位置验收放宽到 5 cm 以吸收格心量化。
            'rotation_distance': 0.025,
            'rotation_position_tolerance': 0.05,
            'localization_timeout': localization_timeout,
            'tracking_localization_timeout': tracking_localization_timeout,
            'terminal_localization_distance': terminal_localization_distance,
            'short_pause_timeout': short_pause_timeout,
            'resume_stable_duration': resume_stable_duration,
            'resume_distance': float(navigator.get('autonomous_fixed_resume_distance', 0.2)),
            'resume_linear_acceleration': 0.5,
            'obstacle_timeout': float(navigator.get('autonomous_fixed_obstacle_timeout', 60.0)),
            'obstacle_check_enabled': obstacle_check_enabled,
        }
        final_yaw = lattice_yaw
        cusp_xy = 0.02
        final_controller_xy = final_xy
        for name, xy, yaw in [('autonomous_fixed_goal_checker', final_controller_xy, final_yaw),
                              ('autonomous_fixed_cusp_checker', cusp_xy, 0.08726646259971647)]:
            controller[name] = {'plugin': 'nav2_controller::StoppedGoalChecker', 'stateful': False,
                                'xy_goal_tolerance': xy, 'yaw_goal_tolerance': yaw,
                                'trans_stopped_velocity': 0.01, 'rot_stopped_velocity': 0.05}
        controller['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
                                               required_movement_radius=0.01,
                                               required_movement_angle=0.05,
                                               movement_time_allowance=120.0)
        config['velocity_smoother']['ros__parameters'].update(
            smoothing_frequency=100.0, max_velocity=[speed, 0.0, 0.4], min_velocity=[-speed, 0.0, -0.4])
        with tempfile.NamedTemporaryFile(mode='w', prefix='autonomous-fixed-', delete=False, encoding='utf-8') as stream:
            yaml.safe_dump(document, stream)
            return stream.name


# 仿真入口的显式覆盖由现有 RewrittenYaml 统一处理，不再生成额外参数文件。
FIXED_PATH_OVERRIDES = {
    'desired_linear_vel': ('fixed_path_max_linear_velocity', 1.5),
    'approach_velocity_scaling_dist': ('fixed_path_approach_velocity_scaling_dist', 0.8),
    'goal_linear_deceleration': ('fixed_path_goal_linear_deceleration', 0.25),
    'goal_final_approach_velocity': ('fixed_path_goal_final_approach_velocity', 0.01),
    'goal_braking_reaction_time': ('fixed_path_goal_braking_reaction_time', 0.1),
    'goal_terminal_lookahead_dist': ('fixed_path_goal_terminal_lookahead_dist', 0.2),
    'goal_terminal_lookahead_reference_speed': ('fixed_path_goal_terminal_lookahead_reference_speed', 0.75),
    'goal_terminal_lookahead_speed_gain': ('fixed_path_goal_terminal_lookahead_speed_gain', 0.1),
    'goal_terminal_lookahead_min_dist': ('fixed_path_goal_terminal_lookahead_min_dist', 0.15),
    'goal_terminal_lookahead_max_dist': ('fixed_path_goal_terminal_lookahead_max_dist', 0.25),
    'rotate_to_heading_angular_vel': ('fixed_path_rotate_to_heading_angular_vel', 0.4),
    'max_angular_accel': ('fixed_path_max_angular_accel', 0.8),
    'rotate_to_heading_kp': ('fixed_path_rotate_to_heading_kp', 1.5),
    'goal_error_log_frequency': ('fixed_path_goal_error_log_frequency', 1.0),
}


# 读取参数文件并解析显式制动覆盖，空覆盖沿用 YAML 中的值。
def resolve_braking_margin_arguments(context):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    values = config.get('controller_server', {}).get('ros__parameters', {}).get('FixedPathController', {})
    defaults = {
        'goal_braking_distance_margin': 0.1,
        'dynamic_goal_braking_margin_enabled': True,
        'goal_braking_min_distance_margin': 0.03,
        'goal_braking_margin_transition_speed': 0.3,
    }
    actions = []
    for key, default in defaults.items():
        explicit = LaunchConfiguration('fixed_path_' + key).perform(context).strip()
        value = explicit if explicit else values.get(key, default)
        if isinstance(default, bool):
            value = str(value).lower()
            if value not in ('true', 'false'):
                raise ValueError('Invalid boolean fixed_path_' + key)
        else:
            value = str(float(value))
        actions.append(SetLaunchConfiguration('resolved_fixed_path_' + key, value))
    for key, (argument, default) in FIXED_PATH_OVERRIDES.items():
        explicit = LaunchConfiguration(argument, default='').perform(context).strip()
        value = float(explicit if explicit else values.get(key, default))
        actions.append(SetLaunchConfiguration('resolved_fixed_path_' + key, str(value)))
    controller = config.get('controller_server', {}).get('ros__parameters', {})
    navigator = config.get('regulated_navigator', {}).get('ros__parameters', {})
    tolerance = LaunchConfiguration('fixed_path_xy_goal_tolerance', default='').perform(context).strip()
    for checker in ['fixed_path_goal_checker', 'stopped_goal_checker']:
        value = controller.get(checker, {}).get('xy_goal_tolerance', 0.01)
        if tolerance and checker == 'fixed_path_goal_checker':
            value = float(tolerance)
        actions.append(SetLaunchConfiguration('resolved_' + checker + '_xy_goal_tolerance', str(value)))
    use_smoother = LaunchConfiguration('use_path_smoother', default='').perform(context).strip().lower()
    if use_smoother and use_smoother not in ('true', 'false'):
        raise ValueError('Invalid boolean use_path_smoother')
    actions.append(SetLaunchConfiguration('resolved_use_path_smoother',
                   use_smoother or str(navigator.get('use_smoother', True)).lower()))
    return actions


# 核对碰撞状态话题和超时参数，再转成供两通道复用的启动配置。
def resolve_collision_state_arguments(context):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    namespace = LaunchConfiguration('namespace').perform(context)
    config = config.get(namespace, config) if namespace else config
    values = config.get('velocity_smoother', {}).get('ros__parameters', {})
    topic = values.get('collision_state_topic', '/collision_state')
    timeout = float(values.get('collision_state_timeout', 0.5))
    if not isinstance(topic, str) or not topic.strip() or not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError('Invalid collision state topic or timeout')
    return [SetLaunchConfiguration('resolved_collision_state_topic', topic),
            SetLaunchConfiguration('resolved_collision_state_timeout', str(timeout))]


def launch_unified(context, rewrites, rviz):
    """从共享 YAML 派生两份配置，为地图、协调器及执行通道分别创建生命周期管理组。"""
    import tempfile

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = arg('namespace').strip('/') if arg('use_namespace').lower() == 'true' else ''
    prefix = '/' + ns + '/' if ns else '/'
    source = arg('params_file')
    substitutions = dict(rewrites)
    enabled = arg('enable_collision_state').lower() == 'true'
    for name in ('regulated_navigator', 'controller_server', 'velocity_smoother'):
        substitutions[name + '.ros__parameters.collision_state_enabled'] = str(enabled).lower()
    profiles = NavigationRewrittenYaml(
        source_file=source, root_key='', param_rewrites=substitutions, convert_types=True)
    # 半自主执行通道固定选择 Lattice，即使共享基线中仍记录 Hybrid 选项。
    adaptive = arg('adaptive_goal_braking_enabled').lower() == 'true'
    fixed_yaml = (RewrittenYaml.perform(profiles, context) if adaptive else
                  profiles.loaded_fixed_path_parameters(context))
    lattice_yaml = profiles.fixed_parameters(context)

    # 读取启动替换后的参数文件，返回可按执行通道独立调整的字典。
    def load_profile(filename):
        with open(filename, encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    fixed = load_profile(fixed_yaml)
    lattice = load_profile(lattice_yaml)
    fixed_controller = fixed['controller_server']['ros__parameters']
    fixed_controller['progress_checker']['movement_time_allowance'] = float(
        arg('fixed_path_progress_timeout'))
    navigator_parameters = fixed['regulated_navigator']['ros__parameters']
    fixed_controller['controller_plugins'] = [
        navigator_parameters.get('fixed_path_controller_id', 'FixedPathController')]
    fixed_controller['goal_checker_plugins'] = [
        navigator_parameters.get('fixed_path_goal_checker_id', 'fixed_path_goal_checker')]
    # 充电专用插件：charging_controller_id／charging_checker_id 非空即在 fixed 通道一并注册，
    # 否则 Charging 下发的 FollowPath 会因控制器名不存在被 controller_server 拒绝；
    # 两者的参数块由共享 YAML 的 controller_server 同名键携带，无需在此展开。
    charging_controller = navigator_parameters.get('charging_controller_id', '')
    if charging_controller:
        fixed_controller['controller_plugins'].append(charging_controller)
    charging_checker = navigator_parameters.get('charging_checker_id', '')
    if charging_checker:
        fixed_controller['goal_checker_plugins'].append(charging_checker)
    if adaptive:
        fixed['velocity_smoother']['ros__parameters'].update(
            max_decel=[-1.0, 0.0, -3.2], immediate_stop_on_zero_command=True)

    commands = [rviz]
    tf_remaps = [('tf', '/tf'), ('tf_static', '/tf_static')]
    use_sim_time = arg('use_sim_time').lower() == 'true'
    gate = arg('use_collision_velocity_gate').lower() == 'true'

    def node(package, executable, name, namespace, params, remaps=()):
        commands.append(Node(
            package=package, executable=executable, name=name, namespace=namespace,
            output='both', parameters=params, remappings=list(remaps),
            respawn=arg('use_respawn').lower() == 'true',
            arguments=['--ros-args', '--log-level', arg('log_level')]))

    # 为指定命名空间添加生命周期管理器，autostart 决定是否自动配置并激活所列节点。
    def manage(label, namespace, names):
        node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_' + label,
             namespace, [{'use_sim_time': use_sim_time, 'autostart':
                          arg('autostart').lower() == 'true', 'node_names': names}])

    node('nav2_map_server', 'map_server', 'map_server', ns,
         [fixed['map_server']['ros__parameters']])
    manage('map', ns, ['map_server'])

    readiness = {}
    for channel, profile in [('fixed_path_exec', fixed), ('lattice_exec', lattice)]:
        channel_ns = prefix + channel
        names = ['controller_server', 'velocity_smoother']
        if channel == 'lattice_exec':
            names.append('planner_server')
        if gate:
            names.append('collision_monitor')
        for name in ('local_costmap', 'global_costmap'):
            values = profile[name][name]['ros__parameters']
            values['map_topic'] = prefix + 'map'
        for name in ('controller_server', 'velocity_smoother'):
            profile[name]['ros__parameters']['speed_limit_topic'] = 'speed_limit'
        # Costmap2DROS 由所属服务器创建，生成文件同时携带服务器及成本地图的完整命名空间键。
        with tempfile.NamedTemporaryFile(
                mode='w', prefix=channel + '-', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump({channel_ns.strip('/'): profile}, stream)
            filename = stream.name
        node('nav2_controller', 'controller_server', 'controller_server', channel_ns,
             [filename], tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', channel_ns,
             [filename], [('cmd_vel', 'cmd_vel_nav'),
                          ('cmd_vel_smoothed', 'cmd_vel_collision_in' if gate else 'cmd_vel')])
        if channel == 'lattice_exec':
            node('nav2_planner', 'planner_server', 'planner_server', channel_ns,
                 [filename], tf_remaps)
        if gate:
            node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', channel_ns,
                 [filename, {'cmd_vel_in_topic': 'cmd_vel_collision_in',
                             'cmd_vel_out_topic': 'cmd_vel'}], tf_remaps)
        manage('execution', channel_ns, names)
        readiness[channel] = [channel_ns + '/' + name for name in names]

    navigator = dict(fixed['regulated_navigator']['ros__parameters'])
    lattice_navigator = lattice['regulated_navigator']['ros__parameters']
    navigator.update({key: value for key, value in lattice_navigator.items()
                      if key.startswith('autonomous_fixed_')})
    # 协调器使用共享参数中的底盘接口，并把规划、跟踪和限速话题分别指向对应执行通道。
    navigator.update(
        planner_id='SmacLattice', use_smoother=False,
        enable_localization_jump_detection=arg('enable_localization_jump_detection').lower() == 'true',
        progress_timeout=float(arg('fixed_path_progress_timeout')),
        compute_path_to_pose_action='lattice_exec/compute_path_to_pose',
        follow_path_action='fixed_path_exec/follow_path',
        stop_cmd_vel_topic='fixed_path_exec/cmd_vel_nav',
        speed_limit_topic='fixed_path_exec/speed_limit',
        chassis_navigation_topic='fixed_path_exec/cmd_vel',
        controller_cmd_vel_topic='lattice_exec/cmd_vel_nav',
        smoothed_cmd_vel_topic='lattice_exec/cmd_vel',
        fixed_execution_nodes=readiness['fixed_path_exec'],
        lattice_execution_nodes=readiness['lattice_exec'] + [prefix + 'map_server'])
    node('nav2_regulated_modules', 'regulated_navigator_node', 'regulated_navigator',
         ns, [navigator], tf_remaps)
    manage('navigator', ns, ['regulated_navigator'])
    return commands


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_regulated_modules')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    # 与 entry.launch.py 相同的语义：use_collision_monitor 是碰撞检测模块总开关，
    # 速度门由独立的 use_collision_velocity_gate 控制。
    use_collision_monitor = LaunchConfiguration('use_collision_monitor', default='false')
    use_obstacle_status = LaunchConfiguration('use_obstacle_status', default='true')
    use_car_op_status_sim = LaunchConfiguration('use_car_op_status_sim', default='true')
    # ooii: 新接口 RobotMoveState（uint8 九值 0–8）；默认 7 = 静止。
    car_op_status_value = LaunchConfiguration('car_op_status_value', default='7')
    car_op_status_topic = LaunchConfiguration('car_op_status_topic', default='/robot_move_state')
    scan_topic = LaunchConfiguration('scan_topic', default='/c200_lidar_node/scan')
    zone_frame = LaunchConfiguration('zone_frame', default='base_link')
    legacy_velocity_logging = PythonExpression([
        "'", LaunchConfiguration('enable_legacy_velocity_logs'), "'.lower() == 'true' or '",
        LaunchConfiguration('enable_velocity_diagnostics'), "'.lower() != 'true'"])
    adaptive_goal_braking_enabled = LaunchConfiguration('adaptive_goal_braking_enabled')
    param_substitutions = {
        **{node + '.ros__parameters.legacy_velocity_logging': legacy_velocity_logging
           for node in ['regulated_navigator', 'velocity_smoother']},
        **{node + '.ros__parameters.' + key: value
           for node in ['controller_server', 'velocity_smoother', 'regulated_navigator']
           for key, value in {
               'collision_state_topic': LaunchConfiguration('resolved_collision_state_topic'),
               'collision_state_timeout': LaunchConfiguration('resolved_collision_state_timeout'),
           }.items()},
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'controller_server.ros__parameters.FixedPathController.adaptive_goal_braking_enabled': adaptive_goal_braking_enabled,
        **{'controller_server.ros__parameters.FixedPathController.' + key:
           LaunchConfiguration('resolved_fixed_path_' + key) for key in
           ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
            'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']},
        **{'controller_server.ros__parameters.FixedPathController.' + key:
           LaunchConfiguration('resolved_fixed_path_' + key) for key in FIXED_PATH_OVERRIDES},
        **{'controller_server.ros__parameters.' + checker + '.xy_goal_tolerance':
           LaunchConfiguration('resolved_' + checker + '_xy_goal_tolerance')
           for checker in ['fixed_path_goal_checker', 'stopped_goal_checker']},
        'regulated_navigator.ros__parameters.use_smoother': LaunchConfiguration('resolved_use_path_smoother'),
    }

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    spdlog_log_dir_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_LOG_DIR', LaunchConfiguration('log_dir'))

    spdlog_console_level_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_CONSOLE_LEVEL', 'info')

    spdlog_file_level_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_FILE_LEVEL', EnvironmentVariable('SPDLOG_WRAPPER_FILE_LEVEL', default_value='trace'))

    spdlog_flush_interval_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_FLUSH_INTERVAL_SECONDS', '1')

    # 声明顶层命名空间；空字符串表示默认全局命名空间。
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    # 控制是否将规控节点放入指定命名空间。
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='False', description='Whether to apply a namespace to the navigation stack')

    # 指定地图文件；默认读取包内地图资源。
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=os.path.join('/userdata/map/pbstream', 'out.yaml'),description='Full path to map yaml file to load')

    # 决定是否使用仿真时钟，影响 TF 和超时判定。
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    # 指定规控栈参数 YAML；后续由 RewrittenYaml 注入启动实参。
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'regulated_modules.yaml'),
        description='Full path to the ROS 2 parameters file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(bringup_dir,'rviz','unified.rviz'))
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz',default_value='false')
    # 碰撞检测模块总开关：与 entry.launch.py 一致，只发状态、不改速度。
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        'use_collision_monitor',
        default_value='false',
        description='Start the collision detection module (dynamic zones + default status source + translator when use_obstacle_status). Status only; never changes velocity')

    # 独立的速度门开关；与状态模块互不影响，可单独或同时启用。
    declare_use_collision_velocity_gate_cmd = DeclareLaunchArgument(
        'use_collision_velocity_gate',
        default_value='false',
        description='Start the Nav2 collision_monitor velocity gate and insert it into the velocity chain')

    # 枚举输出开关：只在模块开启时决定是否启动 translator。
    declare_use_obstacle_status_cmd = DeclareLaunchArgument(
        'use_obstacle_status',
        default_value='true',
        description='With use_collision_monitor: start obstacle_status_translator and publish /collision_state + /obstacle_status_int; it never publishes /cmd_vel')

    # 默认 RobotMoveState 状态源；其它发布者出现时自动让位。
    declare_use_car_op_status_sim_cmd = DeclareLaunchArgument(
        'use_car_op_status_sim',
        default_value='true',
        description='With use_collision_monitor: start the default RobotMoveState source; it yields while another publisher exists')

    declare_car_op_status_value_cmd = DeclareLaunchArgument(
        'car_op_status_value',
        default_value='7',
        description='ooii: default source robot_move_state 0-8 (7 = stationary)')

    declare_car_op_status_topic_cmd = DeclareLaunchArgument(
        'car_op_status_topic',
        default_value='/robot_move_state',
        description='ooii: byd_custom_msgs/RobotMoveState topic shared by the default source and car_op_zones')

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/c200_lidar_node/scan',
        description='Laser scan topic used by the status module translator')

    declare_zone_frame_cmd = DeclareLaunchArgument(
        'zone_frame',
        default_value='base_link',
        description='Frame the dynamic zones and scan are projected into')

    # 默认启动碰撞区域 Marker 可视化，仍可通过启动参数关闭。
    declare_use_collision_visualization_cmd = DeclareLaunchArgument(
        'use_collision_visualization',
        default_value='true',# 导航模式的显示开关，不产生碰撞状态。
        description='Whether to start the Collision Monitor boundary visualizer')

    # 控制 Lifecycle Manager 是否自动激活受管节点。
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup lifecycle nodes')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn nodes if they crash when composition is disabled')

    # 向各 Nav2 节点传递日志级别。
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    declare_adaptive_goal_braking_cmd = DeclareLaunchArgument(
        'adaptive_goal_braking_enabled',
        default_value='false',
        description='Opt in to unvalidated adaptive fixed-path goal braking')

    # 控制定位突跳停车检测；TF 丢失保护不受此开关影响。
    declare_enable_localization_jump_detection_cmd = DeclareLaunchArgument(
        'enable_localization_jump_detection',
        default_value='false',
        description='Whether localization pose jumps cancel control and stop the robot')

    # 配置固定路径无进展的独立超时值，单位秒。
    declare_fixed_path_progress_timeout_cmd = DeclareLaunchArgument(
        'fixed_path_progress_timeout',
        default_value='120.0',
        description='Maximum stationary time allowed in fixed_path mode')

    status_nodes = [
        Node(
            condition=IfCondition(use_collision_monitor),
            package='nav2_collision_monitor',
            executable='car_op_zones.py',
            name='car_op_zones',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # 实车入口固定用实车尺寸档（1.2 × 0.8 m）；仿真入口传 --profile sim。
            arguments=['--drive-monitor', '--frame-id', zone_frame, '--period', '0.1',
                       '--profile', 'real',
                       # ooii: 新接口 RobotMoveState；话题由 car_op_status_topic 统一。
                       '--op-topic', car_op_status_topic]),
        Node(
            condition=IfCondition(PythonExpression([
                "'", use_collision_monitor, "'.lower() == 'true' and '",
                use_car_op_status_sim, "'.lower() == 'true'"])),
            package='nav2_collision_monitor',
            executable='car_op_status_sim.py',
            name='car_op_status_sim',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--mode', 'hold', '--value', car_op_status_value,
                       '--topic', car_op_status_topic, '--yield-to-other-publishers']),
        Node(
            condition=IfCondition(PythonExpression([
                "'", use_collision_monitor, "'.lower() == 'true' and '",
                use_obstacle_status, "'.lower() == 'true'"])),
            package='nav2_collision_monitor',
            executable='obstacle_status_translator.py',
            name='obstacle_status_translator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--scan-topic', scan_topic,
                       '--stop-topic', '/car_op_zones/stop',
                       '--slow-l2-topic', '/car_op_zones/slow_l2',
                       '--slow-l1-topic', '/car_op_zones/slow_l1',
                       '--zone-frame', zone_frame,
                       # 与上面的 car_op_zones 同档；用于屏蔽雷达扫到的自身车体。
                       '--profile', 'real',
                       '--no-passthrough']),
    ]
    panel_remaps = [
        (old + '/' + service, new + '/' + service)
        for old, new in [('lifecycle_manager_navigation', 'lifecycle_manager_navigator'),
                         ('lifecycle_manager_localization', 'lifecycle_manager_map')]
        for service in ('is_active', 'manage_nodes')]
    rviz_cmd = Node(
        package='rviz2', executable='rviz2', condition=IfCondition(use_rviz),
        namespace=PythonExpression(["'", namespace, "' if '", LaunchConfiguration('use_namespace'),
                                    "'.lower() == 'true' else ''"]),
        arguments=['-d', rviz_config_file], parameters=[{'use_sim_time': use_sim_time}],
        remappings=panel_remaps)
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('enable_velocity_diagnostics', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_legacy_velocity_logs', default_value='false'))
    ld.add_action(DeclareLaunchArgument('log_dir', default_value=EnvironmentVariable('SPDLOG_WRAPPER_LOG_DIR', default_value='/tmp/nav2_logs')))
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(spdlog_log_dir_envvar)
    ld.add_action(spdlog_console_level_envvar)
    ld.add_action(spdlog_file_level_envvar)
    ld.add_action(spdlog_flush_interval_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(DeclareLaunchArgument('enable_collision_state', default_value='true',
                                      description='Enable fixed-path collision state gating before S-curve smoothing (default on)'))
    ld.add_action(OpaqueFunction(function=resolve_collision_state_arguments))
    for key in ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
                'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']:
        ld.add_action(DeclareLaunchArgument('fixed_path_' + key, default_value='',
                                           description='Explicit FixedPathController override; empty uses parameter YAML'))
    for argument, _ in FIXED_PATH_OVERRIDES.values():
        ld.add_action(DeclareLaunchArgument(argument, default_value='',
                                           description='Explicit FixedPathController override; empty uses parameter YAML'))
    ld.add_action(DeclareLaunchArgument('fixed_path_xy_goal_tolerance', default_value='',
                                       description='Position tolerance override; empty uses parameter YAML'))
    ld.add_action(DeclareLaunchArgument('use_path_smoother', default_value='',
                                       description='Override navigator path smoothing; empty uses parameter YAML'))
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_collision_velocity_gate_cmd)
    ld.add_action(declare_use_obstacle_status_cmd)
    ld.add_action(declare_use_car_op_status_sim_cmd)
    ld.add_action(declare_car_op_status_value_cmd)
    ld.add_action(declare_car_op_status_topic_cmd)   # ooii: RobotMoveState 话题
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(declare_zone_frame_cmd)
    ld.add_action(declare_use_collision_visualization_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(OpaqueFunction(function=resolve_braking_margin_arguments))
    ld.add_action(declare_adaptive_goal_braking_cmd)
    ld.add_action(declare_enable_localization_jump_detection_cmd)
    ld.add_action(declare_fixed_path_progress_timeout_cmd)
    ld.add_action(OpaqueFunction(function=launch_unified, kwargs={'rewrites':param_substitutions,'rviz':rviz_cmd}))
    for node in status_nodes:
        ld.add_action(node)
    return ld
