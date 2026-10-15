"""noamcl simulation: external localization exclusively publishes map -> base_link.

Subscribe to /scan with the base_link -> base_scan extrinsics. /odometry is
Gazebo velocity feedback for controllers; /motion_state feeds the smoother.
No Gazebo bottom-chassis TF or substitute localization is published here.
"""

import os
import tempfile
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def prepare_external_localization_parameters(context):
    """Keep simulation dynamics, but use the real vehicle's velocity/TF interfaces."""
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    config.pop('amcl', None)

    def adapt_frames(values):
        if isinstance(values, dict):
            for key, value in values.items():
                if key in ('robot_base_frame', 'base_frame_id'):
                    values[key] = 'base_link'
                elif key in ('global_frame', 'global_frame_id', 'odom_frame_id') and value == 'odom':
                    values[key] = 'map'
                elif key in ('odom_topic', 'velocity_odom_topic') and value == '/odom':
                    values[key] = '/odometry'
                else:
                    adapt_frames(value)
        elif isinstance(values, list):
            for value in values:
                adapt_frames(value)

    adapt_frames(config)
    mode = LaunchConfiguration('operation_mode').perform(context)
    navigator = config['regulated_navigator']['ros__parameters']
    navigator.update(operation_mode=mode, use_smoother=mode != 'fixed_path',
                     controller_cmd_vel_topic='cmd_vel_nav', smoothed_cmd_vel_topic='cmd_vel',
                     velocity_odom_topic='/odometry')
    controller = config['controller_server']['ros__parameters']
    controller['odom_topic'] = '/odometry'
    config['velocity_smoother']['ros__parameters']['odom_topic'] = '/motion_state'
    fixed = controller['FixedPathController']
    # Preserve the original simulation wrapper's explicit motion-parameter overrides.
    overrides = {
        'desired_linear_vel': 'fixed_path_max_linear_velocity',
        'approach_velocity_scaling_dist': 'fixed_path_approach_velocity_scaling_dist',
        'goal_linear_deceleration': 'fixed_path_goal_linear_deceleration',
        'goal_final_approach_velocity': 'fixed_path_goal_final_approach_velocity',
        'goal_braking_reaction_time': 'fixed_path_goal_braking_reaction_time',
        'goal_terminal_lookahead_dist': 'fixed_path_goal_terminal_lookahead_dist',
        'goal_terminal_lookahead_reference_speed': 'fixed_path_goal_terminal_lookahead_reference_speed',
        'goal_terminal_lookahead_speed_gain': 'fixed_path_goal_terminal_lookahead_speed_gain',
        'goal_terminal_lookahead_min_dist': 'fixed_path_goal_terminal_lookahead_min_dist',
        'goal_terminal_lookahead_max_dist': 'fixed_path_goal_terminal_lookahead_max_dist',
        'rotate_to_heading_angular_vel': 'fixed_path_rotate_to_heading_angular_vel',
        'max_angular_accel': 'fixed_path_max_angular_accel',
        'rotate_to_heading_kp': 'fixed_path_rotate_to_heading_kp',
        'goal_error_log_frequency': 'fixed_path_goal_error_log_frequency',
    }
    for key, argument in overrides.items():
        explicit = LaunchConfiguration(argument).perform(context).strip()
        if explicit:
            fixed[key] = float(explicit)
    tolerance = float(LaunchConfiguration('fixed_path_xy_goal_tolerance').perform(context))
    controller['fixed_path_goal_checker']['xy_goal_tolerance'] = tolerance
    controller['stopped_goal_checker']['xy_goal_tolerance'] = tolerance if mode == 'fixed_path' else 0.03
    with tempfile.NamedTemporaryFile(mode='w', prefix='noamcl_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        path = stream.name
    return [SetLaunchConfiguration('noamcl_params_file', path)]


def generate_launch_description():
    default_ign_partition = 'myworld_bringup_{}_{}'.format(os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid())
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ign_partition_name = LaunchConfiguration('ign_partition', default=default_ign_partition)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor', default='false')
    use_collision_visualization = LaunchConfiguration('use_collision_visualization', default='true')
    headless = LaunchConfiguration('headless', default='true')
    operation_mode = LaunchConfiguration('operation_mode', default='autonomous')
    enable_localization_jump_detection = LaunchConfiguration('enable_localization_jump_detection', default='false')
    fixed_path_progress_timeout = LaunchConfiguration('fixed_path_progress_timeout', default='120.0')
    fixed_path_max_linear_velocity = LaunchConfiguration('fixed_path_max_linear_velocity', default='1.5')
    fixed_path_approach_velocity_scaling_dist = LaunchConfiguration('fixed_path_approach_velocity_scaling_dist', default='0.8')
    fixed_path_goal_linear_deceleration = LaunchConfiguration('fixed_path_goal_linear_deceleration', default='0.25')
    adaptive_goal_braking_enabled = LaunchConfiguration('adaptive_goal_braking_enabled', default='false')
    fixed_path_goal_final_approach_velocity = LaunchConfiguration('fixed_path_goal_final_approach_velocity', default='')
    fixed_path_goal_braking_reaction_time = LaunchConfiguration('fixed_path_goal_braking_reaction_time', default='0.1')
    fixed_path_goal_braking_distance_margin = LaunchConfiguration('fixed_path_goal_braking_distance_margin', default='')
    fixed_path_dynamic_goal_braking_margin_enabled = LaunchConfiguration('fixed_path_dynamic_goal_braking_margin_enabled', default='')
    fixed_path_goal_braking_min_distance_margin = LaunchConfiguration('fixed_path_goal_braking_min_distance_margin', default='')
    fixed_path_goal_braking_margin_transition_speed = LaunchConfiguration('fixed_path_goal_braking_margin_transition_speed', default='')
    fixed_path_goal_terminal_lookahead_dist = LaunchConfiguration('fixed_path_goal_terminal_lookahead_dist', default='')
    fixed_path_goal_terminal_lookahead_reference_speed = LaunchConfiguration('fixed_path_goal_terminal_lookahead_reference_speed', default='0.75')
    fixed_path_goal_terminal_lookahead_speed_gain = LaunchConfiguration('fixed_path_goal_terminal_lookahead_speed_gain', default='0.1')
    fixed_path_goal_terminal_lookahead_min_dist = LaunchConfiguration('fixed_path_goal_terminal_lookahead_min_dist', default='')
    fixed_path_goal_terminal_lookahead_max_dist = LaunchConfiguration('fixed_path_goal_terminal_lookahead_max_dist', default='')
    fixed_path_xy_goal_tolerance = LaunchConfiguration('fixed_path_xy_goal_tolerance', default='0.01')
    fixed_path_rotate_to_heading_angular_vel = LaunchConfiguration('fixed_path_rotate_to_heading_angular_vel', default='0.4')
    fixed_path_max_angular_accel = LaunchConfiguration('fixed_path_max_angular_accel', default='0.8')
    fixed_path_rotate_to_heading_kp = LaunchConfiguration('fixed_path_rotate_to_heading_kp', default='1.5')
    fixed_path_goal_error_log_frequency = LaunchConfiguration('fixed_path_goal_error_log_frequency', default='1.0')
    fixed_path_stack_start_delay = LaunchConfiguration('fixed_path_stack_start_delay', default='3.0')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay', default='5.0')

    is_autonomous = PythonExpression(["'", operation_mode, "' == 'autonomous'"])
    is_fixed_path = PythonExpression(["'", operation_mode, "' == 'fixed_path'"])
    is_remote = PythonExpression(["'", operation_mode, "' == 'remote'"])
    is_autonomous_gui = PythonExpression(["'", operation_mode, "' == 'autonomous' and '", headless, "' == 'false'"])
    is_autonomous_headless = PythonExpression(["'", operation_mode, "' == 'autonomous' and '", headless, "' == 'true'"])
    is_fixed_path_gui = PythonExpression(["'", operation_mode, "' == 'fixed_path' and '", headless, "' == 'false'"])
    is_remote_gui = PythonExpression(["'", operation_mode, "' == 'remote' and '", headless, "' == 'false'"])

    bringup_dir = get_package_share_directory('myworld_bringup')
    myworld_dir = os.path.join(bringup_dir, 'models', 'myworld2')
    default_map = PythonExpression([
        "'", os.path.join('/userdata/map/pbstream', 'out.yaml'),
        "' if '", operation_mode, "' == 'fixed_path' else '",
        os.path.join(myworld_dir, 'myworld2.yaml'), "'"])
    map_file = LaunchConfiguration('map', default=default_map)
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    robot_description_file = os.path.join(bringup_dir, 'urdf', 'diffbot.urdf')
    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot = ET.fromstring(urdf_file.read())
    # External localization owns map -> base_link. Keep base_link a URDF root;
    # publishing base_footprint -> base_link would give base_link two parents.
    base_joint = robot.find("joint[@name='base_joint']")
    origin = base_joint.find('origin')
    if origin.get('xyz') != '0 0 0' or origin.get('rpy') != '0 0 0':
        raise ValueError('noamcl expects the existing zero-offset base_joint')
    base_joint.find('parent').set('link', 'base_link')
    base_joint.find('child').set('link', 'base_footprint')
    robot_description = ET.tostring(robot, encoding='unicode')

    resource_path = [os.path.join('/opt/ros/humble', 'share'), ':' + os.path.join(bringup_dir, 'models'), ':' + myworld_dir, ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path)
    ign_partition = SetEnvironmentVariable(name='IGN_PARTITION', value=ign_partition_name)

    world_only = LaunchConfiguration('world_file', default=os.path.join(myworld_dir, 'world_only.sdf'))
    ignition_sim = ExecuteProcess(condition=IfCondition(is_autonomous_gui), cmd=['ign', 'gazebo', '-r', '-v', '3', world_only], output='screen')
    ignition_server = ExecuteProcess(condition=IfCondition(is_autonomous_headless), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    fixed_path_ignition_sim = ExecuteProcess(condition=IfCondition(is_fixed_path_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    fixed_path_ignition_server = ExecuteProcess(condition=IfCondition(is_fixed_path), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    remote_ignition_gui = ExecuteProcess(condition=IfCondition(is_remote_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    remote_ignition_server = ExecuteProcess(condition=IfCondition(is_remote), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')

    algorithm_arguments = {'map': map_file,
        **{key: LaunchConfiguration(key) for key in ['enable_velocity_diagnostics', 'log_dir', 'velocity_file_log_frequency', 'velocity_console_log_frequency']},
        'use_sim_time': use_sim_time,
        'params_file': LaunchConfiguration('noamcl_params_file'),
        'rviz_config_file': rviz_config_file,
        'use_rviz': 'false',
        'namespace': '',
        'use_namespace': 'false',
        'use_collision_monitor': use_collision_monitor,
        'use_collision_visualization': use_collision_visualization,
        'autostart': 'true',
        'use_composition': 'False',
        'container_name': 'nav2_regulated_container',
        'use_respawn': 'False',
        'log_level': 'info',
        'operation_mode': operation_mode,
        'enable_localization_jump_detection': enable_localization_jump_detection,
        'fixed_path_progress_timeout': fixed_path_progress_timeout,
        'fixed_path_max_linear_velocity': fixed_path_max_linear_velocity,
        'fixed_path_approach_velocity_scaling_dist': fixed_path_approach_velocity_scaling_dist,
        'fixed_path_goal_linear_deceleration': fixed_path_goal_linear_deceleration,
        'adaptive_goal_braking_enabled': adaptive_goal_braking_enabled,
        'fixed_path_goal_final_approach_velocity': fixed_path_goal_final_approach_velocity,
        'fixed_path_goal_braking_reaction_time': fixed_path_goal_braking_reaction_time,
        'fixed_path_goal_braking_distance_margin': fixed_path_goal_braking_distance_margin,
        'fixed_path_dynamic_goal_braking_margin_enabled': fixed_path_dynamic_goal_braking_margin_enabled,
        'fixed_path_goal_braking_min_distance_margin': fixed_path_goal_braking_min_distance_margin,
        'fixed_path_goal_braking_margin_transition_speed': fixed_path_goal_braking_margin_transition_speed,
        'fixed_path_goal_terminal_lookahead_dist': fixed_path_goal_terminal_lookahead_dist,
        'fixed_path_goal_terminal_lookahead_reference_speed': fixed_path_goal_terminal_lookahead_reference_speed,
        'fixed_path_goal_terminal_lookahead_speed_gain': fixed_path_goal_terminal_lookahead_speed_gain,
        'fixed_path_goal_terminal_lookahead_min_dist': fixed_path_goal_terminal_lookahead_min_dist,
        'fixed_path_goal_terminal_lookahead_max_dist': fixed_path_goal_terminal_lookahead_max_dist,
        'fixed_path_xy_goal_tolerance': fixed_path_xy_goal_tolerance,
        'fixed_path_rotate_to_heading_angular_vel': fixed_path_rotate_to_heading_angular_vel,
        'fixed_path_max_angular_accel': fixed_path_max_angular_accel,
        'fixed_path_rotate_to_heading_kp': fixed_path_rotate_to_heading_kp,
        'fixed_path_goal_error_log_frequency': fixed_path_goal_error_log_frequency,
    }

    regulated_autonomous = IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('nav2_regulated_modules'), '/launch/regulated_modules.launch.py']), condition=IfCondition(is_autonomous),
        launch_arguments=algorithm_arguments.items())

    regulated_fixed_path = TimerAction(period=fixed_path_stack_start_delay, condition=IfCondition(is_fixed_path), actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_regulated_modules'), '/launch/regulated_modules.launch.py']), launch_arguments=algorithm_arguments.items())])

    return LaunchDescription([
        DeclareLaunchArgument('enable_velocity_diagnostics', default_value='true'),
        DeclareLaunchArgument('log_dir', default_value=os.environ.get('SPDLOG_WRAPPER_LOG_DIR', '/tmp/nav2_logs')),
        DeclareLaunchArgument('velocity_file_log_frequency', default_value='100.0'),
        DeclareLaunchArgument('velocity_console_log_frequency', default_value='1.0'),ign_resource_path,
        ign_partition,
        ignition_sim,
        ignition_server,
        fixed_path_ignition_sim,
        fixed_path_ignition_server,
        remote_ignition_gui,
        remote_ignition_server,

        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'use_sim_time': use_sim_time,
                'robot_description': robot_description}]),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use system timestamps by default; true restores Gazebo simulation time'),
        DeclareLaunchArgument('map', default_value=default_map, description='Map YAML: myworld3 for fixed_path, myworld2 for other modes; explicit map overrides the default'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'myworld2.yaml'), description='Explicit simulation parameter file for isolated experiments'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(myworld_dir, 'world_only.sdf'), description='Explicit simulation SDF for isolated experiments'),
        DeclareLaunchArgument('ign_partition', default_value=default_ign_partition, description='Unique Ignition transport partition for this launch'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('use_collision_monitor', default_value='false', description='Enable Collision Monitor in autonomous and fixed-path simulation'),
        DeclareLaunchArgument('use_collision_visualization', default_value='true', description='Enable Collision Monitor boundary visualization in simulation'),
        DeclareLaunchArgument('headless', default_value='true', description='Run only the Ignition Gazebo server by default; false enables Gazebo GUI'),
        DeclareLaunchArgument('operation_mode', default_value='autonomous', choices=['remote', 'autonomous', 'fixed_path'], description='Select the robot operation mode'),
        DeclareLaunchArgument('enable_localization_jump_detection', default_value='false', description='Whether localization pose jumps cancel control and stop the simulated robot'),
        DeclareLaunchArgument('fixed_path_progress_timeout', default_value='120.0', description='Maximum stationary time in fixed-path mode'),
        DeclareLaunchArgument('fixed_path_max_linear_velocity', default_value='1.5', description='Fixed-path controller maximum linear velocity (m/s)'),
        DeclareLaunchArgument('fixed_path_approach_velocity_scaling_dist', default_value='0.8', description='Fixed-path controller approach scaling distance (m)'),
        DeclareLaunchArgument('fixed_path_goal_linear_deceleration', default_value='0.25', description='Fixed-path goal deceleration (m/s^2)'),
        DeclareLaunchArgument('adaptive_goal_braking_enabled', default_value='false', description='Opt in to unvalidated adaptive fixed-path goal braking'),
        DeclareLaunchArgument('fixed_path_goal_final_approach_velocity', default_value='', description='Fixed-path final approach velocity (m/s)'),
        DeclareLaunchArgument('fixed_path_goal_braking_reaction_time', default_value='0.1', description='Fixed-path braking reaction time margin (s)'),
        DeclareLaunchArgument('fixed_path_goal_braking_distance_margin', default_value='', description='Fixed-path high braking margin; empty uses parameter YAML'),
        DeclareLaunchArgument('fixed_path_dynamic_goal_braking_margin_enabled', default_value='', description='Fixed-path dynamic margin override; empty uses parameter YAML'),
        DeclareLaunchArgument('fixed_path_goal_braking_min_distance_margin', default_value='', description='Fixed-path dynamic margin override; empty uses parameter YAML'),
        DeclareLaunchArgument('fixed_path_goal_braking_margin_transition_speed', default_value='', description='Fixed-path dynamic margin override; empty uses parameter YAML'),
        DeclareLaunchArgument('fixed_path_goal_terminal_lookahead_dist', default_value='', description='Fixed-path minimum terminal lookahead distance (m)'),
        DeclareLaunchArgument('fixed_path_goal_terminal_lookahead_reference_speed', default_value='0.75', description='Fixed-path terminal lookahead reference speed (m/s)'),
        DeclareLaunchArgument('fixed_path_goal_terminal_lookahead_speed_gain', default_value='0.1', description='Fixed-path terminal lookahead speed gain (s)'),
        DeclareLaunchArgument('fixed_path_goal_terminal_lookahead_min_dist', default_value='', description='Fixed-path terminal lookahead minimum distance (m)'),
        DeclareLaunchArgument('fixed_path_goal_terminal_lookahead_max_dist', default_value='', description='Fixed-path terminal lookahead maximum distance (m)'),
        DeclareLaunchArgument('fixed_path_xy_goal_tolerance', default_value='0.01', description='Fixed-path position goal tolerance (m)'),
        DeclareLaunchArgument('fixed_path_yaw_goal_tolerance', default_value='0.08726646259971647', description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument('fixed_path_goal_position_hysteresis', default_value='1.0', description='Deprecated compatibility option; terminal stop is latched'),
        DeclareLaunchArgument('fixed_path_rotate_to_heading_angular_vel', default_value='0.4', description='Fixed-path heading alignment velocity (rad/s)'),
        DeclareLaunchArgument('fixed_path_max_angular_accel', default_value='0.8', description='Fixed-path heading alignment acceleration (rad/s^2)'),
        DeclareLaunchArgument('fixed_path_rotate_to_heading_kp', default_value='1.5', description='Fixed-path heading alignment proportional gain'),
        DeclareLaunchArgument('fixed_path_goal_rotate_to_heading_angular_vel', default_value='0.8', description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument('fixed_path_goal_max_angular_accel', default_value='1.6', description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument('fixed_path_goal_rotate_to_heading_kp', default_value='3.0', description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument('fixed_path_goal_position_entry_tolerance', default_value='0.008', description='Deprecated compatibility option; xy goal tolerance is used'),
        DeclareLaunchArgument('fixed_path_goal_error_log_frequency', default_value='1.0', description='Fixed-path goal error log frequency (Hz)'),
        DeclareLaunchArgument('fixed_path_stack_start_delay', default_value='3.0', description='Delay the regulated fixed-path stack for sensors and external localization (s)'),
        DeclareLaunchArgument('rviz_start_delay', default_value='5.0', description='Delay RViz until localization and sensor transforms are ready (s)'),

        # Only sensors and velocity feedback cross the bridge. No Gazebo TF is
        # published; external localization must provide map -> base_link.
        OpaqueFunction(function=prepare_external_localization_parameters),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='simulation_sensor_bridge',
             output='screen', parameters=[{'use_sim_time': use_sim_time,
                 'override_timestamps_with_wall_time': PythonExpression(["'", use_sim_time, "'.lower() == 'false'"])}],
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
             ], remappings=[('/odom', '/odometry')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_clock_bridge',
             condition=IfCondition(use_sim_time), parameters=[{'use_sim_time': True}],
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),

        Node(package='myworld_bringup', executable='odom_to_motion_state.py', name='odom_to_motion_state', output='screen', parameters=[{'use_sim_time': use_sim_time,
                'input_topic': '/odometry',
                'output_topic': '/motion_state'}]),
        Node(condition=IfCondition(is_remote), package='myworld_bringup', executable='chassis_control_to_twist.py', name='chassis_control_to_twist', output='screen',
            parameters=[LaunchConfiguration('noamcl_params_file'), {'use_sim_time': use_sim_time}]),

        regulated_autonomous,
        regulated_fixed_path,

        TimerAction(period=rviz_start_delay, condition=IfCondition(PythonExpression(["'", operation_mode, "' != 'remote' and '", use_rviz, "' == 'true'"])), actions=[Node(package='rviz2',
                    executable='rviz2', name='rviz2', arguments=['-d', rviz_config_file], parameters=[{'use_sim_time': use_sim_time}], output='screen'),
            ]),
    ])
