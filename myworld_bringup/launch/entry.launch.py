"""Navigation entry: external localization exclusively publishes map -> base_link.

Subscribe to /scan with the base_link -> base_scan extrinsics. Raw /odometry
is converted to /motion_state for controllers, the navigator and the smoother.
All modes default to headless Gazebo with its feedback bridges.
use_gazebo:=false switches to external sensor and velocity inputs.
No Gazebo bottom-chassis TF or substitute localization is published here.

use_collision_monitor:=true starts the collision detection module: dynamic
zones (car_op_zones.py --drive-monitor), the default status source, and (when
use_obstacle_status is also true) the status enumerator. The module does not
change velocity. use_obstacle_status defaults true and makes the translator
publish /collision_state (UInt8) and /obstacle_status_int (Int32); it runs with
--no-passthrough and never publishes /cmd_vel. The Nav2 collision_monitor
velocity gate is separate and stays off unless use_collision_velocity_gate is
true.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_ign_partition = 'myworld_bringup_{}_{}'.format(os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid())
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ign_partition_name = LaunchConfiguration('ign_partition', default=default_ign_partition)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    operation_mode = LaunchConfiguration('operation_mode', default='autonomous')
    # Keep the simulator server and sensor bridge active without opening a Gazebo GUI.
    default_use_gazebo = 'true'
    default_collision_visualization = PythonExpression(["'true' if '", operation_mode, "' == 'fixed_path' else 'false'"])
    use_gazebo = LaunchConfiguration('use_gazebo', default=default_use_gazebo)
    default_collision_monitor = 'false'
    # use_collision_monitor 是碰撞检测模块（状态枚举链）总开关；速度门单独用
    # use_collision_velocity_gate，二者互不影响。
    use_collision_monitor = LaunchConfiguration('use_collision_monitor', default=default_collision_monitor)
    use_collision_velocity_gate = LaunchConfiguration('use_collision_velocity_gate', default='false')
    use_obstacle_status = LaunchConfiguration('use_obstacle_status', default='true')
    use_car_op_status_sim = LaunchConfiguration('use_car_op_status_sim', default='true')
    car_op_status_value = LaunchConfiguration('car_op_status_value', default='0')
    use_collision_visualization = LaunchConfiguration('use_collision_visualization', default=default_collision_visualization)
    headless = LaunchConfiguration('headless', default='true')
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
    navigation_stack_start_delay = LaunchConfiguration('navigation_stack_start_delay', default=fixed_path_stack_start_delay)
    rviz_start_delay = LaunchConfiguration('rviz_start_delay', default='5.0')

    is_navigation = PythonExpression(["'", operation_mode, "' != 'remote'"])
    gazebo_enabled = ["'", use_gazebo, "'.lower() == 'true' and "]
    is_autonomous_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' in ('autonomous', 'autonomous_fixed') and '", headless, "' == 'false'"])
    is_autonomous_headless = PythonExpression(gazebo_enabled + ["'", operation_mode, "' in ('autonomous', 'autonomous_fixed') and '", headless, "' == 'true'"])
    is_fixed_path_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'fixed_path' and '", headless, "' == 'false'"])
    is_remote_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'remote' and '", headless, "' == 'false'"])

    bringup_dir = get_package_share_directory('myworld_bringup')
    myworld_dir = os.path.join(bringup_dir, 'models', 'myworld2')
    default_map = PythonExpression([
        "'", os.path.join('/userdata/map/pbstream', 'out.yaml'),
        "' if '", operation_mode, "' in ('fixed_path', 'autonomous', 'autonomous_fixed') else '",
        os.path.join(myworld_dir, 'myworld2.yaml'), "'"])
    map_file = LaunchConfiguration('map', default=default_map)
    rviz_config_file = PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'nav2_autonomous_fixed_view.rviz'),
        "' if '", operation_mode, "' == 'autonomous_fixed' else ",
        "'", os.path.join(bringup_dir, 'rviz', 'nav2_autonomous_view.rviz'),
        "' if '", operation_mode, "' == 'autonomous' else '",
        os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'), "'"])
    robot_description_file = os.path.join(bringup_dir, 'urdf', 'diffbot.urdf')
    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot = ET.fromstring(urdf_file.read())
    # External localization owns map -> base_link. Keep base_link a URDF root;
    # publishing base_footprint -> base_link would give base_link two parents.
    base_joint = robot.find("joint[@name='base_joint']")
    origin = base_joint.find('origin')
    if origin.get('xyz') != '0 0 0' or origin.get('rpy') != '0 0 0':
        raise ValueError('External localization expects the existing zero-offset base_joint')
    base_joint.find('parent').set('link', 'base_link')
    base_joint.find('child').set('link', 'base_footprint')
    # Fixed sensor joints publish on /tf_static; wheel joints follow /joint_states on /tf.
    robot_description = ET.tostring(robot, encoding='unicode')

    resource_path = [os.path.join('/opt/ros/humble', 'share'), ':' + os.path.join(bringup_dir, 'models'), ':' + myworld_dir, ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path)
    ign_partition = SetEnvironmentVariable(name='IGN_PARTITION', value=ign_partition_name)

    world_only = LaunchConfiguration('world_file', default=os.path.join(myworld_dir, 'world_only.sdf'))
    ignition_sim = ExecuteProcess(condition=IfCondition(is_autonomous_gui), cmd=['ign', 'gazebo', '-r', '-v', '3', world_only], output='screen')
    ignition_server = ExecuteProcess(condition=IfCondition(is_autonomous_headless), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    fixed_path_ignition_sim = ExecuteProcess(condition=IfCondition(is_fixed_path_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    fixed_path_ignition_server = ExecuteProcess(condition=IfCondition(PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'fixed_path'"])), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    remote_ignition_gui = ExecuteProcess(condition=IfCondition(is_remote_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    remote_ignition_server = ExecuteProcess(condition=IfCondition(PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'remote'"])), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')

    algorithm_arguments = {'map': map_file,
        **{key: LaunchConfiguration(key) for key in ['enable_velocity_diagnostics', 'enable_legacy_velocity_logs', 'log_dir', 'velocity_file_log_frequency', 'velocity_console_log_frequency']},
        'use_sim_time': use_sim_time,
        'params_file': LaunchConfiguration('params_file'),
        'rviz_config_file': rviz_config_file,
        'use_rviz': 'false',
        'namespace': '',
        'use_namespace': 'false',
        # regulated 的 use_collision_monitor 现在也是状态模块总开关；状态模块已在
        # 下面的 Node 里独立启动，这里必须关掉避免重复节点。速度门单独转发。
        'use_collision_monitor': 'false',
        'use_collision_velocity_gate': use_collision_velocity_gate,
        'use_collision_visualization': use_collision_visualization,
        'use_obstacle_status': 'false',
        'autostart': 'true',
        'use_composition': 'False',
        'container_name': 'nav2_regulated_container',
        'use_respawn': 'False',
        'log_level': 'info',
        'operation_mode': operation_mode,
        # Scene planner selection stays in params_file, rather than the child Launch default.
        'autonomous_planner_id': '',
        'autonomous_mppi_profile': LaunchConfiguration('autonomous_mppi_profile'),
        'use_path_smoother': PythonExpression(["'", operation_mode, "' not in ('fixed_path', 'autonomous_fixed')"]),
        'enable_localization_jump_detection': enable_localization_jump_detection,
        'enable_collision_state': LaunchConfiguration('enable_collision_state'),
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

    # 两种导航模式共用地图、外部定位、反馈接口和启动顺序，由 operation_mode 选择任务流程。
    # 子 Launch 的 use_rviz=false 只作用于组内，保留外层 RViz 的默认 true。
    regulated_navigation = TimerAction(
        period=navigation_stack_start_delay, condition=IfCondition(is_navigation),
        actions=[GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_regulated_modules'), '/launch/regulated_modules.launch.py']),
            launch_arguments=algorithm_arguments.items())])])

    return LaunchDescription([
        DeclareLaunchArgument('enable_velocity_diagnostics', default_value='true'),
        DeclareLaunchArgument('enable_legacy_velocity_logs', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value=os.environ.get('SPDLOG_WRAPPER_LOG_DIR', '/tmp/nav2_logs')),
        DeclareLaunchArgument('velocity_file_log_frequency', default_value=PythonExpression(["'50.0' if '", operation_mode, "' == 'autonomous' else '100.0'"])),
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
        DeclareLaunchArgument('map', default_value=default_map, description='Navigation modes share /userdata/map/pbstream/out.yaml; remote uses myworld2; explicit map overrides the default'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'myworld2.yaml'), description='Explicit simulation parameter file for isolated experiments'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(myworld_dir, 'world_only.sdf'), description='Explicit simulation SDF for isolated experiments'),
        DeclareLaunchArgument('ign_partition', default_value=default_ign_partition, description='Unique Ignition transport partition for this launch'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('use_gazebo', default_value=default_use_gazebo, description='Start Gazebo, sensor bridges and simulated velocity feedback; defaults true in all modes, with the GUI disabled by headless=true'),
        DeclareLaunchArgument('enable_collision_state', default_value='true', description='Enable fixed-path external collision-state gating'),
        DeclareLaunchArgument('use_collision_monitor', default_value=default_collision_monitor, description='Start the collision detection module (dynamic zones + default status source); it publishes status only and never changes velocity'),
        DeclareLaunchArgument('use_collision_velocity_gate', default_value='false', description='Start the Nav2 collision_monitor velocity gate separately; keep false when the status module is the intended collision behaviour'),
        DeclareLaunchArgument('use_collision_visualization', default_value=default_collision_visualization, description='Enable collision boundary visualization; defaults true for fixed_path, false for other modes'),
        DeclareLaunchArgument('use_obstacle_status', default_value='true', description='With use_collision_monitor: start obstacle_status_translator and publish /collision_state (UInt8) and /obstacle_status_int (Int32). The translator never publishes /cmd_vel'),
        DeclareLaunchArgument('use_car_op_status_sim', default_value='true', description='With use_collision_monitor: start the default car_op_status_sim source. It yields while another /car_op_status publisher exists and resumes afterwards'),
        DeclareLaunchArgument('car_op_status_value', default_value='0', description='car_op_status_sim hold value 0-8 (0 = idle forward)'),
        DeclareLaunchArgument('headless', default_value='true', description='Run only the Ignition Gazebo server by default; false enables Gazebo GUI'),
        DeclareLaunchArgument('operation_mode', default_value='autonomous', choices=['remote', 'autonomous', 'fixed_path', 'autonomous_fixed'], description='Select the robot operation mode'),
        DeclareLaunchArgument('autonomous_mppi_profile', default_value='true', description='Apply rotation alignment, terminal approach and reduced MPPI sampling only in autonomous mode'),
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
        DeclareLaunchArgument('navigation_stack_start_delay', default_value=fixed_path_stack_start_delay, description='Shared autonomous/fixed-path stack delay (s); defaults to fixed_path_stack_start_delay for compatibility'),
        DeclareLaunchArgument('rviz_start_delay', default_value='5.0', description='Delay RViz until localization and sensor transforms are ready (s)'),

        # Only sensors and velocity feedback cross the bridge. No Gazebo TF is
        # published; external localization must provide map -> base_link.
        # AMCL 加载保持注释／禁用：不引入 localization_launch.py，由外部定位提供 TF。
        Node(condition=IfCondition(use_gazebo), package='ros_gz_bridge', executable='parameter_bridge', name='simulation_sensor_bridge',
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
             condition=IfCondition(PythonExpression(gazebo_enabled + ["'", use_sim_time, "'.lower() == 'true'"])), parameters=[{'use_sim_time': True}],
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),

        Node(condition=IfCondition(use_gazebo), package='myworld_bringup', executable='odom_to_motion_state.py', name='odom_to_motion_state', output='screen', parameters=[{'use_sim_time': use_sim_time,
                'input_topic': '/odometry',
                'output_topic': '/motion_state'}]),
        # 碰撞检测模块：use_collision_monitor 是总开关；模块只发状态，不改速度。
        # use_obstacle_status 默认 true，决定是否启动 translator 输出枚举：
        # /collision_state (UInt8) 给 regulated，/obstacle_status_int (Int32) 给运控
        # （见 nav2_collision_monitor/doc/OBSTACLE_STATUS.md）。
        Node(condition=IfCondition(use_collision_monitor),
             package='nav2_collision_monitor', executable='car_op_zones.py',
             name='car_op_zones', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--drive-monitor', '--frame-id', 'base_link', '--period', '0.1',
                        '--profile', 'sim']),
        # 默认状态源保住启动时的状态 0 边界框；它带 --yield-to-other-publishers，
        # 窗口 4 再起 car_op_status_sim 时会自动让位，退出后恢复，
        # 避免两个发布者同时发导致动态框跳闪。
        Node(condition=IfCondition(PythonExpression([
                 "'", use_collision_monitor, "'.lower() == 'true' and '",
                 use_car_op_status_sim, "'.lower() == 'true'"])),
             package='nav2_collision_monitor', executable='car_op_status_sim.py',
             name='car_op_status_sim', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--mode', 'hold', '--value', car_op_status_value,
                        '--topic', '/car_op_status',
                        '--yield-to-other-publishers']),
        Node(condition=IfCondition(PythonExpression([
                 "'", use_collision_monitor, "'.lower() == 'true' and '",
                 use_obstacle_status, "'.lower() == 'true'"])),
             package='nav2_collision_monitor', executable='obstacle_status_translator.py',
             name='obstacle_status_translator', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--scan-topic', '/scan',
                        '--stop-topic', '/car_op_zones/stop',
                        '--slow-l2-topic', '/car_op_zones/slow_l2',
                        '--slow-l1-topic', '/car_op_zones/slow_l1',
                        '--zone-frame', 'base_link',
                        '--profile', 'sim',
                        '--no-passthrough']),
        Node(condition=IfCondition(PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'remote'"])), package='myworld_bringup', executable='chassis_control_to_twist.py', name='chassis_control_to_twist', output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}]),

        regulated_navigation,

        TimerAction(period=rviz_start_delay, condition=IfCondition(PythonExpression(["'", operation_mode, "' != 'remote' and '", use_rviz, "' == 'true'"])), actions=[Node(package='rviz2',
                    executable='rviz2', name='rviz2', arguments=['-d', rviz_config_file], parameters=[{'use_sim_time': use_sim_time}], output='screen'),
            ]),
    ])
