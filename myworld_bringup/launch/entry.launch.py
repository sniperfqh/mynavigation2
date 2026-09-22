import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_ign_partition = 'myworld_bringup_{}_{}'.format(
        os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid())
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ign_partition_name = LaunchConfiguration(
        'ign_partition', default=default_ign_partition)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_collision_monitor = LaunchConfiguration(
        'use_collision_monitor', default='false')
    use_collision_visualization = LaunchConfiguration(
        'use_collision_visualization', default='true')
    headless = LaunchConfiguration('headless', default='false')
    operation_mode = LaunchConfiguration('operation_mode', default='autonomous')
    fixed_path_progress_timeout = LaunchConfiguration(
        'fixed_path_progress_timeout', default='120.0')
    fixed_path_max_linear_velocity = LaunchConfiguration(
        'fixed_path_max_linear_velocity', default='1.5')
    fixed_path_approach_velocity_scaling_dist = LaunchConfiguration(
        'fixed_path_approach_velocity_scaling_dist', default='0.8')
    fixed_path_goal_linear_deceleration = LaunchConfiguration(
        'fixed_path_goal_linear_deceleration', default='0.25')
    fixed_path_goal_final_approach_velocity = LaunchConfiguration(
        'fixed_path_goal_final_approach_velocity', default='0.01')
    fixed_path_goal_braking_reaction_time = LaunchConfiguration(
        'fixed_path_goal_braking_reaction_time', default='0.1')
    fixed_path_goal_braking_distance_margin = LaunchConfiguration(
        'fixed_path_goal_braking_distance_margin', default='0.1')
    fixed_path_goal_terminal_lookahead_dist = LaunchConfiguration(
        'fixed_path_goal_terminal_lookahead_dist', default='0.2')
    fixed_path_goal_terminal_lookahead_reference_speed = LaunchConfiguration(
        'fixed_path_goal_terminal_lookahead_reference_speed', default='0.75')
    fixed_path_goal_terminal_lookahead_speed_gain = LaunchConfiguration(
        'fixed_path_goal_terminal_lookahead_speed_gain', default='0.1')
    fixed_path_goal_terminal_lookahead_min_dist = LaunchConfiguration(
        'fixed_path_goal_terminal_lookahead_min_dist', default='0.15')
    fixed_path_goal_terminal_lookahead_max_dist = LaunchConfiguration(
        'fixed_path_goal_terminal_lookahead_max_dist', default='0.25')
    fixed_path_xy_goal_tolerance = LaunchConfiguration(
        'fixed_path_xy_goal_tolerance', default='0.01')
    fixed_path_yaw_goal_tolerance = LaunchConfiguration(
        'fixed_path_yaw_goal_tolerance', default='0.08726646259971647')
    fixed_path_goal_position_hysteresis = LaunchConfiguration(
        'fixed_path_goal_position_hysteresis', default='1.0')
    fixed_path_rotate_to_heading_angular_vel = LaunchConfiguration(
        'fixed_path_rotate_to_heading_angular_vel', default='0.4')
    fixed_path_max_angular_accel = LaunchConfiguration(
        'fixed_path_max_angular_accel', default='0.8')
    fixed_path_rotate_to_heading_kp = LaunchConfiguration(
        'fixed_path_rotate_to_heading_kp', default='1.5')
    fixed_path_goal_rotate_to_heading_angular_vel = LaunchConfiguration(
        'fixed_path_goal_rotate_to_heading_angular_vel', default='0.8')
    fixed_path_goal_max_angular_accel = LaunchConfiguration(
        'fixed_path_goal_max_angular_accel', default='1.6')
    fixed_path_goal_rotate_to_heading_kp = LaunchConfiguration(
        'fixed_path_goal_rotate_to_heading_kp', default='3.0')
    fixed_path_goal_position_entry_tolerance = LaunchConfiguration(
        'fixed_path_goal_position_entry_tolerance', default='0.008')
    fixed_path_goal_error_log_frequency = LaunchConfiguration(
        'fixed_path_goal_error_log_frequency', default='1.0')
    fixed_path_stack_start_delay = LaunchConfiguration(
        'fixed_path_stack_start_delay', default='3.0')
    rviz_start_delay = LaunchConfiguration(
        'rviz_start_delay', default='5.0')

    is_autonomous = PythonExpression([
        "'", operation_mode, "' == 'autonomous'"])
    is_fixed_path = PythonExpression([
        "'", operation_mode, "' == 'fixed_path'"])
    is_remote = PythonExpression([
        "'", operation_mode, "' == 'remote'"])
    is_autonomous_gui = PythonExpression([
        "'", operation_mode, "' == 'autonomous' and '", headless,
        "' == 'false'"])
    is_autonomous_headless = PythonExpression([
        "'", operation_mode, "' == 'autonomous' and '", headless,
        "' == 'true'"])
    is_fixed_path_gui = PythonExpression([
        "'", operation_mode, "' == 'fixed_path' and '", headless,
        "' == 'false'"])
    is_remote_gui = PythonExpression([
        "'", operation_mode, "' == 'remote' and '", headless,
        "' == 'false'"])

    bringup_dir = get_package_share_directory('myworld_bringup')
    launch_file_dir = os.path.dirname(__file__)
    myworld_dir = os.path.join(bringup_dir, 'models', 'myworld2')
    map_file = os.path.join(myworld_dir, 'myworld2.yaml')
    params_file = os.path.join(bringup_dir, 'params', 'myworld2.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    robot_description_file = os.path.join(bringup_dir, 'urdf', 'diffbot.urdf')
    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    resource_path = [
        os.path.join('/opt/ros/humble', 'share'),
        ':' + os.path.join(bringup_dir, 'models'),
        ':' + myworld_dir,
        ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path)
    ign_partition = SetEnvironmentVariable(
        name='IGN_PARTITION',
        value=ign_partition_name)

    world_only = os.path.join(myworld_dir, 'world_only.sdf')
    ignition_sim = ExecuteProcess(
        condition=IfCondition(is_autonomous_gui),
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_only],
        output='screen')
    ignition_server = ExecuteProcess(
        condition=IfCondition(is_autonomous_headless),
        cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
             '-v', '3', world_only],
        output='screen')
    fixed_path_ignition_sim = ExecuteProcess(
        condition=IfCondition(is_fixed_path_gui),
        cmd=['ign', 'gazebo', '-g', '-v', '3', world_only],
        output='screen')
    fixed_path_ignition_server = ExecuteProcess(
        condition=IfCondition(is_fixed_path),
        cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
             '-v', '3', world_only],
        output='screen')
    remote_ignition_gui = ExecuteProcess(
        condition=IfCondition(is_remote_gui),
        cmd=['ign', 'gazebo', '-g', '-v', '3', world_only],
        output='screen')
    remote_ignition_server = ExecuteProcess(
        condition=IfCondition(is_remote),
        cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
             '-v', '3', world_only],
        output='screen')

    localization_arguments = {
        'map': map_file,
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'launch_map_server': 'false',
        'lifecycle_manager_name': 'lifecycle_manager_amcl',
    }
    algorithm_arguments = {
        'map': map_file,
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'rviz_config_file': rviz_config_file,
        'use_collision_monitor': use_collision_monitor,
        'use_collision_visualization': use_collision_visualization,
        'autostart': 'true',
        'use_composition': 'False',
        'container_name': 'nav2_regulated_container',
        'use_respawn': 'False',
        'log_level': 'info',
        'operation_mode': operation_mode,
        'fixed_path_progress_timeout': fixed_path_progress_timeout,
        'fixed_path_max_linear_velocity': fixed_path_max_linear_velocity,
        'fixed_path_approach_velocity_scaling_dist':
            fixed_path_approach_velocity_scaling_dist,
        'fixed_path_goal_linear_deceleration':
            fixed_path_goal_linear_deceleration,
        'fixed_path_goal_final_approach_velocity':
            fixed_path_goal_final_approach_velocity,
        'fixed_path_goal_braking_reaction_time':
            fixed_path_goal_braking_reaction_time,
        'fixed_path_goal_braking_distance_margin':
            fixed_path_goal_braking_distance_margin,
        'fixed_path_goal_terminal_lookahead_dist':
            fixed_path_goal_terminal_lookahead_dist,
        'fixed_path_goal_terminal_lookahead_reference_speed':
            fixed_path_goal_terminal_lookahead_reference_speed,
        'fixed_path_goal_terminal_lookahead_speed_gain':
            fixed_path_goal_terminal_lookahead_speed_gain,
        'fixed_path_goal_terminal_lookahead_min_dist':
            fixed_path_goal_terminal_lookahead_min_dist,
        'fixed_path_goal_terminal_lookahead_max_dist':
            fixed_path_goal_terminal_lookahead_max_dist,
        'fixed_path_xy_goal_tolerance': fixed_path_xy_goal_tolerance,
        'fixed_path_yaw_goal_tolerance': fixed_path_yaw_goal_tolerance,
        'fixed_path_goal_position_hysteresis':
            fixed_path_goal_position_hysteresis,
        'fixed_path_rotate_to_heading_angular_vel':
            fixed_path_rotate_to_heading_angular_vel,
        'fixed_path_max_angular_accel': fixed_path_max_angular_accel,
        'fixed_path_rotate_to_heading_kp': fixed_path_rotate_to_heading_kp,
        'fixed_path_goal_rotate_to_heading_angular_vel':
            fixed_path_goal_rotate_to_heading_angular_vel,
        'fixed_path_goal_max_angular_accel':
            fixed_path_goal_max_angular_accel,
        'fixed_path_goal_rotate_to_heading_kp':
            fixed_path_goal_rotate_to_heading_kp,
        'fixed_path_goal_position_entry_tolerance':
            fixed_path_goal_position_entry_tolerance,
        'fixed_path_goal_error_log_frequency':
            fixed_path_goal_error_log_frequency,
    }

    regulated_autonomous = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/navigation_launch.py']),
        condition=IfCondition(is_autonomous),
        launch_arguments=algorithm_arguments.items())

    regulated_fixed_path = TimerAction(
        period=fixed_path_stack_start_delay,
        condition=IfCondition(is_fixed_path),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/navigation_launch.py']),
            launch_arguments=algorithm_arguments.items())])

    return LaunchDescription([
        ign_resource_path,
        ign_partition,
        ignition_sim,
        ignition_server,
        fixed_path_ignition_sim,
        fixed_path_ignition_server,
        remote_ignition_gui,
        remote_ignition_server,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description}]),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'ign_partition',
            default_value=default_ign_partition,
            description='Unique Ignition transport partition for this launch'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),
        DeclareLaunchArgument(
            'use_collision_monitor',
            default_value='true',
            description='Enable Collision Monitor in autonomous and fixed-path simulation'),
        DeclareLaunchArgument(
            'use_collision_visualization',
            default_value='true',
            description='Enable Collision Monitor boundary visualization in simulation'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Ignition Gazebo server'),
        DeclareLaunchArgument(
            'operation_mode',
            default_value='autonomous',
            choices=['remote', 'autonomous', 'fixed_path'],
            description='Select the robot operation mode'),
        DeclareLaunchArgument(
            'fixed_path_progress_timeout',
            default_value='120.0',
            description='Maximum stationary time in fixed-path mode'),
        DeclareLaunchArgument(
            'fixed_path_max_linear_velocity',
            default_value='1.5',
            description='Fixed-path controller maximum linear velocity (m/s)'),
        DeclareLaunchArgument(
            'fixed_path_approach_velocity_scaling_dist',
            default_value='0.8',
            description='Fixed-path controller approach scaling distance (m)'),
        DeclareLaunchArgument(
            'fixed_path_goal_linear_deceleration',
            default_value='0.25',
            description='Fixed-path goal deceleration (m/s^2)'),
        DeclareLaunchArgument(
            'fixed_path_goal_final_approach_velocity',
            default_value='0.01',
            description='Fixed-path final approach velocity (m/s)'),
        DeclareLaunchArgument(
            'fixed_path_goal_braking_reaction_time',
            default_value='0.1',
            description='Fixed-path braking reaction time margin (s)'),
        DeclareLaunchArgument(
            'fixed_path_goal_braking_distance_margin',
            default_value='0.1',
            description='Fixed-path braking distance margin (m)'),
        DeclareLaunchArgument(
            'fixed_path_goal_terminal_lookahead_dist',
            default_value='0.2',
            description='Fixed-path minimum terminal lookahead distance (m)'),
        DeclareLaunchArgument(
            'fixed_path_goal_terminal_lookahead_reference_speed',
            default_value='0.75',
            description='Fixed-path terminal lookahead reference speed (m/s)'),
        DeclareLaunchArgument(
            'fixed_path_goal_terminal_lookahead_speed_gain',
            default_value='0.1',
            description='Fixed-path terminal lookahead speed gain (s)'),
        DeclareLaunchArgument(
            'fixed_path_goal_terminal_lookahead_min_dist',
            default_value='0.15',
            description='Fixed-path terminal lookahead minimum distance (m)'),
        DeclareLaunchArgument(
            'fixed_path_goal_terminal_lookahead_max_dist',
            default_value='0.25',
            description='Fixed-path terminal lookahead maximum distance (m)'),
        DeclareLaunchArgument(
            'fixed_path_xy_goal_tolerance',
            default_value='0.01',
            description='Fixed-path position goal tolerance (m)'),
        DeclareLaunchArgument(
            'fixed_path_yaw_goal_tolerance',
            default_value='0.08726646259971647',
            description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument(
            'fixed_path_goal_position_hysteresis',
            default_value='1.0',
            description='Deprecated compatibility option; terminal stop is latched'),
        DeclareLaunchArgument(
            'fixed_path_rotate_to_heading_angular_vel',
            default_value='0.4',
            description='Fixed-path heading alignment velocity (rad/s)'),
        DeclareLaunchArgument(
            'fixed_path_max_angular_accel',
            default_value='0.8',
            description='Fixed-path heading alignment acceleration (rad/s^2)'),
        DeclareLaunchArgument(
            'fixed_path_rotate_to_heading_kp',
            default_value='1.5',
            description='Fixed-path heading alignment proportional gain'),
        DeclareLaunchArgument(
            'fixed_path_goal_rotate_to_heading_angular_vel',
            default_value='0.8',
            description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument(
            'fixed_path_goal_max_angular_accel',
            default_value='1.6',
            description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument(
            'fixed_path_goal_rotate_to_heading_kp',
            default_value='3.0',
            description='Deprecated compatibility option; terminal heading is ignored'),
        DeclareLaunchArgument(
            'fixed_path_goal_position_entry_tolerance',
            default_value='0.008',
            description='Deprecated compatibility option; xy goal tolerance is used'),
        DeclareLaunchArgument(
            'fixed_path_goal_error_log_frequency',
            default_value='1.0',
            description='Fixed-path goal error log frequency (Hz)'),
        DeclareLaunchArgument(
            'fixed_path_stack_start_delay',
            default_value='3.0',
            description='Delay the regulated fixed-path stack after AMCL (s)'),
        DeclareLaunchArgument(
            'rviz_start_delay',
            default_value='5.0',
            description='Delay RViz until localization and sensor transforms are ready (s)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        Node(
            package='myworld_bringup',
            executable='odom_to_motion_state.py',
            name='odom_to_motion_state',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/odom',
                'output_topic': '/motion_state'}]),
        Node(
            condition=IfCondition(is_remote),
            package='myworld_bringup',
            executable='chassis_control_to_twist.py',
            name='chassis_control_to_twist',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/localization_launch.py']),
            condition=IfCondition(PythonExpression([
                "'", operation_mode, "' in ['autonomous', 'fixed_path']"])),
            launch_arguments=localization_arguments.items()),

        regulated_autonomous,
        regulated_fixed_path,

        TimerAction(
            period=rviz_start_delay,
            condition=IfCondition(PythonExpression([
                "'", operation_mode, "' != 'remote' and '", use_rviz,
                "' == 'true'"])),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'),
            ]),
    ])
