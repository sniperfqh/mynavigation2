import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('myworld_bringup')
    regulated_launch_file = os.path.join(
        get_package_share_directory('nav2_regulated_modules'),
        'launch',
        'regulated_modules.launch.py')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    operation_mode = LaunchConfiguration('operation_mode')
    fixed_path_progress_timeout = LaunchConfiguration(
        'fixed_path_progress_timeout')
    fixed_path_max_linear_velocity = LaunchConfiguration(
        'fixed_path_max_linear_velocity')
    fixed_path_approach_velocity_scaling_dist = LaunchConfiguration(
        'fixed_path_approach_velocity_scaling_dist')
    fixed_path_xy_goal_tolerance = LaunchConfiguration(
        'fixed_path_xy_goal_tolerance')
    fixed_path_yaw_goal_tolerance = LaunchConfiguration(
        'fixed_path_yaw_goal_tolerance')
    fixed_path_goal_position_hysteresis = LaunchConfiguration(
        'fixed_path_goal_position_hysteresis')
    fixed_path_rotate_to_heading_angular_vel = LaunchConfiguration(
        'fixed_path_rotate_to_heading_angular_vel')
    fixed_path_max_angular_accel = LaunchConfiguration(
        'fixed_path_max_angular_accel')
    fixed_path_rotate_to_heading_kp = LaunchConfiguration(
        'fixed_path_rotate_to_heading_kp')
    fixed_path_goal_rotate_to_heading_angular_vel = LaunchConfiguration(
        'fixed_path_goal_rotate_to_heading_angular_vel')
    fixed_path_goal_max_angular_accel = LaunchConfiguration(
        'fixed_path_goal_max_angular_accel')
    fixed_path_goal_rotate_to_heading_kp = LaunchConfiguration(
        'fixed_path_goal_rotate_to_heading_kp')
    fixed_path_goal_position_entry_tolerance = LaunchConfiguration(
        'fixed_path_goal_position_entry_tolerance')
    fixed_path_goal_error_log_frequency = LaunchConfiguration(
        'fixed_path_goal_error_log_frequency')
    keyboard_input_device = LaunchConfiguration('keyboard_input_device')

    fixed_path = PythonExpression([
        "'", operation_mode, "' == 'fixed_path'"])
    use_smoother = PythonExpression([
        "'false' if ", fixed_path, " else 'true'"])
    fixed_path_controller_velocity = PythonExpression([
        "'", fixed_path_max_linear_velocity, "'"])
    fixed_path_approach_distance = PythonExpression([
        "'", fixed_path_approach_velocity_scaling_dist, "'"])
    effective_xy_goal_tolerance = PythonExpression([
        "'", fixed_path_xy_goal_tolerance, "' if '", operation_mode,
        "' == 'fixed_path' else '0.03'"])
    effective_yaw_goal_tolerance = PythonExpression([
        "'", fixed_path_yaw_goal_tolerance, "' if '", operation_mode,
        "' == 'fixed_path' else '0.15726646259971647'"])
    fixed_path_position_hysteresis = PythonExpression([
        "'", fixed_path_goal_position_hysteresis, "'"])
    fixed_path_heading_velocity = PythonExpression([
        "'", fixed_path_rotate_to_heading_angular_vel, "'"])
    fixed_path_heading_acceleration = PythonExpression([
        "'", fixed_path_max_angular_accel, "'"])
    fixed_path_heading_kp = PythonExpression([
        "'", fixed_path_rotate_to_heading_kp, "'"])
    fixed_path_goal_heading_velocity = PythonExpression([
        "'", fixed_path_goal_rotate_to_heading_angular_vel, "'"])
    fixed_path_goal_heading_acceleration = PythonExpression([
        "'", fixed_path_goal_max_angular_accel, "'"])
    fixed_path_goal_heading_kp = PythonExpression([
        "'", fixed_path_goal_rotate_to_heading_kp, "'"])
    fixed_path_goal_position_entry = PythonExpression([
        "'", fixed_path_goal_position_entry_tolerance, "'"])
    fixed_path_goal_error_log_rate = PythonExpression([
        "'", fixed_path_goal_error_log_frequency, "'"])
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'regulated_navigator.ros__parameters.operation_mode':
                operation_mode,
            'regulated_navigator.ros__parameters.use_smoother':
                use_smoother,
            'regulated_navigator.ros__parameters.controller_cmd_vel_topic':
                'cmd_vel_nav',
            'regulated_navigator.ros__parameters.smoothed_cmd_vel_topic':
                'cmd_vel',
            'regulated_navigator.ros__parameters.velocity_odom_topic':
                '/odom',
            'controller_server.ros__parameters.odom_topic': '/odom',
            'velocity_smoother.ros__parameters.odom_topic': '/odom',
            'controller_server.ros__parameters.FixedPathController.desired_linear_vel':
                fixed_path_controller_velocity,
            'controller_server.ros__parameters.FixedPathController.approach_velocity_scaling_dist':
                fixed_path_approach_distance,
            'controller_server.ros__parameters.stopped_goal_checker.xy_goal_tolerance':
                effective_xy_goal_tolerance,
            'controller_server.ros__parameters.stopped_goal_checker.yaw_goal_tolerance':
                effective_yaw_goal_tolerance,
            'controller_server.ros__parameters.fixed_path_goal_checker.xy_goal_tolerance':
                fixed_path_xy_goal_tolerance,
            'controller_server.ros__parameters.fixed_path_goal_checker.yaw_goal_tolerance':
                fixed_path_yaw_goal_tolerance,
            'controller_server.ros__parameters.FixedPathController.goal_position_hysteresis':
                fixed_path_position_hysteresis,
            'controller_server.ros__parameters.FixedPathController.rotate_to_heading_angular_vel':
                fixed_path_heading_velocity,
            'controller_server.ros__parameters.FixedPathController.max_angular_accel':
                fixed_path_heading_acceleration,
            'controller_server.ros__parameters.FixedPathController.rotate_to_heading_kp':
                fixed_path_heading_kp,
            'controller_server.ros__parameters.FixedPathController.goal_rotate_to_heading_angular_vel':
                fixed_path_goal_heading_velocity,
            'controller_server.ros__parameters.FixedPathController.goal_max_angular_accel':
                fixed_path_goal_heading_acceleration,
            'controller_server.ros__parameters.FixedPathController.goal_rotate_to_heading_kp':
                fixed_path_goal_heading_kp,
            'controller_server.ros__parameters.FixedPathController.goal_position_entry_tolerance':
                fixed_path_goal_position_entry,
            'controller_server.ros__parameters.FixedPathController.goal_error_log_frequency':
                fixed_path_goal_error_log_rate,
        },
        convert_types=True)

    regulated_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(regulated_launch_file),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'rviz_config_file': rviz_config_file,
            'use_rviz': 'false',
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'operation_mode': operation_mode,
            'fixed_path_progress_timeout': fixed_path_progress_timeout,
            'keyboard_input_device': keyboard_input_device,
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                bringup_dir, 'models', 'myworld2', 'myworld2.yaml'),
            description='Full path to the simulation map'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'params', 'myworld2.yaml'),
            description='ROS 2 parameters file'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='RViz configuration file'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate lifecycle nodes'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed regulated modules'),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_regulated_container',
            description='Composable container name'),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn algorithm nodes'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level'),
        DeclareLaunchArgument(
            'operation_mode',
            default_value='autonomous',
            choices=['autonomous', 'fixed_path'],
            description='Regulated modules operation mode'),
        DeclareLaunchArgument(
            'fixed_path_progress_timeout',
            default_value='120.0',
            description='Maximum stationary time in fixed-path mode'),
        DeclareLaunchArgument(
            'fixed_path_max_linear_velocity',
            default_value='0.52',
            description='Fixed-path controller maximum linear velocity (m/s)'),
        DeclareLaunchArgument(
            'fixed_path_approach_velocity_scaling_dist',
            default_value='0.8',
            description='Fixed-path controller approach scaling distance (m)'),
        DeclareLaunchArgument(
            'fixed_path_xy_goal_tolerance',
            default_value='0.01',
            description='Fixed-path position goal tolerance (m)'),
        DeclareLaunchArgument(
            'fixed_path_yaw_goal_tolerance',
            default_value='0.08726646259971647',
            description='Fixed-path heading goal tolerance (rad)'),
        DeclareLaunchArgument(
            'fixed_path_goal_position_hysteresis',
            default_value='1.0',
            description='Fixed-path position reacquisition multiplier'),
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
            description='Fixed-path goal heading maximum velocity (rad/s)'),
        DeclareLaunchArgument(
            'fixed_path_goal_max_angular_accel',
            default_value='1.6',
            description='Fixed-path goal heading acceleration (rad/s^2)'),
        DeclareLaunchArgument(
            'fixed_path_goal_rotate_to_heading_kp',
            default_value='3.0',
            description='Fixed-path goal heading proportional gain'),
        DeclareLaunchArgument(
            'fixed_path_goal_position_entry_tolerance',
            default_value='0.008',
            description='Fixed-path goal alignment entry tolerance (m)'),
        DeclareLaunchArgument(
            'fixed_path_goal_error_log_frequency',
            default_value='1.0',
            description='Fixed-path goal error log frequency (Hz)'),
        DeclareLaunchArgument(
            'keyboard_input_device',
            default_value='/dev/tty',
            description='Remote control input device'),
        regulated_launch,
    ])
