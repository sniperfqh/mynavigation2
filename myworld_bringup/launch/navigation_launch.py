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
    keyboard_input_device = LaunchConfiguration('keyboard_input_device')

    fixed_path = PythonExpression([
        "'", operation_mode, "' == 'fixed_path'"])
    use_smoother = PythonExpression([
        "'false' if ", fixed_path, " else 'true'"])
    fixed_path_controller_velocity = PythonExpression([
        "'", fixed_path_max_linear_velocity, "'"])
    fixed_path_approach_distance = PythonExpression([
        "'", fixed_path_approach_velocity_scaling_dist, "'"])
    fixed_path_yaw_tolerance = PythonExpression([
        "'0.08726646259971647' if ", fixed_path, " else '0.1'"])
    fixed_path_stop_velocity = PythonExpression([
        "'0.005' if ", fixed_path, " else '0.01'"])
    fixed_path_stop_angular_velocity = PythonExpression([
        "'0.02' if ", fixed_path, " else '0.05'"])

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
                '0.01',
            'controller_server.ros__parameters.stopped_goal_checker.yaw_goal_tolerance':
                fixed_path_yaw_tolerance,
            'controller_server.ros__parameters.stopped_goal_checker.trans_stopped_velocity':
                fixed_path_stop_velocity,
            'controller_server.ros__parameters.stopped_goal_checker.rot_stopped_velocity':
                fixed_path_stop_angular_velocity,
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
            default_value='0.8',
            description='Fixed-path RPP maximum linear velocity (m/s)'),
        DeclareLaunchArgument(
            'fixed_path_approach_velocity_scaling_dist',
            default_value='0.8',
            description='Fixed-path RPP approach scaling distance (m)'),
        DeclareLaunchArgument(
            'keyboard_input_device',
            default_value='/dev/tty',
            description='Remote control input device'),
        regulated_launch,
    ])
