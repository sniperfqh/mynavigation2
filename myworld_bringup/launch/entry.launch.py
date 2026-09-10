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
    headless = LaunchConfiguration('headless', default='false')
    operation_mode = LaunchConfiguration('operation_mode', default='autonomous')
    fixed_path_progress_timeout = LaunchConfiguration(
        'fixed_path_progress_timeout', default='120.0')
    fixed_path_max_linear_velocity = LaunchConfiguration(
        'fixed_path_max_linear_velocity', default='0.8')
    fixed_path_approach_velocity_scaling_dist = LaunchConfiguration(
        'fixed_path_approach_velocity_scaling_dist', default='0.8')
    fixed_path_stack_start_delay = LaunchConfiguration(
        'fixed_path_stack_start_delay', default='3.0')
    rviz_start_delay = LaunchConfiguration(
        'rviz_start_delay', default='5.0')

    is_autonomous = PythonExpression([
        "'", operation_mode, "' == 'autonomous'"])
    is_fixed_path = PythonExpression([
        "'", operation_mode, "' == 'fixed_path'"])
    is_autonomous_gui = PythonExpression([
        "'", operation_mode, "' == 'autonomous' and '", headless,
        "' == 'false'"])
    is_autonomous_headless = PythonExpression([
        "'", operation_mode, "' == 'autonomous' and '", headless,
        "' == 'true'"])
    is_fixed_path_gui = PythonExpression([
        "'", operation_mode, "' == 'fixed_path' and '", headless,
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
        cmd=['ign', 'gazebo', '-r', '-s', '-v', '3', world_only],
        output='screen')
    fixed_path_ignition_sim = ExecuteProcess(
        condition=IfCondition(is_fixed_path_gui),
        cmd=['ign', 'gazebo', '-g', '-v', '3', world_only],
        output='screen')
    fixed_path_ignition_server = ExecuteProcess(
        condition=IfCondition(is_fixed_path),
        cmd=['ign', 'gazebo', '-r', '-s', '-v', '3', world_only],
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
            'headless',
            default_value='false',
            description='Run only the Ignition Gazebo server'),
        DeclareLaunchArgument(
            'operation_mode',
            default_value='autonomous',
            choices=['autonomous', 'fixed_path'],
            description='Select the regulated navigation mode'),
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
            condition=IfCondition(use_rviz),
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
