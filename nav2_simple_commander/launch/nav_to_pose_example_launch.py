import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    commander_dir = get_package_share_directory('nav2_simple_commander')
    myworld_dir = os.path.join(commander_dir, 'models', 'myworld2')

    map_yaml_file = os.path.join(myworld_dir, 'myworld2.yaml')
    world = os.path.join(myworld_dir, 'world_only.sdf')
    params_file = os.path.join(commander_dir, 'params', 'myworld2.yaml')
    rviz_config_file = os.path.join(commander_dir, 'rviz', 'nav2_default_view.rviz')
    robot_description_file = os.path.join(commander_dir, 'urdf', 'diffbot.urdf')
    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='myworld2')
    gazebo_partition = f'nav2_simple_commander_nav_to_pose_{os.getpid()}'

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='myworld2',
        description='World name')

    resource_path = [
        os.path.join('/opt/ros/humble', 'share'),
        ':' + os.path.join(commander_dir, 'models'),
        ':' + myworld_dir,
        ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path)

    ign_partition = SetEnvironmentVariable(
        name='IGN_PARTITION',
        value=gazebo_partition)

    gz_partition = SetEnvironmentVariable(
        name='GZ_PARTITION',
        value=gazebo_partition)

    spdlog_log_dir = SetEnvironmentVariable(
        name='SPDLOG_WRAPPER_LOG_DIR',
        value='/tmp/nav2_logs')

    start_gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world],
        output='screen')

    ros_gz_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(commander_dir, 'ros_ign_bridge.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description}])

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(commander_dir, 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'localization_child_frame': 'odom',
            'localization_x': '0.0',
            'localization_y': '0.0',
            'localization_z': '0.0',
            'localization_yaw': '0.0',
            'localization_tf_time_offset': '0.0'}.items())

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(commander_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file}.items())

    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_nav_to_pose',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_partition)
    ld.add_action(gz_partition)
    ld.add_action(spdlog_log_dir)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(ros_gz_bridge_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(demo_cmd)
    return ld
