"""Map the unchanged warehouse using Gazebo's GPU lidar and SLAM Toolbox."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('myworld_bringup')
    scene = os.path.join(share, 'models', 'myworld2')
    partition = LaunchConfiguration('ign_partition')
    mapping = LaunchConfiguration('mapping')
    with open(os.path.join(share, 'urdf', 'diffbot.urdf'), encoding='utf-8') as stream:
        robot = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument('ign_partition', default_value=f'myworld_mapping_{os.getpid()}'),
        DeclareLaunchArgument('mapping', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=os.path.join(scene, 'myworld2.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'params', 'myworld2.yaml')),
        SetEnvironmentVariable('IGN_PARTITION', partition),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            '/opt/ros/humble/share', os.path.join(share, 'models'), scene,
            os.path.join(scene, 'models')])),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
                            os.path.join(scene, 'world_only.sdf')], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': robot}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'ros_ign_bridge.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             condition=IfCondition(mapping),
             parameters=[os.path.join(share, 'params', 'mapping.yaml')], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             condition=UnlessCondition(mapping),
             parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', condition=UnlessCondition(mapping),
             parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='mapping_validation_lifecycle', condition=UnlessCondition(mapping),
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])
