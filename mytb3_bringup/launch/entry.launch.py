import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')


def generate_launch_description():
    bringup_dir = get_package_share_directory('mytb3_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    gazebo_partition = f'mytb3_bringup_{os.getpid()}'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(bringup_dir, 'params', f'{TURTLEBOT3_MODEL}.yaml'))

    robot_name = LaunchConfiguration('robot_name', default=TURTLEBOT3_MODEL)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw = LaunchConfiguration('yaw', default='0.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'))

    world_only = os.path.join(bringup_dir, 'models', 'worlds', 'world_only.sdf')

    ignition_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_only],
        output='screen')

    resource_path = [
        os.path.join('/opt/ros/humble', 'share'),
        ':' + os.path.join(bringup_dir, 'models'),
    ]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
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

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-file', PathJoinSubstitution([
                bringup_dir,
                'models',
                'turtlebot3',
                'model.sdf']),
            '-allow_renaming', 'false',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ])

    return LaunchDescription([
        ign_resource_path,
        gz_resource_path,
        ign_partition,
        gz_partition,
        spdlog_log_dir,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'world_name',
            default_value='turtlebot3_world',
            description='World name'),

        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS 2 parameters file to use'),

        DeclareLaunchArgument(
            'robot_name',
            default_value=TURTLEBOT3_MODEL,
            description='Name of the robot entity'),

        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.0',
            description='Initial robot x pose'),

        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='Initial robot y pose'),

        DeclareLaunchArgument(
            'z_pose',
            default_value='0.01',
            description='Initial robot z pose'),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial robot yaw'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RViz config file to use'),

        ignition_sim,

        TimerAction(period=3.0, actions=[ignition_spawn_entity]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        Node(
            package='mytb3_bringup',
            executable='robot_description_publisher.py',
            name='robot_description_publisher',
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/localization_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'localization_child_frame': 'odom',
                'localization_x': '0.0',
                'localization_y': '0.0',
                'localization_yaw': '0.0',
                'localization_tf_time_offset': '0.0',
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items()),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
