import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    commander_dir = get_package_share_directory('nav2_simple_commander')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
                # Velocity command (ROS2 -> IGN)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (IGN -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # TF (IGN -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera (IGN -> ROS2)
                '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    cpu_lidar = Node(
        package='nav2_simple_commander',
        executable='cpu_lidar',
        name='cpu_lidar',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_image': os.path.join(
                commander_dir, 'models', 'myworld2', 'myworld2.pgm'),
            'map_resolution': 0.05,
            'map_origin_x': -7.0,
            'map_origin_y': -10.5,
            'samples': 720,
            'range_min': 0.12,
            'range_max': 10.0,
            'update_rate': 10.0,
            'sensor_offset_x': 0.18,
        }],
        output='screen'
    )

    return LaunchDescription([
        bridge,
        cpu_lidar,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
