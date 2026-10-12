from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    system_time = PythonExpression(["'", use_sim_time, "'.lower() == 'false'"])
    tf_topic = PythonExpression(["'/_gazebo/tf' if '", use_sim_time, "'.lower() == 'false' else '/tf'"])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{
        'use_sim_time': use_sim_time,
        'override_timestamps_with_wall_time': ParameterValue(system_time, value_type=bool),
    }], arguments=[
                # Velocity command (ROS2 -> IGN)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (IGN -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # TF (IGN -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Joint states (IGN -> ROS2)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Lidar (IGN -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera (IGN -> ROS2)
                '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        remappings=[('/odom/tf', tf_topic)], output='screen')

    # TFMessage has no top-level header, so the native timestamp override skips it.
    tf_system_time = Node(
        package='myworld_bringup', executable='tf_system_time.py',
        name='tf_system_time', condition=UnlessCondition(use_sim_time),
        parameters=[{'use_sim_time': False}], output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_clock_bridge',
        condition=IfCondition(use_sim_time), parameters=[{'use_sim_time': True}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Use system timestamps by default; true restores Gazebo time'),
        bridge, tf_system_time, clock_bridge,
    ])
