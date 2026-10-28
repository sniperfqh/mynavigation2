"""Launch the warehouse, robot, and simulation sensor interfaces only."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory('myworld_bringup')
    scene = os.path.join(share, 'models', 'myworld2')
    world_file = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    publish_map = LaunchConfiguration('publish_map')
    map_file = LaunchConfiguration('map')
    ign_partition = LaunchConfiguration('ign_partition')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')

    with open(os.path.join(share, 'urdf', 'diffbot.urdf'), encoding='utf-8') as stream:
        robot = ET.fromstring(stream.read())
    # The simulation's odometry TF uses odom -> base_footprint; the original
    # URDF continues from base_footprint -> base_link.
    odom_robot_description = ET.tostring(robot, encoding='unicode')

    # Match entry.launch.py when an external localization owns base_link.
    base_joint = robot.find("joint[@name='base_joint']")
    origin = base_joint.find('origin')
    if origin.get('xyz') != '0 0 0' or origin.get('rpy') != '0 0 0':
        raise ValueError('External localization expects the existing zero-offset base_joint')
    base_joint.find('parent').set('link', 'base_link')
    base_joint.find('child').set('link', 'base_footprint')
    external_robot_description = ET.tostring(robot, encoding='unicode')

    system_time = PythonExpression(["'", use_sim_time, "' == 'false'"])
    tf_bridge_topic = PythonExpression([
        "'/tf' if '", use_sim_time, "' == 'true' else '/_gazebo/tf'"])
    system_time_tf = PythonExpression([
        "'", publish_odom_tf, "' == 'true' and '", use_sim_time, "' == 'false'"])
    map_to_odom = PythonExpression([
        "'", publish_map, "' == 'true' and '", publish_odom_tf, "' == 'true'"])
    sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)
    map_server = LifecycleNode(
        condition=IfCondition(publish_map), package='nav2_map_server', executable='map_server',
        name='simulation_map_server', namespace='', output='screen',
        parameters=[{'use_sim_time': sim_time_parameter, 'yaml_filename': map_file}])
    configure_map = RegisterEventHandler(OnProcessStart(
        target_action=map_server, on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server),
            transition_id=Transition.TRANSITION_CONFIGURE))]))
    activate_map = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=map_server, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file', default_value=os.path.join(scene, 'world_only.sdf'),
            description='Gazebo warehouse world, including the diffbot model'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run without the Gazebo GUI'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use Gazebo clock instead of system time'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true', choices=['true', 'false'],
            description='Publish the simulated odom to base_footprint transform'),
        DeclareLaunchArgument(
            'publish_map', default_value='true', choices=['true', 'false'],
            description='Publish the warehouse occupancy grid for RViz'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(scene, 'myworld2.yaml'),
            description='Occupancy-grid YAML corresponding to the Gazebo world'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz for the robot model and simulated lidar'),
        DeclareLaunchArgument(
            'rviz_config_file', default_value=os.path.join(share, 'rviz', 'simulation_only.rviz'),
            description='RViz display configuration'),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value=PythonExpression([
                "'map' if '", publish_map, "' == 'true' and '", publish_odom_tf,
                "' == 'true' else ('odom' if '", publish_odom_tf, "' == 'true' else 'base_link')"]),
            description='RViz fixed frame; defaults to map, odom or base_link according to enabled sources'),
        DeclareLaunchArgument(
            'ign_partition',
            default_value='myworld_simulation_{}_{}'.format(
                os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid()),
            description='Ignition transport partition for this simulation'),

        SetEnvironmentVariable('IGN_PARTITION', ign_partition),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            '/opt/ros/humble/share', os.path.join(share, 'models'), scene,
            os.path.join(scene, 'models')])),
        ExecuteProcess(
            condition=IfCondition(headless),
            cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_file],
            output='screen'),
        ExecuteProcess(
            condition=UnlessCondition(headless),
            cmd=['ign', 'gazebo', '-r', '-v', '3', world_file], output='screen'),

        Node(
            condition=IfCondition(publish_odom_tf), package='robot_state_publisher',
            executable='robot_state_publisher', name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': sim_time_parameter,
                         'robot_description': odom_robot_description}]),
        Node(
            condition=UnlessCondition(publish_odom_tf), package='robot_state_publisher',
            executable='robot_state_publisher', name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': sim_time_parameter,
                         'robot_description': external_robot_description}]),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='simulation_sensor_bridge',
            output='screen', parameters=[{
                'use_sim_time': sim_time_parameter,
                'override_timestamps_with_wall_time': ParameterValue(system_time, value_type=bool)}],
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ], remappings=[('/odom', '/odometry')]),
        Node(
            package='myworld_bringup', executable='odom_to_motion_state.py',
            name='odom_to_motion_state', output='screen',
            parameters=[{'use_sim_time': sim_time_parameter,
                         'input_topic': '/odometry', 'output_topic': '/motion_state'}]),
        Node(
            condition=IfCondition(publish_odom_tf), package='ros_gz_bridge',
            executable='parameter_bridge', name='gazebo_odom_tf_bridge', output='screen',
            parameters=[{'use_sim_time': sim_time_parameter}],
            arguments=['/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/odom/tf', tf_bridge_topic)]),
        Node(
            condition=IfCondition(system_time_tf), package='myworld_bringup',
            executable='tf_system_time.py', name='tf_system_time', output='screen',
            parameters=[{'use_sim_time': False}]),
        Node(
            condition=IfCondition(use_sim_time), package='ros_gz_bridge',
            executable='parameter_bridge', name='gazebo_clock_bridge', output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        # This loads a static occupancy grid; it does not start localization or navigation.
        configure_map,
        activate_map,
        map_server,
        Node(
            condition=IfCondition(map_to_odom), package='tf2_ros',
            executable='static_transform_publisher', name='simulation_map_to_odom', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                       '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(
            condition=IfCondition(use_rviz), package='rviz2', executable='rviz2', name='rviz2',
            output='screen', arguments=['-d', rviz_config_file, '-f', rviz_fixed_frame],
            parameters=[{'use_sim_time': sim_time_parameter}]),
    ])
