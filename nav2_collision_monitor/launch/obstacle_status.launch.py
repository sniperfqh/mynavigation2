"""Collision detection module (status enum output) for the real vehicle.

use_collision_monitor is the module switch (default false): it starts/stops
car_op_status_sim.py, car_op_zones.py and obstacle_status_translator.py.
use_obstacle_status (default true) controls the enum output; with it enabled the translator publishes
/collision_state (std_msgs/UInt8) for the regulated stack and
/obstacle_status_int (std_msgs/Int32) for the vehicle motion controller.

The module is status only: the translator runs with --no-passthrough and never
publishes /cmd_vel. The motion controller consumes the enum and performs
slowdown/stop itself.

The Nav2 collision_monitor velocity gate is NOT started by this launch. On the
real vehicle regulated_modules.launch.py starts it, controlled by that launch's
own use_collision_monitor argument (the velocity gate, not this module). Keep
that one disabled when this status-only module is the intended behaviour,
otherwise two /cmd_vel publishers appear.

Simulation keeps starting the same chain from myworld_bringup/entry.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_collision_monitor = LaunchConfiguration('use_collision_monitor', default='false')
    use_obstacle_status = LaunchConfiguration('use_obstacle_status', default='true')
    use_car_op_status_sim = LaunchConfiguration('use_car_op_status_sim', default='true')
    car_op_status_value = LaunchConfiguration('car_op_status_value', default='0')
    scan_topic = LaunchConfiguration('scan_topic', default='/c200_lidar_node/scan')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    zone_frame = LaunchConfiguration('zone_frame', default='base_link')
    # 本 launch 面向实车，默认实车尺寸档；仿真请显式传 zones_profile:=sim。
    zones_profile = LaunchConfiguration('zones_profile', default='real')

    start_module = IfCondition(use_collision_monitor)
    start_status_source = IfCondition(PythonExpression([
        "'", use_collision_monitor, "'.lower() == 'true' and '",
        use_car_op_status_sim, "'.lower() == 'true'"]))
    start_translator = IfCondition(PythonExpression([
        "'", use_collision_monitor, "'.lower() == 'true' and '",
        use_obstacle_status, "'.lower() == 'true'"]))

    status_nodes = [
        # 默认作业状态源：hold 在 car_op_status_value，外部源出现时自动让位。
        Node(
            condition=start_status_source,
            package='nav2_collision_monitor',
            executable='car_op_status_sim.py',
            name='car_op_status_sim',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--mode', 'hold', '--value', car_op_status_value,
                       '--topic', '/car_op_status', '--yield-to-other-publishers'],
        ),
        # 按作业状态输出三级动态碰撞框。
        Node(
            condition=start_module,
            package='nav2_collision_monitor',
            executable='car_op_zones.py',
            name='car_op_zones',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--drive-monitor', '--frame-id', zone_frame, '--period', '0.1',
                       '--profile', zones_profile],
        ),
        # 雷达 + 动态框 -> /collision_state 与 /obstacle_status_int；不碰速度。
        Node(
            condition=start_translator,
            package='nav2_collision_monitor',
            executable='obstacle_status_translator.py',
            name='obstacle_status_translator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--scan-topic', scan_topic,
                       '--stop-topic', '/car_op_zones/stop',
                       '--slow-l2-topic', '/car_op_zones/slow_l2',
                       '--slow-l1-topic', '/car_op_zones/slow_l1',
                       '--zone-frame', zone_frame,
                       '--profile', zones_profile,
                       '--no-passthrough'],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false',
            description='Start/stop the collision detection module: car_op_status_sim + car_op_zones (+ translator when use_obstacle_status). It never publishes /cmd_vel'),
        DeclareLaunchArgument(
            'use_obstacle_status', default_value='true',
            description='Start obstacle_status_translator and publish /collision_state + /obstacle_status_int; the translator never publishes /cmd_vel'),
        DeclareLaunchArgument(
            'use_car_op_status_sim', default_value='true',
            description='Start the default /car_op_status source; it yields while another publisher exists'),
        DeclareLaunchArgument(
            'car_op_status_value', default_value='0',
            description='Default source hold value 0-8 (0 = idle forward)'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/c200_lidar_node/scan',
            description='Laser scan topic used by the translator'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'zone_frame', default_value='base_link',
            description='Frame the dynamic zones and scan are projected into'),
        DeclareLaunchArgument(
            'zones_profile', default_value='real',
            description='Zone geometry: real = 1.2x0.8 m vehicle, sim = 0.52x0.36 m diffbot'),
        *status_nodes,
    ])
