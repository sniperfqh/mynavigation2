#!/usr/bin/env python3

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import xacro


def main():
    rclpy.init()
    node = rclpy.create_node('robot_description_publisher')

    urdf_path = os.path.join(
        get_package_share_directory('mytb3_bringup'),
        'urdf',
        'turtlebot3_waffle.urdf')
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(String, '/robot_description', qos)
    message = String()
    message.data = doc.toxml()

    def publish_description():
        publisher.publish(message)

    timer = node.create_timer(1.0, publish_description)
    publish_description()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
