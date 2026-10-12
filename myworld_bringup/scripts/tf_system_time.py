#!/usr/bin/env python3
"""Stamp Gazebo dynamic transforms with ROS system time, leaving static TF alone."""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfSystemTime(Node):
    def __init__(self):
        super().__init__('tf_system_time')
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.publisher = self.create_publisher(TFMessage, '/tf', 100)
        self.subscription = self.create_subscription(
            TFMessage, '/_gazebo/tf', self.on_tf,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_tf(self, message):
        stamp = self.system_clock.now().to_msg()
        for transform in message.transforms:
            transform.header.stamp = stamp
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TfSystemTime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
