#!/usr/bin/env python3

import rclpy
from byd_custom_msgs.msg import MotionState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class OdomToMotionState(Node):
    def __init__(self):
        super().__init__('odom_to_motion_state')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/motion_state')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        motion_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.publisher = self.create_publisher(
            MotionState, output_topic, motion_state_qos)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.on_odometry, odom_qos)
        self.get_logger().info(
            f'Converting {input_topic} Odometry to {output_topic} MotionState')

    def on_odometry(self, message):
        output = MotionState()
        output.header = message.header
        output.v_car = float(message.twist.twist.linear.x)
        output.w_car = float(message.twist.twist.angular.z)
        output.v_lift = 0.0
        output.lift_height = 0.0
        output.w_shelf = 0.0
        output.yaw_shelf = 0.0
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMotionState()
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
