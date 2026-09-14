#!/usr/bin/env python3
import math
import time

import rclpy
from byd_custom_msgs.msg import ChassisControl
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ChassisControlToTwist(Node):
    def __init__(self):
        super().__init__('chassis_control_to_twist')
        self.declare_parameter('input_topic', '/downstream/chassis_control')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_linear_velocity', 0.52)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('default_linear_acceleration', 0.5)
        self.declare_parameter('default_angular_acceleration', 1.0)
        self.declare_parameter('max_linear_acceleration', 2.5)
        self.declare_parameter('max_angular_acceleration', 3.2)
        self.declare_parameter('linear_stop_threshold', 0.01)
        self.declare_parameter('angular_stop_threshold', 0.05)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)
        self.default_linear_acceleration = float(self.get_parameter('default_linear_acceleration').value)
        self.default_angular_acceleration = float(self.get_parameter('default_angular_acceleration').value)
        self.max_linear_acceleration = float(self.get_parameter('max_linear_acceleration').value)
        self.max_angular_acceleration = float(self.get_parameter('max_angular_acceleration').value)
        self.linear_stop_threshold = float(self.get_parameter('linear_stop_threshold').value)
        self.angular_stop_threshold = float(self.get_parameter('angular_stop_threshold').value)
        self._validate_parameters()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Twist, self.output_topic, qos)
        self.subscription = self.create_subscription(ChassisControl, self.input_topic, self._on_command, qos)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self._on_timer)
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.linear_acceleration = self.default_linear_acceleration
        self.angular_acceleration = self.default_angular_acceleration
        self.pending_command = None
        self.command_active = False
        self.publisher_conflict = False
        self.last_command_time = time.monotonic()
        self.last_update_time = self.last_command_time
        self.get_logger().info(f'仿真遥控网关已启动：{self.input_topic} -> {self.output_topic}')

    def _validate_parameters(self):
        positive = {
            'publish_frequency': self.publish_frequency,
            'command_timeout': self.command_timeout,
            'max_linear_velocity': self.max_linear_velocity,
            'max_angular_velocity': self.max_angular_velocity,
            'default_linear_acceleration': self.default_linear_acceleration,
            'default_angular_acceleration': self.default_angular_acceleration,
            'max_linear_acceleration': self.max_linear_acceleration,
            'max_angular_acceleration': self.max_angular_acceleration,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} 必须是有限正数，当前值={value}')
        if not self.input_topic or not self.output_topic:
            raise ValueError('input_topic 和 output_topic 不能为空')
        if not math.isfinite(self.linear_stop_threshold) or self.linear_stop_threshold < 0.0:
            raise ValueError('linear_stop_threshold 必须是有限非负数')
        if not math.isfinite(self.angular_stop_threshold) or self.angular_stop_threshold < 0.0:
            raise ValueError('angular_stop_threshold 必须是有限非负数')

    def _on_command(self, message):
        if not all(math.isfinite(value) for value in (message.linear_velocity, message.angular_velocity, message.acceleration)) or message.linear_velocity < 0.0 or message.angular_velocity < 0.0 or message.acceleration < 0.0:
            self.get_logger().error(f'拒绝非法 ChassisControl：op={message.op}，linear_velocity={message.linear_velocity}，angular_velocity={message.angular_velocity}，acceleration={message.acceleration}')
            self._stop_and_clear()
            return
        command = self._convert_command(message)
        if command is None:
            self.get_logger().error(f'拒绝未知 ChassisControl op={message.op}')
            self._stop_and_clear()
            return
        if self.count_publishers(self.output_topic) > 1:
            self.get_logger().error(f'检测到多个 {self.output_topic} Publisher，拒绝新的遥控命令')
            self._stop_and_clear()
            self.publisher_conflict = True
            return
        linear, angular, linear_acceleration, angular_acceleration = command
        self.last_command_time = time.monotonic()
        self.command_active = True
        if self._requires_stop_before_switch(linear, angular):
            self.pending_command = command
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.linear_acceleration = linear_acceleration
            self.angular_acceleration = angular_acceleration
            self.get_logger().info(f'遥控方向切换，先减速到零，pending_op={message.op}')
        else:
            self._apply_command(command)
            self.pending_command = None

    def _convert_command(self, message):
        linear = 0.0
        angular = 0.0
        linear_acceleration = self.default_linear_acceleration
        angular_acceleration = self.default_angular_acceleration
        if message.op == ChassisControl.OP_FORWARD or message.op == ChassisControl.OP_BACKWARD:
            linear = min(float(message.linear_velocity), self.max_linear_velocity)
            if message.op == ChassisControl.OP_BACKWARD:
                linear = -linear
            if message.acceleration > 0.0:
                linear_acceleration = min(float(message.acceleration), self.max_linear_acceleration)
        elif message.op == ChassisControl.OP_TURN_LEFT or message.op == ChassisControl.OP_TURN_RIGHT:
            angular = min(float(message.angular_velocity), self.max_angular_velocity)
            if message.op == ChassisControl.OP_TURN_RIGHT:
                angular = -angular
            if message.acceleration > 0.0:
                angular_acceleration = min(float(message.acceleration), self.max_angular_acceleration)
        else:
            return None
        return linear, angular, linear_acceleration, angular_acceleration

    def _requires_stop_before_switch(self, linear, angular):
        linear_sign_change = (self.target_linear * linear < 0.0 or self.output_linear * linear < 0.0)
        angular_sign_change = (self.target_angular * angular < 0.0 or self.output_angular * angular < 0.0)
        linear_to_angular = abs(self.output_linear) > self.linear_stop_threshold and abs(angular) > 0.0
        angular_to_linear = abs(self.output_angular) > self.angular_stop_threshold and abs(linear) > 0.0
        return self.command_active and (linear_sign_change or angular_sign_change or linear_to_angular or angular_to_linear)

    def _apply_command(self, command):
        self.target_linear, self.target_angular, self.linear_acceleration, self.angular_acceleration = command

    def _on_timer(self):
        now = time.monotonic()
        if self.count_publishers(self.output_topic) > 1:
            if not self.publisher_conflict:
                self.get_logger().error(f'检测到多个 {self.output_topic} Publisher，停止仿真遥控输出')
                self._stop_and_clear()
            self.publisher_conflict = True
            self.last_update_time = now
            return
        if self.publisher_conflict:
            self.publisher_conflict = False
            self.get_logger().info(f'{self.output_topic} Publisher 冲突已解除，等待新的遥控命令')
        if not self.command_active:
            self.last_update_time = now
            return
        if now - self.last_command_time > self.command_timeout:
            self.get_logger().warning(f'遥控命令超过 {self.command_timeout:.3f}s 未更新，仿真机器人停车')
            self._stop_and_clear()
            self.last_update_time = now
            return
        dt = min(max(now - self.last_update_time, 0.0), 0.1)
        self.last_update_time = now
        self.output_linear = self._approach(self.output_linear, self.target_linear, self.linear_acceleration * dt)
        self.output_angular = self._approach(self.output_angular, self.target_angular, self.angular_acceleration * dt)
        if self.pending_command is not None and abs(self.output_linear) <= self.linear_stop_threshold and abs(self.output_angular) <= self.angular_stop_threshold:
            self.output_linear = 0.0
            self.output_angular = 0.0
            self._apply_command(self.pending_command)
            self.pending_command = None
            self.get_logger().info('仿真机器人已停稳，开始执行待处理遥控命令')
        self._publish(self.output_linear, self.output_angular)

    def _stop_and_clear(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.pending_command = None
        self.command_active = False
        self._publish(0.0, 0.0)

    def _publish(self, linear, angular):
        output = Twist()
        output.linear.x = linear
        output.angular.z = angular
        self.publisher.publish(output)

    @staticmethod
    def _approach(current, target, maximum_delta):
        if current < target:
            return min(current + maximum_delta, target)
        if current > target:
            return max(current - maximum_delta, target)
        return target

    def stop(self):
        self._stop_and_clear()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisControlToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
