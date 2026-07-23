#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def read_pgm(path):
    with open(path, 'rb') as pgm_file:
        tokens = []
        while len(tokens) < 4:
            line = pgm_file.readline()
            if not line:
                raise ValueError(f'Invalid PGM header: {path}')
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())

        magic, width, height, max_value = tokens[:4]
        if magic != b'P5' or int(max_value) > 255:
            raise ValueError(
                f'Only 8-bit binary PGM maps are supported: {path}')
        width = int(width)
        height = int(height)
        pixels = pgm_file.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f'Incomplete PGM image data: {path}')
        return width, height, pixels


class CpuLidar(Node):

    def __init__(self):
        super().__init__('cpu_lidar')
        self.declare_parameter('map_image', '')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin_x', -7.0)
        self.declare_parameter('map_origin_y', -10.5)
        self.declare_parameter('samples', 720)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('sensor_offset_x', 0.18)

        map_image = self.get_parameter('map_image').value
        self.width, self.height, self.pixels = read_pgm(map_image)
        self.resolution = self.get_parameter('map_resolution').value
        self.origin_x = self.get_parameter('map_origin_x').value
        self.origin_y = self.get_parameter('map_origin_y').value
        self.samples = self.get_parameter('samples').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.sensor_offset_x = self.get_parameter('sensor_offset_x').value
        update_rate = self.get_parameter('update_rate').value

        self.pose = None
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(1.0 / update_rate, self.publish_scan)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z))
        position = msg.pose.pose.position
        self.pose = (position.x, position.y, yaw)

    def occupied(self, grid_x, grid_y):
        if (grid_x < 0 or grid_x >= self.width
                or grid_y < 0 or grid_y >= self.height):
            return True
        image_y = self.height - 1 - grid_y
        pixel = self.pixels[image_y * self.width + grid_x]
        return pixel < 250

    def ray_range(self, start_x, start_y, angle):
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        grid_x = math.floor((start_x - self.origin_x) / self.resolution)
        grid_y = math.floor((start_y - self.origin_y) / self.resolution)
        step_x = 1 if direction_x >= 0.0 else -1
        step_y = 1 if direction_y >= 0.0 else -1

        next_x = self.origin_x + (grid_x + (step_x > 0)) * self.resolution
        next_y = self.origin_y + (grid_y + (step_y > 0)) * self.resolution
        delta_x = (
            abs(self.resolution / direction_x) if direction_x else math.inf)
        delta_y = (
            abs(self.resolution / direction_y) if direction_y else math.inf)
        distance_x = (
            (next_x - start_x) / direction_x if direction_x else math.inf)
        distance_y = (
            (next_y - start_y) / direction_y if direction_y else math.inf)

        distance = 0.0
        while distance <= self.range_max:
            if self.occupied(grid_x, grid_y):
                return max(distance, self.range_min)
            if distance_x < distance_y:
                grid_x += step_x
                distance = distance_x
                distance_x += delta_x
            else:
                grid_y += step_y
                distance = distance_y
                distance_y += delta_y
        return math.inf

    def publish_scan(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        sensor_x = x + self.sensor_offset_x * math.cos(yaw)
        sensor_y = y + self.sensor_offset_x * math.sin(yaw)
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.samples

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.angle_min = angle_min
        msg.angle_max = math.pi - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [
            self.ray_range(
                sensor_x,
                sensor_y,
                yaw + angle_min + index * angle_increment)
            for index in range(self.samples)
        ]
        self.scan_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CpuLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
