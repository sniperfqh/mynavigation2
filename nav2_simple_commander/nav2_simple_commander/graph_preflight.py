#!/usr/bin/env python3

import argparse
import os
import time

import rclpy
from rclpy.node import Node


CONFLICTING_SERVICES = (
    '/map_server/get_state',
    '/lifecycle_manager_localization/manage_nodes',
    '/lifecycle_manager_navigation/manage_nodes',
)


def main():
    parser = argparse.ArgumentParser(
        description='Check for an existing Nav2 stack or simulation clock.')
    parser.add_argument(
        '--allow-existing',
        choices=('true', 'false'),
        default='false')
    args = parser.parse_args()

    rclpy.init()
    node = Node(f'follow_path_graph_preflight_{os.getpid()}')
    try:
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        service_names = {
            name for name, _ in node.get_service_names_and_types()
        }
        conflicting_services = [
            name for name in CONFLICTING_SERVICES if name in service_names
        ]
        clock_publishers = node.get_publishers_info_by_topic('/clock')

        if not conflicting_services and not clock_publishers:
            node.get_logger().info(
                'ROS graph preflight passed: no existing Nav2 lifecycle '
                'services or /clock publishers found.')
            return 0

        details = []
        if conflicting_services:
            details.append(
                'lifecycle services: ' + ', '.join(conflicting_services))
        if clock_publishers:
            publishers = sorted({
                f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in clock_publishers
            })
            details.append('/clock publishers: ' + ', '.join(publishers))

        message = 'ROS graph conflict detected; ' + '; '.join(details)
        if args.allow_existing == 'true':
            node.get_logger().warning(
                message +
                '. Continuing because allow_existing_ros_graph=true.')
            return 0

        node.get_logger().error(
            message + '. Stop the existing stack before launching this demo, '
            'or explicitly set allow_existing_ros_graph:=true.')
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
