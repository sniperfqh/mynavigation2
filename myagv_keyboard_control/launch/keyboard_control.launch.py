import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("myagv_keyboard_control"))
    params_file = package_share / "config" / "keyboard_control.yaml"
    default_input_device = os.ttyname(sys.stdin.fileno()) if sys.stdin.isatty() else "/dev/tty"
    input_device = LaunchConfiguration("input_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_device",
                default_value=default_input_device,
                description="Terminal device used for keyboard input",
            ),
            Node(
                package="myagv_keyboard_control",
                executable="myagv_keyboard_control_node",
                name="myagv_keyboard_control",
                output="screen",
                emulate_tty=True,
                parameters=[str(params_file), {"input_device": input_device}],
            )
        ]
    )
