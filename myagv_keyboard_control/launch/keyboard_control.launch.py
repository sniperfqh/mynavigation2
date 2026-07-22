from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("myagv_keyboard_control"))
    params_file = package_share / "config" / "keyboard_control.yaml"

    return LaunchDescription(
        [
            Node(
                package="myagv_keyboard_control",
                executable="myagv_keyboard_control_node",
                name="myagv_keyboard_control",
                output="screen",
                emulate_tty=True,
                parameters=[str(params_file)],
            )
        ]
    )
