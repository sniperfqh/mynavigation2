# 中文注释：独立启动键盘遥控节点，加载 YAML 并把当前交互终端传给 C++ 节点。
import os
from pathlib import Path
import sys

# 中文注释：ament 索引用于定位安装空间中的包共享目录。
from ament_index_python.packages import get_package_share_directory
# 中文注释：LaunchDescription 组织参数声明和节点启动动作。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 中文注释：生成键盘遥控 LaunchDescription，不启动规划、控制或 controlpub 节点。
def generate_launch_description():
    # 中文注释：从包共享目录加载默认键盘参数 YAML。
    package_share = Path(get_package_share_directory("myagv_keyboard_control"))
    params_file = package_share / "config" / "keyboard_control.yaml"
    # 中文注释：交互式 Shell 使用真实 /dev/pts/*；非交互环境回退到 /dev/tty。
    default_input_device = os.ttyname(sys.stdin.fileno()) if sys.stdin.isatty() else "/dev/tty"
    # 中文注释：允许命令行通过 input_device:=... 显式覆盖终端设备。
    input_device = LaunchConfiguration("input_device")

    # 中文注释：先声明终端参数，再创建唯一的键盘遥控节点。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_device",
                default_value=default_input_device,
                description="Terminal device used for keyboard input",
            ),
            Node(
                # 中文注释：节点直接发布 /control_to_uart，运行前必须确认没有其他发布者竞争。
                package="myagv_keyboard_control",
                executable="myagv_keyboard_control_node",
                name="myagv_keyboard_control",
                output="screen",
                # 中文注释：为日志输出分配伪终端；键盘输入仍由 input_device 参数指定。
                emulate_tty=True,
                # 中文注释：先加载 YAML，再用 Launch 参数覆盖同名 input_device。
                parameters=[str(params_file), {"input_device": input_device}],
            )
        ]
    )
