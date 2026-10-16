# RViz 独立启动入口。根据命名空间和仿真时间装载可视化配置，不改变导航控制链。

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 独立 RViz 启动描述。按 use_namespace 选择原配置或替换机器人命名空间后的配置。
# 本文件不启动导航与定位；两个 RViz 分支互斥，所选进程退出会触发本 Launch 关闭。
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


# 创建启动实参声明与条件 Action，并返回 LaunchDescription。
# 这里只描述启动关系；节点创建、参数替换与生命周期激活由 Launch 执行阶段完成。
def generate_launch_description():
    # 从 ament 安装索引定位资源目录，不依赖启动终端的当前路径。
    bringup_dir = get_package_share_directory('nav2_regulated_modules')

    # 声明延迟求值占位对象；实际值在 LaunchContext 可用时解析。
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # 声明用户可传入的启动实参及默认值，与构造 Node 时的参数字典不同。
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # 创建 RViz 进程描述，执行时由条件选择一个分支。
    # 无命名空间时直接启动 RViz，加载原配置文件。
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    # 把 RViz 配置中的机器人命名空间占位符替换为当前启动参数。
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace)})

    # 启用命名空间时加载替换后的配置，并重映射 TF、地图与交互话题。
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output='screen',
        remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    # 无命名空间的 RViz 进程退出后结束整个 Launch。
    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # 有命名空间的 RViz 进程退出后结束整个 Launch。
    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # 按依赖顺序组织声明、条件节点和退出事件处理器。
    ld = LaunchDescription()

    # 先注册实参声明，让后续替换与条件求值能读取其值。
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # 两个 RViz Node 条件互斥，不会因为都出现在描述中就同时启动。
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    # 注册所选 RViz 退出时的 Launch 收口事件。
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
