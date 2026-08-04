// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/collision_monitor_node.hpp"

// 中文：独立进程入口只负责初始化 rclcpp、创建 Lifecycle CollisionMonitor、进入 ROS 事件循环并关闭。
// 中文：节点的 configure／activate 等状态转换由外部 Lifecycle Manager 或测试包装器负责。
int main(int argc, char * argv[])
{
  // 中文：解析 ROS 2 命令行参数并初始化底层通信。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_collision_monitor::CollisionMonitor>();
  // 中文：使用节点基础接口 spin，处理 cmd_vel、传感器消息、TF 和参数回调。
  rclcpp::spin(node->get_node_base_interface());
  // 中文：退出事件循环后释放 ROS 资源并返回操作系统状态码。
  rclcpp::shutdown();

  return 0;
}
