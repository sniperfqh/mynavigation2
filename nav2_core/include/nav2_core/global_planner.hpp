// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

// 中文：本文件定义 Planner Server 与所有全局规划算法之间的纯虚接口，不包含具体搜索实现。

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 * 中文：GlobalPlanner 是 NavFn、Smac、Theta* 等规划器通过 Pluginlib 接入 Planner Server 的共同基类。
 * 中文：服务器负责 Action、起点获取、目标变换、插件选择和结果发布；插件负责在给定 Costmap 上生成 Path。
 */
class GlobalPlanner
{
public:
  // 中文：Planner Server 通过该共享指针别名持有不同算法实例，并统一按基类接口调用。
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief Virtual destructor
   * 中文：确保通过 GlobalPlanner 基类指针释放具体规划器时执行派生类析构。
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   * 中文：Planner Server 在 Lifecycle configure 阶段传入父节点、插件实例名、共享 TF Buffer 和
   * 中文：Global Costmap2DROS。插件通常在此读取 `<name>.*` 参数并初始化搜索器或启发式缓存。
   * 中文：这些服务器资源采用共享访问；插件不拥有父节点，也不应自行销毁 TF 或 Costmap。
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   * 中文：Lifecycle cleanup 阶段释放规划器在 configure 中建立的动态参数回调和算法缓存。
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   * 中文：Lifecycle activate 阶段启用规划器资源；Action Server 的激活由 Planner Server 负责。
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   * 中文：Lifecycle deactivate 阶段停止规划器活动资源，为服务器停用或重新配置做准备。
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   * 中文：这是同步规划入口。start 和 goal 由上层准备，插件返回带 Frame／时间信息的 Path；无有效路径
   * 中文：时可返回空路径或抛出 PlannerException，最终 Action 结果由 Planner Server 统一决定。
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
