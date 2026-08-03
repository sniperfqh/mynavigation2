// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_CORE__SMOOTHER_HPP_
#define NAV2_CORE__SMOOTHER_HPP_

// 中文：本文件定义 Smoother Server 与所有路径平滑插件之间的统一生命周期和同步处理接口。

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_core
{

/**
 * @class Smoother
 * @brief smoother interface that acts as a virtual base class for all smoother plugins
 * 中文：Smoother 是 Simple、Savitzky-Golay、Constrained 等平滑器通过 Pluginlib 接入服务器的基类。
 * 中文：服务器负责 SmoothPath Action、插件选择、Costmap／Footprint 订阅、结果发布和可选碰撞检查；
 * 中文：插件只在时间预算内原地修改输入 Path。
 */
class Smoother
{
public:
  // 中文：Smoother Server 的插件映射通过该共享指针别名保存不同算法实例。
  using Ptr = std::shared_ptr<nav2_core::Smoother>;

  /**
   * @brief Virtual destructor
   * 中文：保证经 Smoother 基类指针释放插件时执行派生类完整析构。
   */
  virtual ~Smoother() {}

  /**
   * @brief Configure a smoother plugin and provide shared server resources.
   * @param parent Parent Lifecycle Node hosted by Smoother Server
   * @param name Plugin instance ID and parameter namespace
   * @param tf Shared TF Buffer
   * @param costmap_sub Subscriber providing the current Costmap snapshot
   * @param footprint_sub Subscriber providing the current robot Footprint
   * 中文：Smoother Server 在 configure 阶段调用。派生类按需保存资源并读取 `<name>.*` 参数；
   * 中文：TF、CostmapSubscriber 和 FootprintSubscriber 的创建／销毁责任仍属于服务器。
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;

  /**
   * @brief Method to cleanup resources.
   * 中文：Lifecycle cleanup 阶段释放插件配置资源和算法缓存，不负责销毁服务器共享订阅器。
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   * 中文：Lifecycle activate 阶段启用插件资源；SmoothPath Action 的激活由 Smoother Server 负责。
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   * 中文：Lifecycle deactivate 阶段停止插件活动资源，为 cleanup 或重新配置建立安全边界。
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   * 中文：path 是输入输出引用，成功时直接写回平滑后的 Pose 序列；max_time 来自每次 SmoothPath Goal。
   * 中文：返回值表达算法是否在预算内完成，插件也可抛出 PlannerException；是否执行完整 Footprint
   * 中文：碰撞检查由包外 Smoother Server 根据 Goal 标志决定。
   */
  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SMOOTHER_HPP_
