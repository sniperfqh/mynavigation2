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

#ifndef NAV2_CORE__BEHAVIOR_HPP_
#define NAV2_CORE__BEHAVIOR_HPP_

// 中文：本文件定义 Nav2 行为插件的最小公共契约，只声明接口，不创建节点、Action 或执行线程。

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

namespace nav2_core
{

/**
 * @class Behavior
 * @brief Abstract interface for behaviors to adhere to with pluginlib
 * 中文：Behavior 是 Behavior Server 通过 Pluginlib 加载所有行为插件时使用的抽象基类。
 * 中文：该层只统一配置和 Lifecycle 边界；Spin、BackUp、Wait 等具体执行入口由下游行为实现提供。
 */
class Behavior
{
public:
  // 中文：共享指针别名用于服务器或其他宿主以多态方式持有行为插件实例。
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief Virtual destructor
   * 中文：通过基类指针销毁具体插件时进入派生类析构，避免多态释放不完整。
   */
  virtual ~Behavior() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   * 中文：Behavior Server 在 configure 阶段调用该接口，把父 Lifecycle Node、插件实例名、共享 TF
   * 中文：缓存和 Costmap 碰撞检查器交给插件。插件应在此读取 `<name>.*` 参数并准备自身资源。
   * 中文：parent 使用弱引用，派生类必须先 lock；tf 和 collision_checker 为共享资源，不应擅自销毁。
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   * 中文：Lifecycle cleanup 阶段释放 configure 中创建的发布器、订阅器、Action Server 或缓存资源。
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   * 中文：Lifecycle activate 阶段启用插件业务接口和执行资源；不应在未激活状态接受新行为任务。
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   * 中文：Lifecycle deactivate 阶段停止活动任务和线程，并使插件回到可安全 cleanup 的状态。
   */
  virtual void deactivate() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_HPP_
