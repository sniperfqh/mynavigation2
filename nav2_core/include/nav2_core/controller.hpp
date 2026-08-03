/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

// 中文：本文件定义 Controller Server 与所有局部控制器插件之间的稳定 ABI／API 边界。

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/goal_checker.hpp"


namespace nav2_core
{

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 * 中文：Controller 是 DWB、RPP、MPPI、TEB 和 Rotation Shim 等控制器共同实现的抽象接口。
 * 中文：Controller Server 负责 Action、控制频率、机器人 Pose／速度、进度检查和速度发布；插件只
 * 中文：保存全局路径并在每个控制周期计算一个 TwistStamped，不直接完成 FollowPath Action 编排。
 */
class Controller
{
public:
  // 中文：宿主通过该共享指针别名保存具体控制器，同时只依赖 nav2_core 接口。
  using Ptr = std::shared_ptr<nav2_core::Controller>;


  /**
   * @brief Virtual destructor
   * 中文：保证通过 Controller 基类指针销毁派生插件时执行完整析构链。
   */
  virtual ~Controller() {}

  /**
   * @param  parent pointer to user's node
   * @param  costmap_ros A pointer to the costmap
   * 中文：Controller Server 在 Lifecycle configure 阶段传入父节点、插件实例名、共享 TF Buffer 和
   * 中文：Local Costmap2DROS。派生类通常在此声明参数、创建 Path 处理器并缓存机器人几何信息。
   * 中文：父节点是弱引用；TF 与 Costmap 的生命周期由服务器管理，插件只保存共享访问入口。
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  /**
   * @brief Method to cleanup resources.
   * 中文：释放 configure 阶段创建的参数回调、发布器、内部插件和算法缓存。
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   * 中文：激活控制器及其 Lifecycle 发布器或后台资源，使其可以响应控制周期调用。
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   * 中文：停止控制器活动资源；速度置零和 FollowPath 终态仍由 Controller Server 统一收口。
   */
  virtual void deactivate() = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   * 中文：Controller Server 在接收新 FollowPath Goal 或更新路径时调用。派生类应保存或预处理 Path，
   * 中文：但不能假定后续机器人 Pose 与 Path 已位于同一 Frame，必要变换由具体控制器实现负责。
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   * 中文：这是控制主循环的同步计算入口。pose 是当前机器人位姿，velocity 是当前实测或平滑速度，
   * 中文：goal_checker 是服务器当前选中的非拥有指针；返回值带时间戳和 Frame，随后由服务器发布。
   * 中文：实现无法生成安全命令时可抛出 PlannerException，由 Controller Server 转换为重试或 Action 失败。
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   * 中文：接收 Costmap Filter 或外部限速请求。percentage=true 表示相对插件最大速度的百分比，
   * 中文：false 表示以 m/s 给出绝对线速度上限；插件应在后续每个控制周期应用最新限制。
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_HPP_
