/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#ifndef NAV2_CORE__GOAL_CHECKER_HPP_
#define NAV2_CORE__GOAL_CHECKER_HPP_

// 中文：本文件定义“导航任务是否到达目标”的可插拔判定接口，由 Controller Server 按 Goal 使用。

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace nav2_core
{

/**
 * @class GoalChecker
 * @brief Function-object for checking whether a goal has been reached
 *
 * This class defines the plugin interface for determining whether you have reached
 * the goal state. This primarily consists of checking the relative positions of two poses
 * (which are presumed to be in the same frame). It can also check the velocity, as some
 * applications require that robot be stopped to be considered as having reached the goal.
 * 中文：GoalChecker 同时接收当前 Pose、目标 Pose 和当前速度，可实现仅位置、位置＋朝向或
 * 中文：位置＋朝向＋停车等不同到达语义。两个 Pose 必须已经处于同一坐标系。
 * 中文：Controller Server 每个控制周期调用当前选中的插件，并把同一指针传给 Controller 供算法参考。
 */
class GoalChecker
{
public:
  // 中文：服务器使用该共享指针别名在插件映射中保存一个或多个 GoalChecker 实例。
  typedef std::shared_ptr<nav2_core::GoalChecker> Ptr;

  // 中文：虚析构保证经基类指针释放具体检查器时执行完整派生析构链。
  virtual ~GoalChecker() {}

  /**
   * @brief Initialize any parameters from the NodeHandle
   * @param parent Node pointer for grabbing parameters
   * 中文：Controller Server 配置阶段传入父 Lifecycle Node、插件实例名和 Local Costmap2DROS。
   * 中文：实现应在此读取容差参数；父节点为弱引用，Costmap 为共享资源。
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  // 中文：开始新 Goal 或重置控制状态时清除插件内部的有状态判定，例如“首次满足 XY 容差”标记。
  virtual void reset() = 0;

  /**
   * @brief Check whether the goal should be considered reached
   * @param query_pose The pose to check
   * @param goal_pose The pose to check against
   * @param velocity The robot's current velocity
   * @return True if goal is reached
   * 中文：同步判断当前任务是否到达。返回 true 后由 Controller Server 收口 FollowPath，而不是插件
   * 中文：直接发布 Action Result；实现可以结合线速度和角速度要求机器人完全停稳。
   */
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  /**
   * @brief Get the maximum possible tolerances used for goal checking in the major types.
   * Any field without a valid entry is replaced with std::numeric_limits<double>::lowest()
   * to indicate that it is not measured. For tolerance across multiple entries
   * (e.x. XY tolerances), both fields will contain this value since it is the maximum tolerance
   * that each independent field could be assuming the other has no error (e.x. X and Y).
   * @param pose_tolerance The tolerance used for checking in Pose fields
   * @param vel_tolerance The tolerance used for checking velocity fields
   * @return True if the tolerances are valid to use
   * 中文：向 Controller 或其他调用方公开本插件可能使用的最大 Pose／Twist 容差。未参与判定的字段
   * 中文：应写入 `numeric_limits<double>::lowest()`；返回 false 表示该实现不提供可靠容差信息。
   */
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GOAL_CHECKER_HPP_
