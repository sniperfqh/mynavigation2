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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
  * 中文：只检查位置偏差的 GoalChecker 插件。
 * 中文补充：当前实现实际按顺序检查 XY 位置与 Yaw 朝向，不检查速度。
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
  * 中文：如果 stateful 参数为 true（默认值），该类可以保持状态。
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
  * 中文：这意味着一旦 xy 位置满足条件，goal checker 后续不会再次检查 xy 是否匹配。
 *
 * 中文详细说明：位置检查使用平方距离；位置满足后再通过最短角距离检查目标航向。
 * symmetric_yaw_tolerance=true 时，目标朝向和目标朝向加 π 都可接受，适合前后对称机器人。
 * 需要同时判断停稳速度时，应使用派生类 StoppedGoalChecker。
 */
class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * 中文说明：构造默认 XY／Yaw 容差、stateful、位置阶段标志和对称朝向开关。
   */
  SimpleGoalChecker();
  // Standard GoalChecker Interface
  // 中文：标准 GoalChecker 接口。
  /**
   * 中文说明：声明并读取 XY、Yaw、stateful 与 symmetric_yaw_tolerance 参数，
   * 初始化平方容差缓存并注册动态参数回调。
   * @param parent Controller Server 的 LifecycleNode 弱指针。
   * @param plugin_name 插件实例名及参数命名空间前缀。
   * @param costmap_ros Controller Server 提供的 Costmap 指针；当前实现不使用该参数。
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * 中文说明：把 check_xy_ 恢复为 true，为新任务重新启用位置阶段检查；容差参数保持不变。
   */
  void reset() override;

  /**
   * 中文说明：按“XY 位置→Yaw 朝向”顺序判断目标是否到达。
   * @param query_pose 机器人当前位姿。
   * @param goal_pose 目标位姿。
   * @param velocity 当前速度；该实现忽略速度，仅保留接口兼容性。
   * @return 启用的位置和朝向条件都满足时返回 true；任一条件失败时返回 false。
   * stateful 模式下，位置首次满足后 check_xy_ 被关闭，后续只检查朝向。
   */
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;

  /**
   * 中文说明：返回插件实际使用的 XY 与 Yaw 容差，并把未使用的速度字段标记为无效。
   * @param pose_tolerance 输出位置与朝向容差。
   * @param vel_tolerance 输出速度容差；本插件不检查速度。
   * @return 当前实现始终返回 true，表示容差信息填充成功。
   */
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // 中文：XY 位置容差与 Yaw 朝向容差，单位分别为米和弧度。
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  // 中文：stateful_ 控制位置阶段锁存；check_xy_ 记录当前任务是否仍需检查 XY。
  bool stateful_, check_xy_;
  // 中文：是否把目标朝向和目标朝向加 π 视为等价方向。
  bool symmetric_yaw_tolerance_;
  // Cached squared xy_goal_tolerance_
  // 中文：缓存的 xy_goal_tolerance_ 平方值。
  double xy_goal_tolerance_sq_;
  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // 中文：插件实例名，用于筛选属于本插件命名空间的动态参数。
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
   * @return 参数更新结果；当前实现接受请求并更新匹配的 double／bool 参数。
   * 中文详细说明：可动态更新 XY／Yaw 容差、stateful 和对称朝向开关；XY 更新会同步平方缓存，
   * 参数变化不会自动 reset() 当前任务的阶段状态。
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
