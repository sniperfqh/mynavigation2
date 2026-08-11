// Copyright (c) 2025 Prabhav Saxena
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

#ifndef NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_controller
{

/**
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
  * 中文：只检查 XY 位置并忽略朝向的 GoalChecker 插件。
 *
 * 中文详细说明：该插件只判断当前 XY 位置是否进入目标位置容差圆，不读取目标朝向，也不检查机器人
 * 速度。stateful=true 时，一旦位置首次满足条件就锁存成功，直到 reset() 开启下一次任务；
 * stateful=false 时每个控制周期都重新计算位置误差。
 */
class PositionGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * 中文说明：构造默认 0.25 m XY 容差及其平方缓存，并初始化 stateful 与位置锁存状态。
   */
  PositionGoalChecker();

  /**
   * 中文说明：使用默认析构逻辑释放参数回调句柄等成员，不拥有独立 ROS 节点或 Costmap 资源。
   */
  ~PositionGoalChecker() override = default;

  /**
   * 中文说明：声明并读取 xy_goal_tolerance 与 stateful 参数，计算平方容差并注册动态参数回调。
   * @param parent Controller Server 的 LifecycleNode 弱指针，用于访问参数接口。
   * @param plugin_name 插件实例名，也是私有参数命名空间前缀。
   * @param costmap_ros Controller Server 传入的 Costmap 共享指针；本实现不使用该参数。
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * 中文说明：清除 position_reached_ 锁存状态，为新任务重新启用位置判断；不修改任何参数阈值。
   */
  void reset() override;

  /**
   * 中文说明：判断当前 XY 平方距离是否小于等于 xy_goal_tolerance_sq_。
   * @param query_pose 机器人当前位姿，只读取 position.x 与 position.y。
   * @param goal_pose 目标位姿，只读取 position.x 与 position.y。
   * @param velocity 当前速度；为满足 GoalChecker 接口而传入，本插件完全忽略。
   * @return 位置满足容差或 stateful 已锁存成功时返回 true，否则返回 false。
   */
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;

  /**
   * 中文说明：向调用方报告本插件实际使用的容差字段。
   * @param pose_tolerance 输出 XY 位置容差；不参与检查的字段使用约定值或无效哨兵。
   * @param vel_tolerance 输出速度容差；本插件不检查速度，因此各字段标记为无效。
   * @return 当前实现始终返回 true，表示容差信息可用。
   */
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

  /**
   * @brief Set the XY goal tolerance
   * 中文：设置 XY 目标容差。
   * @param tolerance New tolerance value
   * 中文：新的容差值。
   * 中文详细说明：同时更新 xy_goal_tolerance_ 与其平方缓存；不会清除 stateful 已锁存的成功状态。
   */
  void setXYGoalTolerance(double tolerance);

protected:
  // 中文：XY 目标位置容差，单位为米。
  double xy_goal_tolerance_;
  // 中文：XY 容差平方缓存，用于与平方距离比较，避免控制循环中计算平方根。
  double xy_goal_tolerance_sq_;
  // 中文：是否在首次进入位置容差后锁存成功状态。
  bool stateful_;
  // 中文：记录 stateful 模式下位置是否已经满足过容差。
  bool position_reached_;
  // 中文：插件实例名，用于识别本插件的动态参数。
  std::string plugin_name_;
  // 中文：保存动态参数回调注册句柄，确保回调在插件生命周期内持续有效。
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
  * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
  * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
  * @return 参数更新结果；当前实现接受请求并更新匹配的 double／bool 参数。
   * 中文详细说明：xy_goal_tolerance 更新时同步维护平方缓存，stateful 更新不会主动重置锁存状态。
  */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
