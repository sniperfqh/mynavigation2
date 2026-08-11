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

#include <memory>
#include <string>
#include <limits>
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 中文说明：PositionGoalChecker 是只判断 XY 位置的 GoalChecker。
// 它忽略目标朝向和机器人速度，适合“到达目标点即可、无需对齐航向或停稳”的任务。
// stateful=true 时，一旦位置首次进入容差圆，就锁存成功状态，直到 reset() 开启下一次任务。
namespace nav2_controller
{

// 中文说明：构造函数建立默认 0.25 m 容差及其平方缓存，并初始化状态锁存标志。
// 缓存平方值可以让高频 isGoalReached() 使用平方距离比较，避免每周期计算平方根。
PositionGoalChecker::PositionGoalChecker() : xy_goal_tolerance_(0.25), xy_goal_tolerance_sq_(0.0625), stateful_(true), position_reached_(false) {
}

// 中文说明：初始化插件参数和动态参数回调。
// parent 是 Controller Server 的 LifecycleNode，plugin_name 隔离不同 GoalChecker 实例的参数。
// costmap_ros 在该实现中未使用，因为位置到达判定只依赖当前位姿和目标位姿。
void PositionGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/) {
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  // 中文：运行期更新位置容差或 stateful 模式时，无需重新加载插件。
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&PositionGoalChecker::dynamicParametersCallback, this, _1));
}

// 中文说明：清除“位置已经到达”的锁存状态，为下一次控制任务重新启用位置检查。
void PositionGoalChecker::reset() {
  position_reached_ = false;
}

// 中文说明：判断当前 XY 位置是否进入以目标点为圆心、xy_goal_tolerance_ 为半径的容差圆。
// query_pose 是机器人当前位姿，goal_pose 是目标位姿，速度参数被接口保留但本插件不使用。
// 返回值只表示位置条件；朝向、线速度和角速度均不会改变结果。
bool PositionGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist &) {
  // If stateful and position was already reached, maintain state
  // 中文：stateful 模式下，一旦成功就持续返回 true，防止轻微定位抖动让结果反复切换。
  if (stateful_ && position_reached_) {
    return true;
  }

  // Check if position is within tolerance
  // 中文：直接比较平方距离和平方容差，结果与欧氏距离比较相同，但避免 sqrt 开销。
  double dx = query_pose.position.x - goal_pose.position.x;
  double dy = query_pose.position.y - goal_pose.position.y;

  bool position_reached = (dx * dx + dy * dy <= xy_goal_tolerance_sq_);

  // If stateful, remember that we reached the position
  // 中文：仅在 stateful 开启且本次确实到达时锁存；非 stateful 模式每周期都重新判断。
  if (stateful_ && position_reached) {
    position_reached_ = true;
  }

  return position_reached;
}

// 中文说明：通过 GoalChecker 标准接口向 Controller Server 报告本插件使用的容差。
// pose_tolerance 的 x、y 返回位置容差；不参与判定的字段用 lowest() 标记为无效。
// 该插件不检查速度，所有速度容差字段都返回无效哨兵值。
bool PositionGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;

  // Return zero orientation tolerance as we don't check it
  // 中文：返回单位四元数只是满足消息字段格式；实际 isGoalReached() 完全不读取朝向。
  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

// 中文说明：供外部代码直接设置 XY 容差，并同步维护平方缓存。
// 该入口不会改变 position_reached_，因此 stateful 已锁存成功时仍保持成功，除非调用 reset()。
void nav2_controller::PositionGoalChecker::setXYGoalTolerance(double tolerance) {
  xy_goal_tolerance_ = tolerance;
  xy_goal_tolerance_sq_ = tolerance * tolerance;
}

// 中文说明：处理 xy_goal_tolerance 和 stateful 的运行期参数更新。
// 修改位置容差时同步重算平方缓存；修改 stateful 时不主动清除已有锁存状态。
rcl_interfaces::msg::SetParametersResult PositionGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// 中文说明：向 pluginlib 注册为 nav2_core::GoalChecker 实现，供 Controller Server 按类型名加载。
PLUGINLIB_EXPORT_CLASS(nav2_controller::PositionGoalChecker, nav2_core::GoalChecker)
