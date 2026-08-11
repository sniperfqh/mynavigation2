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

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 中文说明：SimpleGoalChecker 按“先 XY、后 Yaw”的顺序判断是否到达目标。
// stateful=true 时，首次进入 XY 容差后关闭后续 XY 检查，只继续对齐目标朝向；
// symmetric_yaw_tolerance=true 时，目标朝向和目标朝向加 π 都可接受，适合前后对称机器人。
// velocity 参数不参与该插件判定；需要停稳约束时应使用 StoppedGoalChecker。
namespace nav2_controller
{

// 中文说明：初始化位置、航向、状态模式和对称朝向模式的默认值。
// check_xy_ 表示当前任务是否仍需检查位置，xy_goal_tolerance_sq_ 是位置容差的平方缓存。
SimpleGoalChecker::SimpleGoalChecker() : xy_goal_tolerance_(0.25), yaw_goal_tolerance_(0.25), stateful_(true), check_xy_(true), symmetric_yaw_tolerance_(false), xy_goal_tolerance_sq_(0.0625) {
}

// 中文说明：声明并读取插件私有参数，随后注册动态参数回调。
// parent 提供 Controller Server 节点，plugin_name 用于形成如 <plugin>.xy_goal_tolerance 的参数名。
// costmap_ros 未被本实现使用，因为该插件只比较几何位姿，不查询 Costmap。
void SimpleGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/) {
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".stateful", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".symmetric_yaw_tolerance", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
  node->get_parameter(plugin_name + ".symmetric_yaw_tolerance", symmetric_yaw_tolerance_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  // 中文：运行期修改四个参数时直接更新内存阈值，不需要重新 configure Controller Server。
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&SimpleGoalChecker::dynamicParametersCallback, this, _1));
}

// 中文说明：为新任务恢复 XY 检查。
// 该函数不修改容差配置，只清除 stateful 模式在上一任务中锁存的“位置已满足”阶段状态。
void SimpleGoalChecker::reset() {
  check_xy_ = true;
}

// 中文说明：执行目标到达判定。
// query_pose 是当前位姿，goal_pose 是目标位姿，velocity 由接口传入但本插件忽略。
// 数据流为：位置未满足立即 false；位置满足后检查最短航向差；所有启用条件满足才返回 true。
bool SimpleGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist &) {
  if (check_xy_) {
    // 中文：使用 XY 平方距离与平方容差比较，避免在控制循环中计算平方根。
    double dx = query_pose.position.x - goal_pose.position.x, dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // 中文：当前位姿已进入目标位置容差圆。
    // If we are stateful, change the state.
    // 中文：stateful 模式将位置阶段锁存为完成，之后即使定位轻微漂出容差圆，也只继续检查朝向。
    if (stateful_) {
      check_xy_ = false;
    }
  }

  double query_yaw = tf2::getYaw(query_pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  if (symmetric_yaw_tolerance_) {
    // For symmetric robots: accept either goal orientation or goal + 180°
    // 中文：同时计算正向目标和反向目标的最短角距离，任一落入 yaw_goal_tolerance_ 即成功。
    double dyaw_forward = angles::shortest_angular_distance(query_yaw, goal_yaw);
    double dyaw_backward = angles::shortest_angular_distance(query_yaw, angles::normalize_angle(goal_yaw + M_PI));

    bool forward_match = fabs(dyaw_forward) <= yaw_goal_tolerance_;
    bool backward_match = fabs(dyaw_backward) <= yaw_goal_tolerance_;

    return forward_match || backward_match;
  } else {
    // 中文：普通机器人只接受配置的目标朝向，使用最短角距离正确处理 ±π 环绕。
    double dyaw = angles::shortest_angular_distance(query_yaw, goal_yaw);
    return fabs(dyaw) <= yaw_goal_tolerance_;
  }
}

// 中文说明：向 Controller Server 报告位置和航向容差。
// orientation 字段通过绕 Z 轴的四元数编码 yaw_goal_tolerance_；未参与检查的字段使用 lowest() 哨兵值。
// 因本插件不检查速度，所有速度容差字段均标记为无效。
bool SimpleGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

// 中文说明：处理位置容差、航向容差、stateful 和对称朝向模式的动态更新。
// 更新 XY 容差时同步重算平方缓存；参数变化不会自动调用 reset()，当前阶段锁存状态会保留。
rcl_interfaces::msg::SetParametersResult SimpleGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".symmetric_yaw_tolerance") {
        symmetric_yaw_tolerance_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// 中文说明：以 nav2_core::GoalChecker 基类导出，使插件 XML 中的类型可由 pluginlib 实例化。
PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleGoalChecker, nav2_core::GoalChecker)
