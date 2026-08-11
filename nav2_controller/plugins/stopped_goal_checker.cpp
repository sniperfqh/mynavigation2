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

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::hypot;
using std::fabs;

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 中文说明：StoppedGoalChecker 在 SimpleGoalChecker 的位置和朝向条件之外，再要求机器人已经停稳。
// 只有平面线速度模长不超过 trans_stopped_velocity_，且 Z 轴角速度绝对值不超过
// rot_stopped_velocity_ 时才返回到达；因此它能防止机器人高速穿过目标容差区时被提前判定成功。
namespace nav2_controller
{

// 中文说明：复用 SimpleGoalChecker 的默认位姿容差，并设置默认平移、旋转停稳阈值为 0.25。
StoppedGoalChecker::StoppedGoalChecker() : SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25) {
}

// 中文说明：先由父类初始化位置、航向、stateful 和对称朝向参数，
// 再读取本类的角速度与平面线速度停稳阈值，并注册本类动态参数回调。
// 父类回调继续处理位姿相关参数，本类回调只处理两个速度阈值。
void StoppedGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  plugin_name_ = plugin_name;
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  // Add callback for dynamic parameters
  // 中文：速度阈值可在运行期更新，下一次 isGoalReached() 立即使用新值。
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&StoppedGoalChecker::dynamicParametersCallback, this, _1));
}

// 中文说明：先执行父类的位置＋朝向判定，父类失败时直接返回 false；
// 父类成功后，再同时检查角速度绝对值和平面线速度模长是否低于停稳阈值。
// query_pose、goal_pose 进入父类几何判定，velocity 只在本类的第二阶段使用。
bool StoppedGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) {
  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  // 中文：hypot(x, y) 使用平面速度模长，避免只检查单轴而漏掉斜向运动。
  return fabs(velocity.angular.z) <= rot_stopped_velocity_ && hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

// 中文说明：先复用父类填充位姿容差，再用本类的停稳阈值覆盖速度容差。
// 不参与检查的 linear.z、angular.x 和 angular.y 使用 lowest() 标记为无效。
bool StoppedGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  // 中文：父类负责写入 XY 与 Yaw 容差，并先把全部速度字段标为无效。
  bool rtn = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  // 中文：用实际参与停稳判定的平移和旋转阈值覆盖对应速度字段。
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

// 中文说明：处理 rot_stopped_velocity 和 trans_stopped_velocity 的动态更新。
// 位姿容差及 stateful 等父类参数由父类已注册的回调处理，本回调始终接受合法类型的更新请求。
rcl_interfaces::msg::SetParametersResult StoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// 中文说明：以 nav2_core::GoalChecker 接口导出，供 Controller Server 通过 pluginlib 加载停稳目标检查器。
PLUGINLIB_EXPORT_CLASS(nav2_controller::StoppedGoalChecker, nav2_core::GoalChecker)
