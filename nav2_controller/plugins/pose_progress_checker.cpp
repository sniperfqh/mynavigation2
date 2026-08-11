// Copyright (c) 2023 Dexory
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

#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "angles/angles.h"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 中文说明：PoseProgressChecker 在 SimpleProgressChecker 的平移进展判定上增加旋转进展。
// 只要累计平移超过 required_movement_radius，或累计最短角距离超过 required_movement_angle，
// 就刷新基准位姿和基准时间；两者都不足且持续超时，才判定机器人没有进展。
namespace nav2_controller
{

// 中文说明：先复用父类初始化时钟、平移半径、允许时间和父类动态参数回调，
// 再声明并读取本类的最小旋转角阈值，同时注册本类动态参数回调。
// 父类与子类回调分别负责各自参数，运行期修改三类阈值时都能生效。
void PoseProgressChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) {
  plugin_name_ = plugin_name;
  SimpleProgressChecker::initialize(parent, plugin_name);
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".required_movement_angle", rclcpp::ParameterValue(0.5));
  node->get_parameter_or(plugin_name + ".required_movement_angle", required_movement_angle_, 0.5);

  // Add callback for dynamic parameters
  // 中文：该回调只消费 required_movement_angle，父类回调继续消费半径和允许时间。
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&PoseProgressChecker::dynamicParametersCallback, this, _1));
}

// 中文说明：检查平移或旋转是否构成有效进展，并维护与父类相同的基准时间窗口。
// 输入 current_pose 被转换为平面位姿；输出 false 仅表示平移、旋转均不足且允许时间已超时。
bool PoseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose) {
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // 中文：依赖短路求值，首次调用不会读取尚未初始化的 baseline_pose_。
  // baseline_pose is not set.
  // 中文：首次检查直接建立位置和朝向基准，并返回仍有进展。
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if (!baseline_pose_set_ || PoseProgressChecker::isRobotMovedEnough(current_pose2d)) {
    // 中文：平移或旋转任一超过阈值，就从当前位姿重新计时。
    resetBaselinePose(current_pose2d);
    return true;
  }
  // 中文：这里使用“小于等于”，到达 allowance 边界时仍允许继续，超过后才返回 false。
  return clock_->now() - baseline_time_ <= time_allowance_;
}

// 中文说明：以逻辑或组合平移进展和旋转进展。
// 该设计允许原地转向被识别为有效动作，适合控制器执行 RotateToHeading 等阶段。
bool PoseProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose) {
  return pose_distance(pose, baseline_pose_) > radius_ || poseAngleDistance(pose, baseline_pose_) > required_movement_angle_;
}

// 中文说明：通过 angles::shortest_angular_distance 处理正负 π 环绕，再取绝对值获得最小角差。
// 因此从接近 +π 转到接近 -π 时不会被误判为接近 2π 的大角度运动。
double PoseProgressChecker::poseAngleDistance(const geometry_msgs::msg::Pose2D & pose1, const geometry_msgs::msg::Pose2D & pose2) {
  return abs(angles::shortest_angular_distance(pose1.theta, pose2.theta));
}

// 中文说明：运行期更新 required_movement_angle；平移半径和允许时间仍由父类回调更新。
// 本回调不刷新基准位姿，因此新角度阈值立即参与当前窗口的下一次 check()。
rcl_interfaces::msg::SetParametersResult PoseProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_angle") {
        required_movement_angle_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// 中文说明：以 nav2_core::ProgressChecker 接口导出，供 Controller Server 通过 pluginlib 动态加载。
PLUGINLIB_EXPORT_CLASS(nav2_controller::PoseProgressChecker, nav2_core::ProgressChecker)
