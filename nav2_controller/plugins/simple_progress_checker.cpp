// Copyright (c) 2019 Intel Corporation
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

#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// 中文说明：SimpleProgressChecker 是 Controller Server 的进展检查插件。
// 它保存一个“基准位姿＋基准时间”，只考察机器人在 XY 平面内是否移动了足够距离。
// 机器人一旦移动超过 required_movement_radius，就把当前位置和当前时间设为新基准；
// 若始终没有达到该距离，并且持续时间超过 movement_time_allowance，check() 才返回 false。
namespace nav2_controller
{
// 中文说明：初始化插件名、时钟、参数和动态参数回调。
// parent 提供 Controller Server 的 LifecycleNode，plugin_name 用于构造插件私有参数命名空间。
// radius_ 的单位是米，time_allowance_ 的单位是秒；初始化只读取配置，不建立基准位姿。
void SimpleProgressChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) {
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  // 中文：读取最小有效移动半径；未配置时使用 0.5 m。
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  // 中文：先以 double 读取秒数，再转换成 ROS Duration，便于直接与时钟差值比较。
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  // Add callback for dynamic parameters
  // 中文：运行期修改半径或允许时间时，直接更新本插件的判定阈值。
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&SimpleProgressChecker::dynamicParametersCallback, this, _1));
}

// 中文说明：检查当前周期是否仍满足“有进展”条件。
// 输入 current_pose 是当前机器人位姿；输出 true 表示尚未卡滞，false 表示从基准时刻起超时且位移不足。
// 首次调用没有基准位姿，因此会建立基准并返回 true；达到位移阈值时同样刷新基准和计时窗口。
bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose) {
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // 中文：依赖短路求值；未建立基准时不会调用需要读取 baseline_pose_ 的距离检查。
  // baseline_pose is not set.
  // 中文：这样首次检查可以安全地直接进入基准初始化分支。
  geometry_msgs::msg::Pose2D current_pose2d;
  // 中文：ProgressChecker 只需要平面位姿，将 PoseStamped 中的三维 Pose 转成 x、y、theta。
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if ((!baseline_pose_set_) || (isRobotMovedEnough(current_pose2d))) {
    // 中文：首次检查或累计位移超过半径时，以当前位置重新开始一个允许时间窗口。
    resetBaselinePose(current_pose2d);
    return true;
  }
  // 中文：位移尚不足时，只要经过时间未严格超过 allowance 就继续返回 true；超时后返回 false。
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

// 中文说明：开始新任务或控制器要求重置时，仅取消基准有效标志。
// 下一次 check() 会用届时的实际位姿和时间建立新基准，避免沿用上一次任务的进展历史。
void SimpleProgressChecker::reset() {
  baseline_pose_set_ = false;
}

// 中文说明：同步保存新的平面基准位姿、基准时间和有效标志。
// 三个状态必须一起更新，否则位移窗口与时间窗口会来自不同采样时刻。
void SimpleProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose2D & pose) {
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

// 中文说明：判断当前位置相对基准位置的欧氏距离是否严格大于配置半径。
// SimpleProgressChecker 不使用 theta，因此原地旋转不会被视为有效进展。
bool SimpleProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose) {
  return pose_distance(pose, baseline_pose_) > radius_;
}

// 中文说明：计算两个二维位姿在 XY 平面上的欧氏距离；theta 不参与计算。
double SimpleProgressChecker::pose_distance(const geometry_msgs::msg::Pose2D & pose1, const geometry_msgs::msg::Pose2D & pose2) {
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}

// 中文说明：处理本插件命名空间下两个 double 参数的原子参数更新请求。
// 未命中的参数交给节点上的其他回调处理；本回调不重置现有基准，因此新阈值立即作用于当前进展窗口。
rcl_interfaces::msg::SetParametersResult SimpleProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_radius") {
        radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// 中文说明：向 pluginlib 导出实现，使 Controller Server 能按配置中的类型字符串加载该 ProgressChecker。
PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleProgressChecker, nav2_core::ProgressChecker)
