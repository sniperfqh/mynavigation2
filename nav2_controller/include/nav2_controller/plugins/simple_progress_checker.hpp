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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_controller
{
/**
* @class SimpleProgressChecker
* @brief This plugin is used to check the position of the robot to make sure
 * 中文：该插件用于检查机器人位置，确保
* that it is actually progressing towards a goal.
 * 中文：机器人确实在朝目标推进。
 *
 * 中文详细说明：该插件实现 nav2_core::ProgressChecker，在 Controller Server 控制循环中判断机器人
 * 是否持续取得平移进展。它维护基准平面位姿和基准时间；首次检查或位移超过配置半径时刷新基准，
 * 位移不足但尚未超时仍允许继续，位移不足且超过允许时间才返回失败。该类忽略朝向变化。
*/

class SimpleProgressChecker : public nav2_core::ProgressChecker
{
public:
  /**
   * 中文说明：初始化插件运行上下文、参数和动态参数回调。
   * @param parent Controller Server 的 LifecycleNode 弱指针，用于获取时钟、参数和注册回调。
   * @param plugin_name 插件实例名，同时作为 required_movement_radius 与
   * movement_time_allowance 的参数命名空间前缀。
   * 初始化不会建立基准位姿，首次 check() 才会以实际机器人位姿开始计时。
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) override;

  /**
   * 中文说明：检查当前控制周期是否仍满足进展条件。
   * @param current_pose 机器人当前 PoseStamped；实现只使用其中的 XY 平面位置。
   * @return 首次检查、位移超过阈值或尚未超时时返回 true；位移不足且超时返回 false。
   * 达到位移阈值时会把当前位置和当前时间保存为新的基准。
   */
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  /**
   * 中文说明：清除基准有效标志，为新任务重新开始进展统计。
   * 下一次 check() 会建立新的基准位姿和基准时间；参数阈值保持不变。
   */
  void reset() override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * 中文：根据基准位姿计算机器人移动量。
   * @param pose Current pose of the robot
   * 中文：机器人当前位姿。
   * @return true, if movement is greater than radius_, or false
   * 中文：如果移动距离大于 radius_ 则返回 true，否则返回 false。
   * 中文详细说明：比较 pose 与 baseline_pose_ 的 XY 欧氏距离，使用严格大于关系；theta 不参与判断。
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);
  /**
   * @brief Resets baseline pose with the current pose of the robot
   * 中文：使用机器人当前位姿重置基准位姿。
   * @param pose Current pose of the robot
   * 中文：机器人当前位姿。
   * 中文详细说明：同时更新 baseline_pose_、baseline_time_ 和 baseline_pose_set_，确保距离窗口与
   * 时间窗口来自同一采样时刻。
   */
  void resetBaselinePose(const geometry_msgs::msg::Pose2D & pose);

  /**
   * 中文说明：计算两个二维位姿在 XY 平面上的欧氏距离，不使用 theta。
   * @param 第一个参数为待比较位姿。
   * @param 第二个参数为参考位姿。
   * @return 两个位姿的平面直线距离，单位为米。
   */
  static double pose_distance(const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);

  // 中文：节点时钟，用于计算当前时间与基准时间的差值，并兼容 use_sim_time。
  rclcpp::Clock::SharedPtr clock_;

  // 中文：构成有效进展所需的最小 XY 位移半径，单位为米。
  double radius_;
  // 中文：机器人未达到最小位移时允许持续运行的最长时间。
  rclcpp::Duration time_allowance_{0, 0};

  // 中文：当前进展窗口的基准平面位姿。
  geometry_msgs::msg::Pose2D baseline_pose_;
  // 中文：建立当前基准位姿时的节点时间。
  rclcpp::Time baseline_time_;

  // 中文：标记基准位姿与时间是否已经由首次 check() 初始化。
  bool baseline_pose_set_{false};
  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
   * @return 参数处理结果；当前实现接受请求，并更新匹配的 double 参数。
   * 中文详细说明：可动态更新 required_movement_radius 和 movement_time_allowance；更新不会重置
   * 当前基准，因此新阈值从下一次 check() 起作用。
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
