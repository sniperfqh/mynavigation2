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

#ifndef NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_controller
{
/**
* @class PoseProgressChecker
* @brief This plugin is used to check the position and the angle of the robot to make sure
 * 中文：该插件用于检查机器人的位置和角度，确保
* that it is actually progressing or rotating towards a goal.
 * 中文：机器人确实在朝目标推进或旋转。
 *
 * 中文详细说明：该类继承 SimpleProgressChecker，在相同的基准时间窗口内增加旋转进展判定。
 * 机器人只要 XY 位移超过 required_movement_radius，或最短角距离超过 required_movement_angle，
 * 就会刷新基准并继续执行；两者都不足且超时才返回失败，适合包含原地转向阶段的控制器。
*/

class PoseProgressChecker : public SimpleProgressChecker
{
public:
  /**
   * 中文说明：先初始化父类的时钟、平移半径和允许时间，再初始化本类的最小旋转角阈值。
   * @param parent Controller Server 的 LifecycleNode 弱指针。
   * @param plugin_name 插件实例名，也是 required_movement_angle 等参数的命名空间前缀。
   * 父类和子类分别保留动态参数回调，负责各自的参数集合。
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) override;

  /**
   * 中文说明：检查平移或旋转是否构成有效进展，并维护基准时间窗口。
   * @param current_pose 当前机器人位姿，转换为 x、y、theta 后参与判断。
   * @return 首次检查、平移或旋转超过阈值、尚未超时时返回 true；否则返回 false。
   */
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * 中文：根据基准位姿计算机器人移动量。
   * @param pose Current pose of the robot
   * 中文：机器人当前位姿。
   * @return true, if movement is greater than radius_, or false
   * 中文：如果移动距离大于 radius_ 则返回 true，否则返回 false。
   * 中文详细说明：当前实现采用“平移超过父类 radius_ 或旋转超过 required_movement_angle_”的
   * 逻辑或条件，因此原地旋转也可以刷新进展基准。
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);

  /**
   * 中文说明：计算两个平面位姿的最短角距离绝对值。
   * @param 第一个参数为当前位姿。
   * @param 第二个参数为基准位姿。
   * @return 归一化到最短路径后的角度差绝对值，正确处理正负 π 环绕，单位为弧度。
   */
  static double poseAngleDistance(const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);

  // 中文：构成有效旋转进展所需的最小角度，单位为弧度。
  double required_movement_angle_;

  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
   * @return 参数处理结果；当前实现更新匹配的 required_movement_angle double 参数并接受请求。
   * 中文详细说明：父类的平移半径和允许时间仍由父类回调处理，本回调不会重置当前基准。
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
