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

#ifndef NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

namespace nav2_controller
{

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that checks the position difference and velocity
  * 中文：检查位置偏差和速度的 GoalChecker 插件。
 *
 * 中文详细说明：该类继承 SimpleGoalChecker，先复用父类的 XY、Yaw、stateful 和对称朝向条件，
 * 再要求平面线速度模长与 Z 轴角速度绝对值不超过配置阈值。这样可以避免机器人高速穿过目标容差区
 * 时被提前判定成功。
 */
class StoppedGoalChecker : public SimpleGoalChecker
{
public:
  /**
   * 中文说明：构造父类位姿条件，并设置默认平移、旋转停稳阈值。
   */
  StoppedGoalChecker();
  // Standard GoalChecker Interface
  // 中文：标准 GoalChecker 接口。
  /**
   * 中文说明：先初始化父类位姿参数，再声明并读取 rot_stopped_velocity 与
   * trans_stopped_velocity，最后注册本类动态参数回调。
   * @param parent Controller Server 的 LifecycleNode 弱指针。
   * @param plugin_name 插件实例名及参数命名空间前缀。
   * @param costmap_ros 传递给父类的 Costmap 共享指针；当前父类实现不读取 Costmap。
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * 中文说明：执行“父类位姿条件＋本类停稳速度条件”的两阶段目标判定。
   * @param query_pose 机器人当前位姿。
   * @param goal_pose 目标位姿。
   * @param velocity 当前 Twist；使用 linear.x／linear.y 的平面模长和 angular.z。
   * @return 位姿条件、平移停稳条件和旋转停稳条件全部满足时返回 true。
   */
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;

  /**
   * 中文说明：先取得父类的位姿容差，再把平移和旋转停稳阈值写入速度容差字段。
   * @param pose_tolerance 输出父类的 XY 与 Yaw 容差。
   * @param vel_tolerance 输出 linear.x／linear.y 与 angular.z 停稳阈值，其他字段标记为无效。
   * @return 父类容差查询成功时返回 true。
   */
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // 中文：Z 轴角速度停稳阈值和平面线速度模长停稳阈值，单位分别为 rad/s 与 m/s。
  double rot_stopped_velocity_, trans_stopped_velocity_;
  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // 中文：插件实例名，用于识别属于本插件的速度阈值参数。
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
   * @return 参数更新结果；当前实现更新匹配的停稳速度 double 参数并接受请求。
   * 中文详细说明：父类位姿参数由父类回调处理，本回调只处理 rot_stopped_velocity 和
   * trans_stopped_velocity，更新后从下一次目标检查起生效。
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_
