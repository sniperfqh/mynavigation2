// 固定路径目标检查器接口。以位置到达和控制器终点锁存状态判定任务完成，不要求最终航向对齐。

#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_GOAL_CHECKER_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_GOAL_CHECKER_HPP_

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/path_aware_goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_regulated_modules
{

class FixedPathGoalChecker : public nav2_core::GoalChecker, public nav2_core::PathAwareGoalChecker
{
public:
// 构造目标检查器；容差与稳定周期在 initialize 阶段从 ROS 参数读取。
  FixedPathGoalChecker() = default;
  // 读取目标位置、停车速度和稳定周期阈值，并绑定目标检查器运行上下文。
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 清空控制或检查器历史状态，避免跨任务沿用上次进度。
  void reset() override;
// 设置当前固定路径终点，并清除前一条路径的到点状态。
  void setPath(const nav_msgs::msg::Path & path) override;
  // 判断机器人是否到达终点位置容差或越过末段终点平面。
  bool isTerminalPositionReached(const geometry_msgs::msg::Pose & query_pose) override;
  // 综合位置与停车状态判断固定路径是否完成。
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;
  // 向 Nav2 目标检查接口提供当前距离、速度容差。
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & velocity_tolerance) override;

private:
  // 从当前路径搜索区间选出距机器人最近的轨迹点索引。
  std::size_t findNearestIndex(const geometry_msgs::msg::Pose & query_pose) const;

  std::string plugin_name_;
  nav_msgs::msg::Path path_;
  std::size_t goal_tangent_index_ { 0 };
  double terminal_tangent_x_ { 0.0 };
  double terminal_tangent_y_ { 0.0 };
  double xy_goal_tolerance_ { 0.01 };
  double trans_stopped_velocity_ { 0.01 };
  double rot_stopped_velocity_ { 0.05 };
  int position_stable_cycles_ { 10 };
  int stopped_cycles_ { 0 };
  bool path_valid_ { false };
  bool terminal_reached_ { false };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_GOAL_CHECKER_HPP_
