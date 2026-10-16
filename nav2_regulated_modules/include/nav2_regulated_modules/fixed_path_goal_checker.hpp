// 固定路径 GoalChecker：结合末段几何、终点锁存及连续停稳周期判断 FollowPath 完成。
// 终点航向不参与本检查器成功条件；位置状态与控制器外部独立精度验收仍需区分。

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
  // 默认构造仅使用成员初值；容差与连续周期在 initialize 读取，Path 在 setPath 单独提供。
  FixedPathGoalChecker() = default;
  // 从插件参数读取位置／速度阈值及连续周期；位置容差须为有限正数，停稳速度须有限非负，周期至少为 1。
  // position_stable_cycles 计的是连续控制检查次数，不是固定秒数。
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 丢弃旧路径、末段切线、终点锁存和停稳计数，新任务不能继承这些历史。
  void reset() override;
  // 从最后一个有效非重合线段提取归一化切线，并清空先前到点状态。
  // 重复末点会向前回退；全路径没有有效末段切线时拒绝。
  void setPath(const nav_msgs::msg::Path & path) override;
  // 位置在容差内，或已经靠近末段并越过终点切平面时，单向锁存 terminal_reached。
  // 未检查航向与速度，返回 true 仍不等于最终任务完成。
  bool isTerminalPositionReached(const geometry_msgs::msg::Pose & query_pose) override;
  // 先更新终点几何锁存，再在平面线速度及角速度都低于门限时累计连续周期。
  // 任何运动周期清零停稳计数；满足配置周期才返回 true。
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;
  // 通过 Nav2 接口返回 XY 半径与停止速度阈值，未使用的维度写入约定占位。
  // 输出是供控制器参考的阈值，不是当前机器人误差。
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & velocity_tolerance) override;

private:
  // 在整条保存路径中搜索离 query_pose 最近的点，用于末段越界判据。
  // 与控制器只向前搜索的最近点函数不同，此处不维护递增进度。
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
