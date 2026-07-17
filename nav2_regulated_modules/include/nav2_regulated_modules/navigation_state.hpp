#ifndef NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_regulated_modules
{

// 中文注释：显式状态机替代行为树，保证取消、重规划和恢复只有一个确定状态。
enum class NavigationState
{
  IDLE,
  PLANNING,
  SMOOTHING,
  CONTROLLING,
  REPLANNING,
  // 中文注释：不再依赖 nav2_behaviors，恢复阶段只清理代价地图并等待重新规划。
  CLEARING_COSTMAP,
  LOCALIZATION_LOST,
  CANCELING,
  SUCCEEDED,
  FAILED
};

enum class TaskType
{
  NONE,
  TO_POSE,
  THROUGH_POSES,
  TOPIC_GOAL
};

// 中文注释：generation 用于隔离旧任务的异步回调，防止旧路径覆盖新目标。
struct NavigationTask
{
  uint64_t generation{0};
  TaskType type{TaskType::NONE};
  NavigationState state{NavigationState::IDLE};
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  nav_msgs::msg::Path active_path;
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time{0, 0, RCL_ROS_TIME};
  // 中文注释：记录真实位姿进展，避免仅依据控制器速度误判车辆已经移动。
  rclcpp::Time last_progress_time{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_progress_pose;
  int recovery_count{0};
  int consecutive_planning_failures{0};
  float distance_remaining{0.0F};
  std::string last_error;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
