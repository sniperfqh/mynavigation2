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

enum class NavigationMode
{
  REMOTE,
  AUTONOMOUS,
  FIXED_PATH
};

enum class NavigationState
{
  IDLE,
  PLANNING,
  SMOOTHING,
  CONTROLLING,
  REPLANNING,
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
  TOPIC_GOAL,
  FIXED_PATH
};

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
  rclcpp::Time last_progress_time{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_progress_pose;
  int recovery_count{0};
  int consecutive_planning_failures{0};
  double distance_remaining{0.0};
  std::string last_error;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
