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
  FixedPathGoalChecker() = default;
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  void setPath(const nav_msgs::msg::Path & path) override;
  bool isTerminalPositionReached(const geometry_msgs::msg::Pose & query_pose) override;
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & velocity_tolerance) override;

private:
  std::size_t findNearestIndex(const geometry_msgs::msg::Pose & query_pose) const;

  std::string plugin_name_;
  nav_msgs::msg::Path path_;
  std::size_t goal_tangent_index_ { 0 };
  double terminal_tangent_x_ { 0.0 };
  double terminal_tangent_y_ { 0.0 };
  double xy_goal_tolerance_ { 0.01 };
  double trans_stopped_velocity_ { 0.01 };
  double rot_stopped_velocity_ { 0.05 };
  int position_stable_cycles_ { 5 };
  int stopped_cycles_ { 0 };
  bool path_valid_ { false };
  bool terminal_reached_ { false };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_GOAL_CHECKER_HPP_
