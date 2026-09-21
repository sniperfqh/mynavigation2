#include "nav2_regulated_modules/fixed_path_goal_checker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

void FixedPathGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::invalid_argument("FixedPathGoalChecker cannot lock lifecycle node");
  }
  plugin_name_ = plugin_name;
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".trans_stopped_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rot_stopped_velocity", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".position_stable_cycles", rclcpp::ParameterValue(5));
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name_ + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name_ + ".position_stable_cycles", position_stable_cycles_);
  if (!std::isfinite(xy_goal_tolerance_) || !std::isfinite(trans_stopped_velocity_) || !std::isfinite(rot_stopped_velocity_) || xy_goal_tolerance_ <= 0.0 || trans_stopped_velocity_ < 0.0 || rot_stopped_velocity_ < 0.0 || position_stable_cycles_ < 1)
  {
    throw std::invalid_argument("FixedPathGoalChecker parameters are invalid");
  }
  LOG_INFO("固定路径位置 GoalChecker 配置完成，plugin={}，位置容差={:.4f}m，停止线速度={:.3f}m/s，停止角速度={:.3f}rad/s，位置稳定周期={}", plugin_name_, xy_goal_tolerance_, trans_stopped_velocity_, rot_stopped_velocity_, position_stable_cycles_);
}

void FixedPathGoalChecker::reset()
{
  path_ = nav_msgs::msg::Path();
  goal_tangent_index_ = 0;
  terminal_tangent_x_ = 0.0;
  terminal_tangent_y_ = 0.0;
  path_valid_ = false;
  terminal_reached_ = false;
  stopped_cycles_ = 0;
}

void FixedPathGoalChecker::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2)
  {
    throw std::invalid_argument("FixedPathGoalChecker requires at least two path poses");
  }
  std::size_t goal_tangent_index = path.poses.size() - 2;
  const auto & goal = path.poses.back().pose.position;
  while (goal_tangent_index > 0)
  {
    const auto & candidate = path.poses[goal_tangent_index].pose.position;
    if (std::hypot(goal.x - candidate.x, goal.y - candidate.y) > 1e-6)
    {
      break;
    }
    --goal_tangent_index;
  }
  const auto & tangent_start = path.poses[goal_tangent_index].pose.position;
  const double tangent_x = goal.x - tangent_start.x;
  const double tangent_y = goal.y - tangent_start.y;
  const double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length <= 1e-6)
  {
    throw std::invalid_argument("FixedPathGoalChecker path has no valid terminal tangent");
  }
  path_ = path;
  goal_tangent_index_ = goal_tangent_index;
  terminal_tangent_x_ = tangent_x / tangent_length;
  terminal_tangent_y_ = tangent_y / tangent_length;
  path_valid_ = true;
  terminal_reached_ = false;
  stopped_cycles_ = 0;
}

bool FixedPathGoalChecker::isTerminalPositionReached(const geometry_msgs::msg::Pose & query_pose)
{
  if (!path_valid_)
  {
    return false;
  }
  const auto & goal = path_.poses.back().pose.position;
  const double delta_x = query_pose.position.x - goal.x;
  const double delta_y = query_pose.position.y - goal.y;
  const double goal_distance = std::hypot(delta_x, delta_y);
  const double terminal_projection = delta_x * terminal_tangent_x_ + delta_y * terminal_tangent_y_;
  const bool goal_plane_crossed = findNearestIndex(query_pose) >= goal_tangent_index_ && terminal_projection >= 0.0;
  if (goal_distance <= xy_goal_tolerance_ || goal_plane_crossed)
  {
    terminal_reached_ = true;
  }
  return terminal_reached_;
}

bool FixedPathGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist & velocity)
{
  isTerminalPositionReached(query_pose);
  if (!terminal_reached_)
  {
    stopped_cycles_ = 0;
    return false;
  }
  if (std::hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_ && std::abs(velocity.angular.z) <= rot_stopped_velocity_)
  {
    ++stopped_cycles_;
  }
  else
  {
    stopped_cycles_ = 0;
  }
  return stopped_cycles_ >= position_stable_cycles_;
}

bool FixedPathGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & velocity_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;
  velocity_tolerance.linear.x = trans_stopped_velocity_;
  velocity_tolerance.linear.y = trans_stopped_velocity_;
  velocity_tolerance.linear.z = invalid_field;
  velocity_tolerance.angular.x = invalid_field;
  velocity_tolerance.angular.y = invalid_field;
  velocity_tolerance.angular.z = rot_stopped_velocity_;
  return true;
}

std::size_t FixedPathGoalChecker::findNearestIndex(const geometry_msgs::msg::Pose & query_pose) const
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < path_.poses.size(); ++index)
  {
    const auto & position = path_.poses[index].pose.position;
    const double distance = std::hypot(query_pose.position.x - position.x, query_pose.position.y - position.y);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::FixedPathGoalChecker, nav2_core::GoalChecker)
