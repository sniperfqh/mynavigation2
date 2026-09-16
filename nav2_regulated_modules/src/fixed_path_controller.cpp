#include "nav2_regulated_modules/fixed_path_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

void FixedPathController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node)
  {
    throw nav2_core::PlannerException("FixedPathController cannot lock lifecycle node");
  }
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.52));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_position_tolerance", rclcpp::ParameterValue(0.70));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_lateral_tolerance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_max_yaw_error", rclcpp::ParameterValue(0.2617993877991494));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".initial_yaw_tolerance", rclcpp::ParameterValue(0.12217304763960307));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_position_hysteresis", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".alignment_stable_cycles", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  double controller_frequency = 50.0;
  node->get_parameter(plugin_name_ + ".desired_linear_vel", base_linear_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".start_position_tolerance", start_position_tolerance_);
  node->get_parameter(plugin_name_ + ".direct_tracking_lateral_tolerance", direct_tracking_lateral_tolerance_);
  node->get_parameter(plugin_name_ + ".direct_tracking_max_yaw_error", direct_tracking_max_yaw_error_);
  node->get_parameter(plugin_name_ + ".initial_yaw_tolerance", initial_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".goal_position_hysteresis", goal_position_hysteresis_);
  node->get_parameter(plugin_name_ + ".alignment_stable_cycles", alignment_stable_cycles_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter("controller_frequency", controller_frequency);
  if (base_linear_velocity_ <= 0.0 || lookahead_dist_ <= 0.0 || min_lookahead_dist_ <= 0.0 || max_lookahead_dist_ < min_lookahead_dist_ || start_position_tolerance_ <= 0.0 || direct_tracking_lateral_tolerance_ < 0.0 || direct_tracking_max_yaw_error_ <= 0.0 || direct_tracking_max_yaw_error_ > M_PI_2 || initial_yaw_tolerance_ <= 0.0 || initial_yaw_tolerance_ >= direct_tracking_max_yaw_error_ || rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 || approach_velocity_scaling_dist_ <= 0.0 || goal_position_hysteresis_ < 1.0 || alignment_stable_cycles_ < 1 || transform_tolerance_ < 0.0 || controller_frequency <= 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController parameters are invalid");
  }
  speed_limit_ = base_linear_velocity_;
  control_duration_ = 1.0 / controller_frequency;
  LOG_INFO("固定路径控制器配置完成，plugin={}，最大线速度={:.3f}m/s，起点位置容差={:.3f}m，直接跟踪横向容差={:.3f}m，直接跟踪航向门限={:.3f}rad，严格对齐航向容差={:.3f}rad", plugin_name_, base_linear_velocity_, start_position_tolerance_, direct_tracking_lateral_tolerance_, direct_tracking_max_yaw_error_, initial_yaw_tolerance_);
}

void FixedPathController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
  nearest_index_ = 0;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
}

void FixedPathController::activate()
{
  LOG_INFO("固定路径控制器已激活，plugin={}", plugin_name_);
}

void FixedPathController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::SETTLE;
  stable_cycles_ = 0;
  LOG_INFO("固定路径控制器已停用，plugin={}", plugin_name_);
}

void FixedPathController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path.poses.size() < 2 || path.header.frame_id.empty())
  {
    throw nav2_core::PlannerException("Fixed path requires a frame and at least two poses");
  }
  std::size_t tangent_index = 1;
  while (tangent_index < path.poses.size() && poseDistance(path.poses.front(), path.poses[tangent_index]) <= 1e-6)
  {
    ++tangent_index;
  }
  if (tangent_index >= path.poses.size())
  {
    throw nav2_core::PlannerException("Fixed path has no valid tangent");
  }
  std::size_t goal_tangent_index = path.poses.size() - 2;
  while (goal_tangent_index > 0 && poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    --goal_tangent_index;
  }
  if (poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    throw nav2_core::PlannerException("Fixed path has no valid goal tangent");
  }
  const double start_path_yaw = std::atan2(path.poses[tangent_index].pose.position.y - path.poses.front().pose.position.y, path.poses[tangent_index].pose.position.x - path.poses.front().pose.position.x);
  const double goal_path_yaw = std::atan2(path.poses.back().pose.position.y - path.poses[goal_tangent_index].pose.position.y, path.poses.back().pose.position.x - path.poses[goal_tangent_index].pose.position.x);
  const double direction_cosine = std::cos(normalizeAngle(poseYaw(path.poses.front()) - start_path_yaw));
  if (!std::isfinite(direction_cosine) || std::abs(direction_cosine) < 0.5)
  {
    throw nav2_core::PlannerException("Fixed path orientation does not encode a clear direction");
  }
  global_plan_ = path;
  nearest_index_ = 0;
  direction_sign_ = direction_cosine > 0.0 ? 1 : -1;
  start_path_yaw_ = start_path_yaw;
  goal_path_yaw_ = goal_path_yaw;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  phase_ = Phase::ALIGN_START;
  LOG_INFO("固定路径已装载，frame={}，路径点数={}，方向={}，起点切线={:.6f}rad，终点切线={:.6f}rad", global_plan_.header.frame_id, global_plan_.poses.size(), direction_sign_ > 0 ? "forward" : "backward", start_path_yaw_, goal_path_yaw_);
}

geometry_msgs::msg::TwistStamped FixedPathController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (global_plan_.poses.size() < 2 || !goal_checker)
  {
    throw nav2_core::PlannerException("Fixed path controller has no valid plan or goal checker");
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into fixed path frame");
  }
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance))
  {
    throw nav2_core::PlannerException("FixedPathController failed to read StoppedGoalChecker tolerances");
  }
  const double goal_xy_tolerance = pose_tolerance.position.x;
  const double goal_yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
  const double linear_stopped_velocity = velocity_tolerance.linear.x;
  const double angular_stopped_velocity = velocity_tolerance.angular.z;
  if (!std::isfinite(goal_xy_tolerance) || !std::isfinite(goal_yaw_tolerance) || !std::isfinite(linear_stopped_velocity) || !std::isfinite(angular_stopped_velocity) || goal_xy_tolerance <= 0.0 || goal_yaw_tolerance <= 0.0 || linear_stopped_velocity < 0.0 || angular_stopped_velocity < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController requires valid StoppedGoalChecker pose and velocity tolerances");
  }
  const double start_distance = poseDistance(robot_pose, global_plan_.poses.front());
  if (phase_ == Phase::ALIGN_START)
  {
    if (start_distance > start_position_tolerance_)
    {
      throw nav2_core::PlannerException("Robot is outside fixed path start position tolerance");
    }
    const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
    const double yaw_error = normalizeAngle(start_path_yaw_ - vehicle_motion_yaw);
    const double start_delta_x = robot_pose.pose.position.x - global_plan_.poses.front().pose.position.x;
    const double start_delta_y = robot_pose.pose.position.y - global_plan_.poses.front().pose.position.y;
    const double lateral_error = std::abs(-std::sin(start_path_yaw_) * start_delta_x + std::cos(start_path_yaw_) * start_delta_y);
    if (!start_strategy_evaluated_)
    {
      direct_start_tracking_ = lateral_error <= direct_tracking_lateral_tolerance_;
      start_strategy_evaluated_ = true;
      LOG_INFO("固定路径起点策略已锁定，方向={}，起点距离={:.3f}m，横向误差={:.3f}m，运动方向航向误差={:.3f}rad，策略={}", direction_sign_ > 0 ? "forward" : "backward", start_distance, lateral_error, yaw_error, direct_start_tracking_ ? "direct_tracking" : "strict_alignment");
    }
    if (direct_start_tracking_)
    {
      if (std::abs(yaw_error) < direct_tracking_max_yaw_error_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径小横向误差直接进入跟踪，方向={}，横向误差={:.3f}m，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", lateral_error, yaw_error);
      }
      else
      {
        return rotateCommand(yaw_error, velocity);
      }
    }
    else
    {
      if (std::abs(yaw_error) <= initial_yaw_tolerance_ && std::abs(velocity.linear.x) <= linear_stopped_velocity && std::abs(velocity.angular.z) <= angular_stopped_velocity)
      {
        ++stable_cycles_;
      }
      else
      {
        stable_cycles_ = 0;
      }
      if (stable_cycles_ >= alignment_stable_cycles_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径严格起点航向对齐完成，方向={}，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", yaw_error);
        return zeroCommand();
      }
      return rotateCommand(yaw_error, velocity);
    }
  }
  const double goal_distance = poseDistance(robot_pose, global_plan_.poses.back());
  if ((phase_ == Phase::ALIGN_GOAL || phase_ == Phase::SETTLE) && goal_distance > goal_xy_tolerance * goal_position_hysteresis_)
  {
    phase_ = Phase::TRACK_PATH;
    stable_cycles_ = 0;
  }
  if (phase_ == Phase::TRACK_PATH && goal_distance <= goal_xy_tolerance)
  {
    phase_ = Phase::ALIGN_GOAL;
    stable_cycles_ = 0;
    LOG_INFO("固定路径进入终点航向对齐，位置误差={:.4f}m", goal_distance);
  }
  if (phase_ == Phase::ALIGN_GOAL)
  {
    const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
    const double yaw_error = normalizeAngle(goal_path_yaw_ - vehicle_motion_yaw);
    if (std::abs(yaw_error) <= goal_yaw_tolerance && std::abs(velocity.linear.x) <= linear_stopped_velocity && std::abs(velocity.angular.z) <= angular_stopped_velocity)
    {
      ++stable_cycles_;
    }
    else
    {
      stable_cycles_ = 0;
    }
    if (stable_cycles_ >= alignment_stable_cycles_)
    {
      phase_ = Phase::SETTLE;
      LOG_INFO("固定路径终点姿态对齐完成，方向={}", direction_sign_ > 0 ? "forward" : "backward");
      return zeroCommand();
    }
    return rotateCommand(yaw_error, velocity);
  }
  if (phase_ == Phase::SETTLE)
  {
    return zeroCommand();
  }
  nearest_index_ = findNearestIndex(robot_pose);
  const double lookahead_distance = std::clamp(std::max(lookahead_dist_, std::abs(velocity.linear.x) * lookahead_time_), min_lookahead_dist_, max_lookahead_dist_);
  auto carrot = selectCarrot(nearest_index_, lookahead_distance);
  carrot.header.frame_id = global_plan_.header.frame_id;
  carrot.header.stamp = pose.header.stamp;
  geometry_msgs::msg::PoseStamped local_carrot;
  if (!transformPose(costmap_ros_->getBaseFrameID(), carrot, local_carrot))
  {
    throw nav2_core::PlannerException("Unable to transform fixed path lookahead point into robot frame");
  }
  const double carrot_distance_squared = local_carrot.pose.position.x * local_carrot.pose.position.x + local_carrot.pose.position.y * local_carrot.pose.position.y;
  const double curvature = carrot_distance_squared > 1e-6 ? 2.0 * local_carrot.pose.position.y / carrot_distance_squared : 0.0;
  const double remaining = remainingDistance(nearest_index_, robot_pose);
  double linear_magnitude = std::min(base_linear_velocity_, speed_limit_);
  const double approach_scale = std::clamp(remaining / approach_velocity_scaling_dist_, 0.0, 1.0);
  linear_magnitude = std::min(linear_magnitude, std::max(min_approach_linear_velocity_, base_linear_velocity_ * approach_scale));
  if (std::abs(curvature) > 1e-6)
  {
    linear_magnitude = std::min(linear_magnitude, rotate_to_heading_angular_vel_ / std::abs(curvature));
  }
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  command.twist.linear.x = static_cast<double>(direction_sign_) * linear_magnitude;
  command.twist.angular.z = std::clamp(command.twist.linear.x * curvature, -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);
  return command;
}

void FixedPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
  {
    speed_limit_ = base_linear_velocity_;
    return;
  }
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
  {
    LOG_WARN("忽略非法固定路径限速：{}", speed_limit);
    return;
  }
  speed_limit_ = percentage ? base_linear_velocity_ * std::clamp(speed_limit / 100.0, 0.0, 1.0) : std::min(base_linear_velocity_, speed_limit);
}

bool FixedPathController::transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id == frame)
  {
    output = input;
    return true;
  }
  try
  {
    tf_->transform(input, output, frame, tf2::durationFromSec(transform_tolerance_));
    output.header.frame_id = frame;
    return true;
  }
  catch (const tf2::TransformException & error)
  {
    RCLCPP_ERROR(logger_, "Fixed path transform failed: %s", error.what());
    return false;
  }
}

std::size_t FixedPathController::findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::size_t best_index = nearest_index_;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t index = nearest_index_; index < global_plan_.poses.size(); ++index)
  {
    const double distance = poseDistance(robot_pose, global_plan_.poses[index]);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

double FixedPathController::remainingDistance(const std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  double distance = poseDistance(robot_pose, global_plan_.poses[start_index]);
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    distance += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
  }
  return distance;
}

geometry_msgs::msg::PoseStamped FixedPathController::selectCarrot(const std::size_t start_index, const double lookahead_distance) const
{
  double accumulated = 0.0;
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    accumulated += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
    if (accumulated >= lookahead_distance)
    {
      return global_plan_.poses[index];
    }
  }
  return global_plan_.poses.back();
}

geometry_msgs::msg::TwistStamped FixedPathController::zeroCommand() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  return command;
}

geometry_msgs::msg::TwistStamped FixedPathController::rotateCommand(const double yaw_error, const geometry_msgs::msg::Twist & velocity) const
{
  auto command = zeroCommand();
  if (std::abs(yaw_error) <= 1e-6)
  {
    return command;
  }
  const double direction = yaw_error > 0.0 ? 1.0 : -1.0;
  const double stopping_velocity = std::sqrt(2.0 * max_angular_accel_ * std::abs(yaw_error));
  const double target_velocity = direction * std::min(rotate_to_heading_angular_vel_, stopping_velocity);
  command.twist.angular.z = std::clamp(target_velocity, velocity.angular.z - max_angular_accel_ * control_duration_, velocity.angular.z + max_angular_accel_ * control_duration_);
  return command;
}

double FixedPathController::normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double FixedPathController::poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

double FixedPathController::poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::FixedPathController, nav2_core::Controller)
