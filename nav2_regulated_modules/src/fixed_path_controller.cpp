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
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(1.5));
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
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_kp", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.005));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_linear_deceleration", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_final_approach_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_reaction_time", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_distance_margin", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_dist", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_reference_speed", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_speed_gain", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_min_dist", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_max_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_error_log_frequency", rclcpp::ParameterValue(1.0));
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
  node->get_parameter(plugin_name_ + ".rotate_to_heading_kp", rotate_to_heading_kp_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".goal_linear_deceleration", goal_linear_deceleration_);
  node->get_parameter(plugin_name_ + ".goal_final_approach_velocity", goal_final_approach_velocity_);
  node->get_parameter(plugin_name_ + ".goal_braking_reaction_time", goal_braking_reaction_time_);
  node->get_parameter(plugin_name_ + ".goal_braking_distance_margin", goal_braking_distance_margin_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_dist", goal_terminal_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_reference_speed", goal_terminal_lookahead_reference_speed_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_speed_gain", goal_terminal_lookahead_speed_gain_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_min_dist", goal_terminal_lookahead_min_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_max_dist", goal_terminal_lookahead_max_dist_);
  node->get_parameter(plugin_name_ + ".goal_error_log_frequency", goal_error_log_frequency_);
  node->get_parameter(plugin_name_ + ".alignment_stable_cycles", alignment_stable_cycles_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter("controller_frequency", controller_frequency);
  if (base_linear_velocity_ <= 0.0 || lookahead_dist_ <= 0.0 || min_lookahead_dist_ <= 0.0 || max_lookahead_dist_ < min_lookahead_dist_ || start_position_tolerance_ <= 0.0 || direct_tracking_lateral_tolerance_ < 0.0 || direct_tracking_max_yaw_error_ <= 0.0 || direct_tracking_max_yaw_error_ > M_PI_2 || initial_yaw_tolerance_ <= 0.0 || initial_yaw_tolerance_ >= direct_tracking_max_yaw_error_ || rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 || rotate_to_heading_kp_ <= 0.0 || min_approach_linear_velocity_ <= 0.0 || min_approach_linear_velocity_ > base_linear_velocity_ || approach_velocity_scaling_dist_ <= 0.0 || !std::isfinite(goal_linear_deceleration_) || goal_linear_deceleration_ <= 0.0 || !std::isfinite(goal_final_approach_velocity_) || goal_final_approach_velocity_ < 0.0 || !std::isfinite(goal_braking_reaction_time_) || goal_braking_reaction_time_ < 0.0 || !std::isfinite(goal_braking_distance_margin_) || goal_braking_distance_margin_ < 0.0 || !std::isfinite(goal_terminal_lookahead_dist_) || goal_terminal_lookahead_dist_ <= 0.0 || goal_terminal_lookahead_dist_ > lookahead_dist_ || !std::isfinite(goal_terminal_lookahead_reference_speed_) || goal_terminal_lookahead_reference_speed_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_speed_gain_) || goal_terminal_lookahead_speed_gain_ < 0.0 || !std::isfinite(goal_terminal_lookahead_min_dist_) || goal_terminal_lookahead_min_dist_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_max_dist_) || goal_terminal_lookahead_max_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ > goal_terminal_lookahead_max_dist_ || goal_error_log_frequency_ <= 0.0 || alignment_stable_cycles_ < 1 || transform_tolerance_ < 0.0 || controller_frequency <= 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController parameters are invalid");
  }
  speed_limit_ = base_linear_velocity_;
  control_duration_ = 1.0 / controller_frequency;
  LOG_INFO("固定路径控制器配置完成，plugin={}，最大线速度={:.3f}m/s，终点减速度={:.3f}m/s^2，终点微量接近速度={:.3f}m/s，制动反应时间={:.3f}s，制动距离裕量={:.3f}m，终点基础前视={:.3f}m，前视参考速度={:.3f}m/s，前视速度增益={:.3f}s，前视范围=[{:.3f},{:.3f}]m，起点位置容差={:.3f}m，直接跟踪横向容差={:.3f}m，直接跟踪航向门限={:.3f}rad，严格对齐航向容差={:.3f}rad，终点按位置或越界锁存停车", plugin_name_, base_linear_velocity_, goal_linear_deceleration_, goal_final_approach_velocity_, goal_braking_reaction_time_, goal_braking_distance_margin_, goal_terminal_lookahead_dist_, goal_terminal_lookahead_reference_speed_, goal_terminal_lookahead_speed_gain_, goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_, start_position_tolerance_, direct_tracking_lateral_tolerance_, direct_tracking_max_yaw_error_, initial_yaw_tolerance_);
}

void FixedPathController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
  nearest_index_ = 0;
  goal_tangent_index_ = 0;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  terminal_tangent_x_ = 0.0;
  terminal_tangent_y_ = 0.0;
  error_log_initialized_ = false;
}

void FixedPathController::activate()
{
  LOG_INFO("固定路径控制器已激活，plugin={}", plugin_name_);
}

void FixedPathController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::STOPPED;
  terminal_stop_latched_ = true;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
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
  const double goal_tangent_x = path.poses.back().pose.position.x - path.poses[goal_tangent_index].pose.position.x;
  const double goal_tangent_y = path.poses.back().pose.position.y - path.poses[goal_tangent_index].pose.position.y;
  const double goal_tangent_length = std::hypot(goal_tangent_x, goal_tangent_y);
  const double goal_path_yaw = std::atan2(goal_tangent_y, goal_tangent_x);
  const double direction_cosine = std::cos(normalizeAngle(poseYaw(path.poses.front()) - start_path_yaw));
  if (!std::isfinite(direction_cosine) || std::abs(direction_cosine) < 0.5)
  {
    throw nav2_core::PlannerException("Fixed path orientation does not encode a clear direction");
  }
  global_plan_ = path;
  nearest_index_ = 0;
  goal_tangent_index_ = goal_tangent_index;
  direction_sign_ = direction_cosine > 0.0 ? 1 : -1;
  start_path_yaw_ = start_path_yaw;
  goal_path_yaw_ = goal_path_yaw;
  terminal_tangent_x_ = goal_tangent_x / goal_tangent_length;
  terminal_tangent_y_ = goal_tangent_y / goal_tangent_length;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  phase_ = Phase::ALIGN_START;
  error_log_initialized_ = false;
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
  const double linear_stopped_velocity = velocity_tolerance.linear.x;
  const double angular_stopped_velocity = velocity_tolerance.angular.z;
  if (!std::isfinite(goal_xy_tolerance) || !std::isfinite(linear_stopped_velocity) || !std::isfinite(angular_stopped_velocity) || goal_xy_tolerance <= 0.0 || linear_stopped_velocity < 0.0 || angular_stopped_velocity < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController requires valid position and stopped-velocity tolerances");
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
      if (std::abs(yaw_error) <= direct_tracking_max_yaw_error_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径小横向误差直接进入跟踪，方向={}，横向误差={:.3f}m，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", lateral_error, yaw_error);
      }
      else
      {
        return rotateCommand(yaw_error, direct_tracking_max_yaw_error_, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
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
      return rotateCommand(yaw_error, initial_yaw_tolerance_, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
    }
  }
  const double goal_distance = poseDistance(robot_pose, global_plan_.poses.back());
  const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
  const double goal_yaw_error = normalizeAngle(goal_path_yaw_ - vehicle_motion_yaw);
  nearest_index_ = findNearestIndex(robot_pose);
  const auto & goal_position = global_plan_.poses.back().pose.position;
  const double goal_delta_x = robot_pose.pose.position.x - goal_position.x;
  const double goal_delta_y = robot_pose.pose.position.y - goal_position.y;
  const double terminal_projection = goal_delta_x * terminal_tangent_x_ + goal_delta_y * terminal_tangent_y_;
  const bool goal_plane_crossed = nearest_index_ >= goal_tangent_index_ && terminal_projection >= 0.0;
  const double remaining = remainingDistance(nearest_index_, robot_pose);
  const double effective_remaining = std::max(remaining - goal_xy_tolerance, 0.0);
  const double current_linear_velocity = std::max(0.0, static_cast<double>(direction_sign_) * velocity.linear.x);
  const double stopping_distance = current_linear_velocity * current_linear_velocity / (2.0 * goal_linear_deceleration_);
  const double expected_linear_velocity = std::max(0.0, std::min(base_linear_velocity_, speed_limit_));
  const double dynamic_terminal_lookahead_distance = std::clamp(goal_terminal_lookahead_dist_ + goal_terminal_lookahead_speed_gain_ * (expected_linear_velocity - goal_terminal_lookahead_reference_speed_), goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_);
  const bool terminal_condition_reached = goal_distance <= goal_xy_tolerance || goal_plane_crossed;
  if (!terminal_stop_latched_ && terminal_condition_reached)
  {
    terminal_stop_latched_ = true;
    terminal_position_accurate_ = goal_distance <= goal_xy_tolerance;
    terminal_tracking_yaw_error_ = goal_yaw_error;
    phase_ = Phase::STOPPED;
    LOG_INFO("固定路径终点停车已锁存，原因={}，位置误差={:.4f}m，终点纵向投影={:.4f}m，锁存跟踪航向误差={:.3f}rad（{:.2f}deg），停车后不再旋转", goal_distance <= goal_xy_tolerance ? "within_tolerance" : "goal_plane_crossed", goal_distance, terminal_projection, terminal_tracking_yaw_error_, terminal_tracking_yaw_error_ * 180.0 / M_PI);
  }
  else if (terminal_stop_latched_ && !terminal_position_accurate_ && goal_distance <= goal_xy_tolerance)
  {
    terminal_position_accurate_ = true;
    LOG_INFO("固定路径停车后位置精度已进入容差并单向锁存，位置误差={:.4f}m，位置容差={:.4f}m", goal_distance, goal_xy_tolerance);
  }
  if (terminal_stop_latched_)
  {
    logGoalErrors(goal_distance, terminal_tracking_yaw_error_, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, 0.0, stopping_distance, dynamic_terminal_lookahead_distance);
    return zeroCommand();
  }
  const double nominal_lookahead_distance = std::clamp(std::max(lookahead_dist_, std::abs(velocity.linear.x) * lookahead_time_), min_lookahead_dist_, max_lookahead_dist_);
  const double lookahead_distance = std::min(nominal_lookahead_distance, std::max(dynamic_terminal_lookahead_distance, goal_distance));
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
  double linear_magnitude = std::min(base_linear_velocity_, speed_limit_);
  if (std::abs(curvature) > 1e-6)
  {
    linear_magnitude = std::min(linear_magnitude, rotate_to_heading_angular_vel_ / std::abs(curvature));
  }
  const double braking_reaction_distance = current_linear_velocity * goal_braking_reaction_time_ + goal_braking_distance_margin_;
  const double braking_activation_distance = std::max(approach_velocity_scaling_dist_, stopping_distance + braking_reaction_distance);
  if (!goal_braking_active_ && effective_remaining <= braking_activation_distance)
  {
    goal_braking_active_ = true;
    last_braking_command_magnitude_ = linear_magnitude;
    LOG_INFO("固定路径进入终点平滑制动，剩余距离={:.4f}m，当前速度={:.4f}m/s，制动距离={:.4f}m，目标减速度={:.3f}m/s^2", remaining, current_linear_velocity, stopping_distance, goal_linear_deceleration_);
  }
  if (goal_braking_active_)
  {
    const double braking_distance_remaining = std::max(effective_remaining - braking_reaction_distance, 0.0);
    double braking_target = std::min(linear_magnitude, std::sqrt(2.0 * goal_linear_deceleration_ * braking_distance_remaining));
    if (goal_distance > goal_xy_tolerance)
    {
      braking_target = std::min(linear_magnitude, std::max(braking_target, goal_final_approach_velocity_));
    }
    linear_magnitude = std::min(last_braking_command_magnitude_, braking_target);
    last_braking_command_magnitude_ = linear_magnitude;
  }
  logGoalErrors(goal_distance, goal_yaw_error, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, linear_magnitude, stopping_distance, dynamic_terminal_lookahead_distance);
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

bool FixedPathController::isTerminalStopLatched()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_stop_latched_;
}

bool FixedPathController::isTerminalPositionAccurate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_position_accurate_;
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
  auto carrot = global_plan_.poses.back();
  const double extension_distance = std::max(lookahead_distance - accumulated, 0.0);
  carrot.pose.position.x += terminal_tangent_x_ * extension_distance;
  carrot.pose.position.y += terminal_tangent_y_ * extension_distance;
  return carrot;
}

geometry_msgs::msg::TwistStamped FixedPathController::zeroCommand() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  return command;
}

geometry_msgs::msg::TwistStamped FixedPathController::rotateCommand(const double yaw_error, const double yaw_tolerance, const double max_angular_velocity, const double max_angular_accel, const double angular_kp, const geometry_msgs::msg::Twist & velocity) const
{
  auto command = zeroCommand();
  const double remaining_error = std::abs(yaw_error) - yaw_tolerance;
  if (remaining_error <= 0.0)
  {
    return command;
  }
  const double direction = yaw_error > 0.0 ? 1.0 : -1.0;
  const double proportional_velocity = angular_kp * remaining_error;
  const double stopping_velocity = std::sqrt(2.0 * max_angular_accel * remaining_error);
  const double target_velocity = direction * std::min({max_angular_velocity, proportional_velocity, stopping_velocity});
  command.twist.angular.z = std::clamp(target_velocity, velocity.angular.z - max_angular_accel * control_duration_, velocity.angular.z + max_angular_accel * control_duration_);
  return command;
}

void FixedPathController::logGoalErrors(const double position_error, const double yaw_error, const double goal_xy_tolerance, const double terminal_projection, const bool goal_plane_crossed, const double remaining_distance, const double current_linear_velocity, const double target_linear_velocity, const double stopping_distance, const double terminal_lookahead_distance)
{
  const auto now = clock_->now();
  const double log_period = 1.0 / goal_error_log_frequency_;
  if (error_log_initialized_ && now.nanoseconds() >= last_error_log_time_.nanoseconds() && (now - last_error_log_time_).seconds() < log_period)
  {
    return;
  }
  last_error_log_time_ = now;
  error_log_initialized_ = true;
  LOG_INFO("固定路径终点误差：phase={}，位置误差={:.4f}m，路径剩余={:.4f}m，位置容差={:.4f}m，当前速度={:.4f}m/s，目标速度={:.4f}m/s，终点前视={:.4f}m，制动距离={:.4f}m，平滑制动={}，终点纵向投影={:.4f}m，越界={}，停车锁存={}，位置精度锁存={}，跟踪航向误差仅供诊断={:.3f}rad（{:.2f}deg）", phaseName(phase_), position_error, remaining_distance, goal_xy_tolerance, current_linear_velocity, target_linear_velocity, terminal_lookahead_distance, stopping_distance, goal_braking_active_, terminal_projection, goal_plane_crossed, terminal_stop_latched_, terminal_position_accurate_, yaw_error, yaw_error * 180.0 / M_PI);
}

const char * FixedPathController::phaseName(const Phase phase)
{
  switch (phase)
  {
    case Phase::ALIGN_START:
      return "ALIGN_START";
    case Phase::TRACK_PATH:
      return "TRACK_PATH";
    case Phase::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
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
