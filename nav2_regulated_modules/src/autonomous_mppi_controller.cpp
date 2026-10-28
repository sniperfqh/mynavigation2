#include "nav2_regulated_modules/autonomous_mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace nav2_regulated_modules
{

void AutonomousMPPIController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RotationShimController::configure(parent, name, tf, costmap_ros);
  const auto node = parent.lock();
  if (node->get_parameter(name + ".primary_controller").as_string() != "nav2_mppi_controller::MPPIController")
  {
    throw std::invalid_argument("AutonomousMPPIController requires MPPI as its primary controller");
  }
  nominal_speed_ = node->get_parameter(name + ".vx_max").as_double();
  nav2_util::declare_parameter_if_not_declared(node, name + ".autonomous_approach_distance", rclcpp::ParameterValue(1.0));
  approach_distance_ = node->get_parameter(name + ".autonomous_approach_distance").as_double();
  nav2_util::declare_parameter_if_not_declared(node, name + ".autonomous_terminal_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name + ".autonomous_terminal_max_linear_velocity", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".autonomous_terminal_max_angular_velocity", rclcpp::ParameterValue(0.3));
  terminal_distance_ = node->get_parameter(name + ".autonomous_terminal_distance").as_double();
  terminal_linear_speed_ = node->get_parameter(name + ".autonomous_terminal_max_linear_velocity").as_double();
  terminal_angular_speed_ = node->get_parameter(name + ".autonomous_terminal_max_angular_velocity").as_double();
  if (!std::isfinite(nominal_speed_) || nominal_speed_ <= 0.0 || !std::isfinite(approach_distance_) || approach_distance_ <= 0.3)
  {
    throw std::invalid_argument("Invalid autonomous speed or approach distance");
  }
  if (!std::isfinite(terminal_distance_) || terminal_distance_ <= 0.0 || terminal_distance_ >= approach_distance_ || !std::isfinite(terminal_linear_speed_) || terminal_linear_speed_ <= 0.0 || !std::isfinite(terminal_angular_speed_) || terminal_angular_speed_ <= 0.0 || !std::isfinite(control_duration_) || control_duration_ <= 0.0 || !std::isfinite(max_angular_accel_) || max_angular_accel_ <= 0.0)
  {
    throw std::invalid_argument("Invalid autonomous terminal settings");
  }
  external_limit_ = 0.0;
  resetTerminalState();
  RCLCPP_INFO(logger_, "Autonomous controller frequency=%.1fHz model_dt=%.3fs terminal_distance=%.3fm", 1.0 / control_duration_, node->get_parameter(name + ".model_dt").as_double(), terminal_distance_);
}

void AutonomousMPPIController::resetTerminalState()
{
  terminal_state_ = detail::AutonomousTerminalState();
  terminal_goal_.reset();
  logged_terminal_phase_.reset();
  terminal_log_time_ = -1.0;
}

void AutonomousMPPIController::activate()
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  RotationShimController::activate();
  resetTerminalState();
}

void AutonomousMPPIController::deactivate()
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  RotationShimController::deactivate();
  resetTerminalState();
}

void AutonomousMPPIController::cleanup()
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  RotationShimController::cleanup();
  resetTerminalState();
}

void AutonomousMPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  if (path.poses.empty())
  {
    resetTerminalState();
  }
  else
  {
    auto goal = path.poses.back();
    goal.header.frame_id = path.header.frame_id;
    const bool changed = !terminal_goal_ || goal.header.frame_id != terminal_goal_->header.frame_id || std::hypot(goal.pose.position.x - terminal_goal_->pose.position.x, goal.pose.position.y - terminal_goal_->pose.position.y) > 1e-4 || std::abs(angles::shortest_angular_distance(tf2::getYaw(terminal_goal_->pose.orientation), tf2::getYaw(goal.pose.orientation))) > 1e-4;
    if (changed)
    {
      resetTerminalState();
    }
    terminal_goal_ = goal;
  }
  RotationShimController::setPlan(path);
}

void AutonomousMPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
  {
    throw std::invalid_argument("Invalid autonomous speed limit");
  }
  external_limit_ = speed_limit;
  external_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped AutonomousMPPIController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(autonomous_mutex_);
  if (!goal_checker)
  {
    throw nav2_core::PlannerException("Autonomous controller requires a goal checker");
  }
  if (current_path_.poses.empty())
  {
    throw nav2_core::PlannerException("Autonomous controller received an empty path");
  }
  auto goal = current_path_.poses.back();
  goal.header.frame_id = current_path_.header.frame_id;
  goal = poseInControlFrame(goal, pose);
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance))
  {
    throw nav2_core::PlannerException("Invalid autonomous goal tolerances");
  }
  // getTolerances stores yaw tolerance as a quaternion, not a scalar angle.
  const double yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
  if (!std::isfinite(pose_tolerance.position.x) || pose_tolerance.position.x <= 0.0 || pose_tolerance.position.x >= terminal_distance_ || !std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0 || !std::isfinite(velocity_tolerance.linear.x) || velocity_tolerance.linear.x < 0.0 || !std::isfinite(velocity_tolerance.angular.z) || velocity_tolerance.angular.z < 0.0 || !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z))
  {
    throw nav2_core::PlannerException("Invalid autonomous position or yaw tolerance");
  }
  const double distance = std::hypot(goal.pose.position.x - pose.pose.position.x, goal.pose.position.y - pose.pose.position.y);
  double external_cap = nominal_speed_;
  if (external_limit_ > 0.0)
  {
    external_cap = std::min(nominal_speed_, external_percentage_ ? nominal_speed_ * external_limit_ / 100.0 : external_limit_);
  }
  const double yaw_error = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), tf2::getYaw(goal.pose.orientation));
  const double bearing_error = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), std::atan2(goal.pose.position.y - pose.pose.position.y, goal.pose.position.x - pose.pose.position.x));
  const double rotation_limit = std::min(terminal_angular_speed_, rotate_to_heading_angular_vel_) * external_cap / nominal_speed_;
  const auto terminal = detail::autonomousTerminalCommand(terminal_state_, distance, bearing_error, yaw_error, velocity.linear.x, velocity.angular.z, pose_tolerance.position.x, yaw_tolerance, velocity_tolerance.linear.x, velocity_tolerance.angular.z, clock_->now().seconds(), terminal_distance_, std::min(terminal_linear_speed_, external_cap), rotation_limit, max_angular_accel_, control_duration_);
  if (terminal_state_.phase != detail::AutonomousTerminalPhase::TRACK)
  {
    auto command = geometry_msgs::msg::TwistStamped();
    command.header = pose.header;
    command.twist.linear.x = terminal.linear;
    command.twist.angular.z = terminal.angular;
    checkTerminalTrajectory(pose, velocity, command.twist);
    logTerminalState(distance, velocity, command.twist);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }
  const double limit = std::min(external_cap, detail::autonomousApproachLimit(distance, pose_tolerance.position.x, nominal_speed_, approach_distance_));
  primary_controller_->setSpeedLimit(limit, false);
  auto command = computeTrackingCommand(pose, velocity, goal_checker);
  // MPPI may reset its constraints after a long rotation; retain the approach bound on that first output too.
  if (!in_rotation_ && std::abs(command.twist.linear.x) > limit)
  {
    const double scale = limit / std::abs(command.twist.linear.x);
    command.twist.linear.x *= scale;
    command.twist.linear.y *= scale;
    command.twist.angular.z *= scale;
  }
  if (in_rotation_)
  {
    const double cap = rotate_to_heading_angular_vel_ * external_cap / nominal_speed_;
    command.twist.angular.z = std::clamp(command.twist.angular.z, -cap, cap);
  }
  logTerminalState(distance, velocity, command.twist);
  return command;
}

geometry_msgs::msg::PoseStamped AutonomousMPPIController::poseInControlFrame(const geometry_msgs::msg::PoseStamped & point, const geometry_msgs::msg::PoseStamped & robot_pose)
{
  auto input = point;
  // 使用 Controller Server 已获取的同一帧位姿，不为静态路径点制造 now() 查询。
  input.header.stamp = robot_pose.header.stamp;
  if (input.header.frame_id == robot_pose.header.frame_id)
  {
    return input;
  }
  try
  {
    // 非默认坐标系也只使用该帧的缓存；不在 20ms 控制周期中等待未来 TF。
    return tf_->transform(input, robot_pose.header.frame_id, tf2::durationFromSec(0.0));
  }
  catch (const tf2::TransformException & error)
  {
    throw nav2_core::PlannerException(std::string("Autonomous coherent pose transform unavailable: ") + error.what());
  }
}

geometry_msgs::msg::TwistStamped AutonomousMPPIController::computeTrackingCommand(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  if (path_updated_)
  {
    std::optional<geometry_msgs::msg::PoseStamped> sampled;
    for (std::size_t index = 1; index < current_path_.poses.size(); ++index)
    {
      const auto & start = current_path_.poses.front().pose.position;
      const auto & point = current_path_.poses[index].pose.position;
      if (std::hypot(point.x - start.x, point.y - start.y) >= forward_sampling_distance_)
      {
        auto sample = current_path_.poses[index];
        sample.header.frame_id = current_path_.header.frame_id;
        sampled = poseInControlFrame(sample, pose);
        break;
      }
    }
    if (sampled)
    {
      const double bearing = std::atan2(sampled->pose.position.y - pose.pose.position.y, sampled->pose.position.x - pose.pose.position.x);
      const double error = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), bearing);
      const double threshold = in_rotation_ ? angular_disengage_threshold_ : angular_dist_threshold_;
      if (std::abs(error) > threshold)
      {
        in_rotation_ = true;
        auto * costmap = costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        try
        {
          auto command = computeRotateToHeadingCommand(error, pose, velocity);
          last_angular_vel_ = command.twist.angular.z;
          return command;
        }
        catch (const std::runtime_error & collision)
        {
          throw nav2_core::PlannerException(collision.what());
        }
      }
    }
    // 不足前视采样距离的短路径沿用原先交给主控制器的行为。
    path_updated_ = false;
  }
  in_rotation_ = false;
  auto command = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  last_angular_vel_ = command.twist.angular.z;
  return command;
}

void AutonomousMPPIController::checkTerminalTrajectory(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command)
{
  if (measured.linear.x == 0.0 && measured.angular.z == 0.0 && command.linear.x == 0.0 && command.angular.z == 0.0)
  {
    return;
  }
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3)
  {
    throw nav2_core::PlannerException("Autonomous terminal footprint unavailable");
  }
  const auto free = [this, &footprint](const double x, const double y, const double heading)
  {
    const double cost = collision_checker_->footprintCostAtPose(x, y, heading, footprint);
    const bool unknown = cost == nav2_costmap_2d::NO_INFORMATION && costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
    return cost >= 0.0 && !unknown && (cost < nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION);
  };
  if (!detail::autonomousTerminalTrajectory(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation), measured.linear.x, measured.angular.z, command.linear.x, command.angular.z, max_angular_accel_, control_duration_, free))
  {
    throw nav2_core::PlannerException("Autonomous terminal motion or braking intersects an obstacle");
  }
}

void AutonomousMPPIController::logTerminalState(const double distance, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command)
{
  const double time = clock_->now().seconds();
  if (!logged_terminal_phase_ || *logged_terminal_phase_ != terminal_state_.phase || time < terminal_log_time_ || time - terminal_log_time_ >= 1.0)
  {
    RCLCPP_INFO(logger_, "Autonomous terminal phase=%s distance=%.4fm command=(%.4f,%.4f) measured=(%.4f,%.4f)", detail::autonomousTerminalPhaseName(terminal_state_.phase), distance, command.linear.x, command.angular.z, measured.linear.x, measured.angular.z);
    logged_terminal_phase_ = terminal_state_.phase;
    terminal_log_time_ = time;
  }
}

void AutonomousStoppedGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  StoppedGoalChecker::initialize(parent, plugin_name, costmap_ros);
  const auto node = parent.lock();
  autonomous_clock_ = node->get_clock();
  autonomous_logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".settle_duration", rclcpp::ParameterValue(0.5));
  settle_duration_ = node->get_parameter(plugin_name + ".settle_duration").as_double();
  if (!std::isfinite(settle_duration_) || settle_duration_ <= 0.0)
  {
    throw std::invalid_argument("Autonomous settle duration must be positive");
  }
  settled_since_.reset();
  arrival_log_time_ = -1.0;
}

void AutonomousStoppedGoalChecker::reset()
{
  StoppedGoalChecker::reset();
  settled_since_.reset();
  arrival_log_time_ = -1.0;
}

bool AutonomousStoppedGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity)
{
  const bool reached = StoppedGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  const double time = autonomous_clock_->now().seconds();
  const bool stable = detail::autonomousSettled(reached, time, settle_duration_, settled_since_);
  const double distance = std::hypot(query_pose.position.x - goal_pose.position.x, query_pose.position.y - goal_pose.position.y);
  double yaw_error = std::abs(angles::shortest_angular_distance(tf2::getYaw(query_pose.orientation), tf2::getYaw(goal_pose.orientation)));
  if (symmetric_yaw_tolerance_)
  {
    yaw_error = std::min(yaw_error, std::abs(3.14159265358979323846 - yaw_error));
  }
  const double linear = std::hypot(velocity.linear.x, velocity.linear.y);
  const double angular = std::abs(velocity.angular.z);
  if (distance <= std::max(0.3, 2.0 * xy_goal_tolerance_) && (stable || arrival_log_time_ < 0.0 || time < arrival_log_time_ || time - arrival_log_time_ >= 1.0))
  {
    const bool position_ok = distance <= xy_goal_tolerance_;
    const bool yaw_ok = yaw_error <= yaw_goal_tolerance_;
    const bool velocity_ok = linear <= trans_stopped_velocity_ && angular <= rot_stopped_velocity_;
    const double stable_time = settled_since_ ? std::max(0.0, time - *settled_since_) : 0.0;
    RCLCPP_INFO(autonomous_logger_, "Autonomous arrival waiting=%s xy=%.6f/%.6fm yaw=%.6f/%.6frad v=%.6f/%.6fm/s w=%.6f/%.6frad/s stable=%.3f/%.3fs", detail::autonomousArrivalBlocker(position_ok, yaw_ok, velocity_ok, stable), distance, xy_goal_tolerance_, yaw_error, yaw_goal_tolerance_, linear, trans_stopped_velocity_, angular, rot_stopped_velocity_, stable_time, settle_duration_);
    arrival_log_time_ = time;
  }
  return stable;
}

}  // namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousMPPIController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousStoppedGoalChecker, nav2_core::GoalChecker)
