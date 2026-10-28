#ifndef NAV2_REGULATED_MODULES__AUTONOMOUS_MPPI_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__AUTONOMOUS_MPPI_CONTROLLER_HPP_

#include <mutex>
#include <optional>
#include <string>

#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"

namespace nav2_regulated_modules
{

// 仅由自主模式的参数重写选择；固定路径继续加载原插件。
class AutonomousMPPIController : public nav2_rotation_shim_controller::RotationShimController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

private:
  void checkTerminalTrajectory(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command);
  void logTerminalState(double distance, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command);
  void resetTerminalState();
  geometry_msgs::msg::PoseStamped poseInControlFrame(const geometry_msgs::msg::PoseStamped & point, const geometry_msgs::msg::PoseStamped & robot_pose);
  geometry_msgs::msg::TwistStamped computeTrackingCommand(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker);
  std::mutex autonomous_mutex_;
  double nominal_speed_ = 1.5;
  double approach_distance_ = 1.0;
  double external_limit_ = 0.0;
  bool external_percentage_ = false;
  double terminal_distance_ = 0.15;
  double terminal_linear_speed_ = 0.05;
  double terminal_angular_speed_ = 0.3;
  detail::AutonomousTerminalState terminal_state_;
  std::optional<geometry_msgs::msg::PoseStamped> terminal_goal_;
  std::optional<detail::AutonomousTerminalPhase> logged_terminal_phase_;
  double terminal_log_time_ = -1.0;
};

class AutonomousStoppedGoalChecker : public nav2_controller::StoppedGoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::Twist & velocity) override;

private:
  rclcpp::Clock::SharedPtr autonomous_clock_;
  double settle_duration_ = 0.5;
  std::optional<double> settled_since_;
  rclcpp::Logger autonomous_logger_ = rclcpp::get_logger("AutonomousStoppedGoalChecker");
  double arrival_log_time_ = -1.0;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__AUTONOMOUS_MPPI_CONTROLLER_HPP_
