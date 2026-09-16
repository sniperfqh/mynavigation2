#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_regulated_modules
{

class FixedPathController : public nav2_core::Controller
{
  public:
  FixedPathController() = default;
  ~FixedPathController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  enum class Phase
  {
    ALIGN_START, TRACK_PATH, ALIGN_GOAL, SETTLE
  };

  bool transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const;
  std::size_t findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose);
  double remainingDistance(std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const;
  geometry_msgs::msg::PoseStamped selectCarrot(std::size_t start_index, double lookahead_distance) const;
  geometry_msgs::msg::TwistStamped zeroCommand() const;
  geometry_msgs::msg::TwistStamped rotateCommand(double yaw_error, const geometry_msgs::msg::Twist & velocity) const;
  static double normalizeAngle(double angle);
  static double poseYaw(const geometry_msgs::msg::PoseStamped & pose);
  static double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_
  {
    rclcpp::get_logger("FixedPathController")
  };
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  std::mutex mutex_;
  Phase phase_
  {
    Phase::ALIGN_START
  };
  std::size_t nearest_index_
  {
    0
  };
  int direction_sign_
  {
    1
  };
  int stable_cycles_
  {
    0
  };
  bool start_strategy_evaluated_
  {
    false
  };
  bool direct_start_tracking_
  {
    false
  };
  double start_path_yaw_
  {
    0.0
  };
  double goal_path_yaw_
  {
    0.0
  };
  double base_linear_velocity_
  {
    0.52
  };
  double speed_limit_
  {
    0.52
  };
  double lookahead_dist_
  {
    0.45
  };
  double min_lookahead_dist_
  {
    0.25
  };
  double max_lookahead_dist_
  {
    0.75
  };
  double lookahead_time_
  {
    1.5
  };
  double start_position_tolerance_
  {
    0.70
  };
  double direct_tracking_lateral_tolerance_
  {
    0.20
  };
  double direct_tracking_max_yaw_error_
  {
    0.2617993877991494
  };
  double initial_yaw_tolerance_
  {
    0.12217304763960307
  };
  double rotate_to_heading_angular_vel_
  {
    0.4
  };
  double max_angular_accel_
  {
    0.8
  };
  double min_approach_linear_velocity_
  {
    0.01
  };
  double approach_velocity_scaling_dist_
  {
    0.8
  };
  double goal_position_hysteresis_
  {
    1.5
  };
  double transform_tolerance_
  {
    0.2
  };
  double control_duration_
  {
    0.02
  };
  int alignment_stable_cycles_
  {
    5
  };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
