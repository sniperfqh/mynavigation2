#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "nav2_regulated_modules/autonomous_mppi_controller.hpp"
#include "rcl/time.h"

namespace
{

void require(const bool valid, const char * reason)
{
  if (!valid)
  {
    throw std::runtime_error(reason);
  }
}

class RecordingController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string, std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void cleanup() override
  {
  }
  void activate() override
  {
  }
  void deactivate() override
  {
  }
  void setPlan(const nav_msgs::msg::Path &) override
  {
  }
  void setSpeedLimit(const double &, const bool &) override
  {
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) override
  {
    ++calls;
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = 0.6;
    return command;
  }
  int calls = 0;
};

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override
  {
  }
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist &) override
  {
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose.position.x = 0.03;
    pose.orientation.z = std::sin(0.157 / 2.0);
    pose.orientation.w = std::cos(0.157 / 2.0);
    velocity.linear.x = 0.01;
    velocity.angular.z = 0.05;
    return true;
  }
};

class TestController : public nav2_regulated_modules::AutonomousMPPIController
{
public:
  TestController()
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    require(rcl_enable_ros_time_override(clock_->get_clock_handle()) == RCL_RET_OK, "enable numerical ROS clock");
    require(rcl_set_ros_time_override(clock_->get_clock_handle(), 100000000000LL) == RCL_RET_OK, "set numerical ROS clock");
    tf_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_->setUsingDedicatedThread(true);
    recording = std::make_shared<RecordingController>();
    primary_controller_ = recording;
    position_goal_checker_ = std::make_unique<nav2_controller::PositionGoalChecker>();
    forward_sampling_distance_ = 0.5;
    angular_dist_threshold_ = 0.785;
    angular_disengage_threshold_ = 0.174;
    max_angular_accel_ = 0.8;
    control_duration_ = 0.02;
    rotate_to_heading_angular_vel_ = 0.5;
  }
  void cache(const std::string & parent, const std::string & child, const int64_t time)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.header.stamp = rclcpp::Time(time, RCL_ROS_TIME);
    transform.transform.rotation.w = 1.0;
    require(tf_->setTransform(transform, "offline_test"), "cache transform");
  }
  bool nowWouldExtrapolate()
  {
    try
    {
      tf_->lookupTransform("base_link", "map", clock_->now(), rclcpp::Duration::from_seconds(0.0));
    }
    catch (const tf2::ExtrapolationException &)
    {
      return true;
    }
    return false;
  }
  std::shared_ptr<RecordingController> recording;
};

nav_msgs::msg::Path path(const std::string & frame, const double x, const double y)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = frame;
  result.poses.resize(2);
  result.poses[0].pose.orientation.w = 1.0;
  result.poses[1].pose.position.x = x;
  result.poses[1].pose.position.y = y;
  result.poses[1].pose.orientation.w = 1.0;
  return result;
}

}  // namespace

int main()
{
  TestController controller;
  TestGoalChecker checker;
  constexpr int64_t pose_time = 99990322000LL;
  controller.cache("map", "base_link", pose_time);
  require(controller.nowWouldExtrapolate(), "original now() query exceeds TF by 9.678ms");
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = rclcpp::Time(pose_time, RCL_ROS_TIME);
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  controller.setPlan(path("map", 1.0, 0.0));
  const auto started = std::chrono::steady_clock::now();
  auto command = controller.computeVelocityCommands(pose, velocity, &checker);
  require(command.twist.linear.x == 0.6 && controller.recording->calls == 1, "same-frame path uses coherent robot pose despite future TF gap");
  pose.pose.orientation.z = std::sin(1.5707963267948966 / 2.0);
  pose.pose.orientation.w = std::cos(1.5707963267948966 / 2.0);
  controller.setPlan(path("map", 0.0, 1.0));
  command = controller.computeVelocityCommands(pose, velocity, &checker);
  require(command.twist.linear.x == 0.6, "world heading is correctly made relative to robot yaw");
  controller.cache("map", "odom", pose_time);
  controller.setPlan(path("odom", 0.0, 1.0));
  command = controller.computeVelocityCommands(pose, velocity, &checker);
  require(command.twist.linear.x == 0.6, "different frame is transformed at robot pose stamp");
  controller.setPlan(path("missing_frame", 1.0, 0.0));
  const int before_failure = controller.recording->calls;
  bool rejected = false;
  try
  {
    controller.computeVelocityCommands(pose, velocity, &checker);
  }
  catch (const nav2_core::PlannerException &)
  {
    rejected = true;
  }
  require(rejected && controller.recording->calls == before_failure, "missing coherent transform rejects command without MPPI fallback");
  controller.cache("map", "lagged_frame", pose_time - 10000000LL);
  controller.setPlan(path("lagged_frame", 1.0, 0.0));
  rejected = false;
  try
  {
    controller.computeVelocityCommands(pose, velocity, &checker);
  }
  catch (const nav2_core::PlannerException &)
  {
    rejected = true;
  }
  require(rejected, "future transform in a different frame is rejected without waiting or timestamp fallback");
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  auto short_path = path("map", 0.7, 0.0);
  short_path.poses.front().pose.position.x = 0.5;
  controller.setPlan(short_path);
  command = controller.computeVelocityCommands(pose, velocity, &checker);
  require(command.twist.linear.x == 0.6, "path shorter than lookahead still delegates to MPPI");
  controller.setPlan(nav_msgs::msg::Path());
  rejected = false;
  try
  {
    controller.computeVelocityCommands(pose, velocity, &checker);
  }
  catch (const nav2_core::PlannerException &)
  {
    rejected = true;
  }
  require(rejected, "empty path rejected");
  const double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  std::cout << "PASS coherent-pose TF regression: 9.678ms future gap, yaw geometry, cross-frame cache, missing/future rejection; elapsed=" << elapsed_ms << "ms; no ROS nodes or navigation goals created\n";
}
