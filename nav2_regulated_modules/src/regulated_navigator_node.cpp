#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace nav2_regulated_modules
{

class RegulatedNavigator : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using SmoothGoalHandle = rclcpp_action::ClientGoalHandle<SmoothPath>;

  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("regulated_navigator", options)
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    compute_path_action_ =
      declare_parameter<std::string>("compute_path_action", "compute_path_to_pose");
    smooth_path_action_ = declare_parameter<std::string>("smooth_path_action", "smooth_path");
    follow_path_action_ = declare_parameter<std::string>("follow_path_action", "follow_path");
    navigate_to_pose_action_ =
      declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBasedAstar");
    controller_id_ = declare_parameter<std::string>("controller_id", "DWB");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    smoother_id_ = declare_parameter<std::string>("smoother_id", "simple_smoother");
    use_smoother_ = declare_parameter<bool>("use_smoother", true);
    check_smoother_collisions_ = declare_parameter<bool>("check_smoother_collisions", true);
    server_timeout_ = declare_parameter<double>("server_timeout", 5.0);
    smoothing_duration_ = declare_parameter<double>("max_smoothing_duration", 2.0);

    compute_path_client_ =
      rclcpp_action::create_client<ComputePathToPose>(this, compute_path_action_);
    smooth_path_client_ = rclcpp_action::create_client<SmoothPath>(this, smooth_path_action_);
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_action_);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&RegulatedNavigator::onGoalPose, this, std::placeholders::_1));

    navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      navigate_to_pose_action_,
      std::bind(
        &RegulatedNavigator::handleNavigateToPoseGoal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &RegulatedNavigator::handleNavigateToPoseCancel,
        this,
        std::placeholders::_1),
      std::bind(
        &RegulatedNavigator::handleNavigateToPoseAccepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Regulated navigator ready: %s -> %s -> %s, action server: %s",
      compute_path_action_.c_str(),
      use_smoother_ ? smooth_path_action_.c_str() : "skip_smoother",
      follow_path_action_.c_str(),
      navigate_to_pose_action_.c_str());
  }

private:
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    if (navigation_active_) {
      RCLCPP_WARN(get_logger(), "Ignoring goal because another navigation task is active.");
      return;
    }

    navigation_active_ = true;
    active_nav_to_pose_goal_.reset();
    sendComputePathGoal(*goal);
  }

  rclcpp_action::GoalResponse handleNavigateToPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (navigation_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateToPose goal because navigation is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateToPose goal with empty frame_id.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleNavigateToPoseCancel(
    const std::shared_ptr<NavigateGoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "NavigateToPose cancel is accepted, but active sub-goals are not canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleNavigateToPoseAccepted(const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    navigation_active_ = true;
    active_nav_to_pose_goal_ = goal_handle;
    sendComputePathGoal(goal_handle->get_goal()->pose);
  }

  bool waitForActionServers()
  {
    const auto timeout = std::chrono::duration<double>(server_timeout_);
    if (!compute_path_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(get_logger(), "Action server '%s' is not available.", compute_path_action_.c_str());
      return false;
    }
    if (use_smoother_ && !smooth_path_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(get_logger(), "Action server '%s' is not available.", smooth_path_action_.c_str());
      return false;
    }
    if (!follow_path_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(get_logger(), "Action server '%s' is not available.", follow_path_action_.c_str());
      return false;
    }
    return true;
  }

  void sendComputePathGoal(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    if (!waitForActionServers()) {
      finishTask(false);
      return;
    }

    ComputePathToPose::Goal goal;
    goal.goal = goal_pose;
    goal.planner_id = planner_id_;
    goal.use_start = false;

    auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    options.goal_response_callback =
      std::bind(&RegulatedNavigator::onComputePathGoalResponse, this, std::placeholders::_1);
    options.result_callback =
      std::bind(&RegulatedNavigator::onComputePathResult, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Computing path with planner '%s'.", planner_id_.c_str());
    compute_path_client_->async_send_goal(goal, options);
  }

  void onComputePathGoalResponse(const ComputeGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "ComputePathToPose goal was rejected.");
      finishTask(false);
    }
  }

  void onComputePathResult(const ComputeGoalHandle::WrappedResult & result)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        get_logger(), "ComputePathToPose failed with result code %d.",
        static_cast<int>(result.code));
      finishTask(false);
      return;
    }

    if (result.result->path.poses.empty()) {
      RCLCPP_ERROR(get_logger(), "Planner returned an empty path.");
      finishTask(false);
      return;
    }

    if (use_smoother_) {
      sendSmoothPathGoal(result.result->path);
    } else {
      sendFollowPathGoal(result.result->path);
    }
  }

  void sendSmoothPathGoal(const nav_msgs::msg::Path & path)
  {
    SmoothPath::Goal goal;
    goal.path = path;
    goal.smoother_id = smoother_id_;
    const double whole_seconds = std::floor(smoothing_duration_);
    goal.max_smoothing_duration.sec = static_cast<int32_t>(whole_seconds);
    goal.max_smoothing_duration.nanosec =
      static_cast<uint32_t>((smoothing_duration_ - whole_seconds) * 1e9);
    goal.check_for_collisions = check_smoother_collisions_;

    auto options = rclcpp_action::Client<SmoothPath>::SendGoalOptions();
    options.goal_response_callback =
      std::bind(&RegulatedNavigator::onSmoothPathGoalResponse, this, std::placeholders::_1);
    options.result_callback =
      std::bind(&RegulatedNavigator::onSmoothPathResult, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Smoothing path with smoother '%s'.", smoother_id_.c_str());
    smooth_path_client_->async_send_goal(goal, options);
  }

  void onSmoothPathGoalResponse(const SmoothGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "SmoothPath goal was rejected.");
      finishTask(false);
    }
  }

  void onSmoothPathResult(const SmoothGoalHandle::WrappedResult & result)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        get_logger(), "SmoothPath failed with result code %d.", static_cast<int>(result.code));
      finishTask(false);
      return;
    }

    sendFollowPathGoal(result.result->path);
  }

  void sendFollowPathGoal(const nav_msgs::msg::Path & path)
  {
    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

    auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    options.goal_response_callback =
      std::bind(&RegulatedNavigator::onFollowPathGoalResponse, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(
      &RegulatedNavigator::onFollowPathFeedback,
      this,
      std::placeholders::_1,
      std::placeholders::_2);
    options.result_callback =
      std::bind(&RegulatedNavigator::onFollowPathResult, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Following path with controller '%s'.", controller_id_.c_str());
    follow_path_client_->async_send_goal(goal, options);
  }

  void onFollowPathGoalResponse(const FollowGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "FollowPath goal was rejected.");
      finishTask(false);
    }
  }

  void onFollowPathFeedback(
    FollowGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
  {
    RCLCPP_DEBUG(
      get_logger(), "Distance to goal: %.3f, speed: %.3f",
      feedback->distance_to_goal, feedback->speed);
  }

  void onFollowPathResult(const FollowGoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "FollowPath succeeded.");
      finishTask(true);
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath failed with result code %d.", static_cast<int>(result.code));
      finishTask(false);
    }
  }

  void finishTask(bool succeeded)
  {
    if (active_nav_to_pose_goal_) {
      auto result = std::make_shared<NavigateToPose::Result>();
      if (active_nav_to_pose_goal_->is_canceling()) {
        active_nav_to_pose_goal_->canceled(result);
      } else if (succeeded) {
        active_nav_to_pose_goal_->succeed(result);
      } else {
        active_nav_to_pose_goal_->abort(result);
      }
      active_nav_to_pose_goal_.reset();
    }
    navigation_active_ = false;
  }

  std::string goal_topic_;
  std::string compute_path_action_;
  std::string smooth_path_action_;
  std::string follow_path_action_;
  std::string navigate_to_pose_action_;
  std::string planner_id_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string smoother_id_;
  bool use_smoother_;
  bool check_smoother_collisions_;
  double server_timeout_;
  double smoothing_duration_;
  bool navigation_active_{false};

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_server_;
  std::shared_ptr<NavigateGoalHandle> active_nav_to_pose_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace nav2_regulated_modules

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_regulated_modules::RegulatedNavigator>());
  rclcpp::shutdown();
  return 0;
}
