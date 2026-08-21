#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

namespace nav2_regulated_modules
{

void RegulatedNavigator::sendFollowPath(const nav_msgs::msg::Path & path) {
  if (path.poses.empty() || task_.type == TaskType::NONE) {
    handlePlanningFailure("控制路径为空");
    return;
  }
  task_.active_path = path;
  task_.state = NavigationState::CONTROLLING;
  const auto generation = task_.generation;
  const auto sequence = ++follow_sequence_;
  const bool replacing_path = active_follow_goal_ != nullptr;
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = control_module_.controllerId();
  goal.goal_checker_id = control_module_.goalCheckerId();
  if (replacing_path) {
    LOG_DEBUG("更新 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}", generation, sequence, path.poses.size(), control_module_.controllerId());
  } else {
    LOG_INFO("下发 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}，goal_checker_id={}", generation, sequence, path.poses.size(), control_module_.controllerId(), control_module_.goalCheckerId());
  }
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.goal_response_callback = [this, generation, sequence](auto handle) {if (isCurrentFollow(generation, sequence)) {active_follow_goal_ = handle; if (!handle) {startRecovery("控制 Goal 被拒绝");}}};
  options.feedback_callback = [this, generation, sequence](auto, const std::shared_ptr<const FollowPath::Feedback> controller_feedback) {if (isCurrentFollow(generation, sequence)) {task_.distance_remaining = static_cast<double>(controller_feedback->distance_to_goal); current_speed_ = controller_feedback->speed; if (task_.total_path_length > 0.0) {const double completed_ratio = (task_.total_path_length - task_.distance_remaining) / task_.total_path_length; task_.progress = std::max(task_.progress, static_cast<float>(std::clamp(completed_ratio, 0.0, 1.0)));} if (active_navigation_service_goal_) {auto feedback = std::make_shared<NavigationService::Feedback>(); feedback->cur_task_id = task_.task_id; feedback->cur_seg_id = ""; feedback->progress = task_.progress; active_navigation_service_goal_->publish_feedback(feedback);}}};
  options.result_callback = [this, generation, sequence](const auto & result) {if (!isCurrentFollow(generation, sequence)) {return;} active_follow_goal_.reset(); if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {succeedTask();} else if (task_.state != NavigationState::CANCELING) {startRecovery("控制器执行路径失败");}};
  follow_client_->async_send_goal(goal, options);
}

bool RegulatedNavigator::isCurrentFollow(const uint64_t generation, const uint64_t sequence) const {
  return generation == task_.generation && sequence == follow_sequence_;
}

void RegulatedNavigator::stopRobot() {
  if (!stop_cmd_pub_) {return;}
  geometry_msgs::msg::Twist stop;
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  LOG_DEBUG("已向速度链入口连续发布 3 帧零速度");
}

void RegulatedNavigator::onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity) {
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_controller_velocity_ = *velocity;
  has_controller_velocity_ = true;
}

void RegulatedNavigator::onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity) {
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_smoothed_velocity_ = *velocity;
  has_smoothed_velocity_ = true;
}

void RegulatedNavigator::onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_velocity_odometry_ = *odometry;
  has_velocity_odometry_ = true;
}

void RegulatedNavigator::logVelocityChain() {
  if (!active_) {return;}
  geometry_msgs::msg::Twist controller_velocity;
  geometry_msgs::msg::Twist smoothed_velocity;
  nav_msgs::msg::Odometry velocity_odometry;
  bool has_controller_velocity;
  bool has_smoothed_velocity;
  bool has_velocity_odometry;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    controller_velocity = latest_controller_velocity_;
    smoothed_velocity = latest_smoothed_velocity_;
    velocity_odometry = latest_velocity_odometry_;
    has_controller_velocity = has_controller_velocity_;
    has_smoothed_velocity = has_smoothed_velocity_;
    has_velocity_odometry = has_velocity_odometry_;
  }
  if (!has_controller_velocity && !has_smoothed_velocity && !has_velocity_odometry) {return;}
  LOG_INFO("关键速度链：反馈 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 控制器输出/平滑器输入 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 平滑器输出 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}]", velocity_odom_topic_, velocity_odometry.twist.twist.linear.x, velocity_odometry.twist.twist.angular.z, has_velocity_odometry, controller_cmd_vel_topic_, controller_velocity.linear.x, controller_velocity.angular.z, has_controller_velocity, smoothed_cmd_vel_topic_, smoothed_velocity.linear.x, smoothed_velocity.angular.z, has_smoothed_velocity);
}

}  // namespace nav2_regulated_modules
