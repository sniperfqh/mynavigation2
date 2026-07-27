#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

#include "nav2_regulated_modules/navigation_utils.hpp"

namespace nav2_regulated_modules
{

rclcpp_action::GoalResponse RegulatedNavigator::handlePoseGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateToPose::Goal> goal) {
  if (!active_ || !navigation_utils::validPose(goal->pose) || !goal->behavior_tree.empty()) {
    RCLCPP_WARN(get_logger(), "拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse RegulatedNavigator::handlePosesGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateThroughPoses::Goal> goal) {
  const bool poses_valid = !goal->poses.empty() && std::all_of(goal->poses.begin(), goal->poses.end(), [](const auto & pose) {return navigation_utils::validPose(pose);});
  if (!active_ || !poses_valid || !goal->behavior_tree.empty()) {
    RCLCPP_WARN(get_logger(), "拒绝多点目标：节点未激活、目标数组无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RegulatedNavigator::handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal) {
  if (goal == active_pose_goal_) {
    // 中文注释：先让 rclcpp_action 完成 ACCEPT 状态迁移，再由监控周期执行取消收尾。
    cancel_requested_ = true;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse RegulatedNavigator::handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal) {
  if (goal == active_poses_goal_) {
    cancel_requested_ = true;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RegulatedNavigator::handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal) {
  // 中文注释：新目标采用 replace 抢占策略，先结束旧外层目标并取消全部子 Goal。
  preemptCurrentTask();
  active_pose_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::TO_POSE;
  task_.goal = goal->get_goal()->pose;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  startPlanning(false);
}

void RegulatedNavigator::handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal) {
  preemptCurrentTask();
  active_poses_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::THROUGH_POSES;
  task_.goals = goal->get_goal()->poses;
  task_.goal = task_.goals.back();
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  startPlanning(false);
}

void RegulatedNavigator::onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
  if (!active_ || !navigation_utils::validPose(*goal)) {
    RCLCPP_WARN(get_logger(), "忽略无效或未激活状态下的 goal_pose");
    return;
  }
  preemptCurrentTask();
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::TOPIC_GOAL;
  task_.goal = *goal;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  startPlanning(false);
}

void RegulatedNavigator::publishFeedback() {
  if (!active_ || task_.type == TaskType::NONE) {return;}
  geometry_msgs::msg::PoseStamped current_pose;
  if (!lookupCurrentPose(current_pose)) {return;}
  const double navigation_time = (now() - task_.start_time).seconds();
  const double eta = current_speed_ > 0.03 ? task_.distance_remaining / current_speed_ : 0.0;

  if (active_pose_goal_) {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = current_pose;
    feedback->navigation_time = navigation_utils::durationFromSeconds(navigation_time);
    feedback->estimated_time_remaining = navigation_utils::durationFromSeconds(eta);
    feedback->number_of_recoveries = static_cast<int16_t>(task_.recovery_count);
    feedback->distance_remaining = task_.distance_remaining;
    active_pose_goal_->publish_feedback(feedback);
  }
  if (active_poses_goal_) {
    auto feedback = std::make_shared<NavigateThroughPoses::Feedback>();
    feedback->current_pose = current_pose;
    feedback->navigation_time = navigation_utils::durationFromSeconds(navigation_time);
    feedback->estimated_time_remaining = navigation_utils::durationFromSeconds(eta);
    feedback->number_of_recoveries = static_cast<int16_t>(task_.recovery_count);
    feedback->distance_remaining = task_.distance_remaining;
    feedback->number_of_poses_remaining = static_cast<int16_t>(task_.goals.size());
    active_poses_goal_->publish_feedback(feedback);
  }
}

}  // namespace nav2_regulated_modules
