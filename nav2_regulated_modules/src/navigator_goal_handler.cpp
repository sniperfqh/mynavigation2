#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

#include "nav2_regulated_modules/navigation_utils.hpp"

namespace nav2_regulated_modules
{

// 中文注释：校验单点自主 Goal；非自主模式、无效位姿和行为树 XML 请求均被拒绝。
rclcpp_action::GoalResponse RegulatedNavigator::handlePoseGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateToPose::Goal> goal) {
  if (operation_mode_ != NavigationMode::AUTONOMOUS) {
    LOG_WARN("当前模式不接受自主单点导航目标");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!active_ || !navigation_utils::validPose(goal->pose) || !goal->behavior_tree.empty()) {
    LOG_WARN("拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 中文注释：逐个校验多点自主 Goal，并保持本节点“不接受外部行为树”的接口边界。
rclcpp_action::GoalResponse RegulatedNavigator::handlePosesGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateThroughPoses::Goal> goal) {
  if (operation_mode_ != NavigationMode::AUTONOMOUS) {
    LOG_WARN("当前模式不接受自主多点导航目标");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const bool poses_valid = !goal->poses.empty() && std::all_of(goal->poses.begin(), goal->poses.end(), [](const auto & pose) {return navigation_utils::validPose(pose);});
  if (!active_ || !poses_valid || !goal->behavior_tree.empty()) {
    LOG_WARN("拒绝多点目标：节点未激活、目标数组无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 中文注释：记录活动单点 Goal 的取消请求，具体收口由监控周期统一执行。
rclcpp_action::CancelResponse RegulatedNavigator::handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal) {
  if (goal == active_pose_goal_) {
    // 中文注释：先让 rclcpp_action 完成 ACCEPT 状态迁移，再由监控周期执行取消收尾。
    cancel_requested_ = true;
    LOG_DEBUG("收到单点导航取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 中文注释：记录活动多点 Goal 的取消请求，避免直接在 Action 回调中重入任务清理。
rclcpp_action::CancelResponse RegulatedNavigator::handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal) {
  if (goal == active_poses_goal_) {
    cancel_requested_ = true;
    LOG_DEBUG("收到多点导航取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 中文注释：接受单点 Goal 后抢占旧任务，初始化新任务代次并进入首次规划。
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
  LOG_INFO("接受单点导航任务，generation={}，frame={}，goal=({:.3f}, {:.3f})", task_.generation, task_.goal.header.frame_id, task_.goal.pose.position.x, task_.goal.pose.position.y);
  startPlanning(false);
}

// 中文注释：接受多点 Goal 后保存完整目标序列，最后一个 Pose 作为最终终点。
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
  LOG_INFO("接受多点导航任务，generation={}，目标数={}，final_frame={}，final_goal=({:.3f}, {:.3f})", task_.generation, task_.goals.size(), task_.goal.header.frame_id, task_.goal.pose.position.x, task_.goal.pose.position.y);
  startPlanning(false);
}

// 中文注释：兼容 RViz／上游节点的 goal_pose Topic，转换为无外层 Action 句柄的自主任务。
void RegulatedNavigator::onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
  if (operation_mode_ != NavigationMode::AUTONOMOUS) {
    LOG_WARN("当前模式忽略 goal_pose");
    return;
  }
  if (!active_ || !navigation_utils::validPose(*goal)) {
    LOG_WARN("忽略无效或未激活状态下的 goal_pose");
    return;
  }
  preemptCurrentTask();
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::TOPIC_GOAL;
  task_.goal = *goal;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  LOG_INFO("接受 goal_pose 任务，generation={}，frame={}，goal=({:.3f}, {:.3f})", task_.generation, task_.goal.header.frame_id, task_.goal.pose.position.x, task_.goal.pose.position.y);
  startPlanning(false);
}

// 中文注释：查询当前 TF，并向活动外层 Action 发布耗时、ETA、剩余距离和恢复次数。
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
