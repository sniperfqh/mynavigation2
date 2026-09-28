// 导航目标入口。接收标准 Action、固定路径 Action 与话题目标，处理接受、取消和反馈。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

#include "nav2_regulated_modules/navigation_utils.hpp"

namespace nav2_regulated_modules
{

// 校验单目标导航请求，拒绝与当前模式或状态不兼容的目标。
rclcpp_action::GoalResponse RegulatedNavigator::handlePoseGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateToPose::Goal> goal)
{
  if (operation_mode_ != NavigationMode::AUTONOMOUS)
  {
    LOG_WARN("当前模式不接受自主单点导航目标");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!active_ || !navigation_utils::validPose(goal->pose) || !goal->behavior_tree.empty())
  {
    LOG_WARN("拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 校验多目标导航请求，避免空路径或冲突任务进入执行。
rclcpp_action::GoalResponse RegulatedNavigator::handlePosesGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateThroughPoses::Goal> goal)
{
  if (operation_mode_ != NavigationMode::AUTONOMOUS)
  {
    LOG_WARN("当前模式不接受自主多点导航目标");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const bool poses_valid = !goal->poses.empty() && std::all_of(goal->poses.begin(), goal->poses.end(), [](const auto & pose)
  {
    return navigation_utils::validPose(pose);
  }
  );
  if (!active_ || !poses_valid || !goal->behavior_tree.empty())
  {
    LOG_WARN("拒绝多点目标：节点未激活、目标数组无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理单目标 Action 取消请求并传播到下游子目标。
rclcpp_action::CancelResponse RegulatedNavigator::handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal)
{
  if (goal == active_pose_goal_)
  {
    cancel_requested_ = true;
    LOG_DEBUG("收到单点导航取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 处理多目标 Action 取消请求并传播到下游子目标。
rclcpp_action::CancelResponse RegulatedNavigator::handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal)
{
  if (goal == active_poses_goal_)
  {
    cancel_requested_ = true;
    LOG_DEBUG("收到多点导航取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 接管已接受的单目标任务并启动规划流程。
void RegulatedNavigator::handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal)
{
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

// 接管已接受的多目标任务并启动规划流程。
void RegulatedNavigator::handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal)
{
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

// 将话题位姿目标接入同一导航任务流程。
void RegulatedNavigator::onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  if (operation_mode_ != NavigationMode::AUTONOMOUS)
  {
    LOG_WARN("当前模式忽略 goal_pose");
    return;
  }
  if (!active_ || !navigation_utils::validPose(*goal))
  {
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

// 按固定频率向活动 Action 发布当前位置与任务进度。
void RegulatedNavigator::publishFeedback()
{
  if (!active_ || task_.type == TaskType::NONE)
  {
    return;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  if (!lookupCurrentPose(current_pose))
  {
    return;
  }
  const double navigation_time = (now() - task_.start_time).seconds();
  const double eta = current_speed_ > 0.03 ? task_.distance_remaining / current_speed_ : 0.0;

  if (active_pose_goal_)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = current_pose;
    feedback->navigation_time = navigation_utils::durationFromSeconds(navigation_time);
    feedback->estimated_time_remaining = navigation_utils::durationFromSeconds(eta);
    feedback->number_of_recoveries = static_cast<int16_t>(task_.recovery_count);
    feedback->distance_remaining = task_.distance_remaining;
    active_pose_goal_->publish_feedback(feedback);
  }
  if (active_poses_goal_)
  {
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

}
// namespace nav2_regulated_modules
