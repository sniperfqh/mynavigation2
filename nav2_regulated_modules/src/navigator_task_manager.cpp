#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <memory>
#include <string>

namespace nav2_regulated_modules
{

void RegulatedNavigator::cancelSubGoals(const bool invalidate_callbacks) {
  const bool had_compute_pose = active_compute_pose_goal_ != nullptr;
  const bool had_compute_poses = active_compute_poses_goal_ != nullptr;
  const bool had_smooth = active_smooth_goal_ != nullptr;
  const bool had_follow = active_follow_goal_ != nullptr;
  if (invalidate_callbacks) {
    ++plan_sequence_;
    ++follow_sequence_;
  }
  planning_active_ = false;
  if (active_compute_pose_goal_) {
    compute_pose_client_->async_cancel_goal(active_compute_pose_goal_);
    active_compute_pose_goal_.reset();
  }
  if (active_compute_poses_goal_) {
    compute_poses_client_->async_cancel_goal(active_compute_poses_goal_);
    active_compute_poses_goal_.reset();
  }
  if (active_smooth_goal_) {
    smooth_client_->async_cancel_goal(active_smooth_goal_);
    active_smooth_goal_.reset();
  }
  if (active_follow_goal_) {
    follow_client_->async_cancel_goal(active_follow_goal_);
    active_follow_goal_.reset();
  }
  if (had_compute_pose || had_compute_poses || had_smooth || had_follow) {
    LOG_DEBUG("取消子 Goal，compute_pose={}，compute_poses={}，smooth={}，follow={}，invalidate_callbacks={}", had_compute_pose, had_compute_poses, had_smooth, had_follow, invalidate_callbacks);
  }
}

void RegulatedNavigator::cancelTask(const std::string & reason) {
  if (task_.type == TaskType::NONE) {return;}
  const auto generation = task_.generation;
  task_.state = NavigationState::CANCELING;
  ++task_generation_;
  cancelSubGoals(true);
  stopRobot();
  auto pose_result = std::make_shared<NavigateToPose::Result>();
  auto poses_result = std::make_shared<NavigateThroughPoses::Result>();
  if (active_pose_goal_) {
    if (active_pose_goal_->is_canceling()) {
      active_pose_goal_->canceled(pose_result);
    } else {
      active_pose_goal_->abort(pose_result);
    }
    active_pose_goal_.reset();
  }
  if (active_poses_goal_) {
    if (active_poses_goal_->is_canceling()) {
      active_poses_goal_->canceled(poses_result);
    } else {
      active_poses_goal_->abort(poses_result);
    }
    active_poses_goal_.reset();
  }
  if (active_fixed_path_goal_) {
    auto fixed_path_result = std::make_shared<FollowFixedPath::Result>();
    fixed_path_result->success = false;
    if (active_fixed_path_goal_->is_canceling()) {
      active_fixed_path_goal_->canceled(fixed_path_result);
    } else {
      active_fixed_path_goal_->abort(fixed_path_result);
    }
    active_fixed_path_goal_.reset();
  }
  LOG_WARN("导航任务已取消：{}", reason);
  LOG_INFO("导航任务取消收口完成，generation={}，reason={}", generation, reason);
  resetTask();
}

void RegulatedNavigator::preemptCurrentTask() {
  if (task_.type == TaskType::NONE) {return;}
  const auto generation = task_.generation;
  const char * task_type = task_.type == TaskType::TO_POSE ? "to_pose" : task_.type == TaskType::THROUGH_POSES ? "through_poses" : task_.type == TaskType::TOPIC_GOAL ? "topic_goal" : "fixed_path";
  ++task_generation_;
  cancelSubGoals(true);
  stopRobot();
  auto pose_result = std::make_shared<NavigateToPose::Result>();
  auto poses_result = std::make_shared<NavigateThroughPoses::Result>();
  if (active_pose_goal_) {
    active_pose_goal_->abort(pose_result);
    active_pose_goal_.reset();
  }
  if (active_poses_goal_) {
    active_poses_goal_->abort(poses_result);
    active_poses_goal_.reset();
  }
  if (active_fixed_path_goal_) {
    auto fixed_path_result = std::make_shared<FollowFixedPath::Result>();
    fixed_path_result->success = false;
    active_fixed_path_goal_->abort(fixed_path_result);
    active_fixed_path_goal_.reset();
  }
  LOG_INFO("旧导航任务被新任务抢占，generation={}，task_type={}", generation, task_type);
  resetTask();
}

void RegulatedNavigator::succeedTask() {
  const auto generation = task_.generation;
  const auto elapsed = (now() - task_.start_time).seconds();
  task_.state = NavigationState::SUCCEEDED;
  if (active_pose_goal_) {
    active_pose_goal_->succeed(std::make_shared<NavigateToPose::Result>());
    active_pose_goal_.reset();
  }
  if (active_poses_goal_) {
    active_poses_goal_->succeed(std::make_shared<NavigateThroughPoses::Result>());
    active_poses_goal_.reset();
  }
  if (active_fixed_path_goal_) {
    auto fixed_path_result = std::make_shared<FollowFixedPath::Result>();
    fixed_path_result->success = true;
    active_fixed_path_goal_->succeed(fixed_path_result);
    active_fixed_path_goal_.reset();
  }
  LOG_INFO("导航任务执行成功，generation={}，耗时={:.3f}s，恢复次数={}", generation, elapsed, task_.recovery_count);
  resetTask();
}

void RegulatedNavigator::failTask(const std::string & reason) {
  const auto generation = task_.generation;
  task_.state = NavigationState::FAILED;
  cancelSubGoals(true);
  if (active_pose_goal_) {
    active_pose_goal_->abort(std::make_shared<NavigateToPose::Result>());
    active_pose_goal_.reset();
  }
  if (active_poses_goal_) {
    active_poses_goal_->abort(std::make_shared<NavigateThroughPoses::Result>());
    active_poses_goal_.reset();
  }
  if (active_fixed_path_goal_) {
    auto fixed_path_result = std::make_shared<FollowFixedPath::Result>();
    fixed_path_result->success = false;
    active_fixed_path_goal_->abort(fixed_path_result);
    active_fixed_path_goal_.reset();
  }
  LOG_ERROR("导航任务失败，generation={}，恢复次数={}，reason={}", generation, task_.recovery_count, reason);
  resetTask();
}

void RegulatedNavigator::resetTask() {
  task_ = NavigationTask();
  task_.generation = task_generation_;
  updating_path_ = false;
  planning_active_ = false;
  has_last_pose_ = false;
  current_speed_ = 0.0;
  cancel_requested_ = false;
  active_fixed_path_goal_.reset();
}

}  // namespace nav2_regulated_modules
