#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <memory>
#include <string>

namespace nav2_regulated_modules
{

void RegulatedNavigator::cancelSubGoals(const bool invalidate_callbacks) {
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
}

void RegulatedNavigator::cancelTask(const std::string & reason) {
  if (task_.type == TaskType::NONE) {return;}
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
  RCLCPP_WARN(get_logger(), "导航任务已取消：%s", reason.c_str());
  resetTask();
}

void RegulatedNavigator::preemptCurrentTask() {
  if (task_.type == TaskType::NONE) {return;}
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
  resetTask();
}

void RegulatedNavigator::succeedTask() {
  task_.state = NavigationState::SUCCEEDED;
  if (active_pose_goal_) {
    active_pose_goal_->succeed(std::make_shared<NavigateToPose::Result>());
    active_pose_goal_.reset();
  }
  if (active_poses_goal_) {
    active_poses_goal_->succeed(std::make_shared<NavigateThroughPoses::Result>());
    active_poses_goal_.reset();
  }
  RCLCPP_INFO(get_logger(), "导航任务执行成功");
  resetTask();
}

void RegulatedNavigator::failTask(const std::string & reason) {
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
  RCLCPP_ERROR(get_logger(), "导航任务失败：%s", reason.c_str());
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
}

}  // namespace nav2_regulated_modules
