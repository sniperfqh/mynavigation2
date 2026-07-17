#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "nav2_regulated_modules/navigation_utils.hpp"

namespace nav2_regulated_modules
{

bool RegulatedNavigator::dependenciesReady()
{
  const auto timeout = std::chrono::duration<double>(server_timeout_);
  return compute_pose_client_->wait_for_action_server(timeout) &&
         compute_poses_client_->wait_for_action_server(timeout) &&
         (!planning_module_.useSmoother() || smooth_client_->wait_for_action_server(timeout)) &&
         follow_client_->wait_for_action_server(timeout);
}

void RegulatedNavigator::startPlanning(const bool replanning)
{
  if (!active_ || task_.type == TaskType::NONE || planning_active_) {
    return;
  }
  if (!dependenciesReady()) {
    failTask("规划、平滑或控制 Action Server 不可用");
    return;
  }

  planning_active_ = true;
  updating_path_ = replanning;
  task_.state = replanning ? NavigationState::REPLANNING : NavigationState::PLANNING;
  task_.last_replan_time = now();
  const auto generation = task_.generation;
  const auto sequence = ++plan_sequence_;

  if (task_.type == TaskType::THROUGH_POSES) {
    ComputePathThroughPoses::Goal goal;
    goal.goals = task_.goals;
    goal.planner_id = planning_module_.plannerId();
    goal.use_start = false;
    auto options = rclcpp_action::Client<ComputePathThroughPoses>::SendGoalOptions();
    options.goal_response_callback = [this, generation, sequence](auto handle) {
        if (isCurrentPlan(generation, sequence)) {
          active_compute_poses_goal_ = handle;
          if (!handle) {
            planning_active_ = false;
            handlePlanningFailure("多点规划 Goal 被拒绝");
          }
        }
      };
    options.result_callback = [this, generation, sequence](const auto & result) {
        if (!isCurrentPlan(generation, sequence)) {return;}
        planning_active_ = false;
        active_compute_poses_goal_.reset();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
          result.result->path.poses.empty())
        {
          handlePlanningFailure("多点规划失败或返回空路径");
          return;
        }
        onPathReady(result.result->path);
      };
    compute_poses_client_->async_send_goal(goal, options);
  } else {
    ComputePathToPose::Goal goal;
    goal.goal = task_.goal;
    goal.planner_id = planning_module_.plannerId();
    goal.use_start = false;
    auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    options.goal_response_callback = [this, generation, sequence](auto handle) {
        if (isCurrentPlan(generation, sequence)) {
          active_compute_pose_goal_ = handle;
          if (!handle) {
            planning_active_ = false;
            handlePlanningFailure("单点规划 Goal 被拒绝");
          }
        }
      };
    options.result_callback = [this, generation, sequence](const auto & result) {
        if (!isCurrentPlan(generation, sequence)) {return;}
        planning_active_ = false;
        active_compute_pose_goal_.reset();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
          result.result->path.poses.empty())
        {
          handlePlanningFailure("单点规划失败或返回空路径");
          return;
        }
        onPathReady(result.result->path);
      };
    compute_pose_client_->async_send_goal(goal, options);
  }
}

bool RegulatedNavigator::isCurrentPlan(
  const uint64_t generation, const uint64_t sequence) const
{
  return generation == task_.generation && sequence == plan_sequence_;
}

void RegulatedNavigator::onPathReady(const nav_msgs::msg::Path & path)
{
  task_.consecutive_planning_failures = 0;
  pending_raw_path_ = path;
  if (!planning_module_.useSmoother()) {
    sendFollowPath(path);
    return;
  }

  task_.state = NavigationState::SMOOTHING;
  const auto generation = task_.generation;
  const auto sequence = plan_sequence_;
  SmoothPath::Goal goal;
  goal.path = path;
  goal.smoother_id = planning_module_.smootherId();
  goal.max_smoothing_duration = navigation_utils::durationFromSeconds(smoothing_duration_);
  goal.check_for_collisions = check_smoother_collisions_;
  auto options = rclcpp_action::Client<SmoothPath>::SendGoalOptions();
  options.goal_response_callback = [this, generation, sequence](auto handle) {
      if (isCurrentPlan(generation, sequence)) {
        active_smooth_goal_ = handle;
        if (!handle) {
          RCLCPP_WARN(get_logger(), "平滑 Goal 被拒绝，回退使用原始路径");
          sendFollowPath(pending_raw_path_);
        }
      }
    };
  options.result_callback = [this, generation, sequence](const auto & result) {
      if (!isCurrentPlan(generation, sequence)) {return;}
      active_smooth_goal_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        !result.result->path.poses.empty())
      {
        sendFollowPath(result.result->path);
      } else {
        RCLCPP_WARN(get_logger(), "路径平滑失败，回退使用原始规划路径");
        sendFollowPath(pending_raw_path_);
      }
    };
  smooth_client_->async_send_goal(goal, options);
}

void RegulatedNavigator::handlePlanningFailure(const std::string & reason)
{
  task_.last_error = reason;
  ++task_.consecutive_planning_failures;
  if (updating_path_ && active_follow_goal_ &&
    task_.consecutive_planning_failures < planning_module_.maxFailures())
  {
    task_.state = NavigationState::CONTROLLING;
    RCLCPP_WARN(get_logger(), "%s，继续跟随旧路径", reason.c_str());
    return;
  }
  startRecovery(reason);
}

}  // namespace nav2_regulated_modules
