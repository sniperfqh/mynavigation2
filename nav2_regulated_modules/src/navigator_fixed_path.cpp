#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <optional>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 中文注释：校验固定 Path，并把每个 Pose 统一变换到导航全局坐标系后返回新路径。
std::optional<nav_msgs::msg::Path> RegulatedNavigator::prepareFixedPath(const nav_msgs::msg::Path & input) {
  if (input.poses.size() < 2 || input.header.frame_id.empty()) {
    LOG_ERROR("固定路径至少需要两个有效路径点，并且必须提供坐标系");
    return std::nullopt;
  }

  nav_msgs::msg::Path output;
  output.header.frame_id = global_frame_;
  output.header.stamp = now();
  output.poses.reserve(input.poses.size());

  for (const auto & input_pose : input.poses) {
    geometry_msgs::msg::PoseStamped source = input_pose;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = input.header.frame_id;
    }
    if (!navigation_utils::validPose(source)) {
      LOG_ERROR("固定路径包含非有限坐标或无效四元数");
      return std::nullopt;
    }

    try {
      auto target = tf_buffer_->transform(source, global_frame_, tf2::durationFromSec(0.1));
      target.header.frame_id = global_frame_;
      target.header.stamp = output.header.stamp;
      output.poses.push_back(target);
    } catch (const tf2::TransformException & error) {
      LOG_ERROR("固定路径无法转换到 {}：{}", global_frame_, error.what());
      return std::nullopt;
    }
  }

  return output;
}

// 中文注释：只在固定路径模式和激活状态接受结构有效的完整 Path，TF 转换留到 Accepted 回调处理。
rclcpp_action::GoalResponse RegulatedNavigator::handleFixedPathGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const FollowFixedPath::Goal> goal) {
  if (operation_mode_ != NavigationMode::FIXED_PATH) {
    LOG_WARN("当前模式不接受固定路径 Action Goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!active_ || goal->path.poses.size() < 2 || goal->path.header.frame_id.empty()) {
    LOG_WARN("拒绝固定路径 Goal：节点未激活、路径点少于两个或顶层坐标系为空");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto & input_pose : goal->path.poses) {
    auto pose = input_pose;
    if (pose.header.frame_id.empty()) {pose.header.frame_id = goal->path.header.frame_id;}
    if (!navigation_utils::validPose(pose)) {
      LOG_WARN("拒绝固定路径 Goal：路径包含非有限坐标或无效四元数");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 中文注释：固定路径取消只设置请求标志，避免在 Action 回调中重入下游取消和任务复位。
rclcpp_action::CancelResponse RegulatedNavigator::handleFixedPathCancel(const std::shared_ptr<FixedPathHandle> goal) {
  if (goal == active_fixed_path_goal_) {
    cancel_requested_ = true;
    LOG_DEBUG("收到固定路径取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 中文注释：先完成新路径变换和下游可用性检查，再抢占旧任务并启动新的固定路径 Action。
void RegulatedNavigator::handleFixedPathAccepted(const std::shared_ptr<FixedPathHandle> goal) {
  const auto prepared_path = prepareFixedPath(goal->get_goal()->path);
  if (!prepared_path) {
    auto result = std::make_shared<FollowFixedPath::Result>();
    result->success = false;
    goal->abort(result);
    LOG_ERROR("固定路径 Goal 已接受但路径坐标变换失败");
    return;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_))) {
    auto result = std::make_shared<FollowFixedPath::Result>();
    result->success = false;
    goal->abort(result);
    LOG_ERROR("固定路径 Goal 已接受但 FollowPath Action Server 不可用");
    return;
  }

  preemptCurrentTask();
  active_fixed_path_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::FIXED_PATH;
  task_.goal = prepared_path->poses.back();
  task_.active_path = *prepared_path;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  task_.last_progress_pose = geometry_msgs::msg::PoseStamped();
  LOG_INFO("接受固定路径 Action，generation={}，frame={}，路径点数={}", task_.generation, prepared_path->header.frame_id, prepared_path->poses.size());
  sendFollowPath(*prepared_path);
}

// 中文注释：恢复固定路径时重发已保存 Path；自主任务则重新进入规划链。
void RegulatedNavigator::resumeCurrentTask() {
  if (task_.type == TaskType::FIXED_PATH) {
    if (task_.active_path.poses.empty()) {
      failTask("没有可恢复的固定路径");
      return;
    }
    LOG_INFO("恢复固定路径任务，generation={}，路径点数={}", task_.generation, task_.active_path.poses.size());
    sendFollowPath(task_.active_path);
    return;
  }
  LOG_INFO("恢复自主导航任务，generation={}，重新进入规划链", task_.generation);
  startPlanning(false);
}

}  // namespace nav2_regulated_modules
