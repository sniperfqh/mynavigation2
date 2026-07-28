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
    RCLCPP_ERROR(get_logger(), "固定路径至少需要两个有效路径点，并且必须提供坐标系");
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
      RCLCPP_ERROR(get_logger(), "固定路径包含非有限坐标或无效四元数");
      return std::nullopt;
    }

    try {
      auto target = tf_buffer_->transform(source, global_frame_, tf2::durationFromSec(0.1));
      target.header.frame_id = global_frame_;
      target.header.stamp = output.header.stamp;
      output.poses.push_back(target);
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(get_logger(), "固定路径无法转换到 %s：%s", global_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  return output;
}

// 中文注释：接收上游完整路径；首次消息创建固定路径任务，后续消息整路径更新控制器。
void RegulatedNavigator::onFixedPath(const nav_msgs::msg::Path::SharedPtr message) {
  if (!active_ || operation_mode_ != NavigationMode::FIXED_PATH) {
    return;
  }

  auto prepared_path = prepareFixedPath(*message);
  if (!prepared_path) {
    stopRobot();
    return;
  }

  if (!follow_client_->wait_for_action_server(
      std::chrono::duration<double>(server_timeout_)))
  {
    RCLCPP_ERROR(get_logger(), "FollowPath Action Server 不可用");
    stopRobot();
    return;
  }

  if (task_.type != TaskType::NONE && task_.type != TaskType::FIXED_PATH) {
    preemptCurrentTask();
  }
  if (task_.type == TaskType::NONE) {
    task_ = NavigationTask();
    task_.generation = ++task_generation_;
    task_.type = TaskType::FIXED_PATH;
    task_.start_time = now();
  }

  task_.goal = prepared_path->poses.back();
  task_.active_path = *prepared_path;
  task_.last_progress_time = now();
  task_.last_progress_pose = geometry_msgs::msg::PoseStamped();

  if (task_.state == NavigationState::LOCALIZATION_LOST) {
    RCLCPP_WARN(get_logger(), "定位尚未恢复，已保存最新固定路径并继续保持停车");
    return;
  }

  // 中文注释：不取消旧 FollowPath；Controller Server 接受新 Goal 后原子替换当前路径。
  sendFollowPath(*prepared_path);
  RCLCPP_INFO(get_logger(), "固定路径已更新，共 %zu 个路径点", prepared_path->poses.size());
}

// 中文注释：恢复固定路径时重发已保存 Path；自主任务则重新进入规划链。
void RegulatedNavigator::resumeCurrentTask() {
  if (task_.type == TaskType::FIXED_PATH) {
    if (task_.active_path.poses.empty()) {
      failTask("没有可恢复的固定路径");
      return;
    }
    sendFollowPath(task_.active_path);
    return;
  }
  startPlanning(false);
}

}  // namespace nav2_regulated_modules
