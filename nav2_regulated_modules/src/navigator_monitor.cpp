#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace nav2_regulated_modules
{

void RegulatedNavigator::startRecovery(const std::string & reason) {
  if (task_.type == TaskType::NONE) {return;}
  if (task_.recovery_count >= max_recovery_rounds_) {
    failTask(reason + "，恢复次数已用尽");
    return;
  }
  task_.state = NavigationState::CLEARING_COSTMAP;
  task_.last_error = reason;
  ++task_.recovery_count;
  cancelSubGoals(true);
  stopRobot();

  // 中文注释：删除 nav2_behaviors 后，恢复仅清理双 Costmap 并由定时器等待重新规划。
  auto request = std::make_shared<ClearCostmap::Request>();
  if (clear_local_client_->service_is_ready()) {
    clear_local_client_->async_send_request(request);
  }
  if (clear_global_client_->service_is_ready()) {
    clear_global_client_->async_send_request(request);
  }
  recovery_ready_time_ = now() + rclcpp::Duration::from_seconds(costmap_wait_duration_);
}

void RegulatedNavigator::monitorTask() {
  if (!active_ || task_.type == TaskType::NONE) {return;}
  if (cancel_requested_) {
    cancelTask("收到外层导航取消请求");
    return;
  }
  // 中文注释：代价地图清理后非阻塞等待更新，再从当前位置重新规划。
  if (task_.state == NavigationState::CLEARING_COSTMAP) {
    if (now() >= recovery_ready_time_) {
      startPlanning(false);
    }
    return;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  if (!lookupCurrentPose(current_pose)) {
    if ((now() - last_valid_tf_time_).seconds() > localization_timeout_ &&
      task_.state != NavigationState::LOCALIZATION_LOST)
    {
      RCLCPP_ERROR(get_logger(), "自研定位 map->base_link 超时，取消控制并停车");
      cancelSubGoals(true);
      stopRobot();
      task_.state = NavigationState::LOCALIZATION_LOST;
      localization_lost_time_ = now();
      localization_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    return;
  }

  if (task_.state == NavigationState::LOCALIZATION_LOST) {
    if ((now() - localization_lost_time_).seconds() > localization_recovery_timeout_) {
      failTask("自研定位恢复超时");
      return;
    }
    if (localization_stable_since_.nanoseconds() == 0) {
      localization_stable_since_ = now();
    }
    if ((now() - localization_stable_since_).seconds() >= localization_stable_duration_) {
      RCLCPP_INFO(get_logger(), "自研定位连续稳定，重新规划当前任务");
      task_.state = NavigationState::PLANNING;
      startPlanning(false);
    }
    return;
  }

  if (has_last_pose_) {
    const double translation_jump = navigation_utils::poseDistance(current_pose, last_pose_);
    const double rotation_jump = std::abs(navigation_utils::normalizeAngle(navigation_utils::yawFromPose(current_pose) - navigation_utils::yawFromPose(last_pose_)));
    if (translation_jump > max_translation_jump_ || rotation_jump > max_rotation_jump_) {
      RCLCPP_WARN(get_logger(), "检测到定位跳变，废弃旧路径并重新规划");
      cancelSubGoals(true);
      stopRobot();
      last_pose_ = current_pose;
      startPlanning(false);
      return;
    }
  }
  last_pose_ = current_pose;
  has_last_pose_ = true;

  // 中文注释：直接使用 map->base_link 位移判断控制进展，不依赖 odom TF 或里程计消息。
  if (task_.state == NavigationState::CONTROLLING) {
    if (task_.last_progress_pose.header.frame_id.empty() ||
      navigation_utils::poseDistance(current_pose, task_.last_progress_pose) >=
      progress_min_translation_)
    {
      task_.last_progress_pose = current_pose;
      task_.last_progress_time = now();
    } else if ((now() - task_.last_progress_time).seconds() >=
      control_module_.progressTimeout())
    {
      startRecovery("控制期间 map->base_link 位姿长时间无进展");
      return;
    }
  }

  updatePassedGoals(current_pose);
  if (task_.state == NavigationState::CONTROLLING && !planning_active_ &&
    (now() - task_.last_replan_time).seconds() >= planning_module_.replanPeriod())
  {
    startPlanning(true);
  }
}

bool RegulatedNavigator::lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose) {
  try {
    const auto transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, 50ms);
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    last_valid_tf_time_ = now();
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void RegulatedNavigator::updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose) {
  if (task_.type != TaskType::THROUGH_POSES || task_.goals.size() <= 1) {return;}
  while (task_.goals.size() > 1 &&
    navigation_utils::poseDistance(current_pose, task_.goals.front()) <= passed_goal_radius_)
  {
    task_.goals.erase(task_.goals.begin());
  }
}

}  // namespace nav2_regulated_modules
