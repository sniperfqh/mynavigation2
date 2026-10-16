// 周期检查任务取消、定位、进度与恢复。ROS 时间用于导航计时，collision 快照内部另用单调接收时间。
// 固定路径 collision 等待暂停无进展计时，但不关闭取消与定位丢失处理；TF 查询成功不额外证明定位精度。

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

// 在恢复轮次允许时取消子 Goal、发送零速并请求清空代价地图。
// 清图请求异步发送；等待时间到后再尝试恢复，不在此确认清图或实际停车完成。
void RegulatedNavigator::startRecovery(const std::string & reason)
{
  if (task_.type == TaskType::NONE)
  {
    return;
  }
  if (task_.recovery_count >= max_recovery_rounds_)
  {
    failTask(reason + "，恢复次数已用尽");
    return;
  }
  task_.state = NavigationState::CLEARING_COSTMAP;
  ++task_.recovery_count;
  LOG_WARN("进入导航恢复，generation={}，round={}/{}，reason={}", task_.generation, task_.recovery_count, max_recovery_rounds_, reason);
  cancelSubGoals(true);
  stopRobot();

  auto request = std::make_shared<ClearCostmap::Request>();
  if (clear_local_client_->service_is_ready())
  {
    clear_local_client_->async_send_request(request);
  }
  if (clear_global_client_->service_is_ready())
  {
    clear_global_client_->async_send_request(request);
  }
  recovery_ready_time_ = now() + rclcpp::Duration::from_seconds(costmap_wait_duration_);
}

// 依次处理取消、清图等待、TF 丢失／恢复、可选跳变、进度及自主重规划。
// collision 仅暂停固定路径无进展计时，不跳过前面的取消与定位处理。
void RegulatedNavigator::monitorTask()
{
  if (!active_ || task_.type == TaskType::NONE)
  {
    return;
  }
  if (cancel_requested_)
  {
    cancelTask("收到外层导航取消请求");
    return;
  }
  if (task_.state == NavigationState::CLEARING_COSTMAP)
  {
    if (now() >= recovery_ready_time_)
    {
      resumeCurrentTask();
    }
    return;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  // TF 丢失与位姿突跳分开处理：持续无 TF 必停，突跳检测由独立开关控制。
  // 当前位置查询失败进入丢失计时；成功查询后的 last_valid_tf_time 是本机当前 ROS 时间，不是 TF 源时间。
  if (!lookupCurrentPose(current_pose))
  {
    if ((now() - last_valid_tf_time_).seconds() > localization_timeout_ && task_.state != NavigationState::LOCALIZATION_LOST)
    {
      LOG_ERROR("自研定位 map->base_link 超时，取消控制并停车");
      cancelSubGoals(true);
      stopRobot();
      task_.state = NavigationState::LOCALIZATION_LOST;
      localization_lost_time_ = now();
      localization_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    return;
  }

  // 定位恢复需连续稳定一段时间，避免单帧恢复后立即重启旧控制。
  if (task_.state == NavigationState::LOCALIZATION_LOST)
  {
    if ((now() - localization_lost_time_).seconds() > localization_recovery_timeout_)
    {
      failTask("自研定位恢复超时");
      return;
    }
    if (localization_stable_since_.nanoseconds() == 0)
    {
      localization_stable_since_ = now();
    }
    if ((now() - localization_stable_since_).seconds() >= localization_stable_duration_)
    {
      LOG_INFO("自研定位连续稳定，重新规划当前任务");
      task_.state = NavigationState::PLANNING;
      resumeCurrentTask();
    }
    return;
  }

  if (enable_localization_jump_detection_ && has_last_pose_)
  {
    const double translation_jump = navigation_utils::poseDistance(current_pose, last_pose_);
    const double rotation_jump = std::abs(navigation_utils::normalizeAngle(navigation_utils::yawFromPose(current_pose) - navigation_utils::yawFromPose(last_pose_)));
    if (translation_jump > max_translation_jump_ || rotation_jump > max_rotation_jump_)
    {
      LOG_WARN("检测到定位跳变，废弃旧路径并重新规划");
      cancelSubGoals(true);
      stopRobot();
      last_pose_ = current_pose;
      resumeCurrentTask();
      return;
    }
  }
  last_pose_ = current_pose;
  has_last_pose_ = true;

  // 控制期间仅以足够的 map 坐标位移刷新进度时间，静止过久进入恢复。
  if (task_.state == NavigationState::CONTROLLING)
  {
    // 只有固定路径任务受此进度暂停影响；每周期重建进度位置／时间，恢复后不把停车时长算作卡住。
    const bool collision_wait = collision_state_enabled_ && task_.type == TaskType::NAVIGATION_SERVICE && collision_state_.snapshot().stopped();
    if (collision_wait || collision_progress_paused_)
    {
      if (collision_progress_paused_ != collision_wait)
      {
        LOG_INFO("collision 固定路径无进展计时：paused={}，generation={}，继续保留任务与原路径", collision_wait, task_.generation);
      }
      task_.last_progress_pose = current_pose;
      task_.last_progress_time = now();
      collision_progress_paused_ = collision_wait;
    }
    else if (task_.last_progress_pose.header.frame_id.empty() || navigation_utils::poseDistance(current_pose, task_.last_progress_pose) >= progress_min_translation_)
    {
      task_.last_progress_pose = current_pose;
      task_.last_progress_time = now();
    }
    else if ((now() - task_.last_progress_time).seconds() >= control_module_.progressTimeout())
    {
      startRecovery("控制期间 map->base_link 位姿长时间无进展");
      return;
    }
  }

  updatePassedGoals(current_pose);
  if (operation_mode_ == NavigationMode::AUTONOMOUS && task_.state == NavigationState::CONTROLLING && !planning_active_ && (now() - task_.last_replan_time).seconds() >= planning_module_.replanPeriod())
  {
    startPlanning(true);
  }
}

// 尝试获取最近 map→base_link 变换，成功后更新 last_valid_tf_time。
// 当前实现没有额外按消息时间戳计算 TF 年龄；查询成功不能当作独立定位新鲜度／精度验收。
bool RegulatedNavigator::lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  try
  {
    const auto transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, 50ms);
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    last_valid_tf_time_ = now();
    return true;
  }
  catch (const tf2::TransformException &)
  {
    return false;
  }
}

// 多点任务中按 passed_goal_radius（m）移除已接近的前置目标，至少保留末点。
// 这是目标队列维护，不是 FollowPath 成功判定。
void RegulatedNavigator::updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (task_.type != TaskType::THROUGH_POSES || task_.goals.size() <= 1)
  {
    return;
  }
  const auto previous_count = task_.goals.size();
  while (task_.goals.size() > 1 && navigation_utils::poseDistance(current_pose, task_.goals.front()) <= passed_goal_radius_)
  {
    task_.goals.erase(task_.goals.begin());
  }
  if (task_.goals.size() != previous_count)
  {
    LOG_INFO("多点导航已通过前置目标，generation={}，剩余目标数={}", task_.generation, task_.goals.size());
  }
}

}
// namespace nav2_regulated_modules
