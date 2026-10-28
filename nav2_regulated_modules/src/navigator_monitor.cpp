// 周期检查任务取消、定位、进度与恢复。ROS 时间用于导航计时，collision 快照内部另用单调接收时间。
// 固定路径 collision 等待暂停无进展计时，但不关闭取消与定位丢失处理；TF 查询成功不额外证明定位精度。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
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
  // 自主模式还校验 TF 源时间戳；固定路径保留原先的查询成功计时语义。
  if (!lookupCurrentPose(current_pose))
  {
    if (operation_mode_ == NavigationMode::AUTONOMOUS)
    {
      localization_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
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
      if (operation_mode_ == NavigationMode::AUTONOMOUS)
      {
        ++task_.localization_recovery_count;
      }
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
    else if (operation_mode_ == NavigationMode::AUTONOMOUS)
    {
      const double goal_distance = navigation_utils::poseDistance(current_pose, task_.goal);
      const bool terminal = autonomous_terminal_progress_enabled_ ? detail::autonomousTerminalProgressHeading(goal_distance, autonomous_terminal_xy_tolerance_, autonomous_progress_terminal_) : (autonomous_progress_terminal_ ? goal_distance <= 0.35 : goal_distance <= 0.25);
      double heading = navigation_utils::yawFromPose(task_.goal);
      if (!terminal && autonomous_terminal_progress_enabled_ && goal_distance <= autonomous_terminal_distance_ + 0.1)
      {
        heading = std::atan2(task_.goal.pose.position.y - current_pose.pose.position.y, task_.goal.pose.position.x - current_pose.pose.position.x);
      }
      else if (!terminal && !task_.active_path.poses.empty())
      {
        std::size_t nearest = 0;
        double nearest_distance = navigation_utils::poseDistance(current_pose, task_.active_path.poses.front());
        for (std::size_t index = 1; index < task_.active_path.poses.size(); ++index)
        {
          const double distance = navigation_utils::poseDistance(current_pose, task_.active_path.poses[index]);
          if (distance < nearest_distance)
          {
            nearest = index;
            nearest_distance = distance;
          }
        }
        std::size_t ahead = nearest;
        double arc = 0.0;
        while (ahead + 1 < task_.active_path.poses.size() && arc < 0.5)
        {
          arc += navigation_utils::poseDistance(task_.active_path.poses[ahead], task_.active_path.poses[ahead + 1]);
          ++ahead;
        }
        const auto & point = task_.active_path.poses[ahead].pose.position;
        heading = std::atan2(point.y - current_pose.pose.position.y, point.x - current_pose.pose.position.x);
      }
      const double error = std::abs(navigation_utils::normalizeAngle(heading - navigation_utils::yawFromPose(current_pose)));
      const bool first = task_.last_progress_pose.header.frame_id.empty() || !autonomous_progress_heading_error_ || terminal != autonomous_progress_terminal_;
      if (!first && std::abs(navigation_utils::normalizeAngle(heading - autonomous_progress_reference_heading_)) > 0.35)
      {
        // 换路径可改变朝向参照，但不刷新进度时钟。
        autonomous_progress_heading_error_ = error;
        autonomous_progress_reference_heading_ = heading;
      }
      const bool progressed = first || detail::autonomousProgress(navigation_utils::poseDistance(current_pose, task_.last_progress_pose), *autonomous_progress_heading_error_ - error, autonomous_progress_goal_distance_ - goal_distance, goal_distance, progress_min_translation_, autonomous_progress_min_rotation_, autonomous_goal_progress_distance_);
      if (progressed)
      {
        task_.last_progress_pose = current_pose;
        task_.last_progress_time = now();
        autonomous_progress_heading_error_ = error;
        autonomous_progress_goal_distance_ = goal_distance;
        autonomous_progress_terminal_ = terminal;
        autonomous_progress_reference_heading_ = heading;
      }
      else if ((now() - task_.last_progress_time).seconds() >= control_module_.progressTimeout())
      {
        startRecovery("自主路径位置、转向与末端误差均无进展");
        return;
      }
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
  if (operation_mode_ == NavigationMode::AUTONOMOUS && task_.state == NavigationState::CONTROLLING && navigation_utils::poseDistance(current_pose, task_.goal) > 0.3 && !planning_active_ && (now() - task_.last_replan_time).seconds() >= planning_module_.replanPeriod())
  {
    startPlanning(true);
  }
}

// 自主模式按源时间戳拒绝过期／未来 TF；固定路径保留原有查询逻辑。
bool RegulatedNavigator::lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  try
  {
    const auto transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, 50ms);
    if (operation_mode_ == NavigationMode::AUTONOMOUS)
    {
      const auto transform_time = rclcpp::Time(transform.header.stamp, get_clock()->get_clock_type());
      if (transform_time.nanoseconds() <= 0 || !detail::freshAutonomousTransform((now() - transform_time).seconds(), localization_timeout_))
      {
        return false;
      }
      last_valid_tf_time_ = transform_time;
    }
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    if (operation_mode_ != NavigationMode::AUTONOMOUS)
    {
      last_valid_tf_time_ = now();
    }
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
