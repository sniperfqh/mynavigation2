// FollowPath 下发、反馈转发和速度链观察。每次请求记录任务 generation 与 follow_sequence。
// 回调必须同时匹配两个标识才能改当前状态；向 cmd_vel_nav 发零速是停车请求，不是轮速停稳确认。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

namespace nav2_regulated_modules
{

// 按任务类别选择控制器与 GoalChecker，再异步发送完整 Path。
// FollowPath 的响应、反馈和结果均校验 generation／follow_sequence；固定路径绝对限速在接受后发布。
void RegulatedNavigator::sendFollowPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || task_.type == TaskType::NONE)
  {
    handlePlanningFailure("控制路径为空");
    return;
  }
  if (operation_mode_ == NavigationMode::AUTONOMOUS)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!lookupCurrentPose(current_pose))
    {
      cancelSubGoals(true);
      stopRobot();
      task_.state = NavigationState::LOCALIZATION_LOST;
      localization_lost_time_ = now();
      localization_stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      LOG_WARN("自主路径执行前定位未就绪，保持停车并等待新鲜 TF");
      return;
    }
  }
  task_.active_path = path;
  if (operation_mode_ == NavigationMode::AUTONOMOUS)
  {
    task_.distance_remaining = 0.0;
    current_speed_ = 0.0;
  }
  task_.state = NavigationState::CONTROLLING;
  const auto generation = task_.generation;
  const auto sequence = ++follow_sequence_;
  const bool replacing_path = active_follow_goal_ != nullptr;
  if (operation_mode_ == NavigationMode::AUTONOMOUS && !replacing_path)
  {
    // 新任务或真正恢复后的首次执行重建基准；周期换路径继续累计卡死时间。
    task_.last_progress_pose = geometry_msgs::msg::PoseStamped();
    task_.last_progress_time = now();
    autonomous_progress_heading_error_.reset();
  }
  const std::string & controller_id = task_.type == TaskType::NAVIGATION_SERVICE ? fixed_path_controller_id_ : control_module_.controllerId();
  const std::string & goal_checker_id = task_.type == TaskType::NAVIGATION_SERVICE ? fixed_path_goal_checker_id_ : control_module_.goalCheckerId();
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;
  if (replacing_path)
  {
    LOG_DEBUG("更新 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}", generation, sequence, path.poses.size(), controller_id);
  }
  else
  {
    LOG_INFO("下发 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}，goal_checker_id={}", generation, sequence, path.poses.size(), controller_id, goal_checker_id);
  }
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  // 发送请求时还没有可靠的已接受句柄；响应到达后先校验代次，才能保存为当前跟踪目标。
  options.goal_response_callback = [this, generation, sequence](auto handle)
  {
    if (isCurrentFollow(generation, sequence))
    {
      active_follow_goal_ = handle;
      if (!handle)
      {
        startRecovery("控制 Goal 被拒绝");
        return;
      }
      if (task_.type == TaskType::NAVIGATION_SERVICE && speed_limit_pub_)
      {
        nav2_msgs::msg::SpeedLimit speed_limit;
        speed_limit.percentage = false;
        speed_limit.speed_limit = task_.requested_speed;
        speed_limit_pub_->publish(speed_limit);
      }
    }
  };
  options.feedback_callback = [this, generation, sequence](auto, const std::shared_ptr<const FollowPath::Feedback> controller_feedback)
  {
    if (isCurrentFollow(generation, sequence))
    {
      task_.distance_remaining = static_cast<double>(controller_feedback->distance_to_goal);
      current_speed_ = controller_feedback->speed;
      if (task_.total_path_length > 0.0)
      {
        const double completed_ratio = (task_.total_path_length - task_.distance_remaining) / task_.total_path_length;
        task_.progress = std::max(task_.progress, static_cast<float>(std::clamp(completed_ratio, 0.0, 1.0)));
      }
      if (active_navigation_service_goal_)
      {
        auto feedback = std::make_shared<NavigationService::Feedback>();
        feedback->cur_task_id = task_.task_id;
        feedback->cur_seg_id = "";
        feedback->progress = task_.progress;
        active_navigation_service_goal_->publish_feedback(feedback);
      }
    }
  };
  // 下游结果只影响匹配当前 generation／sequence 的任务；迟到结果不能结束新任务。
  options.result_callback = [this, generation, sequence](const auto & result)
  {
    if (!isCurrentFollow(generation, sequence))
    {
      return;
    }
    active_follow_goal_.reset();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      succeedTask();
    }
    else if (task_.state != NavigationState::CANCELING)
    {
      startRecovery("控制器执行路径失败");
    }
  };
  follow_client_->async_send_goal(goal, options);
}

// 比较任务代次与跟踪轮次；同一任务换路径也会使旧 sequence 失效。
// 不能只比较 generation，否则周期重规划的旧跟踪结果可能干扰当前路径。
bool RegulatedNavigator::isCurrentFollow(const uint64_t generation, const uint64_t sequence) const
{
  return generation == task_.generation && sequence == follow_sequence_;
}

// 向速度链入口连续发布三帧零 Twist，增加停车请求被下游收到的机会。
// 不检查发布确认或实际轮速；若没有发布器直接返回，不承诺已经机械停稳。
void RegulatedNavigator::stopRobot()
{
  if (!stop_cmd_pub_)
  {
    return;
  }
  geometry_msgs::msg::Twist stop;
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  LOG_DEBUG("已向速度链入口连续发布 3 帧零速度");
}

// 在 velocity_mutex 保护下缓存控制器最新 Twist 与曾接收标记。
// 缓存用于日志，不代表与平滑器反馈来自严格同一控制周期。
void RegulatedNavigator::onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_controller_velocity_ = *velocity;
  has_controller_velocity_ = true;
}

// 记录平滑后 Twist 供诊断；不再对速度限幅或二次发布。
// 旧快照可持续存在，消费者应另行理解采样新鲜度。
void RegulatedNavigator::onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_smoothed_velocity_ = *velocity;
  has_smoothed_velocity_ = true;
}

// 记录 Odometry 的完整最新样本，仅用于速度链观察。
// 这里不会将里程计位置转换为地图定位。
void RegulatedNavigator::onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_velocity_odometry_ = *odometry;
  has_velocity_odometry_ = true;
}

// 先在锁内复制三路快照，再在锁外写日志，减少回调持锁时间。
// 没有任何样本时不打印；独立订阅的 latest 值不能当作同步传感器帧。
void RegulatedNavigator::logVelocityChain()
{
  if (!active_)
  {
    return;
  }
  geometry_msgs::msg::Twist controller_velocity;
  geometry_msgs::msg::Twist smoothed_velocity;
  nav_msgs::msg::Odometry velocity_odometry;
  bool has_controller_velocity;
  bool has_smoothed_velocity;
  bool has_velocity_odometry;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    controller_velocity = latest_controller_velocity_;
    smoothed_velocity = latest_smoothed_velocity_;
    velocity_odometry = latest_velocity_odometry_;
    has_controller_velocity = has_controller_velocity_;
    has_smoothed_velocity = has_smoothed_velocity_;
    has_velocity_odometry = has_velocity_odometry_;
  }
  if (!has_controller_velocity && !has_smoothed_velocity && !has_velocity_odometry)
  {
    return;
  }
  LOG_DEBUG("关键速度链：反馈 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 控制器输出/平滑器输入 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 平滑器输出 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}]", velocity_odom_topic_, velocity_odometry.twist.twist.linear.x, velocity_odometry.twist.twist.angular.z, has_velocity_odometry, controller_cmd_vel_topic_, controller_velocity.linear.x, controller_velocity.angular.z, has_controller_velocity, smoothed_cmd_vel_topic_, smoothed_velocity.linear.x, smoothed_velocity.angular.z, has_smoothed_velocity);
}

}
// namespace nav2_regulated_modules
