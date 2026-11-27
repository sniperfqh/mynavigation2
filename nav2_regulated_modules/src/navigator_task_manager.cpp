// 统一完成成功、失败、取消和任务状态清理。子 Action 请求数与新鲜停稳检查由交接监控继续跟踪。

#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"

#include <memory>
#include <string>

namespace nav2_regulated_modules
{

// 对已持有句柄发送 async_cancel_goal，再重置本地句柄；可选递增轮次使旧回调失效。
// 未等待取消响应／结果，不能称为下游全部终止；尚未拿到句柄的请求也不由这些分支直接取消。
void RegulatedNavigator::cancelSubGoals(const bool invalidate_callbacks)
{
  const bool had_compute_pose = active_compute_pose_goal_ != nullptr;
  const bool had_follow = active_follow_goal_ != nullptr;
  if (invalidate_callbacks)
  {
    ++plan_sequence_;
    ++follow_sequence_;
  }
  planning_active_ = false;
  // 本函数只取消当前已持有的句柄；异步取消发送后立即 reset，不阻塞等待下游结果。
  if (active_compute_pose_goal_)
  {
    compute_pose_client_->async_cancel_goal(active_compute_pose_goal_);
    active_compute_pose_goal_.reset();
  }
  if (active_follow_goal_)
  {
    followClient()->async_cancel_goal(active_follow_goal_);
    active_follow_goal_.reset();
  }
  if (had_compute_pose || had_follow)
  {
    LOG_DEBUG("取消子 Goal，compute_pose={}，follow={}，invalidate_callbacks={}", had_compute_pose, had_follow, invalidate_callbacks);
  }
}

// 进入取消状态、废弃旧回调、请求停车，再按上层句柄状态返回 canceled 或 abort。
// 只有客户端正在取消的句柄报告 canceled；一般内部取消走 abort，随后清空任务。
void RegulatedNavigator::cancelTask(const std::string & reason)
{
  if (task_.type == TaskType::NONE)
  {
    return;
  }
  if (active_)
  {
    if (task_.state != NavigationState::CANCELING)
    {
      task_.state = NavigationState::CANCELING;
      cancellation_reason_ = reason;
      stopping_since_ = std::chrono::steady_clock::now();
      cancelSubGoals(true);
      stopRobot();
    }
    return;
  }
  cancellation_reason_ = reason;
  task_.state = NavigationState::CANCELING;
  cancelSubGoals(true);
  stopRobot();
  finishCancellation();
}

// 将已取消任务的上层句柄报告为 CANCELED 或 ABORTED，再清理任务快照。正常激活期间由交接监控调用。
void RegulatedNavigator::finishCancellation()
{
  const auto generation = task_.generation;
  ++task_generation_;
  auto pose_result = std::make_shared<NavigateToPose::Result>();
  if (active_pose_goal_)
  {
    if (active_pose_goal_->is_canceling())
    {
      active_pose_goal_->canceled(pose_result);
    }
    else
    {
      active_pose_goal_->abort(pose_result);
    }
    active_pose_goal_.reset();
  }
  if (active_navigation_service_goal_)
  {
    auto navigation_service_result = std::make_shared<NavigationService::Result>();
    navigation_service_result->finish = false;
    if (active_navigation_service_goal_->is_canceling())
    {
      active_navigation_service_goal_->canceled(navigation_service_result);
    }
    else
    {
      active_navigation_service_goal_->abort(navigation_service_result);
    }
    active_navigation_service_goal_.reset();
  }
  LOG_WARN("导航任务已取消并完成收口，generation={}，reason={}", generation, cancellation_reason_);
  resetTask();
}

// 向当前对应 Action 报告成功；固定路径另发 progress=1、finish=true 后清理任务。
// 上层成功由下游结果触发，不直接证明外部精度验收、定位覆盖或真实底盘制动距离合格。
void RegulatedNavigator::succeedTask()
{
  const auto generation = task_.generation;
  if (((active_pose_goal_ && active_pose_goal_->is_canceling()) ||
    (active_navigation_service_goal_ && active_navigation_service_goal_->is_canceling())))
  {
    cancelTask("收到外层导航取消请求");
    return;
  }
  const auto elapsed = (now() - task_.start_time).seconds();
  task_.distance_remaining = 0.0;
  task_.progress = 1.0F;
  task_.state = NavigationState::SUCCEEDED;
  if (active_pose_goal_)
  {
    active_pose_goal_->succeed(std::make_shared<NavigateToPose::Result>());
    active_pose_goal_.reset();
  }
  if (active_navigation_service_goal_)
  {
    auto feedback = std::make_shared<NavigationService::Feedback>();
    feedback->cur_task_id = task_.task_id;
    feedback->cur_seg_id = "";
    feedback->progress = task_.progress;
    active_navigation_service_goal_->publish_feedback(feedback);
    auto navigation_service_result = std::make_shared<NavigationService::Result>();
    navigation_service_result->finish = true;
    active_navigation_service_goal_->succeed(navigation_service_result);
    active_navigation_service_goal_.reset();
  }
    LOG_INFO("导航任务执行成功，generation={}，耗时={:.3f}s，恢复次数={}", generation, elapsed, task_.recovery_count);
  resetTask();
}

// 废弃子 Goal 并向现有上层句柄报告失败，再清理任务快照。
// 失败流程请求 stopRobot 并取消子任务，后续启动仍需经过交接监控的请求收口与实测停稳检查。
void RegulatedNavigator::failTask(const std::string & reason)
{
  if (((active_pose_goal_ && active_pose_goal_->is_canceling()) ||
    (active_navigation_service_goal_ && active_navigation_service_goal_->is_canceling())))
  {
    cancelTask(reason);
    return;
  }
    stopRobot();
  const auto generation = task_.generation;
  task_.state = NavigationState::FAILED;
  cancelSubGoals(true);
  if (active_pose_goal_)
  {
    active_pose_goal_->abort(std::make_shared<NavigateToPose::Result>());
    active_pose_goal_.reset();
  }
  if (active_navigation_service_goal_)
  {
    auto navigation_service_result = std::make_shared<NavigationService::Result>();
    navigation_service_result->finish = false;
    active_navigation_service_goal_->abort(navigation_service_result);
    active_navigation_service_goal_.reset();
  }
  LOG_ERROR("导航任务失败，generation={}，恢复次数={}，reason={}", generation, task_.recovery_count, reason);
  resetTask();
}

// 清除任务与诊断历史；固定路径结束时发布 0 SpeedLimit 清除该任务基准。
// 0 在插件接口中的语义与 collision_task_speed 清理需区分，不代表独立停车命令。
void RegulatedNavigator::resetTask()
{
  frozen_path_ = nav_msgs::msg::Path();
  frozen_segments_.clear();
  frozen_plan_requested_ = false;
  frozen_follow_inflight_ = false;
  frozen_localization_wait_ = false;
  if (task_.type == TaskType::TO_POSE)
  {
    frozen_soft_pause_since_.reset();
    frozen_localization_reason_.clear();
  }
  frozen_stable_since_.reset();
  frozen_obstacle_since_ = -1.0;
  frozen_inhibited_since_.reset();
  frozen_phase_.clear();
  collision_progress_paused_ = false;
  // 固定路径结束清除该任务的限速基准；这条消息不代替 stopRobot 的 Twist 零命令。
    stopping_since_ = std::chrono::steady_clock::now();
  const auto speed_pub = speedPublisher();
  if ((task_.type == TaskType::NAVIGATION_SERVICE ||
    (task_.type == TaskType::TO_POSE)) && speed_pub)
  {
    nav2_msgs::msg::SpeedLimit speed_limit;
    speed_limit.percentage = false;
    speed_limit.speed_limit = 0.0;
    speed_pub->publish(speed_limit);
  }
  task_ = NavigationTask();
  task_.generation = task_generation_;
  planning_active_ = false;
  has_last_pose_ = false;
  current_speed_ = 0.0;
  cancel_requested_ = false;
  active_navigation_service_goal_.reset();
  // 充电状态机归零走统一内部取消：摘回 IDLE、停掉截止与消费定时器、作废在途充电跟随
  // 并请求控制器停止，迟到回调不再打扰新的任务；无活动充电目标时静默返回。
  // 常规互斥下导航任务收口时不应仍有充电目标，此处作为防御性兜底。
  cancelChargingTask("导航任务收口清理充电状态");
}

}
// namespace nav2_regulated_modules
