// 实现双 Action 交接的后台监控，包括生命周期轮询、在途请求计数、停稳确认和待执行 Goal 的启动。
#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav2_regulated_modules
{


// 按当前任务选择 FollowPath 客户端，单点任务进入 lattice_exec，业务固定路径进入 fixed_path_exec。
rclcpp_action::Client<RegulatedNavigator::FollowPath>::SharedPtr RegulatedNavigator::followClient() const
{
  return task_.type == TaskType::TO_POSE ? lattice_follow_client_ : follow_client_;
}

// 为当前执行通道选择 SpeedLimit 发布器，使两条通道的任务限速分别维护。
rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr RegulatedNavigator::speedPublisher() const
{
  return task_.type == TaskType::TO_POSE ? lattice_speed_pub_ : speed_limit_pub_;
}

// 只有规划请求及两通道 FollowPath 的在途计数全部归零，才认为旧子任务已收口。
bool RegulatedNavigator::unifiedDrained() const
{
  return outstanding_plans_ == 0 && fixed_path_outstanding_follows_ == 0 &&
    frozen_outstanding_follows_ == 0;
}

// 检查交接后的有效 MotionState，接收年龄≤反馈超时且线角速度幅值<0.001，同时要求导航输出满足静默窗口。
bool RegulatedNavigator::unifiedStopped() const
{
  const auto feedback = motion_state_subscriber_->latestState();
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - feedback.receive_time).count();
  return feedback.valid && feedback.receive_time > stopping_since_ &&
    age <= get_parameter("chassis_motion_state_timeout").as_double() &&
    std::abs(feedback.linear_velocity) < 1e-3 && std::abs(feedback.angular_velocity) < 1e-3 &&
    chassis_control_subscriber_->navigationOutputStopped(stopping_since_);
}

// 异步轮询两条执行通道的生命周期状态，超时请求从客户端移除，失效缓存按未激活处理。
void RegulatedNavigator::pollExecutionStates()
{
  const auto time = std::chrono::steady_clock::now();
  for (auto * group : {&fixed_execution_, &lattice_execution_})
  {
    for (std::size_t i = 0; i < group->size(); ++i)
    {
      auto & node = group->at(i);
      if (node.request && time - node.last_request >= std::chrono::seconds(1))
      {
        node.client->remove_pending_request(*node.request);
        node.request.reset();
        node.active = false;
      }
      if (node.request || !node.client->service_is_ready())
      {
        continue;
      }
      node.last_request = time;
      node.request = node.client->async_send_request(std::make_shared<StateService::Request>(),
        [group, i](rclcpp::Client<StateService>::SharedFuture future)
        {
          auto & state = group->at(i);
          state.active = future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
          state.last_response = std::chrono::steady_clock::now();
          state.request.reset();
        }).request_id;
    }
  }
}

// 检查所选通道的 Action 可发现性及最近一秒内的 Active 状态，Lattice 还要求规划服务可用。
bool RegulatedNavigator::executionReady(const bool lattice) const
{
  const auto & nodes = lattice ? lattice_execution_ : fixed_execution_;
  const auto time = std::chrono::steady_clock::now();
  const auto & client = lattice ? lattice_follow_client_ : follow_client_;
  return client && client->action_server_is_ready() &&
    (!lattice || compute_pose_client_->action_server_is_ready()) && !nodes.empty() &&
    std::all_of(nodes.begin(), nodes.end(),
      [time](const auto & node)
      {
        return node.active && time - node.last_response <= std::chrono::seconds(1);
      });
}

// 在接收 Goal 前检查生命周期、交接状态、底盘许可与定位，空闲启动还要求旧请求收口和实测停稳。
// 充电任务占用车辆期间（含等待二维码）同样拒绝导航 Goal，与 handleChargingGoal 的
// task_／pending 检查互为镜像，保证两条上层业务不并发驱动同一底盘。
bool RegulatedNavigator::admitUnified(const bool lattice)
{
  if (!active_ || pending_fixed_ || pending_pose_ || task_.state == NavigationState::CANCELING ||
    chargingTaskActive() || !executionReady(lattice) || !chassis_control_subscriber_->canStartNavigation())
  {
    return false;
  }
  if (task_.type == TaskType::NONE && (!unifiedDrained() || !unifiedStopped()))
  {
    return false;
  }
  geometry_msgs::msg::PoseStamped pose;
  return lookupCurrentPose(pose);
}

// 只处理尚未启动的 Goal，已请求取消的句柄返回 CANCELED；内部撤销返回 ABORTED，固定路径句柄收口后清空待执行路径。
void RegulatedNavigator::finishPending(const bool client_cancel)
{
  const auto finish = [client_cancel](auto & handle, const auto & result)
  {
    if (!handle)
    {
      return;
    }
    if (handle->is_canceling())
    {
      handle->canceled(result);
      handle.reset();
    }
    else if (!client_cancel)
    {
      handle->abort(result);
      handle.reset();
    }
  };
  if (pending_pose_)
  {
    finish(pending_pose_, std::make_shared<NavigateToPose::Result>());
  }
  if (pending_fixed_)
  {
    finish(pending_fixed_, std::make_shared<NavigationService::Result>());
    if (!pending_fixed_)
    {
      pending_path_ = nav_msgs::msg::Path();
    }
  }
}

// 先轮询状态并处理取消，再等待旧子任务收口和停稳，最后启动仍有效的待执行 Goal。返回 true 表示本周期由交接流程接管。
bool RegulatedNavigator::monitorUnified()
{
  pollExecutionStates();
  finishPending(true);
  const auto time = std::chrono::steady_clock::now();
  if (task_.state == NavigationState::CANCELING)
  {
    const bool stopped = unifiedDrained() && unifiedStopped();
    const bool timed_out = std::chrono::duration<double>(time - stopping_since_).count() >
      cancel_timeout_;
    if (!stopped && !timed_out)
    {
      return true;
    }
    if (timed_out && !stopped)
    {
      LOG_ERROR("导航交接超时，保持停车，等待终态与新鲜停稳反馈");
      finishPending(false);
    }
    finishCancellation();
  }
  if (task_.type != TaskType::NONE)
  {
    if (!executionReady(task_.type == TaskType::TO_POSE))
    {
      cancelTask("当前导航通道生命周期失效");
      return true;
    }
    return false;
  }
  // 充电跟随同样受固定通道生命周期守护：执行节点掉活即内部取消充电，不等 FollowPath 超时。
  if (chargingFollowActive() && !executionReady(false))
  {
    cancelChargingTask("充电固定执行通道生命周期失效");
    return true;
  }
  if (!pending_pose_ && !pending_fixed_)
  {
    return true;
  }
  if (std::chrono::duration<double>(time - pending_since_).count() > server_timeout_)
  {
    LOG_ERROR("等待启动待执行任务超时");
    finishPending(false);
    return true;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!unifiedDrained() || !unifiedStopped() || !chassis_control_subscriber_->canStartNavigation() ||
    !lookupCurrentPose(pose))
  {
    return true;
  }
  if (!executionReady(pending_pose_ != nullptr))
  {
    finishPending(false);
    return true;
  }
  if (pending_pose_)
  {
    auto goal = std::exchange(pending_pose_, nullptr);
    startPose(goal);
  }
  else
  {
    auto goal = std::exchange(pending_fixed_, nullptr);
    startFixedPath(goal, std::exchange(pending_path_, nav_msgs::msg::Path()));
  }
  return true;
}

}  // namespace nav2_regulated_modules
