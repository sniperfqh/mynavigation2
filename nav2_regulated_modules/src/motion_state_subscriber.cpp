// 订阅 /motion_state 并缓存底盘线／角速度。非法速度不会刷新缓存样本的接收时间。
// 快照读写使用同一互斥锁；过期判定由消费者根据 steady_clock 时间完成，本模块不会定时复制旧消息。

#include "nav2_regulated_modules/motion_state_subscriber.hpp"

#include <cmath>
#include <functional>

#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

// 建立 SensorDataQoS 订阅，速度来自 MotionState 的 v_car/w_car，而非 Odometry.twist。
MotionStateSubscriber::MotionStateSubscriber(nav2_util::LifecycleNode & node)
{
  node.get_parameter_or("legacy_velocity_logging", legacy_velocity_logging_, true);
  subscription_ = node.create_subscription<byd_custom_msgs::msg::MotionState>("/motion_state", rclcpp::SensorDataQoS(), std::bind(&MotionStateSubscriber::onMotionState, this, std::placeholders::_1));
}

// 先拒绝 NaN／Inf，再在锁内更新速度、单调接收时刻和 valid 标志。
// 非法样本不刷新时间，旧样本最终会由消费者判断过期。
void MotionStateSubscriber::onMotionState(const byd_custom_msgs::msg::MotionState::ConstSharedPtr message)
{
  if (!std::isfinite(message->v_car) || !std::isfinite(message->w_car))
  {
    LOG_WARN("忽略非法 MotionState：v_car={}，w_car={}", message->v_car, message->w_car);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_.linear_velocity = message->v_car;
    latest_state_.angular_velocity = message->w_car;
    latest_state_.receive_time = std::chrono::steady_clock::now();
    latest_state_.valid = true;
  }
  if (legacy_velocity_logging_)
  {
    LOG_DEBUG("收到 MotionState：v_car={}，w_car={}，v_lift={}，lift_height={}，w_shelf={}，yaw_shelf={}", message->v_car, message->w_car, message->v_lift, message->lift_height, message->w_shelf, message->yaw_shelf);
  }
}

// 持锁复制快照后返回，调用方不持有内部对象引用；还需检查 valid 和接收年龄。
MotionStateSnapshot MotionStateSubscriber::latestState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_state_;
}

// 在锁内恢复空快照，避免新阶段沿用旧反馈；不发布停车指令。
void MotionStateSubscriber::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = MotionStateSnapshot{};
}

}
// namespace nav2_regulated_modules
