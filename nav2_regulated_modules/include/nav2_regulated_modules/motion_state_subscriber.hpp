// 底盘 MotionState 的线程安全快照接口。只将有限的 v_car/w_car 存入控制参考。
// receive_time 使用 steady_clock 接收时刻，valid 表示曾接收有效反馈，不自动代表该反馈仍新鲜。

#ifndef NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_

#include <chrono>
#include <mutex>

#include "byd_custom_msgs/msg/motion_state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

// 一份独立复制的底盘反馈：linear_velocity 为 m/s，angular_velocity 为 rad/s，receive_time 为本机单调接收时刻。
// valid=false 时速度零值只是占位；valid=true 后还需由消费者检查接收年龄。
struct MotionStateSnapshot
{
  double linear_velocity { 0.0 };
  double angular_velocity { 0.0 };
  std::chrono::steady_clock::time_point receive_time {};
  bool valid { false };
};

// 封装一次订阅与互斥快照；订阅回调不承担远程模式切换，也不会对底盘速度重新积分。
class MotionStateSubscriber
{
  public:
  // 建立 SensorDataQoS 订阅，速度来自 MotionState 的 v_car/w_car，而非 Odometry.twist。
  explicit MotionStateSubscriber(nav2_util::LifecycleNode & node);
  // 持锁复制快照后返回，调用方不持有内部对象引用；还需检查 valid 和接收年龄。
  MotionStateSnapshot latestState() const;
  // 在锁内恢复空快照，避免新阶段沿用旧反馈；不发布停车指令。
  void reset();

private:
  // 先拒绝 NaN／Inf，再在锁内更新速度、单调接收时刻和 valid 标志。
  // 非法样本不刷新时间，旧样本最终会由消费者判断过期。
  void onMotionState(const byd_custom_msgs::msg::MotionState::ConstSharedPtr message);

  mutable std::mutex mutex_;
  bool legacy_velocity_logging_ = true;
  MotionStateSnapshot latest_state_;
  rclcpp::Subscription<byd_custom_msgs::msg::MotionState>::SharedPtr subscription_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
