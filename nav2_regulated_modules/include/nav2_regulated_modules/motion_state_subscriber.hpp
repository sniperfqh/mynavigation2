// 底盘运动状态订阅接口。将下游状态缓存并提供超时判定，供控制指令安全处理使用。

#ifndef NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_

#include <chrono>
#include <mutex>

#include "byd_custom_msgs/msg/motion_state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

struct MotionStateSnapshot
{
  double linear_velocity { 0.0 };
  double angular_velocity { 0.0 };
  std::chrono::steady_clock::time_point receive_time {};
  bool valid { false };
};

class MotionStateSubscriber
{
  public:
  // 创建底盘运动反馈订阅。
  explicit MotionStateSubscriber(nav2_util::LifecycleNode & node);
  // 返回最近一次底盘运动状态及采样时间，供超时判断。
  MotionStateSnapshot latestState() const;
  // 清除缓存的底盘运动状态及采样时间。
  void reset();

private:
  // 缓存最新底盘运动反馈；后续读取需检查消息是否过期。
  void onMotionState(const byd_custom_msgs::msg::MotionState::ConstSharedPtr message);

  mutable std::mutex mutex_;
  MotionStateSnapshot latest_state_;
  rclcpp::Subscription<byd_custom_msgs::msg::MotionState>::SharedPtr subscription_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
