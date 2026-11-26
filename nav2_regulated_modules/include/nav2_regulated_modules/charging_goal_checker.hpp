// 充电电流感知 GoalChecker：继承固定路径检查器的几何／停稳完成条件，新增"检测到
// 充电电流即完成"。电池话题为高频数据，回调只缓存最新采样；到点判定按新鲜度窗口与
// 电流阈值即时短路，供 Controller Server 直接完成并收口零速。

#ifndef NAV2_REGULATED_MODULES__CHARGING_GOAL_CHECKER_HPP_
#define NAV2_REGULATED_MODULES__CHARGING_GOAL_CHECKER_HPP_

#include <functional>
#include <mutex>
#include <string>

#include "byd_custom_msgs/msg/battery_state.hpp"
#include "nav2_core/charging_detection_goal_checker.hpp"
#include "nav2_regulated_modules/fixed_path_goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_regulated_modules
{

class ChargingGoalChecker : public FixedPathGoalChecker, public nav2_core::ChargingDetectionGoalChecker
{
public:
  // 默认构造仅使用成员初值；容差与电池参数在 initialize 读取，路径在 setPath 单独提供。
  ChargingGoalChecker() = default;

  // 几何／停稳容差参数与路径状态委托基类；随后声明充电电流阈值／检测窗口并订阅电池话题。
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 丢弃旧路径、末段切线、终点锁存、停稳计数及电池缓存，新任务不能继承这些历史。
  void reset() override;
  // 检测到新鲜充电电流（窗口内经阈值）立即判定完成，否则沿用基类几何／停稳条件。
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  // Controller Server 到点短路接口：最近窗口内有充电电流即返回 true。
  bool chargingCurrentDetected() const override;

private:
  // 电池话题回调：只缓存最新电流与时间戳，判定在消费时进行（高频到达保持轻量）。
  void batteryStateCallback(const byd_custom_msgs::msg::BatteryState::SharedPtr msg);

  std::string battery_state_topic_{"/agv_battery_state"};
  double charging_current_threshold_{0.05};
  double charging_current_detect_window_{0.5};
  rclcpp::Subscription<byd_custom_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  // 回调写、到点判定读，均走本锁；const 判定内取当前时间需要 mutable 时钟。
  mutable std::mutex battery_mutex_;
  mutable rclcpp::Clock ros_clock_{RCL_ROS_TIME};
  bool has_battery_sample_{false};
  float charging_current_{0.0F};
  rclcpp::Time battery_stamp_{0, 0, RCL_ROS_TIME};
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CHARGING_GOAL_CHECKER_HPP_