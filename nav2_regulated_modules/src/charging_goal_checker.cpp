// 充电电流感知 GoalChecker 实现：几何／停稳完成条件沿用基类，充电电流检测走新增短路分支。
// 检测到新鲜充电电流后，isGoalReached 直接返回 true，Controller Server 据此立即完成，
// 随后的零速收口让底盘停止；未检测到电流时行为与 fixed_path_goal_checker 完全一致。

#include "nav2_regulated_modules/charging_goal_checker.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

// 几何／停稳参数委托基类，随后声明电池参数并订阅电池话题。
void ChargingGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  // 生命周期父节点已销毁时无法读取参数或创建订阅，直接终止插件配置。
  if (!node)
  {
    throw std::invalid_argument("ChargingGoalChecker cannot lock lifecycle node");
  }
  FixedPathGoalChecker::initialize(parent, plugin_name, costmap_ros);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".battery_state_topic", rclcpp::ParameterValue("/agv_battery_state"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".charging_current_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".charging_current_detect_window", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name + ".battery_state_topic", battery_state_topic_);
  node->get_parameter(plugin_name + ".charging_current_threshold", charging_current_threshold_);
  node->get_parameter(plugin_name + ".charging_current_detect_window", charging_current_detect_window_);
  // 负阈值或非正的检测窗口会让检测语义失去边界，直接拒绝配置。
  if (
    !std::isfinite(charging_current_threshold_) || !std::isfinite(charging_current_detect_window_) ||
    charging_current_threshold_ < 0.0 || charging_current_detect_window_ <= 0.0)
  {
    throw std::invalid_argument("ChargingGoalChecker battery parameters are invalid");
  }
  battery_state_sub_ = node->create_subscription<byd_custom_msgs::msg::BatteryState>(
    battery_state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ChargingGoalChecker::batteryStateCallback, this, std::placeholders::_1));
  LOG_INFO(
    "充电电流 GoalChecker 配置完成，plugin={}，电池话题={}，电流阈值={:.3f}A，检测窗口={:.3f}s",
    plugin_name, battery_state_topic_, charging_current_threshold_, charging_current_detect_window_);
}

// 丢弃旧路径、末段切线、终点锁存、停稳计数及电池缓存，新任务不能继承这些历史。
void ChargingGoalChecker::reset()
{
  FixedPathGoalChecker::reset();
  std::lock_guard<std::mutex> lock(battery_mutex_);
  has_battery_sample_ = false;
  charging_current_ = 0.0F;
  battery_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

// 检测到新鲜充电电流立即判定完成，否则沿用基类的几何／停稳完成条件。
bool ChargingGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (chargingCurrentDetected())
  {
    return true;
  }
  return FixedPathGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
}

// 最近检测窗口内出现达到阈值的充电电流即认为已接触上电；时钟回跳或采样过旧不算。
bool ChargingGoalChecker::chargingCurrentDetected() const
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  if (!has_battery_sample_)
  {
    return false;
  }
  const double age_s = (ros_clock_.now() - battery_stamp_).seconds();
  return age_s >= 0.0 && age_s <= charging_current_detect_window_ &&
         static_cast<double>(charging_current_) >= charging_current_threshold_;
}

// 电池话题回调：高频到达，只缓存最新电流与时间戳。
void ChargingGoalChecker::batteryStateCallback(const byd_custom_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  charging_current_ = msg->charging_current;
  battery_stamp_ = rclcpp::Time(msg->header.stamp);
  has_battery_sample_ = true;
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::ChargingGoalChecker, nav2_core::GoalChecker)