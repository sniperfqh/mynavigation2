#include "nav2_regulated_modules/chassis_control_subscriber.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "spdlog_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

ChassisControlSubscriber::ChassisControlSubscriber(
    nav2_util::LifecycleNode & node,
    MotionStateSubscriber & motion_state_subscriber,
    ChassisControlConfig config)
  : node_(node),
    motion_state_subscriber_(motion_state_subscriber),
    config_(std::move(config)),
    linear_planner_(
        1.0 / config_.publish_rate,
        config_.default_linear_speed_max,
        config_.default_linear_accel_max,
        config_.linear_accel_jerk_max,
        config_.linear_decel_max,
        config_.linear_decel_jerk_max),
    angular_planner_(
        1.0 / config_.publish_rate,
        config_.default_angular_speed_max,
        config_.default_angular_accel_max,
        config_.angular_accel_jerk_max,
        config_.angular_decel_max,
        config_.angular_decel_jerk_max)
{
  if (!validateConfig()) {
    LOG_ERROR("ChassisControlConfig 参数校验失败");
    throw std::invalid_argument("ChassisControl 闭环参数非法");
  }
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  subscription_ = node.create_subscription<byd_custom_msgs::msg::ChassisControl>(
      config_.input_topic, qos,
      std::bind(&ChassisControlSubscriber::onChassisControl, this, std::placeholders::_1));
  publisher_ = node.create_publisher<byd_custom_msgs::msg::ControlRes>(
      config_.output_topic, qos);
}

void ChassisControlSubscriber::onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message) {
  if (!active_) {
    LOG_WARN("忽略 Lifecycle 未激活时收到的 ChassisControl");
    return;
  }
  TargetCommand command;
  command.operation = message->op;
  if (!std::isfinite(message->linear_velocity) || !std::isfinite(message->angular_velocity) || !std::isfinite(message->acceleration) || message->linear_velocity < 0.0F || message->angular_velocity < 0.0F || message->acceleration < 0.0F) {
    LOG_ERROR("拒绝非法 ChassisControl：op={}，linear_velocity={}，angular_velocity={}，acceleration={}", static_cast<unsigned int>(message->op), message->linear_velocity, message->angular_velocity, message->acceleration);
    return;
  }
  if (message->op == byd_custom_msgs::msg::ChassisControl::OP_FORWARD || message->op == byd_custom_msgs::msg::ChassisControl::OP_BACKWARD) {
    command.linear_velocity = message->op == byd_custom_msgs::msg::ChassisControl::OP_FORWARD ? message->linear_velocity : -message->linear_velocity;
    command.linear_acceleration = message->acceleration > 0.0F ? std::min(static_cast<double>(message->acceleration), config_.linear_accel_max) : config_.default_linear_accel_max;
    command.angular_velocity = 0.0;
    command.angular_acceleration = config_.default_angular_accel_max;
  } else if (message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_LEFT || message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_RIGHT) {
    command.angular_velocity = message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_LEFT ? message->angular_velocity : -message->angular_velocity;
    command.angular_acceleration = message->acceleration > 0.0F ? std::min(static_cast<double>(message->acceleration), config_.angular_accel_max) : config_.default_angular_accel_max;
    command.linear_velocity = 0.0;
    command.linear_acceleration = config_.default_linear_accel_max;
  } else {
    LOG_ERROR("拒绝未知 ChassisControl op={}", static_cast<unsigned int>(message->op));
    return;
  }
  command.linear_velocity = std::clamp(command.linear_velocity, -config_.linear_speed_max, config_.linear_speed_max);
  command.angular_velocity = std::clamp(command.angular_velocity, -config_.angular_speed_max, config_.angular_speed_max);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_tmie_ = std::chrono::steady_clock::now();
    target_command_ = command;
    has_command_ = true;
  }
  LOG_DEBUG("收到 ChassisControl：op={}，linear_velocity={}，angular_velocity={}，acceleration={}", static_cast<unsigned int>(message->op), message->linear_velocity, message->angular_velocity, message->acceleration);
}

void ChassisControlSubscriber::activate() {
  motion_state_subscriber_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clearControlStateLocked();
    active_ = true;
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / config_.publish_rate));
  timer_ = node_.create_wall_timer(period, std::bind(&ChassisControlSubscriber::processControlCommand, this));
  LOG_INFO("ChassisControl 事件驱动闭环已激活，output_topic={}", config_.output_topic);
}

void ChassisControlSubscriber::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  clearControlStateLocked();
  timer_->cancel();
  LOG_INFO("ChassisControl 事件驱动闭环已停用");
}

void ChassisControlSubscriber::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  clearControlStateLocked();
  motion_state_subscriber_.reset();
}

void ChassisControlSubscriber::processControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {return;}
  if (!has_command_ && !is_remote_control_) {return;}
  const auto publisher_count = node_.count_publishers(config_.output_topic);
  if (publisher_count > 1U) {
    if (!publisher_conflict_) {LOG_ERROR("检测到 {} 个 {} Publisher，停止 ChassisControl 闭环输出", publisher_count, config_.output_topic);}
    publisher_conflict_ = true;
    clearControlStateLocked();
    return;
  }
  if (publisher_conflict_) {
    publisher_conflict_ = false;
    LOG_INFO("{} Publisher 冲突已解除，等待新的 ChassisControl", config_.output_topic);
  }
  const auto now = std::chrono::steady_clock::now();
  if(std::chrono::duration<double>(now - last_command_tmie_).count() > config_.command_timeout){
    has_command_ = false;
    target_command_.linear_velocity = 0.0;
    target_command_.angular_velocity = 0.0;
  }
  if (has_command_ && !is_remote_control_) {
    const auto state = motion_state_subscriber_.latestState();
    const bool state_timed_out = !state.valid || std::chrono::duration<double>(now - state.receive_time).count() > config_.motion_state_timeout;
    if (state_timed_out) {
      LOG_ERROR("MotionState 超时或无有效反馈，停止并等待新的 ChassisControl");
      publishZero();
      clearControlStateLocked();
      return;
    }
    linear_planner_.init(state.linear_velocity);
    angular_planner_.init(state.angular_velocity);
    is_remote_control_ = true;
  }
  //  开始正常控制  if (has_command_ && is_remote_control_) {
  linear_planner_.setDirection(target_command_.linear_velocity, target_command_.linear_acceleration);
  angular_planner_.setDirection(target_command_.angular_velocity, target_command_.angular_acceleration);
  double linear_output = linear_planner_.update();
  double angular_output = angular_planner_.update();
  // 控制状态终结
  if( !has_command_ && std::abs(target_command_.linear_velocity) < 1e-3 && std::abs(linear_output) < 1e-3 && std::abs(target_command_.angular_velocity) < 1e-3 && std::abs(angular_output) < 1e-3){
    const auto state = motion_state_subscriber_.latestState();
    const bool state_timed_out = !state.valid || std::chrono::duration<double>(now - state.receive_time).count() > config_.motion_state_timeout;
    if (state_timed_out) {
      LOG_ERROR("MotionState 超时或无有效反馈，停止并等待新的 ChassisControl");
      publishZero();
      clearControlStateLocked();
      return;
    } 
    if(std::abs(state.linear_velocity)<1e-3 && std::abs(state.angular_velocity)<1e-3){
      is_remote_control_ = false;
      publishZero();
      clearControlStateLocked();
      return;
    }
  }
  publishControl(linear_output, angular_output);
}

void ChassisControlSubscriber::clearControlStateLocked() {
  target_command_ = TargetCommand{};
  has_command_ = false;
  is_remote_control_ = false;
}

void ChassisControlSubscriber::publishControl(const double linear_velocity, const double angular_velocity) {
  byd_custom_msgs::msg::ControlRes output;
  output.v = linear_velocity;
  output.w = angular_velocity;
  output.v_lift = 0.0;
  output.w_rotation = 0.0;
  publisher_->publish(output);
}

void ChassisControlSubscriber::publishZero() { publishControl(0.0, 0.0); }

bool ChassisControlSubscriber::validateConfig() const
{
  // 话题名称不能为空
  if (config_.input_topic.empty()) {
    LOG_ERROR("配置错误: input_topic 不能为空");
    return false;
  }
  if (config_.output_topic.empty()) {
    LOG_ERROR("配置错误: output_topic 不能为空");
    return false;
  }

  // 超时和频率必须为正数
  if (config_.motion_state_timeout <= 0.0) {
    LOG_ERROR("配置错误: motion_state_timeout 必须大于 0，当前值: {}", config_.motion_state_timeout);
    return false;
  }
  if (config_.command_timeout <= 0.0) {
    LOG_ERROR("配置错误: command_timeout 必须大于 0，当前值: {}", config_.command_timeout);
    return false;
  }
  if (config_.publish_rate <= 0.0) {
    LOG_ERROR("配置错误: publish_rate 必须大于 0，当前值: {}", config_.publish_rate);
    return false;
  }

  // 速度上限必须为正数
  if (config_.default_linear_speed_max <= 0.0) {
    LOG_ERROR("配置错误: default_linear_speed_max 必须大于 0，当前值: {}", config_.default_linear_speed_max);
    return false;
  }
  if (config_.linear_speed_max <= 0.0) {
    LOG_ERROR("配置错误: linear_speed_max 必须大于 0，当前值: {}", config_.linear_speed_max);
    return false;
  }
  if (config_.default_angular_speed_max <= 0.0) {
    LOG_ERROR("配置错误: default_angular_speed_max 必须大于 0，当前值: {}", config_.default_angular_speed_max);
    return false;
  }
  if (config_.angular_speed_max <= 0.0) {
    LOG_ERROR("配置错误: angular_speed_max 必须大于 0，当前值: {}", config_.angular_speed_max);
    return false;
  }

  // 加速度上限必须为正数
  if (config_.default_linear_accel_max <= 0.0) {
    LOG_ERROR("配置错误: default_linear_accel_max 必须大于 0，当前值: {}", config_.default_linear_accel_max);
    return false;
  }
  if (config_.linear_accel_max <= 0.0) {
    LOG_ERROR("配置错误: linear_accel_max 必须大于 0，当前值: {}", config_.linear_accel_max);
    return false;
  }
  if (config_.linear_decel_max <= 0.0) {
    LOG_ERROR("配置错误: linear_decel_max 必须大于 0，当前值: {}", config_.linear_decel_max);
    return false;
  }
  if (config_.default_angular_accel_max <= 0.0) {
    LOG_ERROR("配置错误: default_angular_accel_max 必须大于 0，当前值: {}", config_.default_angular_accel_max);
    return false;
  }
  if (config_.angular_accel_max <= 0.0) {
    LOG_ERROR("配置错误: angular_accel_max 必须大于 0，当前值: {}", config_.angular_accel_max);
    return false;
  }
  if (config_.angular_decel_max <= 0.0) {
    LOG_ERROR("配置错误: angular_decel_max 必须大于 0，当前值: {}", config_.angular_decel_max);
    return false;
  }

  // 加加速度上限必须为正数
  if (config_.linear_accel_jerk_max <= 0.0) {
    LOG_ERROR("配置错误: linear_accel_jerk_max 必须大于 0，当前值: {}", config_.linear_accel_jerk_max);
    return false;
  }
  if (config_.linear_decel_jerk_max <= 0.0) {
    LOG_ERROR("配置错误: linear_decel_jerk_max 必须大于 0，当前值: {}", config_.linear_decel_jerk_max);
    return false;
  }
  if (config_.angular_accel_jerk_max <= 0.0) {
    LOG_ERROR("配置错误: angular_accel_jerk_max 必须大于 0，当前值: {}", config_.angular_accel_jerk_max);
    return false;
  }
  if (config_.angular_decel_jerk_max <= 0.0) {
    LOG_ERROR("配置错误: angular_decel_jerk_max 必须大于 0，当前值: {}", config_.angular_decel_jerk_max);
    return false;
  }

  // 交叉约束：default 不能超过 max
  if (config_.default_linear_speed_max > config_.linear_speed_max) {
    LOG_ERROR("配置错误: default_linear_speed_max({}) 不能大于 linear_speed_max({})",
              config_.default_linear_speed_max, config_.linear_speed_max);
    return false;
  }
  if (config_.default_angular_speed_max > config_.angular_speed_max) {
    LOG_ERROR("配置错误: default_angular_speed_max({}) 不能大于 angular_speed_max({})",
              config_.default_angular_speed_max, config_.angular_speed_max);
    return false;
  }
  if (config_.default_linear_accel_max > config_.linear_accel_max) {
    LOG_ERROR("配置错误: default_linear_accel_max({}) 不能大于 linear_accel_max({})",
              config_.default_linear_accel_max, config_.linear_accel_max);
    return false;
  }
  if (config_.default_angular_accel_max > config_.angular_accel_max) {
    LOG_ERROR("配置错误: default_angular_accel_max({}) 不能大于 angular_accel_max({})",
              config_.default_angular_accel_max, config_.angular_accel_max);
    return false;
  }

  // 交叉约束：减速能力应不小于加速能力（工程常识：刹车能力 >= 起步能力）
  if (config_.linear_decel_max < config_.linear_accel_max) {
    LOG_WARN("配置警告: linear_decel_max({}) 小于 linear_accel_max({})，减速能力弱于加速能力，可能导致制动距离过长",
             config_.linear_decel_max, config_.linear_accel_max);
  }
  if (config_.angular_decel_max < config_.angular_accel_max) {
    LOG_WARN("配置警告: angular_decel_max({}) 小于 angular_accel_max({})，转向减速能力弱于加速能力",
             config_.angular_decel_max, config_.angular_accel_max);
  }

  // 交叉约束：加加速度应能支撑对应的加速度变化（经验约束）
  // jerk_max * dt 应能在一两个控制周期内达到目标加速度，否则加加速度限幅会成为瓶颈
  const double min_dt = 1.0 / config_.publish_rate;
  if (config_.linear_accel_jerk_max * min_dt > config_.linear_accel_max) {
    LOG_WARN("配置警告: linear_accel_jerk_max({}) 在单周期内可产生的加速度变化({}) 超过 linear_accel_max({})，加加速度限幅可能成为瓶颈",
             config_.linear_accel_jerk_max,
             config_.linear_accel_jerk_max * min_dt,
             config_.linear_accel_max);
  }

  return true;
}

}  // namespace nav2_regulated_modules