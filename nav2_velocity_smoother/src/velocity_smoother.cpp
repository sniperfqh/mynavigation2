// Copyright (c) 2022 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options) : LifecycleNode("velocity_smoother", "", options), last_command_time_{0, 0, get_clock()->get_clock_type()} {
  // SpdlogWrapper::init("nav2_velocity_smoother", get_name());
  LOG_INFO("Creating velocity smoother");
  LOG_INFO("Velocity smoother limits cmd_vel_nav before publishing final cmd_vel");

}

VelocitySmoother::~VelocitySmoother() {
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2_util::CallbackReturn VelocitySmoother::on_configure(const rclcpp_lifecycle::State &) {
  LOG_INFO("Configuring velocity smoother");
  LOG_INFO("Configuring velocity limits, feedback mode and command topics");
  auto node = shared_from_this();
  std::string feedback_type;
  double velocity_timeout_dbl;

  // Smoothing metadata
  declare_parameter_if_not_declared(node, "smoothing_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, "feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  declare_parameter_if_not_declared(node, "scale_velocities", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, "immediate_stop_on_zero_command", rclcpp::ParameterValue(false));
  node->get_parameter("smoothing_frequency", smoothing_frequency_);
  node->get_parameter("feedback", feedback_type);
  node->get_parameter("scale_velocities", scale_velocities_);
  node->get_parameter("immediate_stop_on_zero_command", immediate_stop_on_zero_command_);
  LOG_INFO("Velocity smoother metadata smoothing_frequency={}, feedback={}, scale_velocities={}, immediate_stop_on_zero_command={}", smoothing_frequency_, feedback_type.c_str(), scale_velocities_, immediate_stop_on_zero_command_);

  // Kinematics
  declare_parameter_if_not_declared(node, "max_velocity", rclcpp::ParameterValue(std::vector<double>{0.50, 0.0, 2.5}));
  declare_parameter_if_not_declared(node, "min_velocity", rclcpp::ParameterValue(std::vector<double>{-0.50, 0.0, -2.5}));
  declare_parameter_if_not_declared(node, "max_accel", rclcpp::ParameterValue(std::vector<double>{2.5, 0.0, 3.2}));
  declare_parameter_if_not_declared(node, "max_decel", rclcpp::ParameterValue(std::vector<double>{-2.5, 0.0, -3.2}));
  node->get_parameter("max_velocity", max_velocities_);
  node->get_parameter("min_velocity", min_velocities_);
  node->get_parameter("max_accel", max_accels_);
  node->get_parameter("max_decel", max_decels_);

  if (max_velocities_.size() != 3 || min_velocities_.size() != 3 || max_accels_.size() != 3 || max_decels_.size() != 3)
  {
    throw std::runtime_error("Kinematic arrays must have three entries");
  }
  for (unsigned int i = 0; i != 3; i++) {
    if (max_decels_[i] > 0.0) {
      throw std::runtime_error("Positive values set of deceleration! These should be negative to slow down!");
    }
    if (max_accels_[i] < 0.0) {
      throw std::runtime_error("Negative values set of acceleration! These should be positive to speed up!");
    }
    if (min_velocities_[i] > max_velocities_[i]) {
      throw std::runtime_error("Min velocities are higher than max velocities!");
    }
  }

  // Get feature parameters
  declare_parameter_if_not_declared(node, "odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared(node, "odom_duration", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, "feedback_correction_time", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, "deadband_velocity", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  declare_parameter_if_not_declared(node, "velocity_timeout", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, "speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  node->get_parameter("odom_topic", odom_topic_);
  node->get_parameter("odom_duration", odom_duration_);
  node->get_parameter("feedback_correction_time", feedback_correction_time_);
  if (!std::isfinite(feedback_correction_time_) || feedback_correction_time_ <= 0.0)
  {
    throw std::runtime_error("feedback_correction_time must be finite and greater than zero");
  }
  node->get_parameter("deadband_velocity", deadband_velocities_);
  node->get_parameter("velocity_timeout", velocity_timeout_dbl);
  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);
  LOG_INFO("Velocity smoother feature parameters odom_topic={}, odom_duration={}, velocity_timeout={}", odom_topic_.c_str(), odom_duration_, velocity_timeout_dbl);

  if (max_velocities_.size() != 3 || min_velocities_.size() != 3 || max_accels_.size() != 3 || max_decels_.size() != 3 || deadband_velocities_.size() != 3)
  {
    throw std::runtime_error("Invalid setting of kinematic and/or deadband limits!" " All limits must be size of 3 representing (x, y, theta).");
  }
  declare_parameter_if_not_declared(node, "jerk_limited_smoothing", rclcpp::ParameterValue(false));
  std::vector<double> jerk_defaults(3, 0.0);
  jerk_defaults[0] = 6.0;
  jerk_defaults[2] = 4.0;
  declare_parameter_if_not_declared(node, "max_accel_jerk", rclcpp::ParameterValue(jerk_defaults));
  jerk_defaults[0] = 8.0;
  jerk_defaults[2] = 6.0;
  declare_parameter_if_not_declared(node, "max_decel_jerk", rclcpp::ParameterValue(jerk_defaults));
  get_parameter("jerk_limited_smoothing", jerk_limited_smoothing_);
  get_parameter("max_accel_jerk", max_accel_jerks_);
  get_parameter("max_decel_jerk", max_decel_jerks_);
  if (jerk_limited_smoothing_)
  {
    if (max_accel_jerks_.size() != 3 || max_decel_jerks_.size() != 3 || !std::isfinite(smoothing_frequency_) || smoothing_frequency_ <= 0.0 || scale_velocities_)
    {
      throw std::runtime_error("S-curve requires three-axis jerk limits, valid frequency and scale_velocities=false");
    }
    for (size_t i = 0; i < 3; ++i)
    {
      const bool axis_enabled = max_velocities_[i] != 0.0 || min_velocities_[i] != 0.0;
      if (!std::isfinite(max_velocities_[i]) || !std::isfinite(min_velocities_[i]) || !std::isfinite(max_accels_[i]) || !std::isfinite(max_decels_[i]) || !std::isfinite(max_accel_jerks_[i]) || !std::isfinite(max_decel_jerks_[i]) || deadband_velocities_[i] != 0.0 || min_velocities_[i] > 0.0 || max_velocities_[i] < 0.0 || (axis_enabled && (max_accels_[i] <= 0.0 || max_decels_[i] >= 0.0 || max_accel_jerks_[i] <= 0.0 || max_decel_jerks_[i] <= 0.0)))
      {
        throw std::runtime_error("S-curve requires finite positive active-axis limits, zero deadband and speed bounds containing zero");
      }
    }
    immediate_stop_on_zero_command_ = false;
    set_parameter(rclcpp::Parameter("immediate_stop_on_zero_command", false));
    LOG_INFO("S-curve enabled: accel_jerk=({},{},{}), decel_jerk=({},{},{}); ordinary zero commands decelerate smoothly", max_accel_jerks_[0], max_accel_jerks_[1], max_accel_jerks_[2], max_decel_jerks_[0], max_decel_jerks_[1], max_decel_jerks_[2]);
  }
  target_maxvx_ = max_velocities_[0];
  target_minvx_ = min_velocities_[0];

  // Get control type
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
    LOG_INFO("Velocity smoother uses OPEN_LOOP feedback from last published command");
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);
    LOG_INFO("Velocity smoother uses CLOSED_LOOP feedback from odometry topic {}", odom_topic_.c_str());
  } else {
    throw std::runtime_error("Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1);
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1), std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1));
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(speed_limit_topic, rclcpp::QoS(10), std::bind(&VelocitySmoother::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_activate(const rclcpp_lifecycle::State &) {
  feedback_initialized_ = false;
  jerk_clock_initialized_ = false;
  for (auto & profile : jerk_profiles_)
  {
    profile.reset();
  }
  LOG_INFO("Activating");
  LOG_INFO("Activating smoothed cmd_vel publisher and smoothing timer");
  smoothed_cmd_pub_->on_activate();
  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(timer_duration_ms)), std::bind(&VelocitySmoother::smootherTimer, this));

  dyn_params_handler_ = this->add_on_set_parameters_callback(std::bind(&VelocitySmoother::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &) {
  feedback_initialized_ = false;
  LOG_INFO("Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  smoothed_cmd_pub_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &) {
  LOG_INFO("Cleaning up");
  smoothed_cmd_pub_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &) {
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  command_ = msg;
  last_command_time_ = now();
  LOG_DEBUG("Received raw cmd_vel linear=({:.3f}, {:.3f}) angular_z={:.3f}", msg->linear.x, msg->linear.y, msg->angular.z);
}

geometry_msgs::msg::Twist VelocitySmoother::feedbackReference(const geometry_msgs::msg::Twist & feedback) const
{
  if (!feedback_initialized_)
  {
    return feedback;
  }
  // 反馈校正权重按控制周期缩放，使 50/100 Hz 使用相同的连续时间响应。
  const double weight = std::clamp(1.0 / (smoothing_frequency_ * feedback_correction_time_), 0.0, 1.0);
  auto reference = last_cmd_;
  reference.linear.x += weight * (feedback.linear.x - last_cmd_.linear.x);
  reference.linear.y += weight * (feedback.linear.y - last_cmd_.linear.y);
  reference.angular.z += weight * (feedback.angular.z - last_cmd_.angular.z);
  return reference;
}

double VelocitySmoother::findEtaConstraint(const double v_curr, const double v_cmd, const double accel, const double decel) {
  // Exploiting vector scaling properties
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  if (dv > v_component_max) {
    return v_component_max / dv;
  }

  if (dv < v_component_min) {
    return v_component_min / dv;
  }

  return -1.0;
}

double VelocitySmoother::applyConstraints(const double v_curr, const double v_cmd, const double accel, const double decel, const double eta) {
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer() {
  // Wait until the first command is received
  if (!command_) {
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  // Check for velocity timeout. If nothing received, publish zeros to apply deceleration
  if (now() - last_command_time_ > velocity_timeout_) {
    if (last_cmd_ == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      jerk_clock_initialized_ = false;
      if (!jerk_limited_smoothing_)
      {
        feedback_initialized_ = false;
      }
      return;
    }
    *command_ = geometry_msgs::msg::Twist();
  }

  stopped_ = false;

  if (immediate_stop_on_zero_command_ && *command_ == geometry_msgs::msg::Twist())
  {
    last_cmd_ = geometry_msgs::msg::Twist();
    stopped_ = true;
    feedback_initialized_ = false;
    smoothed_cmd_pub_->publish(std::move(cmd_vel));
    return;
  }

  // Get current velocity based on feedback type
  geometry_msgs::msg::Twist current_;
  if (open_loop_) {
    current_ = last_cmd_;
  } else {
    const auto feedback = odom_smoother_->getTwist();
    // 首周期及重新激活时从真实反馈起步，不沿用停用前的速度斜坡。
    if (!feedback_initialized_)
    {
      last_cmd_ = feedback;
    }
    current_ = feedbackReference(feedback);
    feedback_initialized_ = true;
  }
  if (jerk_limited_smoothing_)
  {
    const auto tick = std::chrono::steady_clock::now();
    const double dt = jerk_clock_initialized_ ? std::chrono::duration<double>(tick - jerk_last_tick_).count() : 1.0 / smoothing_frequency_;
    jerk_last_tick_ = tick;
    jerk_clock_initialized_ = true;
    if (!std::isfinite(dt) || dt <= 0.0)
    {
      return;
    }
    std::array<double, 3> previous;
    previous[0] = last_cmd_.linear.x;
    previous[1] = last_cmd_.linear.y;
    previous[2] = last_cmd_.angular.z;
    std::array<double, 3> reference;
    reference[0] = current_.linear.x;
    reference[1] = current_.linear.y;
    reference[2] = current_.angular.z;
    std::array<double, 3> requested;
    requested[0] = command_->linear.x;
    requested[1] = command_->linear.y;
    requested[2] = command_->angular.z;
    std::array<double, 3> output;
    for (size_t i = 0; i < 3; ++i)
    {
      const double low = i == 0 ? std::max(min_velocities_[i], target_minvx_) : min_velocities_[i];
      const double high = i == 0 ? std::min(max_velocities_[i], target_maxvx_) : max_velocities_[i];
      // 闭环只校正目标；零指令必须收敛到零，不因里程计滞后产生反向补偿。
      double target = requested[i] == 0.0 ? 0.0 : requested[i] + previous[i] - reference[i];
      target = std::clamp(target, low, high);
      if (target * requested[i] < 0.0)
      {
        target = 0.0;
      }
      // 反向先规划零速；速度限幅消息作为新目标逐步制动，不瞬间截断输出。
      if (target * previous[i] < 0.0)
      {
        target = 0.0;
      }
      const double old_acceleration = jerk_profiles_[i].acceleration();
      output[i] = jerk_profiles_[i].advance(previous[i], target, dt, max_accels_[i], -max_decels_[i], max_accel_jerks_[i], max_decel_jerks_[i], min_velocities_[i], max_velocities_[i]);
      const double acceleration = jerk_profiles_[i].acceleration();
      LOG_DEBUG("S-curve axis={} dt={:.9f} input={:.9f} feedback_reference={:.9f} target={:.9f} output={:.9f} accel={:.9f} jerk={:.9f} max_accel={} max_decel={} max_accel_jerk={} max_decel_jerk={}", i, dt, requested[i], reference[i], target, output[i], acceleration, (acceleration - old_acceleration) / dt, max_accels_[i], max_decels_[i], max_accel_jerks_[i], max_decel_jerks_[i]);
    }
    cmd_vel->linear.x = output[0];
    cmd_vel->linear.y = output[1];
    cmd_vel->angular.z = output[2];
    last_cmd_ = *cmd_vel;
    smoothed_cmd_pub_->publish(std::move(cmd_vel));
    return;
  }
  if (limitv2target) {
    // 当前速度不在 [target_minvx_, target_maxvx_] 区间内：以加速度/减速度步长逐步逼近目标限幅，避免超调
    if (current_.linear.x < target_minvx_ || current_.linear.x > target_maxvx_) {
      max_velocities_[0] = applyConstraints(max_velocities_[0], target_maxvx_, max_accels_[0], max_decels_[0], 1.0);
      min_velocities_[0] = applyConstraints(min_velocities_[0], target_minvx_, max_accels_[0], max_decels_[0], 1.0);
    } else {
      // 当前速度已在目标区间内：直接到位
      max_velocities_[0] = target_maxvx_;
      min_velocities_[0] = target_minvx_;
    }

    // 限幅已移动到目标则结束过渡
    if (std::fabs(max_velocities_[0] - target_maxvx_) < 1e-3 &&
      std::fabs(min_velocities_[0] - target_minvx_) < 1e-3)
    {
      max_velocities_[0] = target_maxvx_;
      min_velocities_[0] = target_minvx_;
      limitv2target = false;
    }
  }

  // Apply absolute velocity restrictions to the command
  command_->linear.x = std::clamp(command_->linear.x, min_velocities_[0], max_velocities_[0]);
  command_->linear.y = std::clamp(command_->linear.y, min_velocities_[1], max_velocities_[1]);
  command_->angular.z = std::clamp(command_->angular.z, min_velocities_[2], max_velocities_[2]);

  // Find if any component is not within the acceleration constraints. If so, store the most
  // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
  // proportionally to follow the same direction, within change of velocity bounds.
  // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
  // output is within limits, even if it deviates from requested command slightly.
  double eta = 1.0;
  if (scale_velocities_) {
    double curr_eta = -1.0;

    curr_eta = findEtaConstraint(current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }
  }

  cmd_vel->linear.x = applyConstraints(current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0], eta);
  cmd_vel->linear.y = applyConstraints(current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1], eta);
  cmd_vel->angular.z = applyConstraints(current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2], eta);
  if (!open_loop_)
  {
    // 反馈修正不能使实际发布指令突破原有单周期加减速度约束。
    cmd_vel->linear.x = applyConstraints(last_cmd_.linear.x, cmd_vel->linear.x, max_accels_[0], max_decels_[0], 1.0);
    cmd_vel->linear.y = applyConstraints(last_cmd_.linear.y, cmd_vel->linear.y, max_accels_[1], max_decels_[1], 1.0);
    cmd_vel->angular.z = applyConstraints(last_cmd_.angular.z, cmd_vel->angular.z, max_accels_[2], max_decels_[2], 1.0);
  }

  cmd_vel->linear.x = std::clamp(cmd_vel->linear.x, min_velocities_[0], max_velocities_[0]);
  cmd_vel->linear.y = std::clamp(cmd_vel->linear.y, min_velocities_[1], max_velocities_[1]);
  cmd_vel->angular.z = std::clamp(cmd_vel->angular.z, min_velocities_[2], max_velocities_[2]);
  // 保留跨死区的内部斜坡，避免 OPEN_LOOP 在首次增量小于死区时停滞。
  last_cmd_ = *cmd_vel;
  // Apply deadband restrictions & publish
  cmd_vel->linear.x = fabs(cmd_vel->linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->linear.x;
  cmd_vel->linear.y = fabs(cmd_vel->linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->linear.y;
  cmd_vel->angular.z = fabs(cmd_vel->angular.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->angular.z;
  // LOG_INFO("command: linear.x:{}, linear.y:{}, angular.z:{}| current:{}, {}, {} | cmd_vel:{}, {}, {}", 
  //   command_->linear.x, command_->linear.y, command_->angular.z,
  //   current_.linear.x,  current_.linear.y,  current_.angular.z,
  //   cmd_vel->linear.x,  cmd_vel->linear.y,  cmd_vel->angular.z
  // );
  // LOG_INFO(" min_velocities_[0]={}, max_velocities_[0]={} ", min_velocities_[0], max_velocities_[0] );

  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

void VelocitySmoother::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  if (msg->percentage)
  {
    RCLCPP_ERROR(this->get_logger(), "Percentage mode is not supported");
    return;
  }

  const double speed_magnitude = std::abs(msg->speed_limit);
  RCLCPP_INFO(this->get_logger(), "set symmetric speed limit = [-%0.3f, %0.3f]", speed_magnitude, speed_magnitude);
  target_maxvx_ = speed_magnitude;
  target_minvx_ = -speed_magnitude;
  limitv2target = true;
}

rcl_interfaces::msg::SetParametersResult VelocitySmoother::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 新模式的限制参数仅允许重新配置生命周期后变更，避免运动中改变可行域。
  if (jerk_limited_smoothing_)
  {
    for (const auto & parameter : parameters)
    {
      const auto & name = parameter.get_name();
      if (name == "jerk_limited_smoothing" || name == "max_accel_jerk" || name == "max_decel_jerk" || name == "max_accel" || name == "max_decel" || name == "max_velocity" || name == "min_velocity" || name == "deadband_velocity" || name == "smoothing_frequency" || name == "scale_velocities" || name == "immediate_stop_on_zero_command")
      {
        result.successful = false;
        result.reason = "S-curve constraints require lifecycle deactivate/cleanup/configure";
        return result;
      }
    }
  }
  for (const auto & parameter : parameters)
  {
    if (parameter.get_name() == "jerk_limited_smoothing" || parameter.get_name() == "max_accel_jerk" || parameter.get_name() == "max_decel_jerk")
    {
      result.successful = false;
      result.reason = "S-curve settings require lifecycle reconfiguration";
      return result;
    }
  }
  for (const auto & parameter : parameters)
  {
    if (parameter.get_name() == "feedback_correction_time" && parameter.get_type() == ParameterType::PARAMETER_DOUBLE && (!std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0))
    {
      result.successful = false;
      result.reason = "feedback_correction_time must be finite and greater than zero";
      return result;
    }
  }

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "smoothing_frequency") {
        smoothing_frequency_ = parameter.as_double();
        LOG_INFO("Updating smoothing_frequency to {}", smoothing_frequency_);
        if (timer_) {
          timer_->cancel();
          timer_.reset();
        }

        double timer_duration_ms = 1000.0 / smoothing_frequency_;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(timer_duration_ms)), std::bind(&VelocitySmoother::smootherTimer, this));
      } else if (name == "feedback_correction_time") {
        feedback_correction_time_ = parameter.as_double();
      } else if (name == "velocity_timeout") {
        velocity_timeout_ = rclcpp::Duration::from_seconds(parameter.as_double());
        LOG_INFO("Updating velocity_timeout to {}", parameter.as_double());
      } else if (name == "odom_duration") {
        odom_duration_ = parameter.as_double();
        LOG_INFO("Updating odom_duration to {}", odom_duration_);
        odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (parameter.as_double_array().size() != 3) {
        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 3", name.c_str());
        result.successful = false;
        break;
      }

      if (name == "max_velocity") {
        max_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating max_velocity limits");
      } else if (name == "min_velocity") {
        min_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating min_velocity limits");
      } else if (name == "max_accel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] < 0.0) {
            RCLCPP_WARN(get_logger(), "Negative values set of acceleration! These should be positive to speed up!");
            result.successful = false;
          }
        }
        max_accels_ = parameter.as_double_array();
        LOG_INFO("Updating max_accel limits");
      } else if (name == "max_decel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] > 0.0) {
            RCLCPP_WARN(get_logger(), "Positive values set of deceleration! These should be negative to slow down!");
            result.successful = false;
          }
        }
        max_decels_ = parameter.as_double_array();
        LOG_INFO("Updating max_decel limits");
      } else if (name == "deadband_velocity") {
        deadband_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating deadband_velocity limits");
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "immediate_stop_on_zero_command") {
        immediate_stop_on_zero_command_ = parameter.as_bool();
        LOG_INFO("Updating immediate_stop_on_zero_command to {}", immediate_stop_on_zero_command_);
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "feedback") {
        if (parameter.as_string() == "OPEN_LOOP") {
          open_loop_ = true;
          odom_smoother_.reset();
          LOG_INFO("Switching velocity smoother feedback to OPEN_LOOP");
        } else if (parameter.as_string() == "CLOSED_LOOP") {
          open_loop_ = false;
          odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
          LOG_INFO("Switching velocity smoother feedback to CLOSED_LOOP");
        } else {
          RCLCPP_WARN(get_logger(), "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
          result.successful = false;
          break;
        }
      } else if (name == "odom_topic") {
        odom_topic_ = parameter.as_string();
        LOG_INFO("Updating odom_topic to {}", odom_topic_.c_str());
        odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
      }
    }
  }

  return result;
}

}  // namespace nav2_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)
