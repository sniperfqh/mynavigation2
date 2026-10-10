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

#ifndef NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
#define NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
// spdlog wrapper for module-specific logging
#include "spdlog_wrapper.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

#include <array>
#include <chrono>
#include "nav2_velocity_smoother/jerk_profile.hpp"

namespace nav2_velocity_smoother
{

/**
 * @class nav2_velocity_smoother::VelocitySmoother
 * @brief This class that smooths cmd_vel velocities for robot bases
 */
class VelocitySmoother : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_velocity_smoother::VelocitySmoother
   * @param options Additional options to control creation of the node.
   */
  explicit VelocitySmoother(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for nav2_velocity_smoother::VelocitySmoother
   */
  ~VelocitySmoother();

  /**
   * @brief Find the scale factor, eta, which scales axis into acceleration range
   * @param v_curr current velocity
   * @param v_cmd commanded velocity
   * @param accel maximum acceleration
   * @param decel maximum deceleration
   * @return Scale factor, eta
   */
  double findEtaConstraint(const double v_curr, const double v_cmd, const double accel, const double decel);

  /**
   * @brief Apply acceleration and scale factor constraints
   * @param v_curr current velocity
   * @param v_cmd commanded velocity
   * @param accel maximum acceleration
   * @param decel maximum deceleration
   * @param eta Scale factor
   * @return Velocity command
   */
  double applyConstraints(const double v_curr, const double v_cmd, const double accel, const double decel, const double eta);

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback for incoming velocity commands
   * @param msg Twist message
   */
  void inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Main worker timer function
   */
  void smootherTimer();

  // 用秒制时间常数校正连续速度斜坡，避免反馈误差形成与频率相关的速度上限。
  geometry_msgs::msg::Twist feedbackReference(const geometry_msgs::msg::Twist & feedback) const;

  /**
  * @brief Callback for speed limiting messages
  * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
  */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);

  /**
   * @brief Dynamic reconfigure callback
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Network interfaces
  std::unique_ptr<nav2_util::OdomSmoother> odom_smoother_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::Twist last_cmd_;
  geometry_msgs::msg::Twist::SharedPtr command_;

  // S 曲线使用上一条实际输出及其加速度，闭环校正不能直接覆盖历史状态。
  bool jerk_limited_smoothing_ = false;
  std::vector<double> max_accel_jerks_;
  std::vector<double> max_decel_jerks_;
  std::array<JerkProfile, 3> jerk_profiles_;
  std::chrono::steady_clock::time_point jerk_last_tick_;
  bool jerk_clock_initialized_ = false;

  // Parameters
  double smoothing_frequency_;
  double odom_duration_;
  double feedback_correction_time_ = 0.1;
  bool feedback_initialized_ = false;
  std::string odom_topic_;
  bool open_loop_;
  bool stopped_{true};
  bool scale_velocities_;
  bool immediate_stop_on_zero_command_ { false };
  std::vector<double> max_velocities_;
  std::vector<double> min_velocities_;
  double target_maxvx_;
  double target_minvx_;
  bool limitv2target{false};
  std::vector<double> max_accels_;
  std::vector<double> max_decels_;
  std::vector<double> deadband_velocities_;
  rclcpp::Duration velocity_timeout_{0, 0};
  rclcpp::Time last_command_time_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_velocity_smoother

#endif  // NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
