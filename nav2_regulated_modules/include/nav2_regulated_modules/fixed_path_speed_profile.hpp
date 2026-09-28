#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_regulated_modules
{

// 纯纵向计算，不依赖 ROS；控制器和单元测试使用同一公式。
struct FixedPathSpeedProfile
{
  double deceleration;
  double stopping_distance;
  double speed_limit;
};

// 命令减速度可随速度增大，但停车距离按更保守的有效减速度估算。
inline FixedPathSpeedProfile calculateFixedPathSpeedProfile(const double measured_speed, const double remaining_distance, const double nominal_speed, const double max_deceleration, const double jerk_limit, const double response_time, const double distance_margin, const double goal_tolerance, const double approach_speed)
{
  if (!std::isfinite(measured_speed) || !std::isfinite(remaining_distance) || !std::isfinite(nominal_speed) || measured_speed < 0.0 || remaining_distance < 0.0 || nominal_speed < 0.0)
  {
    return {0.0, 0.0, 0.0};
  }
  const double minimum_deceleration = std::min(0.25, max_deceleration);
  const double speed_fraction = std::clamp((measured_speed - 0.2) / 1.3, 0.0, 1.0);
  const double deceleration = minimum_deceleration + (max_deceleration - minimum_deceleration) * speed_fraction;
  const double planning_deceleration = std::min(deceleration, 0.4);
  const double jerk_distance = measured_speed * planning_deceleration / (2.0 * jerk_limit);
  const double guard_distance = goal_tolerance + distance_margin + measured_speed * response_time + jerk_distance;
  const double stopping_distance = measured_speed * measured_speed / (2.0 * planning_deceleration) + guard_distance;
  const double available_distance = std::max(0.0, remaining_distance - guard_distance);
  const double braking_speed = std::sqrt(2.0 * planning_deceleration * available_distance);
  const double speed_limit = std::min(nominal_speed, std::max(std::min(approach_speed, nominal_speed), braking_speed));
  return {deceleration, stopping_distance, speed_limit};
}

// 只对正常制动输出做单向速度下降；加速仍由现有速度平滑器处理。
inline double advanceFixedPathBrakingSpeed(const double previous_speed, const double target_speed, const double previous_deceleration, const double max_deceleration, const double jerk_limit, const double dt, double & next_deceleration)
{
  const double requested_deceleration = std::clamp((previous_speed - target_speed) / dt, 0.0, max_deceleration);
  next_deceleration = std::clamp(requested_deceleration, std::max(0.0, previous_deceleration - jerk_limit * dt), std::min(max_deceleration, previous_deceleration + jerk_limit * dt));
  return std::max(target_speed, previous_speed - next_deceleration * dt);
}

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
