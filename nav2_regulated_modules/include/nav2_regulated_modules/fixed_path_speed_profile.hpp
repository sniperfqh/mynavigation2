// 可选自适应终点制动的纯纵向计算。速度、距离、减速度和 jerk 分别为 m/s、m、m/s²、m/s³。
// 本头文件不读参数也不发布速度；上层只有显式选择对应分支才使用这些包络和积分结果。
#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_regulated_modules
{

// 纯纵向计算，不依赖 ROS；控制器和单元测试使用同一公式。
// deceleration 是本轮命令减速度上限 m/s²，stopping_distance 是含预留的估算停车距离 m，speed_limit 是包络上限 m/s。
// 返回全零表示三个运动输入校验失败，不是已经执行停车。
struct FixedPathSpeedProfile
{
  double deceleration;
  double stopping_distance;
  double speed_limit;
};

// 先按速度插值得到命令减速度，再以不超过 0.4 m/s² 的规划减速度估算停车距离。
// 预留 = 目标容差 + 距离裕量 + 速度×响应时间 + jerk 预留；可用距离再反推速度包络。
// 仅先检查三个运动输入，减速度／jerk 等约束须由调用方保证有效，不能误写为函数完成所有参数校验。
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

// 由速度差和 dt 求请求减速度，再限制它相对上周期的变化不超过 jerk_limit×dt。
// 输出不低于 target_speed，next_deceleration 回传下周期状态；dt 必须由调用方保证为正。
inline double advanceFixedPathBrakingSpeed(const double previous_speed, const double target_speed, const double previous_deceleration, const double max_deceleration, const double jerk_limit, const double dt, double & next_deceleration)
{
  const double requested_deceleration = std::clamp((previous_speed - target_speed) / dt, 0.0, max_deceleration);
  next_deceleration = std::clamp(requested_deceleration, std::max(0.0, previous_deceleration - jerk_limit * dt), std::min(max_deceleration, previous_deceleration + jerk_limit * dt));
  return std::max(target_speed, previous_speed - next_deceleration * dt);
}

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
