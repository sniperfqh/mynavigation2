#ifndef NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_TERMINAL_TRIM_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_TERMINAL_TRIM_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

namespace nav2_regulated_modules::detail
{
struct FixedTrimLimits
{
  double max_speed = 0.02;
  double max_step = 0.02;
  double max_distance = 0.03;
  double timeout = 5.0;
  unsigned int max_attempts = 2;
};

// 时间使用单调时钟；平移预算累计实测速度，定位跳变不能重置或增加可用预算。
struct FixedTrimState
{
  bool active = false;
  unsigned int attempts = 0;
  double distance = 0.0;
  double step_distance = 0.0;
  std::optional<double> started;
  std::optional<double> last_time;

  void reset()
  {
    *this = FixedTrimState();
  }
};

inline bool accountFixedTrim(FixedTrimState & state, const double now, const double linear)
{
  if (!std::isfinite(now) || !std::isfinite(linear))
  {
    return false;
  }
  if (state.started && state.last_time)
  {
    const double dt = now - *state.last_time;
    if (dt < 0.0 || dt > 0.2)
    {
      return false;
    }
    // 完成微调后的制动滑行也计入预算；期间原跟踪器不再发出平移指令。
    state.distance += std::abs(linear) * dt;
    state.step_distance += std::abs(linear) * dt;
  }
  state.last_time = now;
  return true;
}

inline bool fixedTrimWithinBudget(const FixedTrimState & state, const FixedTrimLimits & limits, const double now)
{
  return state.distance < limits.max_distance && state.step_distance < limits.max_step && (!state.started || (now >= *state.started && now - *state.started < limits.timeout));
}

inline bool beginFixedTrim(FixedTrimState & state, const FixedTrimLimits & limits, const double now)
{
  if (!std::isfinite(now) || state.active || state.attempts >= limits.max_attempts || state.distance >= limits.max_distance || (state.started && (now < *state.started || now - *state.started >= limits.timeout)))
  {
    return false;
  }
  state.started = state.started ? state.started : std::optional<double>(now);
  state.last_time = now;
  state.step_distance = 0.0;
  ++state.attempts;
  state.active = true;
  return true;
}

// 已在正确行驶朝向内才给平移；速度同时受停车余量、外部限速和剩余位移预算约束。
inline double fixedTrimLinear(const FixedTrimState & state, const FixedTrimLimits & limits, const double distance, const double tolerance, const double bearing_error, const double measured, const double angular, const int direction, const double speed_limit, const double period)
{
  if (!state.active || (direction != 1 && direction != -1) || !std::isfinite(distance) || !std::isfinite(tolerance) || !std::isfinite(bearing_error) || !std::isfinite(measured) || !std::isfinite(angular) || !std::isfinite(speed_limit) || !std::isfinite(period) || tolerance <= 0.0 || period <= 0.0 || speed_limit <= 0.0 || std::abs(bearing_error) > 0.08 || std::abs(angular) > 0.08 || measured * direction < -0.001)
  {
    return 0.0;
  }
  const double remaining = distance - 0.5 * tolerance - 0.2 * std::abs(measured);
  const double budget = std::min(limits.max_step - state.step_distance, limits.max_distance - state.distance);
  if (remaining <= 0.0 || budget <= 0.0)
  {
    return 0.0;
  }
  const double nominal = std::max(0.005, 0.8 * distance);
  const double cap = std::min(std::min(limits.max_speed, speed_limit), std::min(std::sqrt(0.5 * remaining), budget / 0.5));
  const double accelerated = std::max(0.0, measured * direction + 0.25 * period);
  return direction * std::min(nominal, std::min(cap, accelerated));
}
}
#endif
