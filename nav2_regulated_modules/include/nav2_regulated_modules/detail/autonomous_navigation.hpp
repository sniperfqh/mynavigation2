#ifndef NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_NAVIGATION_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_NAVIGATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace nav2_regulated_modules::detail
{

inline bool freshAutonomousTransform(const double age, const double max_age)
{
  return std::isfinite(age) && age >= -0.05 && age <= max_age;
}

inline double autonomousFeedbackDistance(const double path_remaining, const double goal_distance)
{
  if (!std::isfinite(goal_distance) || goal_distance < 0.0)
  {
    throw std::invalid_argument("Invalid autonomous feedback goal distance");
  }
  return std::max(std::isfinite(path_remaining) && path_remaining >= 0.0 ? path_remaining : 0.0, goal_distance);
}

inline double autonomousFeedbackEta(const double remaining, const double yaw_error, const double speed)
{
  if (!std::isfinite(remaining) || remaining < 0.0 || !std::isfinite(yaw_error))
  {
    throw std::invalid_argument("Invalid autonomous ETA geometry");
  }
  // 低速／原地转向也保留航向及停稳估计；这是估计值，不替代目标检查器。
  const double observed_speed = std::isfinite(speed) ? std::abs(speed) : 0.0;
  const double travel_speed = observed_speed > 0.001 ? observed_speed : (remaining <= 0.15 ? 0.05 : 0.2);
  const double estimate = remaining / travel_speed + std::abs(yaw_error) / 0.3 + 0.5;
  return std::min(estimate, static_cast<double>(std::numeric_limits<int32_t>::max()) - 1.0);
}

inline int16_t autonomousFeedbackRecoveries(const int control, const int localization)
{
  const int64_t total = static_cast<int64_t>(std::max(0, control)) + static_cast<int64_t>(std::max(0, localization));
  return static_cast<int16_t>(std::min<int64_t>(total, std::numeric_limits<int16_t>::max()));
}

inline const char * autonomousArrivalBlocker(const bool position_ok, const bool yaw_ok, const bool velocity_ok, const bool stable)
{
  if (!position_ok)
  {
    return "POSITION";
  }
  if (!yaw_ok)
  {
    return "HEADING";
  }
  if (!velocity_ok)
  {
    return "VELOCITY";
  }
  return stable ? "READY" : "SETTLING";
}

// 只在自主模式调用；旋转必须使朝向误差下降，避免来回摆动一直刷新进度。
inline bool autonomousProgress(const double translation, const double heading_improvement, const double distance_improvement, const double goal_distance, const double min_translation, const double min_rotation, const double min_goal_improvement)
{
  return translation >= min_translation || heading_improvement >= min_rotation || (goal_distance <= 0.3 && distance_improvement >= min_goal_improvement);
}

// 在进入位置容差之前连续降低采样约束；保持曲率，交由 MPPI 重新求解。
inline double autonomousApproachLimit(const double distance, const double tolerance, const double nominal, const double approach_distance)
{
  if (distance >= approach_distance)
  {
    return nominal;
  }
  const double far_speed = std::min(nominal, 0.15);
  const double near_speed = std::min(nominal, 0.05);
  const double final_speed = std::min(nominal, 0.03);
  if (distance >= 0.3)
  {
    return far_speed + (nominal - far_speed) * std::clamp((distance - 0.3) / (approach_distance - 0.3), 0.0, 1.0);
  }
  if (distance >= 0.1)
  {
    return near_speed + (far_speed - near_speed) * (distance - 0.1) / 0.2;
  }
  return final_speed + (near_speed - final_speed) * std::clamp((distance - tolerance) / std::max(0.001, 0.1 - tolerance), 0.0, 1.0);
}

inline double autonomousHeadingCommand(const double error, const double measured, const double max_speed, const double acceleration, const double period, const double tolerance)
{
  // 留出下游平滑和反馈延迟的制动角度；目标越过后先令零速，不立即反转。
  const double remaining = std::abs(error) - 0.6 * tolerance - 0.2 * std::abs(measured);
  if (remaining <= 0.0)
  {
    return 0.0;
  }
  if (error * measured < -0.0001)
  {
    return 0.0;
  }
  const double cap = std::min(max_speed, std::sqrt(acceleration * remaining));
  const double desired = std::clamp(1.5 * error, -cap, cap);
  const double limited = std::clamp(desired, measured - acceleration * period, measured + acceleration * period);
  return error > 0.0 ? std::max(0.0, limited) : std::min(0.0, limited);
}

inline bool autonomousSettled(const bool valid, const double now, const double duration, std::optional<double> & since)
{
  if (!valid || !std::isfinite(now))
  {
    since.reset();
    return false;
  }
  if (!since || now < *since)
  {
    since = now;
  }
  return now - *since >= duration;
}

enum class AutonomousTerminalPhase
{
  TRACK,
  ALIGN_POSITION,
  POSITION,
  BRAKE,
  ALIGN_GOAL,
  SETTLE
};

inline const char * autonomousTerminalPhaseName(const AutonomousTerminalPhase phase)
{
  switch (phase)
  {
    case AutonomousTerminalPhase::TRACK:
      return "TRACK";
    case AutonomousTerminalPhase::ALIGN_POSITION:
      return "ALIGN_POSITION";
    case AutonomousTerminalPhase::POSITION:
      return "POSITION";
    case AutonomousTerminalPhase::BRAKE:
      return "BRAKE";
    case AutonomousTerminalPhase::ALIGN_GOAL:
      return "ALIGN_GOAL";
    case AutonomousTerminalPhase::SETTLE:
      return "SETTLE";
  }
  return "UNKNOWN";
}

struct AutonomousTerminalState
{
  AutonomousTerminalPhase phase = AutonomousTerminalPhase::TRACK;
  std::optional<double> brake_since;
};

struct AutonomousTerminalCommand
{
  double linear = 0.0;
  double angular = 0.0;
};

// 同一目标换路径不清空该状态；位置超出真实容差立即重新收敛，不锁存成功。
inline AutonomousTerminalCommand autonomousTerminalCommand(AutonomousTerminalState & state, const double distance, const double bearing_error, const double yaw_error, const double measured_linear, const double measured_angular, const double xy_tolerance, const double yaw_tolerance, const double stopped_linear, const double stopped_angular, const double now, const double enter_distance, const double max_linear, const double max_angular, const double angular_acceleration, const double period)
{
  using Phase = AutonomousTerminalPhase;
  if (!std::isfinite(distance) || !std::isfinite(bearing_error) || !std::isfinite(yaw_error) || !std::isfinite(measured_linear) || !std::isfinite(measured_angular) || !std::isfinite(now))
  {
    state.brake_since.reset();
    throw std::invalid_argument("Non-finite autonomous terminal feedback");
  }
  if (distance > enter_distance + 0.1)
  {
    state = AutonomousTerminalState();
  }
  if (state.phase == Phase::TRACK)
  {
    if (distance > enter_distance)
    {
      return AutonomousTerminalCommand();
    }
    state.phase = Phase::ALIGN_POSITION;
  }
  if ((state.phase == Phase::BRAKE || state.phase == Phase::ALIGN_GOAL || state.phase == Phase::SETTLE) && distance > xy_tolerance)
  {
    state.phase = Phase::ALIGN_POSITION;
    state.brake_since.reset();
  }
  if ((state.phase == Phase::ALIGN_POSITION || state.phase == Phase::POSITION) && distance <= 0.5 * xy_tolerance)
  {
    state.phase = Phase::BRAKE;
    state.brake_since.reset();
  }
  if (state.phase == Phase::BRAKE)
  {
    if (!autonomousSettled(std::abs(measured_linear) <= stopped_linear && std::abs(measured_angular) <= stopped_angular, now, 0.2, state.brake_since))
    {
      return AutonomousTerminalCommand();
    }
    state.phase = Phase::ALIGN_GOAL;
  }
  if (state.phase == Phase::SETTLE && std::abs(yaw_error) > yaw_tolerance)
  {
    state.phase = Phase::ALIGN_GOAL;
  }
  if (state.phase == Phase::ALIGN_GOAL && std::abs(yaw_error) <= 0.6 * yaw_tolerance)
  {
    state.phase = Phase::SETTLE;
  }
  if (state.phase == Phase::SETTLE)
  {
    return AutonomousTerminalCommand();
  }
  AutonomousTerminalCommand command;
  if (state.phase == Phase::ALIGN_GOAL)
  {
    command.angular = std::clamp(autonomousHeadingCommand(yaw_error, measured_angular, max_angular, angular_acceleration, period, yaw_tolerance), -max_angular, max_angular);
    return command;
  }
  constexpr double align_enter = 0.3490658503988659;
  constexpr double align_exit = 0.17453292519943295;
  if (state.phase == Phase::POSITION && std::abs(bearing_error) > align_enter)
  {
    state.phase = Phase::ALIGN_POSITION;
  }
  if (state.phase == Phase::ALIGN_POSITION && std::abs(bearing_error) < align_exit)
  {
    state.phase = Phase::POSITION;
  }
  command.angular = std::clamp(autonomousHeadingCommand(bearing_error, measured_angular, max_angular, angular_acceleration, period, 0.05), -max_angular, max_angular);
  if (state.phase == Phase::POSITION)
  {
    const double braking_cap = std::sqrt(2.0 * 0.25 * std::max(0.0, distance - xy_tolerance / 3.0));
    command.linear = std::min(max_linear, std::min(std::max(0.01, 0.8 * distance), braking_cap)) * std::max(0.0, std::cos(bearing_error));
  }
  return command;
}

inline bool autonomousTerminalProgressHeading(const double distance, const double xy_tolerance, const bool previous)
{
  return previous ? distance <= xy_tolerance : distance <= 0.5 * xy_tolerance;
}

// 回调以实际足迹查询代价；此函数只推演运动与制动，不持有 ROS 节点。
template<typename PoseIsFree>
bool autonomousTerminalTrajectory(double x, double y, double heading, double linear, double angular, const double target_linear, const double target_angular, const double angular_acceleration, const double period, const PoseIsFree & free)
{
  const double max_linear = std::max(std::abs(linear), std::abs(target_linear));
  const double max_angular = std::max(std::abs(angular), std::abs(target_angular));
  const double reaction = 0.1 + std::max(0.25 / 8.0, angular_acceleration / 6.0);
  const double horizon = 0.5 + reaction + std::max(max_linear / 0.25, max_angular / angular_acceleration);
  const double dt = std::min(period, 0.02);
  for (double elapsed = 0.0; elapsed <= horizon + dt; elapsed += dt)
  {
    if (!free(x, y, heading))
    {
      return false;
    }
    if (elapsed >= reaction)
    {
      const double desired_linear = elapsed < 0.5 + reaction ? target_linear : 0.0;
      const double desired_angular = elapsed < 0.5 + reaction ? target_angular : 0.0;
      linear += std::clamp(desired_linear - linear, -0.25 * dt, 0.25 * dt);
      angular += std::clamp(desired_angular - angular, -angular_acceleration * dt, angular_acceleration * dt);
    }
    x += linear * std::cos(heading + 0.5 * angular * dt) * dt;
    y += linear * std::sin(heading + 0.5 * angular * dt) * dt;
    heading += angular * dt;
  }
  return true;
}

}  // namespace nav2_regulated_modules::detail

#endif  // NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_NAVIGATION_HPP_
