// 离散 S 曲线：提前收回加速度，速度目标变化时保留当前加速度。
#ifndef NAV2_VELOCITY_SMOOTHER__JERK_PROFILE_HPP_
#define NAV2_VELOCITY_SMOOTHER__JERK_PROFILE_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace nav2_velocity_smoother
{
class JerkProfile
{
public:
  void reset()
  {
    acceleration_ = 0.0;
  }
  double acceleration() const
  {
    return acceleration_;
  }
  double advance(double velocity, double target, double dt, double accel, double decel, double accel_jerk, double decel_jerk, double minimum, double maximum)
  {
    // 采用两种工况中更严格的约束，保证反向及突然换目标时仍符合两个上限。
    const double amax = std::min(accel, decel);
    const double jerk = std::min(accel_jerk, decel_jerk);
    if (minimum == maximum)
    {
      reset();
      return minimum;
    }
    if (!(dt > 0.0 && amax > 0.0 && jerk > 0.0))
    {
      throw std::invalid_argument("Invalid S-curve limits");
    }
    target = std::clamp(target, minimum, maximum);
    const double change = jerk * dt;
    double lower = std::max(-amax, acceleration_ - change);
    double upper = std::min(amax, acceleration_ + change);
    // v + a*dt + a^2/(2*j) <= 速度上限：为加速度回零预留速度空间。
    const auto envelope = [change, jerk](double distance)
    {
      const double root = std::sqrt(change * change + 2.0 * jerk * std::max(0.0, distance));
      // 有理化表达式避免接近目标时相减损失精度。
      return 2.0 * jerk * std::max(0.0, distance) / (root + change);
    };
    lower = std::max(lower, -envelope(velocity - minimum));
    upper = std::min(upper, envelope(maximum - velocity));
    if (lower > upper + 1e-10)
    {
      throw std::runtime_error("S-curve state outside feasible speed bounds");
    }
    const double error = target - velocity;
    const double desired = error >= 0.0 ? envelope(error) : -envelope(-error);
    const double next_acceleration = std::max(lower, std::min(upper, desired));
    const double next = velocity + next_acceleration * dt;
    // 保存实际离散输出的加速度，确保最终一帧也受相同约束。
    acceleration_ = (next - velocity) / dt;
    return next;
  }
private:
  double acceleration_ = 0.0;
};
}
#endif
