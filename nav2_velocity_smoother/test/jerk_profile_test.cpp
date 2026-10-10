#include <cmath>
#include <iostream>
#include <stdexcept>
#include "nav2_velocity_smoother/jerk_profile.hpp"
using nav2_velocity_smoother::JerkProfile;
void require(bool condition)
{
  if (!condition)
  {
    throw std::runtime_error("S-curve constraint assertion failed");
  }
}
int main()
{
  for (int mode = 0; mode < 3; ++mode)
  {
    JerkProfile curve;
    double velocity = 0.0;
    double acceleration = 0.0;
    const double targets[] =
    {
      .3, 1.5, .4, 0.0, -.3, -1.5, 0.0
    };
    for (double target : targets)
    {
      bool reached = false;
      for (int step = 0; step < 5000; ++step)
      {
        const double dt = mode == 0 ? .01 : mode == 1 ? (step % 2 ? .009 : .011) : (step % 2 ? .002 : .04);
        const double next = curve.advance(velocity, target, dt, 2.5, 1.8, 6.0, 8.0, -1.5, 1.5);
        const double a = (next - velocity) / dt;
        const double j = (a - acceleration) / dt;
        require(std::abs(a) <= 1.8 + 1e-8 && std::abs(j) <= 6.0 + 1e-7 && std::abs(next) <= 1.5 + 1e-9);
        velocity = next;
        acceleration = a;
        if (std::abs(velocity - target) < 1e-10 && std::abs(acceleration) < 1e-8)
        {
          reached = true;
          break;
        }
      }
      require(reached);
    }
  }
  // 目标在加速度尚未回零时突变，检查过渡而非强制重置加速度。
  JerkProfile curve;
  double v = 0.0;
  double previous_a = 0.0;
  for (int i = 0; i < 3000; ++i)
  {
    const double target = i < 25 ? 1.5 : i < 70 ? 0.0 : i < 300 ? -.3 : i < 800 ? 1.5 : 0.0;
    const double next = curve.advance(v, target, .01, 2.5, 2.5, 6.0, 8.0, -1.5, 1.5);
    const double a = (next - v) / .01;
    require(std::abs(a) <= 2.5 + 1e-8 && std::abs((a - previous_a) / .01) <= 6.0 + 1e-7 && std::abs(next) <= 1.5 + 1e-9);
    previous_a = a;
    v = next;
  }
  require(std::abs(v) < 1e-9 && std::abs(previous_a) < 1e-9);
  JerkProfile idle;
  require(idle.advance(0.0, 0.0, .01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) == 0.0);
  std::cout << "PASS start/deceleration/stop/reverse/jitter/retarget/disabled-axis bounds" << std::endl;
}
