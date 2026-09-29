#include <cmath>
#include <iostream>
#include <vector>

#include "nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp"

namespace
{

bool check(const bool condition, const char * message)
{
  if (!condition)
  {
    std::cerr << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  using nav2_regulated_modules::detail::FixedPathStartSpeedGuard;
  using nav2_regulated_modules::detail::StartPathPoint;
  using nav2_regulated_modules::detail::calculateStartRecoveryErrors;
  constexpr double pi = 3.14159265358979323846;
  constexpr double release_yaw = pi / 9.0;
  FixedPathStartSpeedGuard guard;
  if (!check(!guard.evaluateStart(0.20, 0.20), "0.20 m must not trigger the start cap"))
  {
    return 1;
  }
  guard.reset();
  if (!check(guard.evaluateStart(0.21, 0.20), "0.21 m must trigger the start cap"))
  {
    return 1;
  }
  for (int cycle = 0; cycle < 4; ++cycle)
  {
    if (!check(!guard.observe(0.20, release_yaw, 0.20, release_yaw, 5), "release must wait for five stable cycles"))
    {
      return 1;
    }
  }
  if (!check(!guard.observe(0.201, 0.0, 0.20, release_yaw, 5), "lateral deviation must reset the stable count"))
  {
    return 1;
  }
  if (!check(!guard.observe(0.10, release_yaw + 0.001, 0.20, release_yaw, 5), "heading deviation must reset the stable count"))
  {
    return 1;
  }
  for (int cycle = 0; cycle < 4; ++cycle)
  {
    if (!check(!guard.observe(0.20, release_yaw, 0.20, release_yaw, 5), "cap released too early"))
    {
      return 1;
    }
  }
  if (!check(guard.observe(0.20, release_yaw, 0.20, release_yaw, 5) && !guard.active(), "cap must release on the fifth stable cycle"))
  {
    return 1;
  }
  if (!check(!guard.evaluateStart(0.30, 0.20) && !guard.active(), "released cap must not re-arm on the same path"))
  {
    return 1;
  }
  guard.reset();
  if (!check(guard.evaluateStart(0.30, 0.20), "a new path must reset the start cap"))
  {
    return 1;
  }

  std::vector<StartPathPoint> curve;
  curve.emplace_back(0.0, 0.0);
  curve.emplace_back(1.0, 0.0);
  curve.emplace_back(1.0, 1.0);
  double lateral_error = 0.0;
  double heading_error = 0.0;
  if (!check(calculateStartRecoveryErrors(curve, 1, 1.1, 0.4, pi / 2.0, 1, lateral_error, heading_error) && std::abs(lateral_error - 0.1) < 1e-9 && heading_error < 1e-9, "forward recovery must use the nearest curved segment"))
  {
    return 1;
  }
  if (!check(calculateStartRecoveryErrors(curve, 1, 1.1, 0.4, -pi / 2.0, -1, lateral_error, heading_error) && std::abs(lateral_error - 0.1) < 1e-9 && heading_error < 1e-9, "reverse recovery must compare the vehicle backward axis"))
  {
    return 1;
  }
  std::vector<StartPathPoint> repeated_end;
  repeated_end.emplace_back(0.0, 0.0);
  repeated_end.emplace_back(1.0, 0.0);
  repeated_end.emplace_back(1.0, 0.0);
  if (!check(calculateStartRecoveryErrors(repeated_end, 2, 0.5, 0.1, 0.0, 1, lateral_error, heading_error) && std::abs(lateral_error - 0.1) < 1e-9, "a repeated terminal point must fall back to a valid segment"))
  {
    return 1;
  }
  std::vector<StartPathPoint> invalid_path;
  invalid_path.emplace_back(0.0, 0.0);
  invalid_path.emplace_back(0.0, 0.0);
  if (!check(!calculateStartRecoveryErrors(invalid_path, 0, 0.0, 0.0, 0.0, 1, lateral_error, heading_error), "a path without a valid tangent must not release the cap"))
  {
    return 1;
  }
  return 0;
}
