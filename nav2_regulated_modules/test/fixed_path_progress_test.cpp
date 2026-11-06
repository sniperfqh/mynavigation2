// 固定路径投影弧长回归：离散点切换、双向点序、转弯、重复点和自交局部性；不启动 ROS。
#include "nav2_regulated_modules/detail/fixed_path_progress.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using nav2_regulated_modules::detail::projectedRemainingDistance;
using nav2_regulated_modules::detail::StartPathPoint;
std::size_t checks = 0;

void require(const bool value, const std::string & message)
{
  ++checks;
  if (!value)
  {
    throw std::runtime_error(message);
  }
}

void near(const double actual, const double expected, const std::string & message)
{
  require(std::isfinite(actual) && std::abs(actual - expected) <= 1e-9, message);
}

// 已知在线段上的位置应满足：剩余弧长等于后续段长加当前段未走的部分。
void checkSegments(const std::vector<StartPathPoint> & path)
{
  double total = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    total += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
  }
  double prefix = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-6)
    {
      prefix += length;
      continue;
    }
    const auto check_fraction = [&](const double fraction)
    {
      const std::size_t nearest = fraction <= 0.5 ? index : index + 1;
      const double x = path[index].x + fraction * dx;
      const double y = path[index].y + fraction * dy;
      near(projectedRemainingDistance(path, nearest, x, y), total - prefix - fraction * length, "remaining arc changed with nearest point");
    };
    check_fraction(0.0);
    check_fraction(0.25);
    check_fraction(0.499999);
    check_fraction(0.500001);
    check_fraction(0.75);
    check_fraction(1.0);
    const double before = projectedRemainingDistance(path, index, path[index].x + 0.499999 * dx, path[index].y + 0.499999 * dy);
    const double after = projectedRemainingDistance(path, index + 1, path[index].x + 0.500001 * dx, path[index].y + 0.500001 * dy);
    near(before - after, 0.000002 * length, "nearest-point midpoint introduced a distance step");
    prefix += length;
  }
}

void straightPaths()
{
  const double start_x = 4.94;
  const double start_y = 9.41;
  const double dx = 8.31 - start_x;
  const double dy = 11.70 - start_y;
  const double length = std::hypot(dx, dy);
  for (int resolution = 0; resolution < 3; ++resolution)
  {
    const double step = resolution == 0 ? 0.1 : resolution == 1 ? 0.05 : 0.01;
    std::vector<StartPathPoint> path;
    for (double distance = 0.0; distance < length; distance += step)
    {
      path.emplace_back(start_x + dx * distance / length, start_y + dy * distance / length);
    }
    path.emplace_back(8.31, 11.70);
    checkSegments(path);
    std::reverse(path.begin(), path.end());
    checkSegments(path);
  }
}

void cornersAndCurves()
{
  std::vector<StartPathPoint> corner;
  corner.emplace_back(0.0, 0.0);
  corner.emplace_back(1.0, 0.0);
  corner.emplace_back(1.0, 1.0);
  corner.emplace_back(2.0, 1.0);
  checkSegments(corner);
  std::reverse(corner.begin(), corner.end());
  checkSegments(corner);
  std::vector<StartPathPoint> curve;
  for (int index = 0; index <= 32; ++index)
  {
    const double angle = std::acos(-1.0) * index / 64.0;
    curve.emplace_back(std::cos(angle), std::sin(angle));
  }
  checkSegments(curve);
  std::reverse(curve.begin(), curve.end());
  checkSegments(curve);
}

void duplicatesAndBounds()
{
  std::vector<StartPathPoint> path;
  path.emplace_back(0.0, 0.0);
  path.emplace_back(0.0, 0.0);
  path.emplace_back(1.0, 0.0);
  path.emplace_back(1.0, 0.0);
  path.emplace_back(2.0, 0.0);
  path.emplace_back(2.0, 0.0);
  checkSegments(path);
  for (std::size_t nearest = 2; nearest <= 3; ++nearest)
  {
    near(projectedRemainingDistance(path, nearest, 0.75, 0.3), 1.25, "lateral offset or duplicate point changed arc progress");
  }
  near(projectedRemainingDistance(path, 0, -0.2, 0.0), 2.0, "position before path should clamp to its start");
  near(projectedRemainingDistance(path, 5, 2.2, 0.0), 0.0, "position after path should clamp to its end");
}

void localProjectionAtCrossing()
{
  std::vector<StartPathPoint> path;
  path.emplace_back(-1.0, 0.0);
  path.emplace_back(0.0, 0.0);
  path.emplace_back(1.0, 0.0);
  path.emplace_back(1.0, 1.0);
  path.emplace_back(0.0, 1.0);
  path.emplace_back(0.0, 0.0);
  path.emplace_back(0.0, -1.0);
  near(projectedRemainingDistance(path, 1, 0.0, 0.01), 5.0, "projection jumped to a distant branch at crossing");
  near(projectedRemainingDistance(path, 5, 0.0, -0.01), 0.99, "later crossing should use the caller's nearest-point anchor");
}

void invalidGeometry()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<StartPathPoint> path;
  require(std::isnan(projectedRemainingDistance(path, 0, 0.0, 0.0)), "empty path accepted");
  path.emplace_back(0.0, 0.0);
  require(std::isnan(projectedRemainingDistance(path, 0, 0.0, 0.0)), "single-point path accepted");
  path.emplace_back(0.0, 0.0);
  require(std::isnan(projectedRemainingDistance(path, 0, 0.0, 0.0)), "zero-length path accepted");
  path.back().x = 1.0;
  require(std::isnan(projectedRemainingDistance(path, path.size(), 0.0, 0.0)), "invalid nearest index accepted");
  require(std::isnan(projectedRemainingDistance(path, 0, nan, 0.0)), "non-finite robot position accepted");
  path.back().x = nan;
  require(std::isnan(projectedRemainingDistance(path, 0, 0.0, 0.0)), "non-finite path accepted");
}
}

int main()
{
  try
  {
    straightPaths();
    cornersAndCurves();
    duplicatesAndBounds();
    localProjectionAtCrossing();
    invalidGeometry();
    std::cout << "PASS fixed_path_progress checks=" << checks << '\n';
    return 0;
  }
  catch (const std::exception & error)
  {
    std::cerr << "FAIL fixed_path_progress: " << error.what() << '\n';
    return 1;
  }
}
