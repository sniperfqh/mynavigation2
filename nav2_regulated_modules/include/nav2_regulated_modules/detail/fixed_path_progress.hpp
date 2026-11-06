// 在原最近路径点相邻的有效线段上投影，计算剩余弧长；不改变最近点搜索或停车判据。
#ifndef NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_PROGRESS_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_PROGRESS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp"

namespace nav2_regulated_modules
{
namespace detail
{

// 最近点前后各选一条有效线段，重复点跳过；不另行搜索远处路径分支。
// 进度沿 Path 点序增加，与车辆前进／倒车的速度符号无关；无有效投影时返回 NaN。
inline double projectedRemainingDistance(const std::vector<StartPathPoint> & path, const std::size_t nearest_index, const double robot_x, const double robot_y)
{
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  if (path.size() < 2 || nearest_index >= path.size() || !std::isfinite(robot_x) || !std::isfinite(robot_y))
  {
    return invalid;
  }
  struct Projection
  {
    double distance_squared = std::numeric_limits<double>::infinity();
    double progress = 0.0;
  };
  Projection before;
  Projection after;
  bool after_found = false;
  double total_length = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared))
    {
      return invalid;
    }
    const double length = std::sqrt(length_squared);
    if (length_squared > 1e-12 && (index < nearest_index || !after_found))
    {
      const double offset_x = robot_x - path[index].x;
      const double offset_y = robot_y - path[index].y;
      const double fraction = std::clamp((offset_x * dx + offset_y * dy) / length_squared, 0.0, 1.0);
      const double error_x = offset_x - fraction * dx;
      const double error_y = offset_y - fraction * dy;
      auto & candidate = index < nearest_index ? before : after;
      candidate.distance_squared = error_x * error_x + error_y * error_y;
      candidate.progress = total_length + fraction * length;
      if (index >= nearest_index)
      {
        after_found = true;
      }
    }
    total_length += length;
  }
  const auto & projection = before.distance_squared <= after.distance_squared ? before : after;
  if (!std::isfinite(total_length) || !std::isfinite(projection.distance_squared))
  {
    return invalid;
  }
  // 只减去沿路径的投影进度，不把已走过的距离或横向偏差加回剩余距离。
  return std::max(0.0, total_length - projection.progress);
}

}  // namespace detail
}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_PROGRESS_HPP_
