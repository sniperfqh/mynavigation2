// 每条新路径的起步保护与局部几何误差计算。保护距离按路径投影进度（m）判断，航向误差使用 rad。
// 保护先启用，走满距离且误差连续达标才单向解除；新路径必须 reset，不能借旧任务稳定计数解除限速。
#ifndef NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace nav2_regulated_modules
{
namespace detail
{

struct StartPathPoint
{
  // 保存二维路径点，point_x／point_y 单位 m；构造时不检查有限性或连接关系。
  StartPathPoint(const double point_x, const double point_y)
  : x(point_x), y(point_y)
  {
  }

  double x;
  double y;
};

// 实测速度非有限或其绝对值超过新路径上限时要求先预减速。
// 仅报告是否需要制动，速度曲线由调用方与平滑器推进。
inline bool requiresStartPrebraking(const double measured_speed, const double speed_limit)
{
  return !std::isfinite(measured_speed) || std::abs(measured_speed) > speed_limit;
}

// 把当前位置投影到前向候选有效线段，输出横向距离 m、航向绝对误差 rad 和可选弧长进度 m。
// 倒车比较车体反向轴；末段全为重复点时回退有效切线，完全无有效线段返回 false。
inline bool calculateStartRecoveryErrors(const std::vector<StartPathPoint> & path, const std::size_t nearest_index, const double robot_x, const double robot_y, const double vehicle_yaw, const int direction_sign, double & lateral_error, double & heading_error, double * progress = nullptr)
{
  if (path.size() < 2 || !std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(vehicle_yaw) || (direction_sign != 1 && direction_sign != -1))
  {
    return false;
  }

  constexpr double pi = 3.14159265358979323846;
  const std::size_t first_segment = std::min(nearest_index > 0 ? nearest_index - 1 : 0, path.size() - 2);
  double best_distance_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  double prefix_length = 0.0;
  auto consider_segment = [&](const std::size_t index)
  {
    const auto & start = path[index];
    const auto & end = path[index + 1];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared) || length_squared <= 1e-12)
    {
      return;
    }
    const double offset_x = robot_x - start.x;
    const double offset_y = robot_y - start.y;
    // 投影参数限制在 [0,1]：先求在线段上的最近点，再比较各有效线段距离，避免无限直线延长造成假进度。
    const double projection = std::clamp((offset_x * dx + offset_y * dy) / length_squared, 0.0, 1.0);
    const double closest_x = start.x + projection * dx;
    const double closest_y = start.y + projection * dy;
    const double distance_squared = (robot_x - closest_x) * (robot_x - closest_x) + (robot_y - closest_y) * (robot_y - closest_y);
    if (!std::isfinite(distance_squared) || distance_squared >= best_distance_squared)
    {
      return;
    }
    best_distance_squared = distance_squared;
    lateral_error = std::abs(dx * offset_y - dy * offset_x) / std::sqrt(length_squared);
    const double path_yaw = std::atan2(dy, dx);
    // 倒车运动轴 = 车头 yaw＋π；用运动方向与路径切线比较，而不是始终以车头正向比较。
    const double motion_yaw = vehicle_yaw + (direction_sign < 0 ? pi : 0.0);
    const double yaw_difference = path_yaw - motion_yaw;
    heading_error = std::abs(std::atan2(std::sin(yaw_difference), std::cos(yaw_difference)));
    if (progress)
    {
      *progress = prefix_length + projection * std::sqrt(length_squared);
    }
    found = true;
  };

  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    if (index >= first_segment)
    {
      consider_segment(index);
    }
    prefix_length += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
  }
  // 前向候选没有有效段时，在先前线段中重新选最近有效段及其切线。
  if (!found)
  {
    prefix_length = 0.0;
    for (std::size_t index = 0; index < first_segment; ++index)
    {
      consider_segment(index);
      prefix_length += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
    }
  }
  return found && (!progress || std::isfinite(*progress));
}

// 每条新路径统一限速；距离与误差同时达标后单向解除。
class FixedPathStartSpeedGuard
{
public:
  // 新路径开始时清空评估／稳定／初始进度状态；同路径已解除状态不会在 observe 中重入。
  void reset()
  {
    evaluated_ = false;
    active_ = false;
    stable_cycles_ = 0;
    initial_progress_ = std::numeric_limits<double>::quiet_NaN();
  }

  // 每条路径首次评估时启用起步保护，已评估的同一路径不重复启用。
  bool evaluateStart()
  {
    if (!evaluated_)
    {
      evaluated_ = true;
      active_ = true;
    }
    return active_;
  }

  // 先走满相对初始投影进度的保护距离，再检查横向／航向并累计连续周期。
  // 进度无效、回到保护区或误差越界会重置稳定计数；达标只在本路径上单向解除。
  bool observe(const double lateral_error, const double heading_error, const double lateral_tolerance, const double heading_tolerance, const int required_stable_cycles, const double progress, const double protected_distance)
  {
    if (!active_)
    {
      return false;
    }
    if (!std::isfinite(progress) || progress < 0.0)
    {
      resetStability();
      return false;
    }
    if (!std::isfinite(initial_progress_))
    {
      initial_progress_ = progress;
    }
    // 保护结束位置同时受“从首次有效投影再走一段”与“至少到绝对保护距离”限制。
    if (progress < std::max(initial_progress_ + protected_distance, protected_distance) || !std::isfinite(lateral_error) || !std::isfinite(heading_error) || lateral_error > lateral_tolerance || heading_error > heading_tolerance)
    {
      stable_cycles_ = 0;
      return false;
    }
    ++stable_cycles_;
    if (stable_cycles_ < required_stable_cycles)
    {
      return false;
    }
    active_ = false;
    return true;
  }

  // 仅清零连续达标计数，不重新启用已解除的起步限速，也不清理新路径基准。
  void resetStability()
  {
    stable_cycles_ = 0;
  }

  // 读取起步限速是否尚生效，不代表当前速度已经受限到该值。
  bool active() const
  {
    return active_;
  }

private:
  bool evaluated_ = false;
  bool active_ = false;
  int stable_cycles_ = 0;
  double initial_progress_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace detail
}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_
