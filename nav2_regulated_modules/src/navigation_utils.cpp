// 导航通用计算实现。封装姿态、距离及其他任务流程共用的计算。

#include "nav2_regulated_modules/navigation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nav2_regulated_modules
{
  namespace navigation_utils
{

// 将非负秒数拆成秒和纳秒；负输入按零处理。
builtin_interfaces::msg::Duration durationFromSeconds(const double seconds)
{
  const auto safe_seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(safe_seconds));
  duration.nanosec = static_cast<uint32_t>((safe_seconds - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}

// 仅计算平面位置距离，不将航向和高度计入结果。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

// 从姿态四元数求平面航向角，返回弧度。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 将角度规约到 [-π, π]，避免跨越边界时产生虚假的大角度差。
double normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 拒绝缺少坐标系、含非有限位置或零长度四元数的位姿。
bool validPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return !pose.header.frame_id.empty() && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) && norm > 1e-6;
}

}
// namespace navigation_utils
}
// namespace nav2_regulated_modules
