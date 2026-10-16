// 基础转换与二维几何实现，供目标校验、进度和日志共用。
// 这些函数不查询 TF；计算距离前应确保两个 Pose 位于相同坐标系。

#include "nav2_regulated_modules/navigation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nav2_regulated_modules
{
  namespace navigation_utils
{

// 把秒数转换为 ROS Duration 消息，不读取系统时钟；输入的业务合法性由调用方负责。
builtin_interfaces::msg::Duration durationFromSeconds(const double seconds)
{
  const auto safe_seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(safe_seconds));
  duration.nanosec = static_cast<uint32_t>((safe_seconds - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}

// 对同坐标系的两点计算 XY 平面距离，单位 m；忽略高度。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

// 从四元数取平面 yaw，单位 rad，不查询 TF。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 把角差折回主值区间，避免 ±π 边界把小误差计算成接近一整圈。
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

// 检查 frame_id、坐标与四元数的实现约束；通过并不等于目标可规划或 TF 可用。
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
