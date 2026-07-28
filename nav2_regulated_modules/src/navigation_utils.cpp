#include "nav2_regulated_modules/navigation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nav2_regulated_modules
{
namespace navigation_utils
{

// 中文注释：把秒数安全转换为 ROS Duration；负值按零处理，整数与纳秒部分分别写入消息。
builtin_interfaces::msg::Duration durationFromSeconds(const double seconds) {
  const auto safe_seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(safe_seconds));
  duration.nanosec = static_cast<uint32_t>((safe_seconds - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}

// 中文注释：计算两个导航位姿在 XY 平面的直线距离，用于进展、路点通过和定位跳变判断。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second) {
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

// 中文注释：按标准四元数公式提取绕 Z 轴偏航角。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose) {
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 中文注释：循环加减 2π，把任意角度收敛到 [-π, π]。
double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 中文注释：拒绝空坐标系、非有限位置或零范数／非有限四元数，避免无效 Goal 进入 Nav2。
bool validPose(const geometry_msgs::msg::PoseStamped & pose) {
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return !pose.header.frame_id.empty() && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) && norm > 1e-6;
}

}  // namespace navigation_utils
}  // namespace nav2_regulated_modules
