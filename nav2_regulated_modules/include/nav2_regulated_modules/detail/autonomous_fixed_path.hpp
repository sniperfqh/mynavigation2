#ifndef NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_PATH_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_PATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules::detail
{
struct FixedSegment
{
  std::size_t first = 0;
  std::size_t last = 0;
  int direction = 0;
  bool rotation = false;
};

inline double fixedDistance(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

inline double fixedAngle(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

// 原始姿态编码车头方向；重复位置不能简单删除，否则会丢失原地旋转。
inline std::vector<FixedSegment> splitFixedPath(const nav_msgs::msg::Path & path)
{
  if (path.header.frame_id.empty() || path.poses.empty())
  {
    throw std::invalid_argument("Frozen path is empty or has no frame");
  }
  for (const auto & point : path.poses)
  {
    const auto & p = point.pose.position;
    const auto & q = point.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if ((!point.header.frame_id.empty() && point.header.frame_id != path.header.frame_id) || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
    {
      throw std::invalid_argument("Frozen path contains invalid pose or mixed frames");
    }
  }
  std::vector<FixedSegment> segments;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i)
  {
    const auto & a = path.poses[i];
    const auto & b = path.poses[i + 1];
    const double distance = fixedDistance(a, b);
    const bool rotation = distance <= 1e-6 && std::abs(fixedAngle(tf2::getYaw(b.pose.orientation) - tf2::getYaw(a.pose.orientation))) > 1e-4;
    if (distance <= 1e-6 && !rotation)
    {
      continue;
    }
    if (distance > 1.0)
    {
      throw std::invalid_argument("Frozen path contains a discontinuity larger than 1m");
    }
    int direction = 0;
    if (!rotation)
    {
      const double tangent = std::atan2(b.pose.position.y - a.pose.position.y, b.pose.position.x - a.pose.position.x);
      const double cosine = std::cos(fixedAngle(tf2::getYaw(a.pose.orientation) - tangent));
      if (std::abs(cosine) < 0.5)
      {
        throw std::invalid_argument("Frozen path has ambiguous driving direction");
      }
      direction = cosine > 0.0 ? 1 : -1;
    }
    if (!segments.empty() && !rotation && !segments.back().rotation && segments.back().direction == direction)
    {
      segments.back().last = i + 1;
    }
    else
    {
      FixedSegment segment;
      segment.first = i;
      segment.last = i + 1;
      segment.direction = direction;
      segment.rotation = rotation;
      segments.push_back(segment);
    }
  }
  if (segments.empty())
  {
    // 单点或完全重合路径只允许停稳／航向验收，不能进入平移跟踪器。
    FixedSegment segment;
    segment.last = path.poses.size() - 1;
    segment.rotation = true;
    segments.push_back(segment);
  }
  return segments;
}

// FNV-1a 仅用于同任务几何一致性诊断；不包含消息时间戳，不作为安全认证。
inline uint64_t fixedPathFingerprint(const nav_msgs::msg::Path & path)
{
  uint64_t hash = 14695981039346656037ULL;
  const auto append = [&hash](const void * bytes, const std::size_t size)
  {
    const auto * data = static_cast<const unsigned char *>(bytes);
    for (std::size_t i = 0; i < size; ++i)
    {
      hash = (hash ^ data[i]) * 1099511628211ULL;
    }
  };
  append(path.header.frame_id.data(), path.header.frame_id.size());
  for (const auto & point : path.poses)
  {
    const auto & p = point.pose.position;
    const auto & q = point.pose.orientation;
    const double values[] =
    {
      p.x, p.y, p.z, q.x, q.y, q.z, q.w
    };
    append(values, sizeof(values));
  }
  return hash;
}

// 只在当前段、旧进度向前的有限弧长内匹配，避免自交路径跳到后续圈。
inline std::size_t advanceFixedCursor(const nav_msgs::msg::Path & path, const FixedSegment & segment, const std::size_t previous, const geometry_msgs::msg::PoseStamped & pose, const double window = 0.5)
{
  std::size_t best = std::clamp(previous, segment.first, segment.last);
  double best_distance = fixedDistance(path.poses[best], pose);
  double arc = 0.0;
  for (std::size_t i = best + 1; i <= segment.last; ++i)
  {
    arc += fixedDistance(path.poses[i - 1], path.poses[i]);
    if (arc > window)
    {
      break;
    }
    const double distance = fixedDistance(path.poses[i], pose);
    if (distance < best_distance)
    {
      best = i;
      best_distance = distance;
    }
  }
  return best;
}
}
#endif
