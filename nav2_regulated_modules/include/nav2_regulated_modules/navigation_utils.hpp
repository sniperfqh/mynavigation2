#ifndef NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_regulated_modules
{
namespace navigation_utils
{

// 中文注释：集中放置无状态导航计算，避免业务编排文件重复实现数学和校验逻辑。
builtin_interfaces::msg::Duration durationFromSeconds(double seconds);

double poseDistance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second);

double yawFromPose(const geometry_msgs::msg::PoseStamped & pose);

double normalizeAngle(double angle);

bool validPose(const geometry_msgs::msg::PoseStamped & pose);

}  // namespace navigation_utils
}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
