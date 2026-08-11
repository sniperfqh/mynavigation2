#ifndef NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_regulated_modules
{
namespace navigation_utils
{

// 中文注释：集中放置无状态导航计算，避免业务编排文件重复实现数学和校验逻辑。
// 中文注释：把浮点秒数裁剪到非负区间并转换为 ROS Duration 消息。
builtin_interfaces::msg::Duration durationFromSeconds(double seconds);

// 中文注释：计算两个 Pose 在 XY 平面的欧氏距离，不考虑高度和朝向。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

// 中文注释：从 Pose 四元数提取绕 Z 轴的偏航角。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose);

// 中文注释：把角度循环归一化到 [-pi, pi]。
double normalizeAngle(double angle);

// 中文注释：检查 frame_id、位置有限性和四元数范数，过滤不可用导航位姿。
bool validPose(const geometry_msgs::msg::PoseStamped & pose);

}  // namespace navigation_utils
}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
