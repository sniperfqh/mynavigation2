// 导航几何与状态辅助接口。封装规划和控制流程共用的轻量计算，避免不同回调产生不一致判定。

#ifndef NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_regulated_modules
{
  namespace navigation_utils
{

// 将浮点秒数安全换算为 ROS Duration 消息，供 Action 超时字段使用。
builtin_interfaces::msg::Duration durationFromSeconds(double seconds);

// 计算两帧位姿的平面距离，单位米。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

// 从四元数提取平面航向角，结果单位为弧度。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose);

// 将角度规约到约定区间，避免跨越正负 π 时误判。
double normalizeAngle(double angle);

// 校验坐标系、位置有限值和非零四元数；无效位姿返回 false。
bool validPose(const geometry_msgs::msg::PoseStamped & pose);

}
// namespace navigation_utils
}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
