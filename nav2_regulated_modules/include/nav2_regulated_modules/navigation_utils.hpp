// 导航数据转换与几何小工具。二维距离用 m，航向和角差用 rad，时长输入为秒。
// 有效 Pose 检查是数值／结构检查，不代表坐标变换存在、定位新鲜或目标可通行。

#ifndef NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_regulated_modules
{
  namespace navigation_utils
{

// 把秒数转换为 ROS Duration 消息，不读取系统时钟；输入的业务合法性由调用方负责。
builtin_interfaces::msg::Duration durationFromSeconds(double seconds);

// 对同坐标系的两点计算 XY 平面距离，单位 m；忽略高度。
double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

// 从四元数取平面 yaw，单位 rad，不查询 TF。
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose);

// 把角差折回主值区间，避免 ±π 边界把小误差计算成接近一整圈。
double normalizeAngle(double angle);

// 检查 frame_id、坐标与四元数的实现约束；通过并不等于目标可规划或 TF 可用。
bool validPose(const geometry_msgs::msg::PoseStamped & pose);

}
// namespace navigation_utils
}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_UTILS_HPP_
