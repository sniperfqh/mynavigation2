#ifndef NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_regulated_modules
{

// 中文注释：运行模式只允许在启动时选择，避免运行期切换遗留旧 Action 或速度发布者。
enum class NavigationMode
{
  // 中文注释：遥控模式只启动键盘控制，不进入本导航状态机。
  REMOTE,
  // 中文注释：自主模式执行规划、平滑、控制和周期重规划完整链路。
  AUTONOMOUS,
  // 中文注释：固定路径模式跳过规划和平滑，直接更新 FollowPath。
  FIXED_PATH
};

// 中文注释：显式状态机替代行为树，保证取消、重规划和恢复只有一个确定状态。
enum class NavigationState
{
  // 中文注释：当前没有活动任务。
  IDLE,
  // 中文注释：正在请求首次全局路径。
  PLANNING,
  // 中文注释：全局路径已经返回，正在请求平滑。
  SMOOTHING,
  // 中文注释：控制器正在跟随当前路径。
  CONTROLLING,
  // 中文注释：控制期间正在计算替换路径。
  REPLANNING,
  // 中文注释：不再依赖 nav2_behaviors，恢复阶段只清理代价地图并等待重新规划。
  CLEARING_COSTMAP,
  // 中文注释：map 到 base_link 的动态 TF 超时，任务保持停车等待恢复。
  LOCALIZATION_LOST,
  // 中文注释：正在取消子 Action 并结束外层目标。
  CANCELING,
  // 中文注释：任务已到达终点并准备复位。
  SUCCEEDED,
  // 中文注释：任务无法继续并准备复位。
  FAILED
};

// 中文注释：任务类型决定目标来源以及定位恢复后应重新规划还是重发固定路径。
enum class TaskType
{
  // 中文注释：没有任务。
  NONE,
  // 中文注释：来自 NavigateToPose Action 的单点任务。
  TO_POSE,
  // 中文注释：来自 NavigateThroughPoses Action 的多点任务。
  THROUGH_POSES,
  // 中文注释：来自 goal_pose Topic 的兼容单点任务。
  TOPIC_GOAL,
  // 中文注释：来自 fixed_path Topic 的完整路径任务。
  FIXED_PATH
};

// 中文注释：generation 用于隔离旧任务的异步回调，防止旧路径覆盖新目标。
struct NavigationTask
{
  uint64_t generation{0};
  // 中文注释：标识外层目标来源，控制恢复分支选择。
  TaskType type{TaskType::NONE};
  // 中文注释：记录当前状态机阶段。
  NavigationState state{NavigationState::IDLE};
  // 中文注释：单点终点；多点与固定路径任务保存最后一个 Pose。
  geometry_msgs::msg::PoseStamped goal;
  // 中文注释：多点自主导航尚未通过的目标序列。
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  // 中文注释：当前交给控制器的路径，也是固定路径恢复时的重发数据。
  nav_msgs::msg::Path active_path;
  // 中文注释：以下时间和计数用于反馈、重规划节流、进展判断与恢复限次。
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time{0, 0, RCL_ROS_TIME};
  // 中文注释：记录真实位姿进展，避免仅依据控制器速度误判车辆已经移动。
  rclcpp::Time last_progress_time{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_progress_pose;
  int recovery_count{0};
  int consecutive_planning_failures{0};
  // 中文注释：FollowPath 反馈的剩余距离，供外层导航反馈使用。
  float distance_remaining{0.0F};
  // 中文注释：保存最近失败原因，便于恢复和日志诊断。
  std::string last_error;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
