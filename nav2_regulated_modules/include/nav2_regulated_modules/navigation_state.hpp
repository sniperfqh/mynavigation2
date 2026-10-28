// 导航模式、任务类别和执行阶段的共享定义；当前实现仍由启动模式决定可接受的入口。
// generation 区分任务，规划和跟踪另有序号；三个维度配合丢弃迟到回调，不能只凭 Action 句柄非空判断当前任务。

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

// 三种互斥入口：遥控不运行导航栈；自主与固定路径共用规控节点。
enum class NavigationMode
{
  REMOTE,
  AUTONOMOUS,
  FIXED_PATH
};

// 单个任务的执行阶段；异步回调必须结合 generation 判断状态是否仍属于当前任务。
enum class NavigationState
{
  IDLE,
  PLANNING,
  SMOOTHING,
  CONTROLLING,
  REPLANNING,
  CLEARING_COSTMAP,
  LOCALIZATION_LOST,
  CANCELING,
  SUCCEEDED,
  FAILED
};

// 区分 Action 与话题入口，以便向正确的客户端反馈结果。
enum class TaskType
{
  NONE,
  TO_POSE,
  THROUGH_POSES,
  TOPIC_GOAL,
  NAVIGATION_SERVICE
};

// 当前任务的共享快照；代次、路径和计时字段用于丢弃过时回调并监控进度。
struct NavigationTask
{
  // 每接管一个新任务递增，作为异步规划及跟踪回调的有效性标识。
  // 每次任务接管递增；旧回调携带旧值时应被忽略，不能把复用成员地址当同一任务。
  uint64_t generation { 0 };
  TaskType type { TaskType::NONE };
  NavigationState state { NavigationState::IDLE };
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  nav_msgs::msg::Path active_path;
  // 以下时间用于总耗时、重规划节流和无进展超时判断。
  // ROS 时间用于导航耗时；暂停仿真／时钟回跳会影响此类计时，与 steady_clock 的接收超时不同。
  rclcpp::Time start_time { 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_replan_time { 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_progress_time { 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped last_progress_pose;
  // 恢复次数与连续规划失败次数分别受各自上限约束。
  int recovery_count { 0 };
  int localization_recovery_count = 0;
  int consecutive_planning_failures { 0 };
  double distance_remaining { 0.0 };
  std::string task_id;
  double total_path_length { 0.0 };
  // 固定路径请求速度与前后向标记不直接改变普通导航控制器参数。
  // 正的任务速度基准 m/s；运动方向由 motion_direction 与路径姿态另行编码。
  double requested_speed { 0.0 };
  uint8_t motion_direction { 0 };
  float progress { 0.0F };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
