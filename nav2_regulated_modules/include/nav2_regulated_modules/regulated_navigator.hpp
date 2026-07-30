#ifndef NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
#define NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_regulated_modules/action/follow_fixed_path.hpp"
#include "nav2_regulated_modules/control_module.hpp"
#include "nav2_regulated_modules/navigation_state.hpp"
#include "nav2_regulated_modules/planning_module.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_regulated_modules
{

// 中文注释：单一状态所有者负责规控编排，各职责实现分布在独立 cpp 中，避免跨对象状态竞争。
class RegulatedNavigator : public nav2_util::LifecycleNode
{
public:
  // 中文注释：为六类 Nav2 Action 和清图 Service 建立简短类型别名。
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using FollowFixedPath = nav2_regulated_modules::action::FollowFixedPath;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  // 中文注释：客户端句柄对应规划、平滑、控制子 Goal，服务端句柄对应对外导航 Goal。
  using ComputePoseHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using ComputePosesHandle = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;
  using SmoothHandle = rclcpp_action::ClientGoalHandle<SmoothPath>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using NavigatePoseHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using NavigatePosesHandle = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;
  using FixedPathHandle = rclcpp_action::ServerGoalHandle<FollowFixedPath>;

  // 中文注释：构造生命周期节点并声明全部参数，不在构造阶段连接外部服务器。
  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 中文注释：配置回调读取参数并创建全部 ROS 通信接口。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  // 中文注释：激活回调开放任务入口并建立 Lifecycle Bond。
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  // 中文注释：停用回调取消任务、停车并销毁 Lifecycle Bond。
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  // 中文注释：清理回调释放配置阶段创建的客户端、服务端、订阅、定时器和 TF。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  // 中文注释：关闭回调在进程退出前再次收口活动任务。
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // 中文注释：校验单点导航 Goal 是否可由当前自主模式执行。
  rclcpp_action::GoalResponse handlePoseGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const NavigateToPose::Goal> goal);
  // 中文注释：校验多点导航 Goal 的模式、位姿数组和行为树字段。
  rclcpp_action::GoalResponse handlePosesGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const NavigateThroughPoses::Goal> goal);
  // 中文注释：接收单点外层 Goal 的取消请求。
  rclcpp_action::CancelResponse handlePoseCancel(
    const std::shared_ptr<NavigatePoseHandle> goal);
  // 中文注释：接收多点外层 Goal 的取消请求。
  rclcpp_action::CancelResponse handlePosesCancel(
    const std::shared_ptr<NavigatePosesHandle> goal);
  // 中文注释：初始化已接受的单点任务并启动规划。
  void handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal);
  // 中文注释：初始化已接受的多点任务并启动规划。
  void handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal);
  // 中文注释：把 goal_pose Topic 输入转换为自主单点任务。
  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  // 中文注释：校验固定路径业务 Goal 的模式、激活状态和 Path 基本结构。
  rclcpp_action::GoalResponse handleFixedPathGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const FollowFixedPath::Goal> goal);
  // 中文注释：记录固定路径业务 Goal 的取消请求，由监控周期统一收口。
  rclcpp_action::CancelResponse handleFixedPathCancel(
    const std::shared_ptr<FixedPathHandle> goal);
  // 中文注释：完成 Path 变换和下游检查后，以新 Action Goal 替换旧任务。
  void handleFixedPathAccepted(const std::shared_ptr<FixedPathHandle> goal);
  // 中文注释：校验固定路径并统一变换到全局坐标系。
  std::optional<nav_msgs::msg::Path> prepareFixedPath(
    const nav_msgs::msg::Path & path);
  // 中文注释：按固定频率把当前位姿、耗时、剩余距离和恢复次数反馈给外层 Action。
  void publishFeedback();

  // 中文注释：等待规划、可选平滑和控制 Action Server 就绪。
  bool dependenciesReady();
  // 中文注释：发起首次规划或控制期间的周期重规划。
  void startPlanning(bool replanning);
  // 中文注释：判断规划异步回调是否仍属于当前任务和当前请求。
  bool isCurrentPlan(uint64_t generation, uint64_t sequence) const;
  // 中文注释：接续规划结果到可选平滑或直接控制。
  void onPathReady(const nav_msgs::msg::Path & path);
  // 中文注释：处理规划失败、旧路径保留和恢复切换。
  void handlePlanningFailure(const std::string & reason);

  // 中文注释：把路径发送给 FollowPath 并安装 Goal、反馈和结果回调。
  void sendFollowPath(const nav_msgs::msg::Path & path);
  // 中文注释：判断控制异步回调是否仍属于当前任务和当前请求。
  bool isCurrentFollow(uint64_t generation, uint64_t sequence) const;
  // 中文注释：向速度链入口重复发布零 Twist，形成停车兜底。
  void stopRobot();

  // 中文注释：取消当前子任务、停车并异步清理双代价地图。
  void startRecovery(const std::string & reason);
  // 中文注释：按任务类型选择重新规划或重发固定路径。
  void resumeCurrentTask();
  // 中文注释：周期监控取消、恢复、定位、进展和重规划条件。
  void monitorTask();
  // 中文注释：通过 TF 查询当前 map 到 base_link 位姿。
  bool lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose);
  // 中文注释：更新多点任务中已经通过的目标序列。
  void updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose);

  // 中文注释：取消全部规划、平滑和控制子 Goal，并可使旧回调失效。
  void cancelSubGoals(bool invalidate_callbacks);
  // 中文注释：按外层 Action 状态取消或终止当前任务。
  void cancelTask(const std::string & reason);
  // 中文注释：以新任务替换当前任务并终止旧外层 Goal。
  void preemptCurrentTask();
  // 中文注释：成功完成外层 Action 并复位内部状态。
  void succeedTask();
  // 中文注释：失败终止外层 Action 并复位内部状态。
  void failTask(const std::string & reason);
  // 中文注释：清空任务瞬态数据并回到空闲状态。
  void resetTask();

  // 中文注释：状态标志与递增序号共同保证异步 Action 回调只能修改当前任务。
  bool configured_{false};
  bool active_{false};
  bool planning_active_{false};
  bool updating_path_{false};
  bool has_last_pose_{false};
  bool cancel_requested_{false};
  uint64_t task_generation_{0};
  uint64_t plan_sequence_{0};
  uint64_t follow_sequence_{0};
  // 中文注释：任务快照、启动模式和规划／控制策略模块。
  NavigationTask task_;
  NavigationMode operation_mode_{NavigationMode::AUTONOMOUS};
  PlanningModule planning_module_;
  ControlModule control_module_;

  // 中文注释：接口名称、坐标系、超时、频率和恢复阈值均在 configure 阶段从参数读取。
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string goal_topic_;
  std::string fixed_path_action_;
  double server_timeout_{5.0};
  double cancel_timeout_{2.0};
  double smoothing_duration_{2.0};
  double feedback_frequency_{5.0};
  double costmap_wait_duration_{0.8};
  double passed_goal_radius_{0.7};
  double localization_timeout_{0.3};
  double max_translation_jump_{0.3};
  double max_rotation_jump_{0.35};
  double progress_min_translation_{0.1};
  double localization_recovery_timeout_{10.0};
  double localization_stable_duration_{0.5};
  int max_recovery_rounds_{2};
  bool check_smoother_collisions_{true};
  double current_speed_{0.0};

  // 中文注释：时间戳和缓存数据用于判断定位丢失／恢复、跳变、重规划等待和原始路径回退。
  rclcpp::Time last_valid_tf_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_lost_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_ready_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_pose_;
  nav_msgs::msg::Path pending_raw_path_;

  // 中文注释：以下 ROS 接口分别连接 Nav2 子 Action、对外导航 Action 与清图 Service。
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_pose_client_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_poses_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr navigate_poses_server_;
  rclcpp_action::Server<FollowFixedPath>::SharedPtr fixed_path_server_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_global_client_;

  // 中文注释：活动 Goal 句柄用于取消、抢占以及过滤已经过期的异步结果。
  ComputePoseHandle::SharedPtr active_compute_pose_goal_;
  ComputePosesHandle::SharedPtr active_compute_poses_goal_;
  SmoothHandle::SharedPtr active_smooth_goal_;
  FollowHandle::SharedPtr active_follow_goal_;
  std::shared_ptr<NavigatePoseHandle> active_pose_goal_;
  std::shared_ptr<NavigatePosesHandle> active_poses_goal_;
  std::shared_ptr<FixedPathHandle> active_fixed_path_goal_;

  // 中文注释：Topic、停车发布器、定时器和 TF 接口构成非 Action 数据流与健康监控链。
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
