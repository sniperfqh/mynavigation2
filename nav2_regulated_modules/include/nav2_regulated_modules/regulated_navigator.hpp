// 规控生命周期节点总接口。协调标准导航、固定路径、规划、控制、恢复、反馈及底盘速度链。

#ifndef NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
#define NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "byd_custom_msgs/action/navigation_service.hpp"
#include "byd_custom_msgs/msg/navi_segment.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_regulated_modules/chassis_control_subscriber.hpp"
#include "nav2_regulated_modules/control_module.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_regulated_modules/navigation_state.hpp"
#include "nav2_regulated_modules/planning_module.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

class RegulatedNavigator : public nav2_util::LifecycleNode
{
  public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using NavigationService = byd_custom_msgs::action::NavigationService;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  using ComputePoseHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using ComputePosesHandle = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;
  using SmoothHandle = rclcpp_action::ClientGoalHandle<SmoothPath>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using NavigatePoseHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using NavigatePosesHandle = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;
  using NavigationServiceHandle = rclcpp_action::ServerGoalHandle<NavigationService>;

  // 声明规控运行参数；真正的 ROS 实体在生命周期配置阶段建立。
  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 读取并验证参数、创建 Action 客户端及 ROS 通信对象；失败则阻止节点激活。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  // 激活生命周期发布器及任务入口，允许外部请求进入。
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  // 停止接收新任务、取消在途目标并输出零速。
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  // 销毁配置阶段创建的接口与资源，恢复未配置状态。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  // 在 ROS 关闭前取消任务并执行安全停车。
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // 校验单目标导航请求，拒绝与当前模式或状态不兼容的目标。
  rclcpp_action::GoalResponse handlePoseGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateToPose::Goal> goal);
  // 校验多目标导航请求，避免空路径或冲突任务进入执行。
  rclcpp_action::GoalResponse handlePosesGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateThroughPoses::Goal> goal);
  // 处理单目标 Action 取消请求并传播到下游子目标。
  rclcpp_action::CancelResponse handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal);
  // 处理多目标 Action 取消请求并传播到下游子目标。
  rclcpp_action::CancelResponse handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal);
  // 接管已接受的单目标任务并启动规划流程。
  void handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal);
  // 接管已接受的多目标任务并启动规划流程。
  void handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal);
  // 将话题位姿目标接入同一导航任务流程。
  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  // 校验固定路径请求的路径、方向与速度约束。
  rclcpp_action::GoalResponse handleNavigationServiceGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigationService::Goal> goal);
  // 响应固定路径任务取消并使下游跟踪停车。
  rclcpp_action::CancelResponse handleNavigationServiceCancel(const std::shared_ptr<NavigationServiceHandle> goal);
  // 接管固定路径 Action 并准备跟踪路径。
  void handleNavigationServiceAccepted(const std::shared_ptr<NavigationServiceHandle> goal);
  // 把分段任务转换为带运动方向的固定路径；非法段返回空结果。
  std::optional<nav_msgs::msg::Path> prepareFixedPath(const std::vector<byd_custom_msgs::msg::NaviSegment> & segments);
  // 将输入路径段转换为贝塞尔曲线所需的控制点。
  void liner2to4point(const nav_msgs::msg::Path & input_path, nav_msgs::msg::Path & output_path);
  // 计算三次贝塞尔曲线在给定参数处的位姿。
  geometry_msgs::msg::PoseStamped bezier3(const nav_msgs::msg::Path & input_path, double t, bool use_result);
  // 累计路径各点间的弧长，供等距采样使用。
  void computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> & poses, std::vector<double> & arc_lengths);
  // 通过弧长表反查贝塞尔参数，实现近似等距离采样。
  double findTfromArcLength(const std::vector<double> & arc_lengths, const std::vector<double> & ts, double target_length);
  // 按目标间隔沿贝塞尔曲线均匀生成路径点。
  void generateBezierUniformPoints(const nav_msgs::msg::Path & input_path, double interval, nav_msgs::msg::Path & output_path);
  // 按固定频率向活动 Action 发布当前位置与任务进度。
  void publishFeedback();
  // 把任务有效速度上限发布给下游速度平滑器。
  void publishSpeedLimit();

  // 检查规划、平滑和跟踪 Action 服务是否已就绪。
  bool dependenciesReady();
  // 为当前任务发送新规划请求；重规划时保留任务代次用于回调过滤。
  void startPlanning(bool replanning);
  // 比较任务代次和规划序号，丢弃旧规划异步回调。
  bool isCurrentPlan(uint64_t generation, uint64_t sequence) const;
  // 接收有效规划路径，按配置执行平滑或直接交给跟踪器。
  void onPathReady(const nav_msgs::msg::Path & path);
  // 记录规划失败并依据连续失败次数触发恢复或任务失败。
  void handlePlanningFailure(const std::string & reason);

  // 把有效路径提交给控制服务器，并注册反馈与结果回调。
  void sendFollowPath(const nav_msgs::msg::Path & path);
  // 发布固定路径及左右可视化边界，不参与实际控制输出。
  void publishFixedPath(const nav_msgs::msg::Path & path);
  // 比较任务代次和跟踪序号，防止旧 FollowPath 回调污染新任务。
  bool isCurrentFollow(uint64_t generation, uint64_t sequence) const;
  // 向控制速度入口发布全零指令，确保取消或异常时停车。
  void stopRobot();
  // 记录控制器原始速度，供速度链诊断。
  void onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 记录速度平滑器输出，供速度链诊断。
  void onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 记录实际里程计速度，供上下游速度比较。
  void onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
  // 周期性汇总控制器、平滑器与里程计速度。
  void logVelocityChain();

  // 取消当前子目标并进入恢复流程，避免继续执行失效路径。
  void startRecovery(const std::string & reason);
  // 恢复后对当前任务重新规划或继续固定路径跟踪。
  void resumeCurrentTask();
  // 周期检查定位、任务进展及重规划时机，必要时停车或恢复。
  void monitorTask();
  // 通过 TF 获取当前车体在全局坐标系中的位姿。
  bool lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose);
  // 根据机器人当前位置移除已经通过的多点目标。
  void updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose);

  // 取消仍在执行的下游规划、平滑或跟踪 Action 目标。
  void cancelSubGoals(bool invalidate_callbacks);
  // 取消当前任务，通知客户端并输出停车指令。
  void cancelTask(const std::string & reason);
  // 在新目标接管前终止旧任务并隔离旧回调。
  void preemptCurrentTask();
  // 向客户端报告任务成功并清理在途资源。
  void succeedTask();
  // 向客户端报告失败原因并清理在途资源。
  void failTask(const std::string & reason);
  // 清理任务状态、序号及活动 Action 句柄。
  void resetTask();

  bool configured_ { false };
  bool active_ { false };
  bool planning_active_ { false };
  bool updating_path_ { false };
  bool has_last_pose_ { false };
  bool cancel_requested_ { false };
  uint64_t task_generation_ { 0 };
  uint64_t plan_sequence_ { 0 };
  uint64_t follow_sequence_ { 0 };
  NavigationTask task_;
  NavigationMode operation_mode_ { NavigationMode::AUTONOMOUS };
  std::unique_ptr<MotionStateSubscriber> motion_state_subscriber_;
  std::unique_ptr<ChassisControlSubscriber> chassis_control_subscriber_;
  PlanningModule planning_module_;
  ControlModule control_module_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string goal_topic_;
  std::string navigation_service_action_;
  std::string fixed_path_visualization_topic_;
  std::string fixed_path_boundaries_topic_;
  std::string fixed_path_controller_id_;
  std::string fixed_path_goal_checker_id_;
  std::string controller_cmd_vel_topic_;
  std::string smoothed_cmd_vel_topic_;
  std::string velocity_odom_topic_;
  std::string speed_limit_topic_;
  double server_timeout_ { 5.0 };
  double cancel_timeout_ { 2.0 };
  double smoothing_duration_ { 2.0 };
  double feedback_frequency_ { 5.0 };
  double costmap_wait_duration_ { 0.8 };
  double passed_goal_radius_ { 0.7 };
  double localization_timeout_ { 0.3 };
  bool enable_localization_jump_detection_ { false };
  double max_translation_jump_ { 0.3 };
  double max_rotation_jump_ { 0.35 };
  double progress_min_translation_ { 0.1 };
  double localization_recovery_timeout_ { 10.0 };
  double localization_stable_duration_ { 0.5 };
  int max_recovery_rounds_ { 2 };
  bool check_smoother_collisions_ { true };
  double current_speed_ { 0.0 };
  double fixed_path_step_ { 0.1 };
  double fixed_path_max_speed_ { 1.5 };
  double fixed_path_boundary_half_width_ { 0.4 };
  double velocity_log_frequency_ { 1.0 };

  rclcpp::Time last_valid_tf_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_lost_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_stable_since_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time recovery_ready_time_ { 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped last_pose_;
  nav_msgs::msg::Path pending_raw_path_;
  geometry_msgs::msg::Twist latest_controller_velocity_;
  geometry_msgs::msg::Twist latest_smoothed_velocity_;
  nav_msgs::msg::Odometry latest_velocity_odometry_;
  std::mutex velocity_mutex_;
  bool has_controller_velocity_ { false };
  bool has_smoothed_velocity_ { false };
  bool has_velocity_odometry_ { false };

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_pose_client_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_poses_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr navigate_poses_server_;
  rclcpp_action::Server<NavigationService>::SharedPtr navigation_service_server_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_global_client_;

  ComputePoseHandle::SharedPtr active_compute_pose_goal_;
  ComputePosesHandle::SharedPtr active_compute_poses_goal_;
  SmoothHandle::SharedPtr active_smooth_goal_;
  FollowHandle::SharedPtr active_follow_goal_;
  std::shared_ptr<NavigatePoseHandle> active_pose_goal_;
  std::shared_ptr<NavigatePosesHandle> active_poses_goal_;
  std::shared_ptr<NavigationServiceHandle> active_navigation_service_goal_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smoothed_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_odom_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr fixed_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr fixed_path_boundaries_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr velocity_log_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
