// 方向感知的固定路径 Controller 插件声明：起点策略、前视跟踪、终点制动与零速锁存。
// 调用方通过 FollowPath 提供完整 Path；路径姿态编码车头方向，插件不重置定位，不直接向串口发送数据。

#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/terminal_aware_controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_regulated_modules
{

class FixedPathController : public nav2_core::Controller, public nav2_core::TerminalAwareController
{
  public:
// 构造控制器；插件资源与路径状态在 configure 和 setPlan 阶段建立。
  // 默认构造仅使用成员初值；实际依赖和参数在 configure 建立，不能把构造成功视为插件已配置。
  FixedPathController() = default;
  // 说明 ~FixedPathController 对应的接口行为；调用方需遵守该类的任务状态约束。
  // 默认析构由成员管理资源；生命周期 cleanup／deactivate 的业务职责仍需宿主按约定调用。
  ~FixedPathController() override = default;

  // 锁定生命周期父节点并读取插件命名空间参数，检查约束后初始化 TF、代价地图和控制周期。
  // 成员初值与 declare_parameter 默认及 YAML 覆盖可能不同；角度为 rad，距离为 m，速度为 m/s。
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 清空路径、起步／制动历史和共享依赖，供后续重新 configure。
  // 不直接发布 Twist，也不证明底盘停稳。
  void cleanup() override;
  // 记录插件已激活；插件由 Controller Server 调度，不能独立启动导航。
  void activate() override;
  // 将内部阶段切到 STOPPED 并锁存停止请求，不从这里发布速度。
  // 真正的零速输出仍由后续调用与 Controller Server 收口负责。
  void deactivate() override;
  // 接收至少两点的完整 Path，复位最近点、起步保护与终点锁存，提取有效首末切线。
  // 通过首点姿态与路径切线区分前进／倒车；新路径会重新开始起步保护，不继承旧任务状态。
  void setPlan(const nav_msgs::msg::Path & path) override;
  // 本周期从路径坐标系位姿、实测 Twist 和 GoalChecker 容差生成控制目标。
  // 执行顺序：起点对齐／保护 → 前视几何 → 终点制动／锁存 → 方向化 Twist；异常以 PlannerException 交给宿主处理。
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;
  // 按 percentage 区分百分比与绝对上限，结果不能超过 base_linear_velocity。
  // NO_SPEED_LIMIT 表示恢复基础上限，非有限／负值被忽略；不在这里生成制动曲线。
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  // 在锁内返回停车锁存位；锁存只说明控制器持续要求零速，不说明轮速已为零。
  bool isTerminalStopLatched() override;
  // 在锁内返回控制器维护的位置状态，终点收敛分支可逐周期复核该值。
  // 该布尔值不包含独立采样窗覆盖率或实际物理定位误差。
  bool isTerminalPositionAccurate() override;
  // 终点收敛开启时向宿主申请 1 秒稳定精度复核，否则返回 0。
  // 此处只报告时长，计时状态在 Controller Server 一侧维护。
  double getTerminalStableDuration() const override
  {
    return terminal_convergence_enabled_ ? 1.0 : 0.0;
  }

private:
  enum class Phase
  {
    ALIGN_START, TRACK_PATH, STOPPED
  };

  // 同坐标系直接复制，否则通过 TF 在 transform_tolerance（s）内转换。
  // 变换失败返回 false，调用方不得用未变换坐标继续计算。
  bool transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const;
  // 只在当前 nearest_index 及其之后搜索最近离散点，避免进度倒退。
  // 输出是路径索引而非弧长；同距离候选保留先遇点。
  std::size_t findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose);
  // 当前位置到选定路径点的距离加上后续折线长度，单位 m。
  // 这与到终点的直线距离不同，弯道与偏离路径时不能混用。
  double remainingDistance(std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const;
  // 从 start_index 向前累计离散弧长，返回首个达到前视距离的路径点。
  // 若余下路径不够长，沿末段单位切线外推；外推只服务曲率计算，不改变业务终点。
  geometry_msgs::msg::PoseStamped selectCarrot(std::size_t start_index, double lookahead_distance) const;
  // 生成带底盘坐标系和当前时钟时间戳的零 TwistStamped，不发送消息。
  geometry_msgs::msg::TwistStamped zeroCommand() const;
  // 将剩余角误差同时受比例项、最大角速度及角减速度包络约束，再限制单周期角速度变化。
  // 角误差单位 rad；纵向速度保持零，控制周期从配置频率得到。
  geometry_msgs::msg::TwistStamped rotateCommand(double yaw_error, double yaw_tolerance, double max_angular_velocity, double max_angular_accel, double angular_kp, const geometry_msgs::msg::Twist & velocity) const;
  // 按 ROS 时钟限频保存终点误差、剩余路程和制动状态；时钟回跳允许重新记录。
  // 日志只观察已算出的量，不改变停车、精度或任务状态。
  void logGoalErrors(double position_error, double yaw_error, double goal_xy_tolerance, double terminal_projection, bool goal_plane_crossed, double remaining_distance, double current_linear_velocity, double target_linear_velocity, double stopping_distance, double terminal_lookahead_distance);
  // 将内部阶段枚举映射到稳定字符串，仅用于日志；未知值返回 UNKNOWN。
  static const char * phaseName(Phase phase);
  // 用 atan2(sin θ, cos θ) 将任意角差折回主值区间，θ 的单位为 rad。
  static double normalizeAngle(double angle);
  // 从四元数提取绕竖直轴的平面航向，单位 rad；不验证输入所在坐标系。
  static double poseYaw(const geometry_msgs::msg::PoseStamped & pose);
  // 计算两个同坐标系位置的二维欧氏距离，忽略 z，单位 m。
  static double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ { rclcpp::get_logger("FixedPathController") };
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  // 下列为每条 Path 的运行历史，setPlan 必须清理；与 configure 阶段读到的参数分开维护。
  nav_msgs::msg::Path global_plan_;
  std::vector<detail::StartPathPoint> start_path_points_;
  detail::FixedPathStartSpeedGuard start_speed_guard_;
  // 串行化 setPlan、计算、限速和终点状态读取，保证每帧使用同一条 Path 的状态。
  std::mutex mutex_;
  Phase phase_ { Phase::ALIGN_START };
  std::size_t nearest_index_ { 0 };
  std::size_t goal_tangent_index_ { 0 };
  int direction_sign_ { 1 };
  int stable_cycles_ { 0 };
  bool start_strategy_evaluated_ { false };
  bool direct_start_tracking_ { false };
  bool goal_braking_active_ { false };
  bool terminal_stop_latched_ { false };
  bool terminal_position_accurate_ { false };
  double start_path_yaw_ { 0.0 };
  double goal_path_yaw_ { 0.0 };
  double terminal_tangent_x_ { 0.0 };
  double terminal_tangent_y_ { 0.0 };
  // 以下成员是构造回退初值，configure 还会声明／读取参数，项目 YAML 与 Launch 可进一步覆盖。
  double base_linear_velocity_ { 1.5 };
  double speed_limit_ { 1.5 };
  double lookahead_dist_ { 0.45 };
  double min_lookahead_dist_ { 0.25 };
  double max_lookahead_dist_ { 0.75 };
  double lookahead_time_ { 1.5 };
  double start_position_tolerance_ { 1.20 };
  double direct_tracking_lateral_tolerance_ { 0.20 };
  double start_offset_speed_limit_ = 0.30;
  double start_speed_limit_distance_ = 0.50;
  bool start_speed_prebraking_ = false;
  double start_speed_release_yaw_tolerance_ = 0.3490658503988659;
  int start_speed_release_stable_cycles_ = 10;
  double direct_tracking_max_yaw_error_ { 0.2617993877991494 };
  double initial_yaw_tolerance_ { 0.05235987755982989 };
  double rotate_to_heading_angular_vel_ { 0.4 };
  double max_angular_accel_ { 0.8 };
  double rotate_to_heading_kp_ { 1.5 };
  double min_approach_linear_velocity_ { 0.005 };
  double approach_velocity_scaling_dist_ { 0.8 };
  double goal_linear_deceleration_ { 0.25 };
  double goal_final_approach_velocity_ { 0.01 };
  // 停车入口 m：0 沿用 GoalChecker 容差；正值单独指定停车半径且不得大于该容差，精度半径保持。
  double goal_stop_entry_tolerance_ { 0.0 };
  // 该成员初值不等于 configure 后生效值；终点复核行为由实际读取的参数决定。
  bool terminal_convergence_enabled_ = false;
  double goal_braking_reaction_time_ { 0.1 };
  double goal_braking_distance_margin_ { 0.1 };
  bool dynamic_goal_braking_margin_enabled_ = false;
  double goal_braking_min_distance_margin_ = 0.03;
  double goal_braking_margin_transition_speed_ = 0.3;
  // false 采用固定减速度与单调命令，true 采用自适应速度包络与 jerk 积分。
  bool adaptive_goal_braking_enabled_ { false };
  double adaptive_goal_max_deceleration_ { 1.0 };
  double adaptive_goal_jerk_limit_ { 2.0 };
  double adaptive_goal_approach_speed_ { 0.03 };
  double adaptive_goal_response_time_ { 0.5 };
  double adaptive_goal_distance_margin_ { 0.01 };
  double goal_terminal_lookahead_dist_ { 0.2 };
  double goal_terminal_lookahead_reference_speed_ { 0.75 };
  double goal_terminal_lookahead_speed_gain_ { 0.1 };
  double goal_terminal_lookahead_min_dist_ { 0.15 };
  double goal_terminal_lookahead_max_dist_ { 0.25 };
  double terminal_tracking_yaw_error_ { 0.0 };
  double last_braking_command_magnitude_ { 0.0 };
  double last_braking_deceleration_ { 0.0 };
  rclcpp::Time last_braking_update_time_ { 0, 0, RCL_ROS_TIME };
  double goal_error_log_frequency_ { 1.0 };
  double transform_tolerance_ { 0.2 };
  double control_duration_ { 0.01 };
  int alignment_stable_cycles_ { 1 };
  rclcpp::Time last_error_log_time_ { 0, 0, RCL_ROS_TIME };
  bool error_log_initialized_ { false };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
