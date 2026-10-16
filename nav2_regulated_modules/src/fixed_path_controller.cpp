// 固定路径控制实现：一次调用根据路径、位姿、速度和 GoalChecker 输出一帧 TwistStamped。
// 先检查起点与反馈，再处理方向化几何、起步保护及终点制动；停止锁存后不再执行终点旋转。
// 实际底盘速度还经过速度平滑与下游适配，插件返回零命令不等于已有实测停稳证据。
//
// 功能参数：纵向制动、末端收敛和距离裕量分别配置。
// 1. adaptive_goal_braking_enabled：false 使用固定减速度制动，true 使用自适应制动。
//    固定减速度制动使用反应距离和单调命令；自适应制动使用速度相关包络及 jerk 约束。
// 2. terminal_convergence_enabled：独立决定末端前视、当前精度复核及向宿主申请连续复核的时长。
//    它可与固定减速度制动或自适应制动分别组合。
// 3. dynamic_goal_braking_margin_enabled：在固定减速度制动中按实测速度插值距离裕量。
//    自适应制动的距离预留由 adaptive_goal_* 参数计算。
// configure 声明的缺省组合依次为 false／true／true，即“固定减速度制动＋末端收敛＋动态裕量”。
// YAML／Launch 可以覆盖缺省值；实际后台配置须读运行参数，不能从本文件推断已加载哪组值。
// 两种制动共用起点策略、起步限速、路径方向、曲率限速和终点零速锁存，不改变 Action 接口。

#include "nav2_regulated_modules/fixed_path_controller.hpp"
#include "nav2_regulated_modules/detail/terminal_position.hpp"
#include "nav2_regulated_modules/detail/terminal_lookahead.hpp"
#include "nav2_regulated_modules/detail/braking_margin.hpp"
#include "nav2_regulated_modules/fixed_path_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 锁定生命周期父节点并读取插件命名空间参数，检查约束后初始化 TF、代价地图和控制周期。
// 成员初值与 declare_parameter 默认及 YAML 覆盖可能不同；角度为 rad，距离为 m，速度为 m/s。
void FixedPathController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  // 生命周期父节点已销毁时无法读取参数或创建 TF 接口，直接终止插件配置。
  if (!node)
  {
    throw nav2_core::PlannerException("FixedPathController cannot lock lifecycle node");
  }
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_position_tolerance", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_lateral_tolerance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_offset_speed_limit", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_limit_distance", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_release_yaw_tolerance", rclcpp::ParameterValue(0.3490658503988659));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_release_stable_cycles", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_max_yaw_error", rclcpp::ParameterValue(0.2617993877991494));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".initial_yaw_tolerance", rclcpp::ParameterValue(0.05235987755982989));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_kp", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.005));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_linear_deceleration", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_final_approach_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_stop_entry_tolerance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_reaction_time", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_distance_margin", rclcpp::ParameterValue(0.1));
  // 固定减速度制动的裕量开关：true 在 low→high 间随绝对车速连续插值；false 固定使用 high。
  // low/high 分别为 goal_braking_min_distance_margin／goal_braking_distance_margin，单位 m。
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".dynamic_goal_braking_margin_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_min_distance_margin", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_margin_transition_speed", rclcpp::ParameterValue(0.3));
  // 纵向算法选择：false 使用固定减速度算法；true 使用下方 adaptive_goal_* 参数。
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_braking_enabled", rclcpp::ParameterValue(false));
  // 末端收敛与上述选择正交：true 使用实测速度算末端前视、混合前视并逐周期复核精度。
  // getTerminalStableDuration() 同时向 Controller Server 返回 1 s；false 返回 0，不申请这层连续复核。
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_convergence_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_max_deceleration", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_jerk_limit", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_approach_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_response_time", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_distance_margin", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_dist", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_reference_speed", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_speed_gain", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_min_dist", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_max_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_error_log_frequency", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".alignment_stable_cycles", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  double controller_frequency = 100.0;
  node->get_parameter(plugin_name_ + ".desired_linear_vel", base_linear_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".start_position_tolerance", start_position_tolerance_);
  node->get_parameter(plugin_name_ + ".direct_tracking_lateral_tolerance", direct_tracking_lateral_tolerance_);
  node->get_parameter(plugin_name_ + ".start_offset_speed_limit", start_offset_speed_limit_);
  node->get_parameter(plugin_name_ + ".start_speed_limit_distance", start_speed_limit_distance_);
  node->get_parameter(plugin_name_ + ".start_speed_release_yaw_tolerance", start_speed_release_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".start_speed_release_stable_cycles", start_speed_release_stable_cycles_);
  node->get_parameter(plugin_name_ + ".direct_tracking_max_yaw_error", direct_tracking_max_yaw_error_);
  node->get_parameter(plugin_name_ + ".initial_yaw_tolerance", initial_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_kp", rotate_to_heading_kp_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".goal_linear_deceleration", goal_linear_deceleration_);
  node->get_parameter(plugin_name_ + ".goal_final_approach_velocity", goal_final_approach_velocity_);
  node->get_parameter(plugin_name_ + ".goal_stop_entry_tolerance", goal_stop_entry_tolerance_);
  node->get_parameter(plugin_name_ + ".goal_braking_reaction_time", goal_braking_reaction_time_);
  node->get_parameter(plugin_name_ + ".goal_braking_distance_margin", goal_braking_distance_margin_);
  node->get_parameter(plugin_name_ + ".dynamic_goal_braking_margin_enabled", dynamic_goal_braking_margin_enabled_);
  node->get_parameter(plugin_name_ + ".goal_braking_min_distance_margin", goal_braking_min_distance_margin_);
  node->get_parameter(plugin_name_ + ".goal_braking_margin_transition_speed", goal_braking_margin_transition_speed_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_braking_enabled", adaptive_goal_braking_enabled_);
  node->get_parameter(plugin_name_ + ".terminal_convergence_enabled", terminal_convergence_enabled_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_max_deceleration", adaptive_goal_max_deceleration_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_jerk_limit", adaptive_goal_jerk_limit_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_approach_speed", adaptive_goal_approach_speed_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_response_time", adaptive_goal_response_time_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_distance_margin", adaptive_goal_distance_margin_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_dist", goal_terminal_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_reference_speed", goal_terminal_lookahead_reference_speed_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_speed_gain", goal_terminal_lookahead_speed_gain_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_min_dist", goal_terminal_lookahead_min_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_max_dist", goal_terminal_lookahead_max_dist_);
  node->get_parameter(plugin_name_ + ".goal_error_log_frequency", goal_error_log_frequency_);
  node->get_parameter(plugin_name_ + ".alignment_stable_cycles", alignment_stable_cycles_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter("controller_frequency", controller_frequency);
  // 两分支均执行这组公共参数检查，各字段按下方实际条件校验；终点基础前视不能大于常规基础前视。
  if (base_linear_velocity_ <= 0.0 || lookahead_dist_ <= 0.0 || min_lookahead_dist_ <= 0.0 || max_lookahead_dist_ < min_lookahead_dist_ || start_position_tolerance_ <= 0.0 || direct_tracking_lateral_tolerance_ < 0.0 || direct_tracking_max_yaw_error_ <= 0.0 || direct_tracking_max_yaw_error_ > M_PI_2 || initial_yaw_tolerance_ <= 0.0 || initial_yaw_tolerance_ >= direct_tracking_max_yaw_error_ || rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 || rotate_to_heading_kp_ <= 0.0 || min_approach_linear_velocity_ <= 0.0 || min_approach_linear_velocity_ > base_linear_velocity_ || approach_velocity_scaling_dist_ <= 0.0 || !std::isfinite(goal_linear_deceleration_) || goal_linear_deceleration_ <= 0.0 || !std::isfinite(goal_final_approach_velocity_) || goal_final_approach_velocity_ < 0.0 || !std::isfinite(goal_braking_reaction_time_) || goal_braking_reaction_time_ < 0.0 || !std::isfinite(goal_braking_distance_margin_) || goal_braking_distance_margin_ < 0.0 || !std::isfinite(goal_terminal_lookahead_dist_) || goal_terminal_lookahead_dist_ <= 0.0 || goal_terminal_lookahead_dist_ > lookahead_dist_ || !std::isfinite(goal_terminal_lookahead_reference_speed_) || goal_terminal_lookahead_reference_speed_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_speed_gain_) || goal_terminal_lookahead_speed_gain_ < 0.0 || !std::isfinite(goal_terminal_lookahead_min_dist_) || goal_terminal_lookahead_min_dist_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_max_dist_) || goal_terminal_lookahead_max_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ > goal_terminal_lookahead_max_dist_ || goal_error_log_frequency_ <= 0.0 || alignment_stable_cycles_ < 1 || transform_tolerance_ < 0.0 || controller_frequency <= 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController parameters are invalid");
  }
  if (!std::isfinite(direct_tracking_lateral_tolerance_) || !std::isfinite(start_offset_speed_limit_) || start_offset_speed_limit_ <= 0.0 || !std::isfinite(start_speed_limit_distance_) || start_speed_limit_distance_ <= 0.0 || !std::isfinite(start_speed_release_yaw_tolerance_) || start_speed_release_yaw_tolerance_ <= 0.0 || start_speed_release_yaw_tolerance_ >= M_PI || start_speed_release_stable_cycles_ < 1)
  {
    throw nav2_core::PlannerException("FixedPathController start speed guard parameters are invalid");
  }
  speed_limit_ = base_linear_velocity_;
  if (!std::isfinite(goal_stop_entry_tolerance_) || goal_stop_entry_tolerance_ < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController stop entry tolerance must be finite and nonnegative");
  }
  if (dynamic_goal_braking_margin_enabled_ && !detail::validBrakingMargin(goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_))
  {
    throw nav2_core::PlannerException("FixedPathController dynamic braking margin parameters are invalid");
  }
  LOG_INFO("连续制动裕量：enabled={}，minimum={:.3f}m，high={:.3f}m，transition={:.3f}m/s，adaptive_branch={}", dynamic_goal_braking_margin_enabled_, goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_, adaptive_goal_braking_enabled_);
  control_duration_ = 1.0 / controller_frequency;
  // 只有自适应制动启用时，才校验其专属减速度／jerk／接近速度／延迟／距离裕量。
  // 关闭自适应后仍会执行前面的公共检查和已启用动态裕量的检查，并非跳过所有制动参数校验。
  if (adaptive_goal_braking_enabled_ && (!std::isfinite(adaptive_goal_max_deceleration_) || adaptive_goal_max_deceleration_ <= 0.0 || adaptive_goal_max_deceleration_ > 1.0 || !std::isfinite(adaptive_goal_jerk_limit_) || adaptive_goal_jerk_limit_ <= 0.0 || !std::isfinite(adaptive_goal_approach_speed_) || adaptive_goal_approach_speed_ <= 0.0 || !std::isfinite(adaptive_goal_response_time_) || adaptive_goal_response_time_ < 0.0 || !std::isfinite(adaptive_goal_distance_margin_) || adaptive_goal_distance_margin_ < 0.0))
  {
    throw nav2_core::PlannerException("FixedPathController adaptive braking parameters are invalid");
  }
  LOG_INFO("固定路径控制器配置完成，plugin={}，最大线速度={:.3f}m/s，终点减速度={:.3f}m/s^2，终点微量接近速度={:.3f}m/s，制动反应时间={:.3f}s，制动距离裕量={:.3f}m，终点基础前视={:.3f}m，前视参考速度={:.3f}m/s，前视速度增益={:.3f}s，前视范围=[{:.3f},{:.3f}]m，起点位置容差={:.3f}m，直接跟踪横向容差={:.3f}m，直接跟踪航向门限={:.3f}rad，严格对齐航向容差={:.3f}rad，终点按位置或越界锁存停车", plugin_name_, base_linear_velocity_, goal_linear_deceleration_, goal_final_approach_velocity_, goal_braking_reaction_time_, goal_braking_distance_margin_, goal_terminal_lookahead_dist_, goal_terminal_lookahead_reference_speed_, goal_terminal_lookahead_speed_gain_, goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_, start_position_tolerance_, direct_tracking_lateral_tolerance_, direct_tracking_max_yaw_error_, initial_yaw_tolerance_);
  LOG_INFO("固定路径起步限速配置：每条路径保护距离={:.3f}m，限速={:.3f}m/s，解除横向误差≤{:.3f}m，航向误差≤{:.3f}rad，连续周期={}", start_speed_limit_distance_, start_offset_speed_limit_, direct_tracking_lateral_tolerance_, start_speed_release_yaw_tolerance_, start_speed_release_stable_cycles_);
}

// 清空路径、起步／制动历史和共享依赖，供后续重新 configure。
// 不直接发布 Twist，也不证明底盘停稳。
void FixedPathController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
  start_path_points_.clear();
  start_speed_guard_.reset();
  nearest_index_ = 0;
  start_speed_prebraking_ = false;
  goal_tangent_index_ = 0;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  // 策略只按首次横向偏差选择；航向条件随后各分支再判断，不能把两者写成同一个首次布尔条件。
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  last_braking_deceleration_ = 0.0;
  last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  terminal_tangent_x_ = 0.0;
  terminal_tangent_y_ = 0.0;
  error_log_initialized_ = false;
}

// 记录插件已激活；插件由 Controller Server 调度，不能独立启动导航。
void FixedPathController::activate()
{
  LOG_INFO("固定路径控制器已激活，plugin={}", plugin_name_);
}

// 将内部阶段切到 STOPPED 并锁存停止请求，不从这里发布速度。
// 真正的零速输出仍由后续调用与 Controller Server 收口负责。
void FixedPathController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::STOPPED;
  terminal_stop_latched_ = true;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  stable_cycles_ = 0;
  LOG_INFO("固定路径控制器已停用，plugin={}", plugin_name_);
}

// 接收至少两点的完整 Path，复位最近点、起步保护与终点锁存，提取有效首末切线。
// 通过首点姿态与路径切线区分前进／倒车；新路径会重新开始起步保护，不继承旧任务状态。
void FixedPathController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 缺少坐标系或不足两个点时无法定义起点、终点及运动切线。
  if (path.poses.size() < 2 || path.header.frame_id.empty())
  {
    throw nav2_core::PlannerException("Fixed path requires a frame and at least two poses");
  }
  std::size_t tangent_index = 1;
  // 跳过与起点重合的前缀点，寻找首段真正具有长度的路径切线。
  while (tangent_index < path.poses.size() && poseDistance(path.poses.front(), path.poses[tangent_index]) <= 1e-6)
  {
    ++tangent_index;
  }
  // 所有路径点均与起点重合时，无法计算前进／后退的初始方向。
  if (tangent_index >= path.poses.size())
  {
    throw nav2_core::PlannerException("Fixed path has no valid tangent");
  }
  std::size_t goal_tangent_index = path.poses.size() - 2;
  // 从终点向前跳过重复点，选出最后一段有效切线用于越界停车判定。
  while (goal_tangent_index > 0 && poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    --goal_tangent_index;
  }
  // 回溯后仍无有效末段，终点平面和末段延长方向都无法定义。
  if (poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    throw nav2_core::PlannerException("Fixed path has no valid goal tangent");
  }
  const double start_path_yaw = std::atan2(path.poses[tangent_index].pose.position.y - path.poses.front().pose.position.y, path.poses[tangent_index].pose.position.x - path.poses.front().pose.position.x);
  const double goal_tangent_x = path.poses.back().pose.position.x - path.poses[goal_tangent_index].pose.position.x;
  const double goal_tangent_y = path.poses.back().pose.position.y - path.poses[goal_tangent_index].pose.position.y;
  const double goal_tangent_length = std::hypot(goal_tangent_x, goal_tangent_y);
  const double goal_path_yaw = std::atan2(goal_tangent_y, goal_tangent_x);
  const double direction_cosine = std::cos(normalizeAngle(poseYaw(path.poses.front()) - start_path_yaw));
  // 首点车头朝向必须能明确区分沿切线前进还是逆切线倒车，近乎垂直则拒绝。
  if (!std::isfinite(direction_cosine) || std::abs(direction_cosine) < 0.5)
  {
    throw nav2_core::PlannerException("Fixed path orientation does not encode a clear direction");
  }
  global_plan_ = path;
  start_path_points_.clear();
  start_path_points_.reserve(path.poses.size());
  for (const auto & path_pose : path.poses)
  {
    start_path_points_.emplace_back(path_pose.pose.position.x, path_pose.pose.position.y);
  }
  start_speed_guard_.reset();
  nearest_index_ = 0;
  start_speed_prebraking_ = true;
  goal_tangent_index_ = goal_tangent_index;
  // 车头与起点切线同向记为前进，反向记为倒车；后续速度输出使用此符号。
  direction_sign_ = direction_cosine > 0.0 ? 1 : -1;
  start_path_yaw_ = start_path_yaw;
  goal_path_yaw_ = goal_path_yaw;
  terminal_tangent_x_ = goal_tangent_x / goal_tangent_length;
  terminal_tangent_y_ = goal_tangent_y / goal_tangent_length;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  last_braking_deceleration_ = 0.0;
  last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  phase_ = Phase::ALIGN_START;
  error_log_initialized_ = false;
  LOG_INFO("固定路径已装载，frame={}，路径点数={}，方向={}，起点切线={:.6f}rad，终点切线={:.6f}rad", global_plan_.header.frame_id, global_plan_.poses.size(), direction_sign_ > 0 ? "forward" : "backward", start_path_yaw_, goal_path_yaw_);
}

// 本周期从路径坐标系位姿、实测 Twist 和 GoalChecker 容差生成控制目标。
// 执行顺序：起点对齐／保护 → 终点几何／锁停 → 前视曲率 → 纵向制动 → 方向化 Twist。
// 已锁停时提前返回零命令；PlannerException 交给宿主按 failure_tolerance 等策略处理。
geometry_msgs::msg::TwistStamped FixedPathController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 路径和 GoalChecker 是本周期求速度与到点容差的前提，缺失时不能继续控制。
  if (global_plan_.poses.size() < 2 || !goal_checker)
  {
    throw nav2_core::PlannerException("Fixed path controller has no valid plan or goal checker");
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  // 所有路径距离均在路径坐标系中计算，TF 失败不能用原始位姿代替。
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into fixed path frame");
  }
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  // 目标检查器提供位置容差与停稳速度阈值；读取失败时不继续计算本周期控制命令。
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance))
  {
    throw nav2_core::PlannerException("FixedPathController failed to read StoppedGoalChecker tolerances");
  }
  const double goal_xy_tolerance = pose_tolerance.position.x;
  const double linear_stopped_velocity = velocity_tolerance.linear.x;
  const double angular_stopped_velocity = velocity_tolerance.angular.z;
  if (goal_stop_entry_tolerance_ > goal_xy_tolerance)
  {
    throw nav2_core::PlannerException("FixedPathController stop entry tolerance exceeds goal tolerance");
  }
  // 两种制动共用停车入口：entry>0 用独立入口，entry=0 沿用 GoalChecker 的 XY 容差。
  // 正入口不得大于 XY 容差；例如入口 8 mm、精度半径 10 mm 时，停车门槛比精度门槛更靠近终点。
  // 该入口用于锁停及制动距离预留，不会把 GoalChecker 的精度半径改成 8 mm。
  const double stop_tolerance = goal_stop_entry_tolerance_ > 0.0 ? goal_stop_entry_tolerance_ : goal_xy_tolerance;
  // 拒绝非有限或负容差，避免后续停车锁存和停稳判定失去边界。
  if (!std::isfinite(goal_xy_tolerance) || !std::isfinite(linear_stopped_velocity) || !std::isfinite(angular_stopped_velocity) || goal_xy_tolerance <= 0.0 || linear_stopped_velocity < 0.0 || angular_stopped_velocity < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController requires valid position and stopped-velocity tolerances");
  }
  // 自适应制动或末端收敛任一启用时，拒绝非有限线／角速度并返回零命令。
  // 自适应关闭且末端收敛开启时，同样执行此零速保护。
  if ((adaptive_goal_braking_enabled_ || terminal_convergence_enabled_) && (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z)))
  {
    LOG_ERROR("固定路径里程计速度无效，输出零速保护");
    return zeroCommand();
  }
  const double start_distance = poseDistance(robot_pose, global_plan_.poses.front());
  // 移动中接收新路径时先平滑减速，不把减速和原地转向计入保护距离。
  if (start_speed_prebraking_)
  {
    if (detail::requiresStartPrebraking(velocity.linear.x, std::min(start_offset_speed_limit_, speed_limit_)))
    {
      LOG_DEBUG("固定路径起步预减速：实测速度={:.6f}，起步上限={:.6f}", velocity.linear.x, std::min(start_offset_speed_limit_, speed_limit_));
      return zeroCommand();
    }
    start_speed_prebraking_ = false;
  }
  // 起点阶段先决定是否需要原地转向；进入 TRACK_PATH 后不再重新选择策略。
  if (phase_ == Phase::ALIGN_START)
  {
    // 机器人离首点过远时，不允许直接驶向路径以补偿起点偏差。
    if (start_distance > start_position_tolerance_)
    {
      throw nav2_core::PlannerException("Robot is outside fixed path start position tolerance");
    }
    // 倒车时以车体反向轴作为运动方向，统一与路径切线比较航向误差。
    const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
    const double yaw_error = normalizeAngle(start_path_yaw_ - vehicle_motion_yaw);
    const double start_delta_x = robot_pose.pose.position.x - global_plan_.poses.front().pose.position.x;
    const double start_delta_y = robot_pose.pose.position.y - global_plan_.poses.front().pose.position.y;
    const double lateral_error = std::abs(-std::sin(start_path_yaw_) * start_delta_x + std::cos(start_path_yaw_) * start_delta_y);
    // 每条新路径只根据首次横向误差锁定一次起点策略，避免阈值附近来回切换。
    if (!start_strategy_evaluated_)
    {
      direct_start_tracking_ = lateral_error <= direct_tracking_lateral_tolerance_;
      start_strategy_evaluated_ = true;
      if (start_speed_guard_.evaluateStart())
      {
        LOG_INFO("固定路径起步限速已进入：横向偏差={:.3f}m，航向误差={:.3f}rad，纵向上限={:.3f}m/s，保护距离={:.3f}m", lateral_error, yaw_error, start_offset_speed_limit_, start_speed_limit_distance_);
      }
      LOG_INFO("固定路径起点策略已锁定，方向={}，起点距离={:.3f}m，横向误差={:.3f}m，运动方向航向误差={:.3f}rad，策略={}", direction_sign_ > 0 ? "forward" : "backward", start_distance, lateral_error, yaw_error, direct_start_tracking_ ? "direct_tracking" : "strict_alignment");
    }
    // 横向误差较小时采用较宽的航向门槛，尽量直接进入路径跟踪。
    if (direct_start_tracking_)
    {
      // 已满足直接跟踪门槛，切换状态并继续执行本周期的路径控制。
      if (std::abs(yaw_error) <= direct_tracking_max_yaw_error_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径小横向误差直接进入跟踪，方向={}，横向误差={:.3f}m，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", lateral_error, yaw_error);
      }
      else
      {
        // 航向尚未进入宽门槛，本周期只输出原地转向，不输出纵向速度。
        return rotateCommand(yaw_error, direct_tracking_max_yaw_error_, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
      }
    }
    else
    {
      // 横向偏差较大时，只按更严的航向容差决定何时结束原地旋转，不再等待车速停稳。
      if (std::abs(yaw_error) <= initial_yaw_tolerance_)
      {
        ++stable_cycles_;
      }
      else
      {
        // 航向重新超出容差时清零计数，避免累计不连续的对齐周期。
        stable_cycles_ = 0;
      }
      // 默认只需当前周期达标：立即切到跟踪，并沿本周期后续逻辑输出受限纵向速度。
      if (stable_cycles_ >= alignment_stable_cycles_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径严格起点航向对齐完成并进入低速跟踪，方向={}，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", yaw_error);
      }
      else
      {
        // 严格分支已单独判断 3 度切换门槛；转向指令按完整误差计算，避免在门槛外速度趋零而迟迟无法进入跟踪。
        return rotateCommand(yaw_error, 0.0, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
      }
    }
  }
  const double goal_distance = poseDistance(robot_pose, global_plan_.poses.back());
  // 终点航向只用于跟踪诊断；倒车仍按车体反向轴计算，不参与终点成功条件。
  const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
  const double goal_yaw_error = normalizeAngle(goal_path_yaw_ - vehicle_motion_yaw);
  nearest_index_ = findNearestIndex(robot_pose);
  // 仅在已触发起步限速时复核当前最近有效路径段；连续达标后单向解除。
  if (start_speed_guard_.active())
  {
    double local_lateral_error = 0.0;
    double local_heading_error = 0.0;
    double start_progress = 0.0;
    if (detail::calculateStartRecoveryErrors(start_path_points_, nearest_index_, robot_pose.pose.position.x, robot_pose.pose.position.y, poseYaw(robot_pose), direction_sign_, local_lateral_error, local_heading_error, &start_progress))
    {
      LOG_DEBUG("固定路径起步保护：路径进度={:.6f}m，保护距离={:.6f}m，横向误差={:.6f}m，航向误差={:.6f}rad", start_progress, start_speed_limit_distance_, local_lateral_error, local_heading_error);
      if (start_speed_guard_.observe(local_lateral_error, local_heading_error, direct_tracking_lateral_tolerance_, start_speed_release_yaw_tolerance_, start_speed_release_stable_cycles_, start_progress, start_speed_limit_distance_))
      {
        LOG_INFO("固定路径起步限速已解除，路径进度={:.3f}m，横向误差={:.3f}m，运动方向航向误差={:.3f}rad，连续达标周期={}", start_progress, local_lateral_error, local_heading_error, start_speed_release_stable_cycles_);
      }
    }
    else
    {
      start_speed_guard_.resetStability();
    }
  }
  // 速度幅值取基础上限、任务限速、当前起步上限三者最小值；方向符号在输出末尾才附加。
  const double effective_linear_limit = std::min(std::min(base_linear_velocity_, speed_limit_), start_speed_guard_.active() ? start_offset_speed_limit_ : base_linear_velocity_);
  LOG_DEBUG("固定路径起步速度链：保护={}，任务限速={:.6f}，有效上限={:.6f}", start_speed_guard_.active(), speed_limit_, effective_linear_limit);
  const auto & goal_position = global_plan_.poses.back().pose.position;
  const double goal_delta_x = robot_pose.pose.position.x - goal_position.x;
  const double goal_delta_y = robot_pose.pose.position.y - goal_position.y;
  // 终点纵向投影 = 位置误差向量 · 末段单位切线；投影≥0 表示到达或越过末段终点平面。
  const double terminal_projection = goal_delta_x * terminal_tangent_x_ + goal_delta_y * terminal_tangent_y_;
  // 只有已跟踪到末段且沿末段切线越过终点平面，才视为纵向越界。
  const bool goal_plane_crossed = nearest_index_ >= goal_tangent_index_ && terminal_projection >= 0.0;
  const double remaining = remainingDistance(nearest_index_, robot_pose);
  const double effective_remaining = std::max(remaining - stop_tolerance, 0.0);
  // 【速度基准】两分支均以非负幅值计算制动，倒车符号在最终 Twist 输出处添加。
  // adaptive_goal_braking_enabled=false：max(0, 任务方向×实测线速度)，反向残余运动按 0 计。
  // 自适应：abs(实测线速度)，把反向残余运动也计入停车距离；此选择与末端收敛开关无关。
  const double current_linear_velocity = adaptive_goal_braking_enabled_ ? std::abs(velocity.linear.x) : std::max(0.0, static_cast<double>(direction_sign_) * velocity.linear.x);
  // 【停车距离】固定减速度制动先计算 v²÷(2×goal_linear_deceleration_)，触发判断时再加反应距离。
  // 自适应调用纯函数：先将速度 0.2～1.5 m/s 映射为减速度，再以不超过 0.4 m/s² 的规划减速度估距。
  // 自适应预留 = 停车入口 + adaptive 距离裕量 + v×响应时间 + v×规划减速度÷(2×jerk 上限)。
  // 自适应停车距离 = v²÷(2×规划减速度) + 自适应预留，speed_limit 再由剩余可用距离反推。
  // speed_profile.deceleration 参与包络规划及日志；后续积分器实际传入的是 adaptive_goal_max_deceleration_。
  const auto speed_profile = adaptive_goal_braking_enabled_ ? calculateFixedPathSpeedProfile(current_linear_velocity, remaining, effective_linear_limit, adaptive_goal_max_deceleration_, adaptive_goal_jerk_limit_, adaptive_goal_response_time_, adaptive_goal_distance_margin_, stop_tolerance, adaptive_goal_approach_speed_) : FixedPathSpeedProfile{0.0, 0.0, 0.0};
  const double stopping_distance = adaptive_goal_braking_enabled_ ? speed_profile.stopping_distance : current_linear_velocity * current_linear_velocity / (2.0 * goal_linear_deceleration_);
  // 【末端前视】收敛开启用实测速度幅值，关闭用当前有效任务上限；不是按自适应制动开关选择。
  const double expected_linear_velocity = terminal_convergence_enabled_ ? std::abs(velocity.linear.x) : std::max(0.0, effective_linear_limit);
  // 末段前视随有效速度变化，但始终限制在配置的最小和最大距离之间。
  const double dynamic_terminal_lookahead_distance = std::clamp(goal_terminal_lookahead_dist_ + goal_terminal_lookahead_speed_gain_ * (expected_linear_velocity - goal_terminal_lookahead_reference_speed_), goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_);
  // 进入配置停车入口（未指定时用 XY 容差）或越过末段终点平面时锁停，准确性仍按 XY 容差另算。
  const auto terminal_position = detail::terminalPosition(goal_distance, goal_xy_tolerance, goal_stop_entry_tolerance_, goal_plane_crossed, terminal_stop_latched_, terminal_position_accurate_);
  const bool terminal_condition_reached = terminal_position.stopped;
  // 首次触发终点条件时锁存停车；位置是否准确单独记录，不把越界误报为到点成功。
  if (!terminal_stop_latched_ && terminal_condition_reached)
  {
    // 自适应分支以 >0.02 m/s 触发诊断警告；两分支仍执行相同锁停，警告本身不改变速度算法。
    if (adaptive_goal_braking_enabled_ && current_linear_velocity > 0.02)
    {
      LOG_WARN("固定路径终点在非低速状态下触发安全停车，实测线速度={:.4f}m/s", current_linear_velocity);
    }
    terminal_stop_latched_ = true;
    terminal_position_accurate_ = terminal_position.accurate;
    terminal_tracking_yaw_error_ = goal_yaw_error;
    phase_ = Phase::STOPPED;
    LOG_INFO("固定路径终点停车已锁存，原因={}，位置误差={:.4f}m，终点纵向投影={:.4f}m，锁存跟踪航向误差={:.3f}rad（{:.2f}deg），停车后不再旋转", goal_distance <= stop_tolerance ? "within_tolerance" : "goal_plane_crossed", goal_distance, terminal_projection, terminal_tracking_yaw_error_, terminal_tracking_yaw_error_ * 180.0 / M_PI);
  }
  // 【停车后的精度】收敛开启时按当前 XY 误差复核；仅有正入口时采用 terminalPosition 的精度结果。
  // 入口为 0 且收敛关闭时，下面的 else if 将“已进入精度半径”单向锁存为 true。
  else if (terminal_stop_latched_ && (goal_stop_entry_tolerance_ > 0.0 || terminal_convergence_enabled_))
  {
    // 收敛开启直接重算当前位置；收敛关闭且入口为正时，辅助函数也不继承历史准确状态。
    terminal_position_accurate_ = terminal_convergence_enabled_ ? goal_distance <= goal_xy_tolerance : terminal_position.accurate;
  }
  else if (terminal_stop_latched_ && !terminal_position_accurate_ && goal_distance <= goal_xy_tolerance)
  {
    terminal_position_accurate_ = true;
    LOG_INFO("固定路径停车后位置精度已进入容差并单向锁存，位置误差={:.4f}m，位置容差={:.4f}m", goal_distance, goal_xy_tolerance);
  }
  // 两分支一旦锁停便保持零速且不再转向；锁停后位置漂出容差不会重新开始跟踪。
  // 收敛开启时，连续 1 s 精度复核及停稳 2 s 后的超限成功兜底由 Controller Server 实现。
  // 兜底还需满足宿主的新鲜反馈／定位等条件；本插件返回零命令不直接返回 Action 成功。
  if (terminal_stop_latched_)
  {
    // 自适应分支在这里再次重算精度，即使 terminal_convergence_enabled=false 也会执行。
    // 此处覆盖前面的历史锁存结果，使自适应分支始终以当前 XY 误差作为准确性依据。
    if (adaptive_goal_braking_enabled_)
    {
      // 位置精度使用当前位姿，而非越过终点瞬间的锁存结果。
      terminal_position_accurate_ = goal_distance <= goal_xy_tolerance;
      // 当前 XY 超容差且纵向速度已低于停稳门槛时抛异常；此条件本身未检查角速度停稳。
      // 异常先交宿主按 failure_tolerance 处理；容忍时间耗尽可能先结束任务，不能保证等到 2 s 成功兜底。
      if (!terminal_position_accurate_ && current_linear_velocity <= linear_stopped_velocity)
      {
        throw nav2_core::PlannerException("Fixed path stopped beyond terminal position tolerance");
      }
    }
    logGoalErrors(goal_distance, terminal_tracking_yaw_error_, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, 0.0, stopping_distance, dynamic_terminal_lookahead_distance);
    return zeroCommand();
  }
  const double nominal_lookahead_distance = std::clamp(std::max(lookahead_dist_, std::abs(velocity.linear.x) * lookahead_time_), min_lookahead_dist_, max_lookahead_dist_);
  // 远离终点按速度选常规前视；接近终点时收缩到不小于末段动态前视的距离。
  const double normal_lookahead_distance = std::min(nominal_lookahead_distance, std::max(dynamic_terminal_lookahead_distance, goal_distance));
  // 收敛关闭直接使用 normal；开启时在距终点 0.5→0.1 m 区间平滑混合到末端前视。
  // 该开关同时影响横向几何和精度复核，但不会替换上面的纵向制动算法。
  const double lookahead_distance = terminal_convergence_enabled_ ? detail::terminalLookahead(normal_lookahead_distance, dynamic_terminal_lookahead_distance, goal_distance) : normal_lookahead_distance;
  auto carrot = selectCarrot(nearest_index_, lookahead_distance);
  carrot.header.frame_id = global_plan_.header.frame_id;
  carrot.header.stamp = pose.header.stamp;
  geometry_msgs::msg::PoseStamped local_carrot;
  // 曲率需要车体坐标系下的前视点；转换失败时不能用错误坐标计算转向。
  if (!transformPose(costmap_ros_->getBaseFrameID(), carrot, local_carrot))
  {
    throw nav2_core::PlannerException("Unable to transform fixed path lookahead point into robot frame");
  }
  const double carrot_distance_squared = local_carrot.pose.position.x * local_carrot.pose.position.x + local_carrot.pose.position.y * local_carrot.pose.position.y;
  // 前视点几乎与车体重合时曲率分母趋零，改用零曲率避免速度命令发散。
  // 车体坐标中的 Pure Pursuit 曲率 κ=2y÷(x²+y²)，单位 1/m；分母过小时使用零曲率。
  const double curvature = carrot_distance_squared > 1e-6 ? 2.0 * local_carrot.pose.position.y / carrot_distance_squared : 0.0;
  double linear_magnitude = effective_linear_limit;
  // 弯道曲率非零时再受最大角速度约束，直线段保持原线速度上限。
  if (std::abs(curvature) > 1e-6)
  {
    linear_magnitude = std::min(linear_magnitude, rotate_to_heading_angular_vel_ / std::abs(curvature));
  }
  // 【固定减速度制动裕量】动态开启时按 abs(v)÷transition 从 low 插值到 high，达到过渡速度后固定 high。
  // 动态关闭、速度无效或选择自适应时，辅助函数返回 high；braking_reaction_distance 用于固定减速度制动。
  const double effective_braking_margin = detail::brakingMargin(dynamic_goal_braking_margin_enabled_, adaptive_goal_braking_enabled_, velocity.linear.x, goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_);
  const double braking_reaction_distance = current_linear_velocity * goal_braking_reaction_time_ + effective_braking_margin;
  // 【进入制动】effective_remaining 已扣除停车入口，两分支都与 approach_velocity_scaling_dist_ 取较大值。
  // 固定减速度制动触发距离 = max(接近距离, 基础停车距离 + v×goal_braking_reaction_time_ + 距离裕量)。
  // 自适应制动触发距离 = max(接近距离, 已含 adaptive_goal_* 预留的停车距离)。
  const double braking_activation_distance = adaptive_goal_braking_enabled_ ? std::max(approach_velocity_scaling_dist_, stopping_distance) : std::max(approach_velocity_scaling_dist_, stopping_distance + braking_reaction_distance);
  // 首次进入减速区才锁定制动状态与当前命令，防止剩余距离波动反复切换阶段。
  if (!goal_braking_active_ && effective_remaining <= braking_activation_distance)
  {
    goal_braking_active_ = true;
    last_braking_command_magnitude_ = linear_magnitude;
    // 自适应首次进入制动时初始化 jerk 积分历史；setPlan 还会在每条新 Path 开始前清理历史。
    if (adaptive_goal_braking_enabled_)
    {
      last_braking_deceleration_ = 0.0;
      last_braking_update_time_ = clock_->now();
      LOG_INFO("固定路径进入自适应终点制动，剩余距离={:.4f}m，当前速度={:.4f}m/s，预留停车距离={:.4f}m，本周期减速度上限={:.3f}m/s^2", remaining, current_linear_velocity, stopping_distance, speed_profile.deceleration);
    }
    else
    {
      // 固定减速度制动在此记录阶段切入；加减速平滑由下游速度平滑器执行。
      LOG_INFO("固定路径进入终点平滑制动，剩余距离={:.4f}m，当前速度={:.4f}m/s，制动距离={:.4f}m，目标减速度={:.3f}m/s^2", remaining, current_linear_velocity, stopping_distance, goal_linear_deceleration_);
    }
  }
  // 只有制动阶段才改变纵向目标速度；此前由限速及曲率约束控制。
  if (goal_braking_active_)
  {
    // 【自适应输出】包络先给目标速度，再由上一命令、上一减速度和 dt 逐步逼近。
    // helper 中请求减速度 = clamp((上一命令−目标速度)÷dt, 0, adaptive 最大减速度)。
    // 下一减速度限制在“上一减速度±jerk×dt”内，输出 = max(目标速度, 上一命令−下一减速度×dt)。
    // 包络目标增大时，max(目标速度, 积分速度) 允许输出随之增大。
    if (adaptive_goal_braking_enabled_)
    {
      const auto update_time = clock_->now();
      double dt = control_duration_;
      // 有上次更新时间时使用真实周期；首周期使用配置的控制周期。
      if (last_braking_update_time_.nanoseconds() != 0)
      {
        dt = (update_time - last_braking_update_time_).seconds();
        // 仿真时钟倒退会使历史制动积分失效，重置状态并输出零速。
        if (dt < 0.0)
        {
          goal_braking_active_ = false;
          last_braking_deceleration_ = 0.0;
          last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          LOG_WARN("固定路径控制时钟回跳，重置制动状态并输出零速");
          return zeroCommand();
        }
        // 同一仿真时间戳可能覆盖多个控制回调，使用名义周期避免除零或停滞。
        if (dt == 0.0)
        {
          dt = control_duration_;
        }
      }
      // 限定时间步长，防止长时间调度间隔一次性放大命令变化。
      // 积分步长限制为 5～100 ms；ROS 时钟冻结时使用名义周期，回跳则在前面清制动状态并请求零速。
      dt = std::clamp(dt, 0.005, 0.1);
      const double braking_target = std::min(linear_magnitude, speed_profile.speed_limit);
      double next_deceleration = 0.0;
      // 这里传最大减速度参数，而非 speed_profile.deceleration，不能把日志值当成本次积分器硬上限。
      // jerk 约束针对正常制动积分；曲率／任务限速可再裁剪，终点锁停／异常零速也不经过这层积分。
      const double braking_command = advanceFixedPathBrakingSpeed(last_braking_command_magnitude_, braking_target, last_braking_deceleration_, adaptive_goal_max_deceleration_, adaptive_goal_jerk_limit_, dt, next_deceleration);
      linear_magnitude = std::min(linear_magnitude, braking_command);
      last_braking_command_magnitude_ = linear_magnitude;
      last_braking_deceleration_ = next_deceleration;
      last_braking_update_time_ = update_time;
    }
    else
    {
      // 【固定减速度制动输出】可用距离 = max(0, 剩余路程−停车入口−反应距离)，各项单位为 m。
      // 目标幅值 = min(当前几何上限, √(2×固定减速度×可用距离))，尚未到停车入口时再保留微量接近速度。
      // 此处直接计算目标，不在插件内按 jerk 积分；下游平滑器负责自己的加减速约束。
      const double braking_distance_remaining = std::max(effective_remaining - braking_reaction_distance, 0.0);
      double braking_target = std::min(linear_magnitude, std::sqrt(2.0 * goal_linear_deceleration_ * braking_distance_remaining));
      // 尚未进入位置容差时保留配置的微量接近速度，避免提前完全停住。
      if (goal_distance > stop_tolerance)
      {
        braking_target = std::min(linear_magnitude, std::max(braking_target, goal_final_approach_velocity_));
      }
      // 取上一命令与当前目标的较小值，使制动命令幅值单调下降。
      linear_magnitude = detail::boundedBrakingCommand(last_braking_command_magnitude_, braking_target);
      last_braking_command_magnitude_ = linear_magnitude;
    }
  }
  logGoalErrors(goal_distance, goal_yaw_error, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, linear_magnitude, stopping_distance, dynamic_terminal_lookahead_distance);
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  // 路径方向决定纵向命令正负；曲率再映射为受上限约束的角速度。
  // 最终加上方向符号：前进为正，倒车为负；ω=v×κ，随后角速度再限幅。
  command.twist.linear.x = static_cast<double>(direction_sign_) * linear_magnitude;
  command.twist.angular.z = std::clamp(command.twist.linear.x * curvature, -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);
  return command;
}

// 按 percentage 区分百分比与绝对上限，结果不能超过 base_linear_velocity。
// NO_SPEED_LIMIT 表示恢复基础上限，非有限／负值被忽略；不在这里生成制动曲线。
void FixedPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2 的“无限速”哨兵值表示恢复插件基础速度，而不是要求停车。
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
  {
    speed_limit_ = base_linear_velocity_;
    return;
  }
  // 非有限或负限速不覆盖当前有效值，避免异常消息污染速度链。
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
  {
    LOG_WARN("忽略非法固定路径限速：{}", speed_limit);
    return;
  }
  // 百分比以基础速度为基数，绝对值直接按 m/s 限幅；两者均不能超过基础速度。
  speed_limit_ = percentage ? base_linear_velocity_ * std::clamp(speed_limit / 100.0, 0.0, 1.0) : std::min(base_linear_velocity_, speed_limit);
}

// 在锁内返回停车锁存位；锁存只说明控制器持续要求零速，不说明轮速已为零。
bool FixedPathController::isTerminalStopLatched()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_stop_latched_;
}

// 在锁内返回控制器维护的位置状态，终点收敛分支可逐周期复核该值。
// 该布尔值不包含独立采样窗覆盖率或实际物理定位误差。
bool FixedPathController::isTerminalPositionAccurate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_position_accurate_;
}

// 同坐标系直接复制，否则通过 TF 在 transform_tolerance（s）内转换。
// 变换失败返回 false，调用方不得用未变换坐标继续计算。
bool FixedPathController::transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const
{
  // 坐标系已一致时直接复制，避免不必要的 TF 查询及时间外推失败。
  if (input.header.frame_id == frame)
  {
    output = input;
    return true;
  }
  // 坐标系不同才在配置的 TF 容差内请求变换。
  try
  {
    tf_->transform(input, output, frame, tf2::durationFromSec(transform_tolerance_));
    output.header.frame_id = frame;
    return true;
  }
  // TF 不可用时返回失败，由上层拒绝用未转换的位姿计算控制命令。
  catch (const tf2::TransformException & error)
  {
    RCLCPP_ERROR(logger_, "Fixed path transform failed: %s", error.what());
    return false;
  }
}

// 只在当前 nearest_index 及其之后搜索最近离散点，避免进度倒退。
// 输出是路径索引而非弧长；同距离候选保留先遇点。
std::size_t FixedPathController::findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::size_t best_index = nearest_index_;
  double best_distance = std::numeric_limits<double>::max();
  // 仅向前搜索，防止定位抖动让最近点索引回退并重复跟踪已通过路径。
  for (std::size_t index = nearest_index_; index < global_plan_.poses.size(); ++index)
  {
    const double distance = poseDistance(robot_pose, global_plan_.poses[index]);
    // 发现更近的候选点才更新索引；相等时保留先遇到的路径点。
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

// 当前位置到选定路径点的距离加上后续折线长度，单位 m。
// 这与到终点的直线距离不同，弯道与偏离路径时不能混用。
double FixedPathController::remainingDistance(const std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  double distance = poseDistance(robot_pose, global_plan_.poses[start_index]);
  // 当前位姿到最近路径点的距离加上其后各段弧长，得到控制器使用的剩余行程。
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    distance += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
  }
  return distance;
}

// 从 start_index 向前累计离散弧长，返回首个达到前视距离的路径点。
// 若余下路径不够长，沿末段单位切线外推；外推只服务曲率计算，不改变业务终点。
geometry_msgs::msg::PoseStamped FixedPathController::selectCarrot(const std::size_t start_index, const double lookahead_distance) const
{
  double accumulated = 0.0;
  // 沿离散路径累计弧长，优先选第一个达到前视距离的实际路径点。
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    accumulated += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
    // 距离已满足时直接返回路径点，不对中途段进行外推。
    if (accumulated >= lookahead_distance)
    {
      return global_plan_.poses[index];
    }
  }
  // 路径末段不足前视距离时沿终点切线延长 carrot，避免终点前视塌缩。
  auto carrot = global_plan_.poses.back();
  const double extension_distance = std::max(lookahead_distance - accumulated, 0.0);
  carrot.pose.position.x += terminal_tangent_x_ * extension_distance;
  carrot.pose.position.y += terminal_tangent_y_ * extension_distance;
  return carrot;
}

// 生成带底盘坐标系和当前时钟时间戳的零 TwistStamped，不发送消息。
geometry_msgs::msg::TwistStamped FixedPathController::zeroCommand() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  return command;
}

// 将剩余角误差同时受比例项、最大角速度及角减速度包络约束，再限制单周期角速度变化。
// 角误差单位 rad；纵向速度保持零，控制周期从配置频率得到。
geometry_msgs::msg::TwistStamped FixedPathController::rotateCommand(const double yaw_error, const double yaw_tolerance, const double max_angular_velocity, const double max_angular_accel, const double angular_kp, const geometry_msgs::msg::Twist & velocity) const
{
  auto command = zeroCommand();
  const double remaining_error = std::abs(yaw_error) - yaw_tolerance;
  // 航向已进入容差时不继续旋转，直接保持零速度。
  if (remaining_error <= 0.0)
  {
    return command;
  }
  // 航向误差正负决定转向方向；幅值同时受比例项、最大角速度和制动能力约束。
  const double direction = yaw_error > 0.0 ? 1.0 : -1.0;
  const double proportional_velocity = angular_kp * remaining_error;
  const double stopping_velocity = std::sqrt(2.0 * max_angular_accel * remaining_error);
  const double target_velocity = direction * std::min({max_angular_velocity, proportional_velocity, stopping_velocity});
  command.twist.angular.z = std::clamp(target_velocity, velocity.angular.z - max_angular_accel * control_duration_, velocity.angular.z + max_angular_accel * control_duration_);
  return command;
}

// 按 ROS 时钟限频保存终点误差、剩余路程和制动状态；时钟回跳允许重新记录。
// 日志只观察已算出的量，不改变停车、精度或任务状态。
void FixedPathController::logGoalErrors(const double position_error, const double yaw_error, const double goal_xy_tolerance, const double terminal_projection, const bool goal_plane_crossed, const double remaining_distance, const double current_linear_velocity, const double target_linear_velocity, const double stopping_distance, const double terminal_lookahead_distance)
{
  const auto now = clock_->now();
  const double log_period = 1.0 / goal_error_log_frequency_;
  // 仿真时钟未回跳且距上次记录不足周期时跳过日志，不影响速度计算。
  if (error_log_initialized_ && now.nanoseconds() >= last_error_log_time_.nanoseconds() && (now - last_error_log_time_).seconds() < log_period)
  {
    return;
  }
  last_error_log_time_ = now;
  error_log_initialized_ = true;
  LOG_INFO("固定路径终点误差：phase={}，位置误差={:.4f}m，路径剩余={:.4f}m，位置容差={:.4f}m，当前速度={:.4f}m/s，目标速度={:.4f}m/s，终点前视={:.4f}m，制动距离={:.4f}m，平滑制动={}，终点纵向投影={:.4f}m，越界={}，停车锁存={}，位置精度锁存={}，跟踪航向误差仅供诊断={:.3f}rad（{:.2f}deg）", phaseName(phase_), position_error, remaining_distance, goal_xy_tolerance, current_linear_velocity, target_linear_velocity, terminal_lookahead_distance, stopping_distance, goal_braking_active_, terminal_projection, goal_plane_crossed, terminal_stop_latched_, terminal_position_accurate_, yaw_error, yaw_error * 180.0 / M_PI);
}

// 将内部阶段枚举映射到稳定字符串，仅用于日志；未知值返回 UNKNOWN。
const char * FixedPathController::phaseName(const Phase phase)
{
  // 三个已定义控制阶段分别映射为稳定的诊断名称。
  switch (phase)
  {
    // 起点位置／航向检查阶段。
    case Phase::ALIGN_START:
      return "ALIGN_START";
    // 前视路径跟踪与终点制动阶段。
    case Phase::TRACK_PATH:
      return "TRACK_PATH";
    // 终点或停用后的零速锁存阶段。
    case Phase::STOPPED:
      return "STOPPED";
  }
  // 防御性兜底：遇到未列举枚举值时仍返回可读文本。
  return "UNKNOWN";
}

// 用 atan2(sin θ, cos θ) 将任意角差折回主值区间，θ 的单位为 rad。
double FixedPathController::normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 从四元数提取绕竖直轴的平面航向，单位 rad；不验证输入所在坐标系。
double FixedPathController::poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

// 计算两个同坐标系位置的二维欧氏距离，忽略 z，单位 m。
double FixedPathController::poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::FixedPathController, nav2_core::Controller)
