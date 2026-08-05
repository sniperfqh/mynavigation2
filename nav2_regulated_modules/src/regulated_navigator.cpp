#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

namespace nav2_regulated_modules
{

// 中文注释：构造函数只声明运行参数；ROS 通信接口延迟到 Lifecycle configure 阶段创建。
RegulatedNavigator::RegulatedNavigator(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("regulated_navigator", "", options) {
  // 中文注释：构造阶段只声明参数，通信接口在 configure 阶段创建。
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("operation_mode", "autonomous");
  declare_parameter("goal_topic", "goal_pose");
  declare_parameter("fixed_path_action", "follow_fixed_path");
  declare_parameter("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter("navigate_through_poses_action", "navigate_through_poses");
  declare_parameter("compute_path_to_pose_action", "compute_path_to_pose");
  declare_parameter("compute_path_through_poses_action", "compute_path_through_poses");
  declare_parameter("smooth_path_action", "smooth_path");
  declare_parameter("follow_path_action", "follow_path");
  declare_parameter("planner_id", "GridBasedAstar");
  declare_parameter("controller_id", "RPP");
  declare_parameter("goal_checker_id", "stopped_goal_checker");
  declare_parameter("smoother_id", "simple_smoother");
  declare_parameter("use_smoother", true);
  declare_parameter("check_smoother_collisions", true);
  declare_parameter("server_timeout", 5.0);
  declare_parameter("cancel_timeout", 2.0);
  declare_parameter("max_smoothing_duration", 2.0);
  declare_parameter("feedback_frequency", 5.0);
  declare_parameter("replan_frequency", 1.0);
  declare_parameter("max_consecutive_planning_failures", 3);
  declare_parameter("max_recovery_rounds", 2);
  declare_parameter("costmap_update_wait_duration", 0.8);
  declare_parameter("passed_goal_radius", 0.7);
  declare_parameter("localization_timeout", 0.3);
  declare_parameter("max_localization_translation_jump", 0.3);
  declare_parameter("max_localization_rotation_jump", 0.35);
  declare_parameter("progress_timeout", 10.0);
  declare_parameter("progress_min_translation", 0.1);
  declare_parameter("localization_recovery_timeout", 10.0);
  declare_parameter("localization_stable_duration", 0.5);
  declare_parameter("stop_cmd_vel_topic", "cmd_vel_nav");
}

// 中文注释：读取参数、配置策略模块，并创建 Action、Service、Topic、TF 和定时器接口。
nav2_util::CallbackReturn RegulatedNavigator::on_configure(const rclcpp_lifecycle::State &) {
  // 中文注释：读取配置并初始化规划、控制、恢复和定位监控所需接口。
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  const auto operation_mode = get_parameter("operation_mode").as_string();
  if (operation_mode == "remote") {
    operation_mode_ = NavigationMode::REMOTE;
  } else if (operation_mode == "autonomous") {
    operation_mode_ = NavigationMode::AUTONOMOUS;
  } else if (operation_mode == "fixed_path") {
    operation_mode_ = NavigationMode::FIXED_PATH;
  } else {
    LOG_ERROR("不支持的 operation_mode：{}", operation_mode);
    return nav2_util::CallbackReturn::FAILURE;
  }
  goal_topic_ = get_parameter("goal_topic").as_string();
  fixed_path_action_ = get_parameter("fixed_path_action").as_string();
  server_timeout_ = get_parameter("server_timeout").as_double();
  cancel_timeout_ = get_parameter("cancel_timeout").as_double();
  smoothing_duration_ = get_parameter("max_smoothing_duration").as_double();
  check_smoother_collisions_ = get_parameter("check_smoother_collisions").as_bool();
  feedback_frequency_ = get_parameter("feedback_frequency").as_double();
  max_recovery_rounds_ = get_parameter("max_recovery_rounds").as_int();
  costmap_wait_duration_ = get_parameter("costmap_update_wait_duration").as_double();
  passed_goal_radius_ = get_parameter("passed_goal_radius").as_double();
  localization_timeout_ = get_parameter("localization_timeout").as_double();
  max_translation_jump_ = get_parameter("max_localization_translation_jump").as_double();
  max_rotation_jump_ = get_parameter("max_localization_rotation_jump").as_double();
  progress_min_translation_ = get_parameter("progress_min_translation").as_double();
  localization_recovery_timeout_ = get_parameter("localization_recovery_timeout").as_double();
  localization_stable_duration_ = get_parameter("localization_stable_duration").as_double();

  planning_module_.configure(get_parameter("planner_id").as_string(), get_parameter("smoother_id").as_string(), get_parameter("use_smoother").as_bool(), get_parameter("replan_frequency").as_double(), get_parameter("max_consecutive_planning_failures").as_int());
  control_module_.configure(get_parameter("controller_id").as_string(), get_parameter("goal_checker_id").as_string(), get_parameter("progress_timeout").as_double());

  compute_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, get_parameter("compute_path_to_pose_action").as_string());
  compute_poses_client_ = rclcpp_action::create_client<ComputePathThroughPoses>(this, get_parameter("compute_path_through_poses_action").as_string());
  smooth_client_ = rclcpp_action::create_client<SmoothPath>(this, get_parameter("smooth_path_action").as_string());
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, get_parameter("follow_path_action").as_string());
  clear_local_client_ = create_client<ClearCostmap>("local_costmap/clear_entirely_local_costmap");
  clear_global_client_ = create_client<ClearCostmap>("global_costmap/clear_entirely_global_costmap");

  navigate_pose_server_ = rclcpp_action::create_server<NavigateToPose>(this, get_parameter("navigate_to_pose_action").as_string(), std::bind(&RegulatedNavigator::handlePoseGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handlePoseCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handlePoseAccepted, this, std::placeholders::_1));
  navigate_poses_server_ = rclcpp_action::create_server<NavigateThroughPoses>(this, get_parameter("navigate_through_poses_action").as_string(), std::bind(&RegulatedNavigator::handlePosesGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handlePosesCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handlePosesAccepted, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&RegulatedNavigator::onTopicGoal, this, std::placeholders::_1));
  if (operation_mode_ == NavigationMode::FIXED_PATH) {
    // 中文注释：业务 Action 提供取消、反馈和结果；新 Goal 通过 replace 语义替换旧路径任务。
    fixed_path_server_ = rclcpp_action::create_server<FollowFixedPath>(this, fixed_path_action_, std::bind(&RegulatedNavigator::handleFixedPathGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handleFixedPathCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handleFixedPathAccepted, this, std::placeholders::_1));
  }
  // 中文注释：取消、定位丢失或失败时直接向控制链入口发布零速度，形成停车兜底。
  stop_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("stop_cmd_vel_topic").as_string(), rclcpp::SystemDefaultsQoS());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto feedback_period = std::chrono::duration<double>(1.0 / feedback_frequency_);
  feedback_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(feedback_period), std::bind(&RegulatedNavigator::publishFeedback, this));
  monitor_timer_ = create_wall_timer(200ms, std::bind(&RegulatedNavigator::monitorTask, this));

  configured_ = true;
  LOG_INFO("独立规控导航器配置完成，operation_mode={}", operation_mode);
  return nav2_util::CallbackReturn::SUCCESS;
}

// 中文注释：激活业务入口并创建 Lifecycle Bond，随后开始接受导航任务。
nav2_util::CallbackReturn RegulatedNavigator::on_activate(const rclcpp_lifecycle::State &) {
  // 中文注释：入口最后激活，确保规划、控制和平滑服务器已由 Lifecycle Manager 拉起。
  active_ = true;
  createBond();
  LOG_INFO("独立规控导航器已激活");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 中文注释：停止接受业务并取消当前任务，确保节点停用后不再输出运动指令。
nav2_util::CallbackReturn RegulatedNavigator::on_deactivate(const rclcpp_lifecycle::State &) {
  // 中文注释：停用时先取消底层任务，防止 Lifecycle 关闭后仍继续输出速度。
  active_ = false;
  cancelTask("节点停用");
  destroyBond();
  LOG_INFO("独立规控导航器已停用");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 中文注释：取消任务并释放 configure 阶段创建的全部 ROS 接口，回到未配置状态。
nav2_util::CallbackReturn RegulatedNavigator::on_cleanup(const rclcpp_lifecycle::State &) {
  cancelTask("节点清理");
  navigate_pose_server_.reset();
  navigate_poses_server_.reset();
  fixed_path_server_.reset();
  compute_pose_client_.reset();
  compute_poses_client_.reset();
  smooth_client_.reset();
  follow_client_.reset();
  clear_local_client_.reset();
  clear_global_client_.reset();
  goal_sub_.reset();
  stop_cmd_pub_.reset();
  feedback_timer_.reset();
  monitor_timer_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  configured_ = false;
  LOG_INFO("独立规控导航器已清理");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 中文注释：进程关闭前再次执行任务取消和停车收口。
nav2_util::CallbackReturn RegulatedNavigator::on_shutdown(const rclcpp_lifecycle::State &) {
  cancelTask("节点关闭");
  LOG_INFO("独立规控导航器已关闭");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_regulated_modules
