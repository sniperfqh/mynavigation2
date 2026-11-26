// 声明参数并按生命周期创建双 Action、规划和跟踪客户端、遥控接口及通道探测，生效值由节点回退、YAML 和 launch 重写共同决定。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;

namespace nav2_regulated_modules
{

// 仅声明运行参数；ROS 实体在 configure 建立，业务入口在 activate 后才允许接收。
// 构造中的 false 等值是节点回退，不覆盖 Launch 从项目 YAML 注入的配置。
RegulatedNavigator::RegulatedNavigator(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("regulated_navigator", "", options)
{
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("fixed_execution_nodes", std::vector<std::string>{});
  declare_parameter("lattice_execution_nodes", std::vector<std::string>{});
  declare_parameter("tf_diagnostics_enabled", true);
  declare_parameter("tf_diagnostics_log_frequency", 1.0);
  declare_parameter("tf_diagnostics_map_base_frequency", 100.0);
  // 节点回退为关闭，统一 launch 依据 enable_collision_state 向两个执行通道及协调器注入一致的开关。
  declare_parameter("collision_state_enabled", false);
  declare_parameter("collision_state_topic", "/collision_state");
  declare_parameter("collision_state_timeout", 0.5);
  declare_parameter("qrcode_topic", "/qrcode");
  declare_parameter("navigation_service_action", "navigation_service");
  declare_parameter("charging_action", "charging");
  declare_parameter("charging_path_step", 0.04);
  declare_parameter("charging_path_extension", 0.3);
  declare_parameter("charging_qr_wait_timeout", 2.0);
  declare_parameter("charging_qr_max_age", 0.5);
  declare_parameter("fixed_path_visualization_topic", "fixed_path_plan");
  declare_parameter("fixed_path_boundaries_topic", "fixed_path_boundaries");
  declare_parameter("fixed_path_boundary_half_width", 0.4);
  declare_parameter("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter("compute_path_to_pose_action", "compute_path_to_pose");
  declare_parameter("follow_path_action", "follow_path");
  declare_parameter("planner_id", "GridBasedAstar");
  declare_parameter("controller_id", "RPP");
  declare_parameter("fixed_path_controller_id", "FixedPathController");
  declare_parameter("fixed_path_goal_checker_id", "fixed_path_goal_checker");
  declare_parameter("charging_controller_id", "ChargingController");
  declare_parameter("charging_checker_id", "charging_goal_checker");
  declare_parameter("goal_checker_id", "stopped_goal_checker");
  declare_parameter("smoother_id", "simple_smoother");
  declare_parameter("use_smoother", true);
  declare_parameter("check_smoother_collisions", true);
  declare_parameter("server_timeout", 5.0);
  declare_parameter("cancel_timeout", 2.0);
  declare_parameter("max_smoothing_duration", 2.0);
  declare_parameter("feedback_frequency", 5.0);
  declare_parameter("replan_frequency", 1.0);
  declare_parameter("autonomous_fixed_max_speed", 0.5);
  declare_parameter("autonomous_fixed_xy_tolerance", 0.01);
  declare_parameter("autonomous_fixed_yaw_tolerance", 0.15726646259971647);
  // 中间段节点回退容差为 0.01 米，统一 Lattice 配置将其覆盖为 0.02 米。
  declare_parameter("autonomous_fixed_cusp_xy_tolerance", 0.01);
  // 冻结路径分段判据，必须与控制器一致；状态格路径的毫米级拼接残差需按半格归为旋转段。
  declare_parameter("autonomous_fixed_rotation_distance", 1e-6);
  // 旋转段的位置验收半径，与控制器同名参数一致；默认 0 表示沿用中间段容差。
  declare_parameter("autonomous_fixed_rotation_position_tolerance", 0.0);
  declare_parameter("autonomous_fixed_obstacle_timeout", 60.0);
  declare_parameter("autonomous_fixed_resume_distance", 0.20);
  {
    declare_parameter("autonomous_fixed_short_pause_timeout", 0.5);
    declare_parameter("autonomous_fixed_resume_stable_duration", 0.15);
    declare_parameter("autonomous_fixed_tracking_localization_timeout", 0.6);
    declare_parameter("autonomous_fixed_terminal_localization_distance", 0.5);
  }
  declare_parameter("max_consecutive_planning_failures", 3);
  declare_parameter("max_recovery_rounds", 2);
  declare_parameter("costmap_update_wait_duration", 0.8);
  declare_parameter("passed_goal_radius", 0.7);
  declare_parameter("localization_timeout", 0.3);
  declare_parameter("enable_localization_jump_detection", false);
  declare_parameter("max_localization_translation_jump", 0.3);
  declare_parameter("max_localization_rotation_jump", 0.35);
  declare_parameter("progress_timeout", 10.0);
  declare_parameter("progress_min_translation", 0.1);
  declare_parameter("localization_recovery_timeout", 10.0);
  declare_parameter("localization_stable_duration", 0.5);
  declare_parameter("fixed_path_step", 0.1);
  declare_parameter("fixed_path_max_speed", 1.5);
  declare_parameter("stop_cmd_vel_topic", "cmd_vel_nav");
  declare_parameter("controller_cmd_vel_topic", "cmd_vel_nav");
  declare_parameter("smoothed_cmd_vel_topic", "cmd_vel");
  declare_parameter("velocity_odom_topic", "/odometry");
  declare_parameter("velocity_log_frequency", 1.0);
  declare_parameter("legacy_velocity_logging", true);
  declare_parameter<std::string>("speed_limit_topic", "speed_limit");
  declare_parameter<std::string>("chassis_input_topic", "/downstream/chassis_control");
  declare_parameter<std::string>("chassis_output_topic", "/control_to_uart");
  declare_parameter<std::string>("chassis_navigation_topic", "/cmd_vel");
  declare_parameter("chassis_motion_state_timeout", 0.2);
  declare_parameter("chassis_command_timeout", 0.15);
  declare_parameter("chassis_publish_rate", 100.0);
  declare_parameter("chassis_default_linear_speed_max", 1.5);
  declare_parameter("chassis_linear_speed_max", 0.3);
  declare_parameter("chassis_default_angular_speed_max", 0.5);
  declare_parameter("chassis_angular_speed_max", 0.3);
  declare_parameter("chassis_default_linear_accel_max", 2.0);
  declare_parameter("chassis_linear_accel_max", 3.0);
  declare_parameter("chassis_linear_decel_max", 4.0);
  declare_parameter("chassis_default_angular_accel_max", 1.5);
  declare_parameter("chassis_angular_accel_max", 2.0);
  declare_parameter("chassis_angular_decel_max", 3.0);
  declare_parameter("chassis_linear_accel_jerk_max", 6.0);
  declare_parameter("chassis_linear_decel_jerk_max", 8.0);
  declare_parameter("chassis_angular_accel_jerk_max", 4.0);
  declare_parameter("chassis_angular_decel_jerk_max", 6.0);
}

// 读取并校验配置，再创建客户端、Action 服务、订阅与定时器。
// 两个上层 Action 在配置阶段建立，接收 Goal 时核对激活状态、执行通道和底盘准入条件。
nav2_util::CallbackReturn RegulatedNavigator::on_configure(const rclcpp_lifecycle::State &)
{
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  navigation_service_action_ = get_parameter("navigation_service_action").as_string();
  charging_action_ = get_parameter("charging_action").as_string();
  charging_path_step_ = get_parameter("charging_path_step").as_double();
  charging_path_extension_ = get_parameter("charging_path_extension").as_double();
  charging_qr_wait_timeout_ = get_parameter("charging_qr_wait_timeout").as_double();
  charging_qr_max_age_ = get_parameter("charging_qr_max_age").as_double();
  fixed_path_visualization_topic_ = get_parameter("fixed_path_visualization_topic").as_string();
  fixed_path_boundaries_topic_ = get_parameter("fixed_path_boundaries_topic").as_string();
  fixed_path_controller_id_ = get_parameter("fixed_path_controller_id").as_string();
  fixed_path_goal_checker_id_ = get_parameter("fixed_path_goal_checker_id").as_string();
  charging_controller_id_ = get_parameter("charging_controller_id").as_string();
  charging_checker_id_ = get_parameter("charging_checker_id").as_string();
  // 按 collision_state_enabled 参数配置状态消费，与两条执行通道的速度门是否创建分别控制。
  collision_state_enabled_ = get_parameter("collision_state_enabled").as_bool();
  const auto collision_topic = get_parameter("collision_state_topic").as_string();
  collision_state_.configure(get_parameter("collision_state_timeout").as_double());
  if (collision_state_enabled_ && collision_topic.empty())
  {
    throw std::invalid_argument("collision_state_topic must not be empty");
  }
  collision_params_handler_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters)
    {
      if (nav2_util::isCollisionStateParameter(parameter.get_name()))
      {
        result.successful = false;
        result.reason = "collision state settings require lifecycle cleanup/configure";
        break;
      }
    }
    return result;
  });
  if (collision_state_enabled_)
  {
    collision_state_sub_ = create_subscription<std_msgs::msg::UInt8>(collision_topic, rclcpp::QoS(1).best_effort().durability_volatile(), [this](std_msgs::msg::UInt8::ConstSharedPtr msg)
    {
      collision_state_.update(msg->data);
    });
  }
  server_timeout_ = get_parameter("server_timeout").as_double();
  cancel_timeout_ = get_parameter("cancel_timeout").as_double();
  smoothing_duration_ = get_parameter("max_smoothing_duration").as_double();
  check_smoother_collisions_ = get_parameter("check_smoother_collisions").as_bool();
  feedback_frequency_ = get_parameter("feedback_frequency").as_double();
  max_recovery_rounds_ = get_parameter("max_recovery_rounds").as_int();
  costmap_wait_duration_ = get_parameter("costmap_update_wait_duration").as_double();
  passed_goal_radius_ = get_parameter("passed_goal_radius").as_double();
  localization_timeout_ = get_parameter("localization_timeout").as_double();
  enable_localization_jump_detection_ = get_parameter("enable_localization_jump_detection").as_bool();
  max_translation_jump_ = get_parameter("max_localization_translation_jump").as_double();
  max_rotation_jump_ = get_parameter("max_localization_rotation_jump").as_double();
  progress_min_translation_ = get_parameter("progress_min_translation").as_double();
  localization_recovery_timeout_ = get_parameter("localization_recovery_timeout").as_double();
  localization_stable_duration_ = get_parameter("localization_stable_duration").as_double();
  fixed_path_step_ = get_parameter("fixed_path_step").as_double();
  fixed_path_max_speed_ = get_parameter("fixed_path_max_speed").as_double();
  fixed_path_boundary_half_width_ = get_parameter("fixed_path_boundary_half_width").as_double();
  controller_cmd_vel_topic_ = get_parameter("controller_cmd_vel_topic").as_string();
  smoothed_cmd_vel_topic_ = get_parameter("smoothed_cmd_vel_topic").as_string();
  velocity_odom_topic_ = get_parameter("velocity_odom_topic").as_string();
  speed_limit_topic_ = get_parameter("speed_limit_topic").as_string();
  velocity_log_frequency_ = get_parameter("velocity_log_frequency").as_double();
  if (!std::isfinite(fixed_path_step_) || fixed_path_step_ <= 0.0)
  {
    LOG_ERROR("fixed_path_step 必须为有限正数，当前值={}", fixed_path_step_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!std::isfinite(fixed_path_max_speed_) || fixed_path_max_speed_ <= 0.0)
  {
    LOG_ERROR("fixed_path_max_speed 必须为有限正数，当前值={}", fixed_path_max_speed_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!std::isfinite(fixed_path_boundary_half_width_) || fixed_path_boundary_half_width_ <= 0.0)
  {
    LOG_ERROR("fixed_path_boundary_half_width 必须为有限正数，当前值={}", fixed_path_boundary_half_width_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!std::isfinite(velocity_log_frequency_) || velocity_log_frequency_ <= 0.0)
  {
    LOG_ERROR("velocity_log_frequency 必须为有限正数，当前值={}", velocity_log_frequency_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (fixed_path_controller_id_.empty())
  {
    LOG_ERROR("fixed_path_controller_id 不能为空");
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (fixed_path_goal_checker_id_.empty())
  {
    LOG_ERROR("fixed_path_goal_checker_id 不能为空");
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (charging_controller_id_.empty())
  {
    LOG_ERROR("charging_controller_id 不能为空");
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (charging_checker_id_.empty())
  {
    LOG_ERROR("charging_checker_id 不能为空");
    return nav2_util::CallbackReturn::FAILURE;
  }

  planning_module_.configure(get_parameter("planner_id").as_string(), get_parameter("smoother_id").as_string(), get_parameter("use_smoother").as_bool(), get_parameter("replan_frequency").as_double(), get_parameter("max_consecutive_planning_failures").as_int());
  control_module_.configure(get_parameter("controller_id").as_string(), get_parameter("goal_checker_id").as_string(), get_parameter("progress_timeout").as_double());
  motion_state_subscriber_ = std::make_unique<MotionStateSubscriber>(*this);
  ChassisControlConfig chassis_control_config;
  chassis_control_config.unified_output = true;
  chassis_control_config.navigation_topic = get_parameter("chassis_navigation_topic").as_string();
  chassis_control_config.request_takeover = [this]()
  {
    finishPending(false);
    cancelTask("后台遥控／统一出口保护接管，旧路径不自动恢复");
  };
  chassis_control_config.navigation_active = [this]()
  {
    return active_ && (task_.type != TaskType::NONE &&
      task_.state != NavigationState::CANCELING);
  };
  chassis_control_config.navigation_drained = [this]()
  {
    return unifiedDrained();
  };
  {
    chassis_control_config.lattice_navigation_topic = get_parameter("smoothed_cmd_vel_topic").as_string();
    chassis_control_config.navigation_source = [this]()
    {
      if (task_.state != NavigationState::CONTROLLING || !active_follow_goal_)
      {
        return -1;
      }
      return task_.type == TaskType::TO_POSE ? 1 : 0;
    };
    chassis_control_config.navigation_generation = [this]() {return follow_sequence_;};
  }
  chassis_control_config.input_topic = get_parameter("chassis_input_topic").as_string();
  chassis_control_config.output_topic = get_parameter("chassis_output_topic").as_string();
  chassis_control_config.motion_state_timeout = get_parameter("chassis_motion_state_timeout").as_double();
  chassis_control_config.command_timeout = get_parameter("chassis_command_timeout").as_double();
  chassis_control_config.publish_rate = get_parameter("chassis_publish_rate").as_double();
  chassis_control_config.default_linear_speed_max = get_parameter("chassis_default_linear_speed_max").as_double();
  chassis_control_config.linear_speed_max = get_parameter("chassis_linear_speed_max").as_double();
  chassis_control_config.default_angular_speed_max = get_parameter("chassis_default_angular_speed_max").as_double();
  chassis_control_config.angular_speed_max = get_parameter("chassis_angular_speed_max").as_double();
  chassis_control_config.default_linear_accel_max = get_parameter("chassis_default_linear_accel_max").as_double();
  chassis_control_config.linear_accel_max = get_parameter("chassis_linear_accel_max").as_double();
  chassis_control_config.linear_decel_max = get_parameter("chassis_linear_decel_max").as_double();
  chassis_control_config.default_angular_accel_max = get_parameter("chassis_default_angular_accel_max").as_double();
  chassis_control_config.angular_accel_max = get_parameter("chassis_angular_accel_max").as_double();
  chassis_control_config.angular_decel_max = get_parameter("chassis_angular_decel_max").as_double();
  chassis_control_config.linear_accel_jerk_max = get_parameter("chassis_linear_accel_jerk_max").as_double();
  chassis_control_config.linear_decel_jerk_max = get_parameter("chassis_linear_decel_jerk_max").as_double();
  chassis_control_config.angular_accel_jerk_max = get_parameter("chassis_angular_accel_jerk_max").as_double();
  chassis_control_config.angular_decel_jerk_max = get_parameter("chassis_angular_decel_jerk_max").as_double();

  try {
    chassis_control_subscriber_ = std::make_unique<ChassisControlSubscriber>(*this, *motion_state_subscriber_, chassis_control_config);
  }
  catch (const std::invalid_argument & error)
  {
    LOG_ERROR("ChassisControl 闭环配置失败：{}", error.what());
    motion_state_subscriber_.reset();
    return nav2_util::CallbackReturn::FAILURE;
  }

  compute_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, get_parameter("compute_path_to_pose_action").as_string());
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, get_parameter("follow_path_action").as_string());
  clear_local_client_ = create_client<ClearCostmap>("local_costmap/clear_entirely_local_costmap");
  clear_global_client_ = create_client<ClearCostmap>("global_costmap/clear_entirely_global_costmap");

  navigate_pose_server_ = rclcpp_action::create_server<NavigateToPose>(this, get_parameter("navigate_to_pose_action").as_string(), std::bind(&RegulatedNavigator::handlePoseGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handlePoseCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handlePoseAccepted, this, std::placeholders::_1));
  // 充电动作在各非遥控模式均可用；路径生成与控制尚未实现，当前仅在接受回调处记录占位结果。
  charging_server_ = rclcpp_action::create_server<Charging>(this, charging_action_, std::bind(&RegulatedNavigator::handleChargingGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handleChargingCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handleChargingAccepted, this, std::placeholders::_1));
  qr_code_sub_ = this->create_subscription<byd_custom_msgs::msg::QrCode>(get_parameter("qrcode_topic").as_string(), rclcpp::SensorDataQoS(), std::bind(&RegulatedNavigator::qrCodeMsgCallback, this, std::placeholders::_1));
  {
    navigation_service_server_ = rclcpp_action::create_server<NavigationService>(this, navigation_service_action_, std::bind(&RegulatedNavigator::handleNavigationServiceGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handleNavigationServiceCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handleNavigationServiceAccepted, this, std::placeholders::_1));
    fixed_path_pub_ = create_publisher<nav_msgs::msg::Path>(fixed_path_visualization_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    fixed_path_boundaries_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(fixed_path_boundaries_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }
  stop_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("stop_cmd_vel_topic").as_string(), rclcpp::SystemDefaultsQoS());
  {
    const std::vector<std::string> fixed_parameters =
    {
      "autonomous_fixed_max_speed", "autonomous_fixed_xy_tolerance", "autonomous_fixed_yaw_tolerance", "autonomous_fixed_cusp_xy_tolerance", "autonomous_fixed_rotation_distance", "autonomous_fixed_obstacle_timeout", "autonomous_fixed_resume_distance", "autonomous_fixed_short_pause_timeout", "autonomous_fixed_resume_stable_duration", "autonomous_fixed_tracking_localization_timeout", "autonomous_fixed_terminal_localization_distance"
    };
    for (const auto & parameter : fixed_parameters)
    {
      const double value = get_parameter(parameter).as_double();
      if (!std::isfinite(value) || value <= 0.0)
      {
        throw std::invalid_argument(std::string(parameter) + " must be finite and positive");
      }
    }
    if (get_parameter("autonomous_fixed_xy_tolerance").as_double() > 0.01)
    {
      throw std::invalid_argument("autonomous_fixed_xy_tolerance must not exceed 0.01 m");
    }
    if (get_parameter("autonomous_fixed_resume_stable_duration").as_double() >= get_parameter("autonomous_fixed_short_pause_timeout").as_double())
    {
      throw std::invalid_argument("Autonomous fixed resume stability must fit inside the short pause window");
    }
    frozen_status_sub_ = create_subscription<std_msgs::msg::String>("lattice_exec/autonomous_fixed/execution_status", rclcpp::QoS(10), std::bind(&RegulatedNavigator::onFixedExecutionStatus, this, std::placeholders::_1));
    frozen_path_pub_ = create_publisher<nav_msgs::msg::Path>("autonomous_fixed/plan", rclcpp::QoS(1).reliable().transient_local());
  }
  const auto velocity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  {
    controller_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(controller_cmd_vel_topic_, velocity_qos, std::bind(&RegulatedNavigator::onControllerVelocity, this, std::placeholders::_1));
    smoothed_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(smoothed_cmd_vel_topic_, velocity_qos, std::bind(&RegulatedNavigator::onSmoothedVelocity, this, std::placeholders::_1));
    if (velocity_odom_topic_.substr(velocity_odom_topic_.find_last_of('/') + 1) == "motion_state")
    {
      velocity_odom_sub_ = create_subscription<byd_custom_msgs::msg::MotionState>(velocity_odom_topic_, velocity_qos, std::bind(&RegulatedNavigator::onVelocityMotionState, this, std::placeholders::_1));
    }
    else
    {
      velocity_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(velocity_odom_topic_, velocity_qos, std::bind(&RegulatedNavigator::onVelocityOdometry, this, std::placeholders::_1));
    }
  }
  // 普通发布器用于绝对速度基准；它不是最终底盘速度出口。
  speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>( speed_limit_topic_, rclcpp::QoS(10));

  {
    lattice_follow_client_ = rclcpp_action::create_client<FollowPath>(this, "lattice_exec/follow_path");
    lattice_stop_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "lattice_exec/cmd_vel_nav", rclcpp::SystemDefaultsQoS());
    lattice_speed_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
      "lattice_exec/speed_limit", rclcpp::QoS(10));
    clear_local_client_ = create_client<ClearCostmap>(
      "fixed_path_exec/local_costmap/clear_entirely_local_costmap");
    clear_global_client_ = create_client<ClearCostmap>(
      "lattice_exec/global_costmap/clear_entirely_global_costmap");
    for (const auto lattice : {false, true})
    {
      auto & nodes = lattice ? lattice_execution_ : fixed_execution_;
      const auto names = get_parameter(lattice ? "lattice_execution_nodes" :
        "fixed_execution_nodes").as_string_array();
      for (const auto & name : names)
      {
        ExecutionNode node;
        node.client = create_client<StateService>(name + "/get_state");
        nodes.push_back(std::move(node));
      }
    }
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto feedback_period = std::chrono::duration<double>(1.0 / feedback_frequency_);
  feedback_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(feedback_period), std::bind(&RegulatedNavigator::publishFeedback, this));
  monitor_timer_ = create_wall_timer(200ms, std::bind(&RegulatedNavigator::monitorTask, this));
  if (get_parameter("legacy_velocity_logging").as_bool())
  {
    const auto velocity_log_period = std::chrono::duration<double>(1.0 / velocity_log_frequency_);
    velocity_log_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(velocity_log_period), std::bind(&RegulatedNavigator::logVelocityChain, this));
  }

  LOG_INFO("独立规控导航器配置完成，定位跳变保护={}，平移阈值={:.3f}m，旋转阈值={:.3f}rad，速度日志={}Hz，反馈={}，控制器输出={}，平滑器输出={}", enable_localization_jump_detection_ ? "启用" : "关闭", max_translation_jump_, max_rotation_jump_, velocity_log_frequency_, velocity_odom_topic_, controller_cmd_vel_topic_, smoothed_cmd_vel_topic_);
  return nav2_util::CallbackReturn::SUCCESS;
}

// 激活固定路径可视化与底盘支路，重置 collision 接收历史，再开放业务入口。
// 这只表示生命周期就绪，不生成任务，也不证明定位／轮速已经新鲜。
nav2_util::CallbackReturn RegulatedNavigator::on_activate(const rclcpp_lifecycle::State &)
{
  if (frozen_path_pub_)
  {
    frozen_path_pub_->on_activate();
  }
  collision_state_.reset();
  collision_progress_paused_ = false;
  if (fixed_path_pub_)
  {
    fixed_path_pub_->on_activate();
  }
  if (fixed_path_boundaries_pub_)
  {
    fixed_path_boundaries_pub_->on_activate();
  }
  if (chassis_control_subscriber_)
  {
    chassis_control_subscriber_->activate();
  }
  active_ = true;
  createBond();
  LOG_INFO("独立规控导航器已激活");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 先关闭业务许可，再停用底盘定时器并请求取消导航任务。
// 固定路径可视化停用、bond 销毁与实际机械停稳是不同事件。
nav2_util::CallbackReturn RegulatedNavigator::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (frozen_path_pub_)
  {
    frozen_path_pub_->on_deactivate();
  }
  active_ = false;
  if (chassis_control_subscriber_)
  {
    chassis_control_subscriber_->deactivate();
  }
  finishPending(false);
  cancelTask("节点停用");
  collision_state_.reset();
  collision_progress_paused_ = false;
  if (fixed_path_pub_)
  {
    fixed_path_pub_->on_deactivate();
  }
  if (fixed_path_boundaries_pub_)
  {
    fixed_path_boundaries_pub_->on_deactivate();
  }
  destroyBond();
  LOG_INFO("独立规控导航器已停用");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 释放配置阶段创建的 ROS 句柄、反馈缓存与 TF 对象。
// cleanup 是资源清理，不是控制权移交或完整的下游停稳等待。
nav2_util::CallbackReturn RegulatedNavigator::on_cleanup(const rclcpp_lifecycle::State &)
{
  frozen_status_sub_.reset();
  frozen_path_pub_.reset();
  collision_state_sub_.reset();
  collision_params_handler_.reset();
  collision_state_.reset();
  collision_progress_paused_ = false;
  cancelTask("节点清理");
  navigate_pose_server_.reset();
  navigation_service_server_.reset();
  {
    // 充电状态机随服务器一起清理：摘回 IDLE、停掉截止与消费定时器并摘除句柄
    // （cancelTask 已重置其余任务状态并请求在途跟随停止）。
    std::lock_guard<std::mutex> lock(charging_mutex_);
    charging_phase_ = ChargingPhase::IDLE;
    charging_qr_timeout_timer_.reset();
    charging_qr_poll_timer_.reset();
    charging_follow_goal_.reset();
    active_charging_goal_.reset();
  }
  charging_server_.reset();
  compute_pose_client_.reset();
  follow_client_.reset();
  lattice_follow_client_.reset();
  lattice_stop_pub_.reset();
  lattice_speed_pub_.reset();
  fixed_execution_.clear();
  lattice_execution_.clear();
  outstanding_plans_ = 0;
  fixed_path_outstanding_follows_ = 0;
  frozen_outstanding_follows_ = 0;
  clear_local_client_.reset();
  clear_global_client_.reset();
  if (chassis_control_subscriber_)
  {
    chassis_control_subscriber_->reset();
  }
  chassis_control_subscriber_.reset();
  motion_state_subscriber_.reset();
  qr_code_sub_.reset();
  controller_velocity_sub_.reset();
  smoothed_velocity_sub_.reset();
  velocity_odom_sub_.reset();
  stop_cmd_pub_.reset();
  fixed_path_pub_.reset();
  fixed_path_boundaries_pub_.reset();
  feedback_timer_.reset();
  monitor_timer_.reset();
  velocity_log_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    has_controller_velocity_ = false;
    has_smoothed_velocity_ = false;
    has_velocity_odometry_ = false;
  }
  tf_diagnostics_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  LOG_INFO("独立规控导航器已清理");
  return nav2_util::CallbackReturn::SUCCESS;
}

// 停用底盘支路并调用任务取消流程；是否发零速取决于当前任务状态。
// 不能仅凭本回调返回 SUCCESS 推断 UART 和轮速已完成停车。
nav2_util::CallbackReturn RegulatedNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (chassis_control_subscriber_)
  {
    chassis_control_subscriber_->deactivate();
  }
  cancelTask("节点关闭");
  LOG_INFO("独立规控导航器已关闭");
  return nav2_util::CallbackReturn::SUCCESS;
}

}
// namespace nav2_regulated_modules
