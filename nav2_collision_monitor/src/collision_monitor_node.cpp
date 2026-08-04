// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

// 中文：本文件实现 Collision Monitor 的生命周期和实时速度决策闭环：
// 中文：cmd_vel 输入 -> 传感器点合并 -> 区域动作计算 -> 安全 cmd_vel 输出。
CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("collision_monitor", "", options), process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}}, stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0) {
  // 中文：节点初始为非 active；robot_action_prev_ 使用特殊负值仅用于首次状态变化日志。
}

CollisionMonitor::~CollisionMonitor() {
  // 中文：对象销毁前释放区域和数据源共享指针，具体订阅器由各自析构函数关闭。
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  // 中文：configure 阶段只装配 ROS 资源和配置对象，不处理速度；实际安全处理从 activate 开始。
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  // 中文：TF Listener 把 /tf 和 /tf_static 写入 Buffer，传感器 Source 与动态 Footprint 共用它。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;

  // Obtaining ROS parameters
  // 中文：getParameters() 还会创建 polygons_ 和 sources_，任一配置错误都会让节点 configure 失败。
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_in_topic, 1, std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, 1);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  // 中文：激活顺序先打开 Publisher 和区域可视化，再设置 process_active_，最后建立 Lifecycle Bond。
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  // 中文：即使尚未收到 cmd_vel，也立即发布一次区域，保证 RViz 启动后能看到安全边界。
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  // 中文：先关掉 process_active_，防止停用过程中新的输入速度继续触发区域判断。
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  // 中文：清理后不保留旧的传感器点、区域对象或 TF Listener，下一次 configure 会重新建立全部状态。
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  // 中文：shutdown 是 Lifecycle 的最终状态边界；实际资源释放主要已在 cleanup 完成。
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  // 中文：输入速度是整个安全门的触发时钟；每收到一帧就用同一时刻采集所有传感器数据。
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  process({msg->linear.x, msg->linear.y, msg->angular.z});
}

void CollisionMonitor::publishVelocity(const Action & robot_action) {
  // 中文：零速度具有特殊发布策略：刚停车时继续发一段时间，随后停止重复发零消息，减少下游负载。
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(std::string & cmd_vel_in_topic, std::string & cmd_vel_out_topic) {
  // 中文：集中读取节点级参数，并把公共 TF／超时配置传给每个 Polygon 和 Source。
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance = tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout = rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
  nav2_util::declare_parameter_if_not_declared(node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction = get_parameter("base_shift_correction").as_bool();

  nav2_util::declare_parameter_if_not_declared(node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ = rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    // 中文：区域参数错误会阻断节点启动，因为没有安全区域时不应假装安全监控已生效。
    return false;
  }

  if (!configureSources(base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(const std::string & base_frame_id, const tf2::Duration & transform_tolerance) {
  // 中文：按配置顺序创建区域，保持 polygons_ 的顺序；顺序影响同等级动作的最后触发区域日志。
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        // 中文：普通 Polygon 可用于静态 STOP、SLOWDOWN 或动态 Footprint APPROACH。
        polygons_.push_back(std::make_shared<Polygon>(node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        // 中文：Circle 用常数时间的半径判断替代通用点内算法，适合高频或近似圆形区域。
        polygons_.push_back(std::make_shared<Circle>(node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(get_logger(), "[%s]: Unknown polygon type: %s", polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(const std::string & base_frame_id, const std::string & odom_frame_id, const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout, const bool base_shift_correction) {
  // 中文：按 observation_sources 构建异构数据源列表；每个 Source 都向统一 Point 数组追加数据。
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(node, source_name + ".type", rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        // 中文：LaserScan 适配器将有效射线端点投影到 base frame。
        std::shared_ptr<Scan> s = std::make_shared<Scan>(node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        // 中文：PointCloud 适配器先按高度裁剪，再投影三维点到二维安全平面。
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        // 中文：Range 适配器把单次扇形量测离散为多个障碍点。
        std::shared_ptr<Range> r = std::make_shared<Range>(node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else {  // Error if something else
        RCLCPP_ERROR(get_logger(), "[%s]: Unknown source type: %s", source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity & cmd_vel_in) {
  // 中文：每帧决策都共享一个 curr_time，保证所有 Source 的超时和 TF 查询使用同一时间基准。
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  // 中文：不同传感器的点在 Source 内完成坐标变换，这里只负责合并，不再区分来源。
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    if (source->getEnabled()) {
      source->getData(curr_time, collision_points);
    }
  }

  // By default - there is no action
  // 中文：默认透传输入速度；后续区域只能保持或降低速度，不能放大速度。
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    // 中文：逐个区域求安全约束；STOP 一旦产生即提前结束，因为它比其他动作更保守。
    if (!polygon->getEnabled()) {
      continue;
    }
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.action_type != robot_action_prev_.action_type) {
    // 中文：日志只在状态变化时输出，避免高频 cmd_vel 造成重复日志。
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
  }

  // Publish required robot velocity
  publishVelocity(robot_action);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdown(const std::shared_ptr<Polygon> polygon, const std::vector<Point> & collision_points, const Velocity & velocity, Action & robot_action) const {
  // 中文：静态区域动作依赖点计数阈值；多个 SLOWDOWN 区域通过 Velocity::operator< 选择更小速度。
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(const std::shared_ptr<Polygon> polygon, const std::vector<Point> & collision_points, const Velocity & velocity, Action & robot_action) const {
  // 中文：APPROACH 先刷新动态 Footprint，再预测碰撞时间；速度比例由剩余安全时间决定。
  polygon->updatePolygon();

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0) {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::printAction(const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const {
  // 中文：把内部动作转换为可读日志，方便确认到底是哪个区域改变了输出速度。
  if (robot_action.action_type == STOP) {
    RCLCPP_INFO(get_logger(), "Robot to stop due to %s polygon", action_polygon->getName().c_str());
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(get_logger(), "Robot to slowdown for %f percents due to %s polygon", action_polygon->getSlowdownRatio() * 100, action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(get_logger(), "Robot to approach for %f seconds away from collision", action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(get_logger(), "Robot to continue normal operation");
  }
}

void CollisionMonitor::publishPolygons() const {
  // 中文：只发布启用区域；区域发布器是 Lifecycle Publisher，必须先在 on_activate 中激活。
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getEnabled()) {
      polygon->publish();
    }
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：同时支持独立 executable 和 ROS 2 component container 两种加载方式。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
