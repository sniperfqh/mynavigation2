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

#include "nav2_collision_monitor/polygon.hpp"

#include <exception>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

// 中文：Polygon 实现安全区域的公共行为：参数装配、动态启停、可视化、点内判断和碰撞时间预测。
// 中文：Circle 只重写几何相关函数，APPROACH 的动态 Footprint 仍由本类统一管理。
Polygon::Polygon(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & polygon_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const tf2::Duration & transform_tolerance) : node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING), slowdown_ratio_(0.0), footprint_sub_(nullptr), tf_buffer_(tf_buffer), base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance) {
  // 中文：公共成员只建立对象级上下文；具体 action_type、顶点和可视化选项在 configure() 读取。
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon() {
  // 中文：清理顶点、动态参数回调和对象持有的轻量状态；FootprintSubscriber 由 unique_ptr 自动释放。
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  poly_.clear();
  dyn_params_handler_.reset();
}

bool Polygon::configure() {
  // 中文：配置顺序是参数 -> 可选 Footprint 订阅 -> 可视化消息缓存与发布器 -> 动态参数回调。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_pub_topic, footprint_topic)) {
    return false;
  }

  if (!footprint_topic.empty()) {
    // 中文：只有 APPROACH 多边形会提供 Footprint Topic；STOP／SLOWDOWN 使用静态几何。
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(node, footprint_topic, *tf_buffer_, base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  if (visualize_) {
    // 中文：可视化缓存使用 geometry_msgs::Point32，运行时发布时再补充时间戳和 frame_id。
    // Fill polygon_ points for future usage
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(polygon_pub_topic, polygon_qos);
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&Polygon::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

void Polygon::activate() {
  // 中文：只激活可视化 Publisher；碰撞判断本身由 CollisionMonitor 的 process_active_ 控制。
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void Polygon::deactivate() {
  // 中文：停用可视化消息输出，不修改区域参数或动态 Footprint 缓存。
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

std::string Polygon::getName() const {
  // 中文：返回配置中的实例名，用于日志标识和测试断言。
  return polygon_name_;
}

ActionType Polygon::getActionType() const {
  // 中文：返回该区域对应的 STOP、SLOWDOWN 或 APPROACH 行为。
  return action_type_;
}

bool Polygon::getEnabled() const {
  // 中文：主循环用该开关跳过动态禁用的安全区域。
  return enabled_;
}

int Polygon::getMaxPoints() const {
  // 中文：返回超过该数量才触发区域动作的障碍点阈值。
  return max_points_;
}

double Polygon::getSlowdownRatio() const {
  // 中文：返回 SLOWDOWN 对输入速度施加的比例；其他动作类型通常不会使用它。
  return slowdown_ratio_;
}

double Polygon::getTimeBeforeCollision() const {
  // 中文：返回 APPROACH 期望保持的最小碰撞时间窗口，单位为秒。
  return time_before_collision_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const {
  // 中文：复制当前顶点，避免调用方直接修改 Polygon 内部几何状态。
  poly = poly_;
}

void Polygon::updatePolygon() {
  // 中文：从 FootprintSubscriber 取机器人坐标系下的最新轮廓，同时更新碰撞判断和可视化缓存。
  if (footprint_sub_ != nullptr) {
    // Get latest robot footprint from footprint subscriber
    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    std::size_t new_size = footprint_vec.size();
    poly_.resize(new_size);
    polygon_.points.resize(new_size);

    geometry_msgs::msg::Point32 p_s;
    for (std::size_t i = 0; i < new_size; i++) {
      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
      p_s.x = footprint_vec[i].x;
      p_s.y = footprint_vec[i].y;
      polygon_.points[i] = p_s;
    }
  }
}

int Polygon::getPointsInside(const std::vector<Point> & points) const {
  // 中文：对输入点逐个执行点内测试，输出数量而非布尔值，以支持 max_points 阈值策略。
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

double Polygon::getCollisionTime(const std::vector<Point> & collision_points, const Velocity & velocity) const {
  // 中文：APPROACH 模型的核心预测：假设机器人按 velocity 运动，在 time_before_collision_ 内离散前推，
  // 中文：检查障碍点相对机器人是否进入当前 Footprint／Circle 区域。
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each simulation step
  std::vector<Point> points_transformed = collision_points;

  // Check static polygon
  // 中文：先检查当前姿态，已在区域内代表立即碰撞，直接返回 0 秒。
  if (getPointsInside(points_transformed) >= max_points_) {
    return 0.0;
  }

  // Robot movement simulation
  // 中文：每步同时推进 pose 和 velocity；旋转速度会让后续线速度方向随时间变化。
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time interval
    // NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a collision
    // as if robot was moved with given velocity
    if (getPointsInside(points_transformed) > max_points_) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

void Polygon::publish() const {
  // 中文：发布的是当前机器人坐标系下的区域轮廓，供 RViz 检查配置和动态 Footprint 是否正确。
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s = std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygon_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

bool Polygon::getCommonParameters(std::string & polygon_pub_topic) {
  // 中文：解析所有形状共享的行为参数；失败时返回 false，让 Lifecycle configure 终止。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Get action type.
    // 中文：字符串参数是外部配置入口，内部统一映射为 ActionType 枚举。
    // Leave it not initialized: the will cause an error if it will not set.
    nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".action_type", rclcpp::PARAMETER_STRING);
    const std::string at_str = node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else {  // Error if something else
      RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
      return false;
    }

    nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".enabled", rclcpp::ParameterValue(true));
    enabled_ = node->get_parameter(polygon_name_ + ".enabled").as_bool();

    nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();

    if (action_type_ == SLOWDOWN) {
      // 中文：只有减速区域需要读取 slowdown_ratio，比例越小表示越保守。
      nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
      slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
    }

    if (action_type_ == APPROACH) {
      // 中文：APPROACH 需要碰撞时间窗口和模拟步长；步长越小预测越精细但计算量越大。
      nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".time_before_collision", rclcpp::ParameterValue(2.0));
      time_before_collision_ = node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
      nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".simulation_time_step", rclcpp::ParameterValue(0.1));
      simulation_time_step_ = node->get_parameter(polygon_name_ + ".simulation_time_step").as_double();
    }

    nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
    visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
    if (visualize_) {
      // 中文：可视化开启时才声明发布 Topic，避免无意义地创建参数和 Publisher。
      // Get polygon topic parameter in case if it is going to be published
      nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name_));
      polygon_pub_topic = node->get_parameter(polygon_name_ + ".polygon_pub_topic").as_string();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "[%s]: Error while getting common polygon parameters: %s", polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool Polygon::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic) {
  // 中文：基类默认实现处理静态多边形和动态 Footprint 两种配置分支；Circle 重写该函数读取 radius。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  try {
    if (action_type_ == APPROACH) {
      // 中文：APPROACH 的区域几何来自 Costmap Footprint Topic，不需要在 YAML 重复写 points。
      // Obtain the footprint topic to make a footprint subscription for approach polygon
      nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".footprint_topic", rclcpp::ParameterValue("local_costmap/published_footprint"));
      footprint_topic = node->get_parameter(polygon_name_ + ".footprint_topic").as_string();

      // This is robot footprint: do not need to get polygon points from ROS parameters.
      // It will be set dynamically later.
      return true;
    } else {
      // 中文：STOP／SLOWDOWN 必须提供至少四个顶点构成的闭合多边形点列。
      // Make it empty otherwise
      footprint_topic.clear();
    }

    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row = node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for points format correctness
    // 中文：数组长度必须为偶数且至少包含四个点；奇数或过短配置会被拒绝。
    if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(logger_, "[%s]: Polygon has incorrect points description", polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    // 中文：按 x、y 交替读取一维参数数组，依次追加到内部顶点列表。
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "[%s]: Error while getting polygon parameters: %s", polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

rcl_interfaces::msg::SetParametersResult Polygon::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  // 中文：当前只动态处理 enabled，几何和动作参数仍保持 configure 时的值，避免运行中重建资源。
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == polygon_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

inline bool Polygon::isPointInside(const Point & point) const {
  // 中文：水平射线从 point 沿 x 正方向发出；与多边形边界交点次数为奇数时判定在内部。
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_[i].x + (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) / (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

}  // namespace nav2_collision_monitor
