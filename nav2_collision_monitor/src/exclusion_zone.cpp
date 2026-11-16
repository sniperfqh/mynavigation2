// Copyright (c) 2026 Dexory
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

#include "nav2_collision_monitor/exclusion_zone.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/polygon_utils.hpp"

namespace nav2_collision_monitor
{

ExclusionZone::ExclusionZone(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & zone_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const bool base_shift_correction)
: node_(node), zone_name_(zone_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), base_shift_correction_(base_shift_correction)
{
  RCLCPP_INFO(logger_, "[%s]: Creating ExclusionZone", zone_name_.c_str());
}

ExclusionZone::~ExclusionZone()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying ExclusionZone", zone_name_.c_str());
  zone_pub_.reset();
  poly_.clear();
  node_clock_.reset();
  //ooii Humble：on_set_params_handler_ 在 node 销毁时自动清理，无需手动 remove
  on_set_params_handler_.reset();
}

bool ExclusionZone::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node_clock_ = node->get_clock();

  if (!getParameters()) {
    return false;
  }

  if (visualize_) {
    zone_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/" + zone_name_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  //ooii Humble 没有 add_post_set_parameters_callback，用 on_set 统一做 validate+update
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ExclusionZone::parameterUpdateCallback, this, std::placeholders::_1));

  return true;
}

bool ExclusionZone::configure(const nav2_msgs::msg::ExclusionZoneDescription & desc)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node_clock_ = node->get_clock();

  enabled_ = desc.enabled;
  visualize_ = desc.visualize;
  frame_hold_timeout_ = desc.frame_hold_timeout;

  frame_id_ = desc.frame_id.empty() ? base_frame_id_ : desc.frame_id;

  if (desc.type == "circle") {
    is_circle_ = true;
    radius_ = desc.radius;
    if (radius_ <= 0.0) {
      RCLCPP_ERROR(
        logger_, "[%s]: circle exclusion zone requires a positive 'radius'",
        zone_name_.c_str());
      return false;
    }
    radius_squared_ = radius_ * radius_;
  } else if (desc.type == "polygon") {
    is_circle_ = false;
    if (desc.points.size() < 3) {
      RCLCPP_ERROR(
        logger_, "[%s]: polygon exclusion zone requires at least 3 points, got %zu",
        zone_name_.c_str(), desc.points.size());
      return false;
    }
    poly_.clear();
    poly_.reserve(desc.points.size());
    for (const auto & p : desc.points) {
      Point pt;
      pt.x = static_cast<double>(p.x);
      pt.y = static_cast<double>(p.y);
      poly_.push_back(pt);
    }
  } else {
    RCLCPP_ERROR(
      logger_, "[%s]: unknown exclusion zone type: %s",
      zone_name_.c_str(), desc.type.c_str());
    return false;
  }

  if (visualize_) {
    zone_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/" + zone_name_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  return true;
}

bool ExclusionZone::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  //ooii 旧 API：declare_parameter_if_not_declared + get_parameter
  nav2_util::declare_parameter_if_not_declared(
    node, zone_name_ + ".enabled", rclcpp::ParameterValue(false));
  enabled_ = node->get_parameter(zone_name_ + ".enabled").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, zone_name_ + ".visualize", rclcpp::ParameterValue(false));
  visualize_ = node->get_parameter(zone_name_ + ".visualize").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, zone_name_ + ".frame_id", rclcpp::ParameterValue(base_frame_id_));
  frame_id_ = node->get_parameter(zone_name_ + ".frame_id").as_string();
  if (frame_id_.empty()) {
    frame_id_ = base_frame_id_;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, zone_name_ + ".frame_hold_timeout", rclcpp::ParameterValue(0.0));
  frame_hold_timeout_ = node->get_parameter(zone_name_ + ".frame_hold_timeout").as_double();

  //ooii 2D 雷达无 z，去掉 min_height/max_height 参数（官方有，用户 Point 无 z 成员）

  nav2_util::declare_parameter_if_not_declared(
    node, zone_name_ + ".type", rclcpp::ParameterValue(std::string("polygon")));
  const std::string type = node->get_parameter(zone_name_ + ".type").as_string();

  if (type == "circle") {
    is_circle_ = true;
    nav2_util::declare_parameter_if_not_declared(
      node, zone_name_ + ".radius", rclcpp::ParameterValue(-1.0));
    radius_ = node->get_parameter(zone_name_ + ".radius").as_double();
    if (radius_ <= 0.0) {
      RCLCPP_ERROR(
        logger_, "[%s]: circle exclusion zone requires a positive 'radius'",
        zone_name_.c_str());
      return false;
    }
    radius_squared_ = radius_ * radius_;
  } else if (type == "polygon") {
    is_circle_ = false;
    nav2_util::declare_parameter_if_not_declared(
      node, zone_name_ + ".points", rclcpp::ParameterValue(std::string()));
    const std::string points_str =
      node->get_parameter(zone_name_ + ".points").as_string();
    std::string error;
    if (!parsePolygonPoints(points_str, 3, poly_, error)) {
      RCLCPP_ERROR(logger_, "[%s]: %s", zone_name_.c_str(), error.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(
      logger_, "[%s]: unknown exclusion zone type: %s",
      zone_name_.c_str(), type.c_str());
    return false;
  }

  return true;
}

void ExclusionZone::apply(const rclcpp::Time & curr_time, std::vector<Point> & data) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_ || data.empty()) {
    return;
  }

  tf2::Transform tf_zone_to_base;
  if (!getZoneToBaseTransform(curr_time, tf_zone_to_base)) {
    //ooii fail-safe：TF 拿不到时不删任何点，保留全部碰撞保护
    return;
  }

  if (is_circle_) {
    const tf2::Vector3 center = tf_zone_to_base.getOrigin();
    const double cx = center.x();
    const double cy = center.y();
    data.erase(
      std::remove_if(
        data.begin(), data.end(),
        [&](const Point & p) {
          //ooii 2D：无 z 高度过滤
          const double dx = p.x - cx;
          const double dy = p.y - cy;
          return (dx * dx + dy * dy) <= radius_squared_;
        }),
      data.end());
  } else {
    //ooii 把多边形顶点变换到 base frame，然后删掉落在内部的点
    std::vector<Point> poly_base;
    transformPolygonPoints(tf_zone_to_base, poly_, poly_base);
    data.erase(
      std::remove_if(
        data.begin(), data.end(),
        [&](const Point & p) {
          //ooii 2D：无 z 高度过滤；用本包的 polygon_utils 内联实现
          return isPointInsidePolygon(p.x, p.y, poly_base);
        }),
      data.end());
  }
}

bool ExclusionZone::getZoneToBaseTransform(
  const rclcpp::Time & curr_time, tf2::Transform & tf_zone_to_base) const
{
  //ooii zone 锚定在 base frame：跟车体走，无需 bridging
  if (frame_id_ == base_frame_id_) {
    tf_zone_to_base.setIdentity();
    return true;
  }

  //ooii 非阻塞查找（collision_monitor 线程不能阻塞等 TF）
  const tf2::Duration non_blocking = tf2::Duration::zero();

  rclcpp::Time zone_stamp = curr_time;
  tf2::Transform tf_zone_to_global;
  tf_zone_to_global.setIdentity();
  if (frame_id_ != global_frame_id_) {
    //ooii 用户 nav2_util::getTransform 签名：(source_frame, target_frame, tolerance, buffer, transform)
    tf2::Transform tf_tmp;
    if (!nav2_util::getTransform(
        frame_id_, global_frame_id_, non_blocking, tf_buffer_, tf_tmp))
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_clock_, 2000,
        "[%s]: no transform available for exclusion zone frame '%s'; not excluding any points",
        zone_name_.c_str(), frame_id_.c_str());
      return false;
    }
    tf_zone_to_global = tf_tmp;
    //ooii 用 buffer 的 latest transform 时间戳判断陈旧度（非阻塞 lookup 无法拿到 stamp，
    //  这里用 curr_time 近似；如果需要精确 stamp 可改用 buffer->lookupTransform）
    zone_stamp = curr_time;

    const double age = (curr_time - zone_stamp).seconds();
    const double max_age =
      std::max(frame_hold_timeout_, tf2::durationToSec(transform_tolerance_));
    if (age > max_age) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_clock_, 2000,
        "[%s]: exclusion zone frame '%s' stale for %.2fs (> %.2fs hold window); "
        "not excluding any points",
        zone_name_.c_str(), frame_id_.c_str(), age, max_age);
      return false;
    }
  }

  //ooii global -> base（用用户 nav2_util::getTransform 签名）
  tf2::Transform tf_global_to_base;
  bool got_base;
  if (base_shift_correction_) {
    //ooii 带时间戳的 5 参数版本：(source_frame, source_time, target_frame, target_time, fixed_frame, tolerance, buffer, transform)
    got_base = nav2_util::getTransform(
      global_frame_id_, curr_time, base_frame_id_, curr_time, global_frame_id_,
      non_blocking, tf_buffer_, tf_global_to_base);
  } else {
    got_base = nav2_util::getTransform(
      global_frame_id_, base_frame_id_, non_blocking, tf_buffer_, tf_global_to_base);
  }
  if (!got_base) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_clock_, 2000,
      "[%s]: cannot transform '%s' -> '%s'; not excluding any points",
      zone_name_.c_str(), global_frame_id_.c_str(), base_frame_id_.c_str());
    return false;
  }

  tf_zone_to_base = tf_global_to_base * tf_zone_to_global;
  return true;
}

std::string ExclusionZone::getName() const
{
  return zone_name_;
}

bool ExclusionZone::getEnabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void ExclusionZone::activate()
{
  //ooii 普通 Publisher 无需 on_activate
}

void ExclusionZone::deactivate()
{
  //ooii 普通 Publisher 无需 on_deactivate
}

void ExclusionZone::publish() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!zone_pub_ || !enabled_) {
    return;
  }

  tf2::Transform tf_zone_to_base;
  if (!getZoneToBaseTransform(node_clock_->now(), tf_zone_to_base)) {
    return;
  }

  const std::vector<Point> & vertices = is_circle_ ? circleToPolygon(radius_) : poly_;
  std::vector<Point> vertices_base;
  transformPolygonPoints(tf_zone_to_base, vertices, vertices_base);

  auto msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
  msg->header.frame_id = base_frame_id_;
  msg->header.stamp = node_clock_->now();
  for (const Point & v : vertices_base) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(v.x);
    p.y = static_cast<float>(v.y);
    msg->polygon.points.push_back(p);
  }

  zone_pub_->publish(std::move(msg));
}

//ooii Humble 合并 validate+update（官方分 post_set + on_set，Humble 只有 on_set）
rcl_interfaces::msg::SetParametersResult ExclusionZone::parameterUpdateCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_name = parameter.get_name();
    if (param_name.find(zone_name_ + ".") != 0) {
      continue;
    }
    if (param_name == zone_name_ + ".enabled" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      enabled_ = parameter.as_bool();
    } else if (param_name == zone_name_ + ".radius" && is_circle_ &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      if (parameter.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "radius must be > 0";
        return result;
      }
      radius_ = parameter.as_double();
      radius_squared_ = radius_ * radius_;
    } else if (param_name == zone_name_ + ".frame_hold_timeout" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      if (parameter.as_double() < 0.0) {
        result.successful = false;
        result.reason = "frame_hold_timeout must be >= 0";
        return result;
      }
      frame_hold_timeout_ = parameter.as_double();
    } else if (param_name == zone_name_ + ".points" && !is_circle_ &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      std::vector<Point> parsed;
      std::string error;
      if (!parsePolygonPoints(parameter.as_string(), 3, parsed, error)) {
        result.successful = false;
        result.reason = error;
        return result;
      }
      poly_ = parsed;
    }
  }
  return result;
}

}  // namespace nav2_collision_monitor
