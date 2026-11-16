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

#include "nav2_collision_monitor/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), source_timeout_(source_timeout),
  base_shift_correction_(base_shift_correction)
{
}

Source::~Source()
{
  //ooii 先清排除区（zone 的 on_set 回调需在 service 销毁前卸载）
  exclusion_zones_.clear();
  add_exclusion_zone_srv_.reset();
  remove_exclusion_zone_srv_.reset();
}

bool Source::configure()
{
  auto node = node_.lock();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Source::dynamicParametersCallback, this, std::placeholders::_1));

  //ooii 从参数读取静态排除区列表并逐个配置
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".exclusion_zones", rclcpp::ParameterValue(std::vector<std::string>()));
  const std::vector<std::string> zone_names =
    node->get_parameter(source_name_ + ".exclusion_zones").as_string_array();

  for (const std::string & zone_name : zone_names) {
    auto zone = std::make_shared<ExclusionZone>(
      node_, zone_name, tf_buffer_, base_frame_id_, global_frame_id_,
      transform_tolerance_, base_shift_correction_);
    if (!zone->configure()) {
      RCLCPP_ERROR(
        logger_, "[%s]: failed to configure exclusion zone '%s'",
        source_name_.c_str(), zone_name.c_str());
      return false;
    }
    exclusion_zones_[zone_name] = zone;
  }

  if (!zone_names.empty()) {  //ooii 有排除区才创建运行时服务
    add_exclusion_zone_srv_ = node->create_service<nav2_msgs::srv::AddExclusionZone>(
      source_name_ + "/add_exclusion_zone",
      std::bind(
        &Source::addExclusionZoneCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    remove_exclusion_zone_srv_ = node->create_service<nav2_msgs::srv::RemoveExclusionZone>(
      source_name_ + "/remove_exclusion_zone",
      std::bind(
        &Source::removeExclusionZoneCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  return true;
}

//ooii 非虚包装：先取原始点，再套 exclusion zone 过滤
bool Source::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  if (!getSourceData(curr_time, data)) {
    return false;
  }

  for (const auto & [name, zone] : exclusion_zones_) {
    (void)name;
    zone->apply(curr_time, data);
  }

  return true;
}

void Source::activate()
{
  for (auto & [name, zone] : exclusion_zones_) {
    (void)name;
    zone->activate();
  }
}

void Source::deactivate()
{
  for (auto & [name, zone] : exclusion_zones_) {
    (void)name;
    zone->deactivate();
  }
}

void Source::publishExclusionZones() const
{
  for (const auto & [name, zone] : exclusion_zones_) {
    (void)name;
    zone->publish();
  }
}

void Source::addExclusionZoneCallback(
  const std::shared_ptr<nav2_msgs::srv::AddExclusionZone::Request> request,
  std::shared_ptr<nav2_msgs::srv::AddExclusionZone::Response> response)
{
  const std::string & zone_name = request->zone.zone_name;
  if (zone_name.empty()) {
    response->success = false;
    response->message = "zone_name must not be empty";
    return;
  }

  auto node = node_.lock();
  if (!node) {
    response->success = false;
    response->message = "node is not available";
    return;
  }

  auto zone = std::make_shared<ExclusionZone>(
    node_, zone_name, tf_buffer_, base_frame_id_, global_frame_id_,
    transform_tolerance_, base_shift_correction_);
  if (!zone->configure(request->zone)) {
    response->success = false;
    response->message = "failed to configure exclusion zone '" + zone_name + "'";
    return;
  }

  std::lock_guard<std::mutex> lock(exclusion_zones_mutex_);
  exclusion_zones_[zone_name] = zone;
  response->success = true;
  response->message = "exclusion zone '" + zone_name + "' added";
  RCLCPP_INFO(
    logger_, "[%s]: added exclusion zone '%s' at runtime",
    source_name_.c_str(), zone_name.c_str());
}

void Source::removeExclusionZoneCallback(
  const std::shared_ptr<nav2_msgs::srv::RemoveExclusionZone::Request> request,
  std::shared_ptr<nav2_msgs::srv::RemoveExclusionZone::Response> response)
{
  std::lock_guard<std::mutex> lock(exclusion_zones_mutex_);

  if (request->remove_all) {
    const std::size_t count = exclusion_zones_.size();
    exclusion_zones_.clear();
    response->success = true;
    response->message = "removed " + std::to_string(count) + " exclusion zone(s)";
    return;
  }

  const auto it = exclusion_zones_.find(request->zone_name);
  if (it == exclusion_zones_.end()) {
    response->success = false;
    response->message = "exclusion zone '" + request->zone_name + "' not found";
    return;
  }
  exclusion_zones_.erase(it);
  response->success = true;
  response->message = "exclusion zone '" + request->zone_name + "' removed";
}

void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set default topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(source_name_ + ".enabled").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".source_timeout",
    rclcpp::ParameterValue(source_timeout_.seconds()));      // node source_timeout by default
  source_timeout_ = rclcpp::Duration::from_seconds(
    node->get_parameter(source_name_ + ".source_timeout").as_double());
}

bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (source_timeout_.seconds() != 0.0 && dt > source_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

bool Source::getEnabled() const
{
  return enabled_;
}

std::string Source::getSourceName() const
{
  return source_name_;
}

rclcpp::Duration Source::getSourceTimeout() const
{
  return source_timeout_;
}

rcl_interfaces::msg::SetParametersResult
Source::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if(param_name.find(source_name_ + ".") != 0) {
      continue;
    }
    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == source_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

bool Source::getTransform(
  const rclcpp::Time & curr_time,
  const std_msgs::msg::Header & data_header,
  tf2::Transform & tf_transform) const
{
  if (base_shift_correction_) {
    if (
      !nav2_util::getTransform(
        data_header.frame_id, data_header.stamp,
        base_frame_id_, curr_time, global_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return false;
    }
  } else {
    if (
      !nav2_util::getTransform(
        data_header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return false;
    }
  }
  return true;
}

}  // namespace nav2_collision_monitor
