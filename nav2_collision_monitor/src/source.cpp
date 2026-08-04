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

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 中文：Source 只保存所有传感器共有的运行上下文；具体消息格式和坐标点生成由派生类实现。
// 中文：数据源的输出契约是“追加 base_frame_id_ 坐标系下、当前时刻有效的二维点”。
Source::Source(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const std::string & global_frame_id, const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout, const bool base_shift_correction) : node_(node), source_name_(source_name), tf_buffer_(tf_buffer), base_frame_id_(base_frame_id), global_frame_id_(global_frame_id), transform_tolerance_(transform_tolerance), source_timeout_(source_timeout), base_shift_correction_(base_shift_correction) {
  // 中文：构造函数不创建订阅器，避免在 Lifecycle 尚未 configure 时触碰 ROS 资源。
}

Source::~Source() {
  // 中文：派生类析构函数会先释放具体订阅器；这里没有需要主动销毁的公共资源。
}

bool Source::configure() {
  // 中文：每个 Source 注册自己的动态参数回调，当前只支持运行时切换 enabled。
  auto node = node_.lock();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&Source::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

void Source::getCommonParameters(std::string & source_topic) {
  // 中文：所有传感器共享 Topic 和 enabled 参数，参数命名空间由 source_name_ 隔离。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".topic", rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(source_name_ + ".enabled").as_bool();
}

bool Source::sourceValid(const rclcpp::Time & source_time, const rclcpp::Time & curr_time) const {
  // 中文：用消息时间与 CollisionMonitor 当前时间的差值判断数据是否过期；未来时间不被这里拒绝，
  // 中文：具体时间语义仍由上层 TF 查询处理。
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (dt > source_timeout_) {
    RCLCPP_WARN(logger_, "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. " "Ignoring the source.", source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

bool Source::getEnabled() const {
  // 中文：主节点在每次处理循环调用该函数，禁用源不会参与点集合合并。
  return enabled_;
}

rcl_interfaces::msg::SetParametersResult Source::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  // 中文：参数回调不重建订阅器，只即时更新 enabled_；这样可以低成本暂停某一路传感器。
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == source_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_collision_monitor
