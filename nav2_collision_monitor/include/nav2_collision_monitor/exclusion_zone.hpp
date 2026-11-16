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

#ifndef NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_
#define NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2/time.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_msgs/msg/exclusion_zone_description.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

//ooii 官方 ExclusionZone：排除区，在 source 点送入碰撞检测前先把落在区内的点删掉
// 适配 Humble 旧 API：nav2_util::LifecycleNode + tf2_ros::Buffer + declare_parameter_if_not_declared
class ExclusionZone
{
public:
  ExclusionZone(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & zone_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const bool base_shift_correction);

  ~ExclusionZone();

  bool configure();

  bool configure(const nav2_msgs::msg::ExclusionZoneDescription & desc);

  void activate();
  void deactivate();

  //ooii 删除落在排除区内的点（2D，用户 Point 无 z 成员，去掉高度过滤）
  void apply(const rclcpp::Time & curr_time, std::vector<Point> & data) const;

  std::string getName() const;

  bool getEnabled() const;

  void publish() const;

protected:
  bool getParameters();

  //ooii 合并 validate+update：Humble 没有 add_post_set_parameters_callback，用 on_set 统一处理
  rcl_interfaces::msg::SetParametersResult parameterUpdateCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  bool getZoneToBaseTransform(
    const rclcpp::Time & curr_time, tf2::Transform & tf_zone_to_base) const;

  // ----- Variables -----
  nav2_util::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};
  rclcpp::Clock::SharedPtr node_clock_;
  mutable std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

  std::string zone_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string base_frame_id_;
  std::string global_frame_id_;
  tf2::Duration transform_tolerance_;
  bool base_shift_correction_;

  double frame_hold_timeout_{0.0};

  std::string frame_id_;
  bool is_circle_{false};
  std::vector<Point> poly_;
  double radius_{0.0};
  double radius_squared_{0.0};
  bool enabled_{false};

  bool visualize_{false};
  //ooii 用普通 Publisher（用户 Source 体系不是 lifecycle publisher）
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr zone_pub_;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_
