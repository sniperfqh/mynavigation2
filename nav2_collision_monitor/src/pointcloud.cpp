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

#include "nav2_collision_monitor/pointcloud.hpp"

#include <cmath>
#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

PointCloud::PointCloud(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const std::string & global_frame_id, const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout, const bool base_shift_correction) : Source(node, source_name, tf_buffer, base_frame_id, global_frame_id, transform_tolerance, source_timeout, base_shift_correction), data_(nullptr) {
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud() {
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

void PointCloud::configure() {
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(source_topic, pointcloud_qos, std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));
}

void PointCloud::getData(const rclcpp::Time & curr_time, std::vector<Point> & data) const {
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  tf2::Transform tf_transform;
  if (base_shift_correction_) {
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    if (!nav2_util::getTransform(data_->header.frame_id, data_->header.stamp, base_frame_id_, curr_time, global_frame_id_, transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  } else {
    // Obtaining the transform to get data from source frame to base frame without time shift
    // considered. Less accurate but much more faster option not dependent on state estimation
    // frames.
    if (!nav2_util::getTransform(data_->header.frame_id, base_frame_id_, transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  // Refill data array with PointCloud points in base frame
  std::vector<Point> source_data;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      source_data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
  denoise(source_data);
  data.insert(data.end(), source_data.begin(), source_data.end());
}

void PointCloud::denoise(std::vector<Point> & data) const
{
  if (data.empty() || noise_min_neighbors_ <= 0)
  {
    return;
  }

  const double radius_squared = noise_radius_ * noise_radius_;
  std::vector<Point> kept;
  kept.reserve(data.size());
  for (size_t i = 0; i < data.size(); ++i)
  {
    int neighbors = 0;
    for (size_t j = 0; j < data.size(); ++j)
    {
      if (i == j)
      {
        continue;
      }
      const double dx = data[i].x - data[j].x;
      const double dy = data[i].y - data[j].y;
      if (dx * dx + dy * dy <= radius_squared && ++neighbors >= noise_min_neighbors_)
      {
        break;
      }
    }
    if (neighbors >= noise_min_neighbors_)
    {
      kept.push_back(data[i]);
    }
  }
  data.swap(kept);
}

void PointCloud::getParameters(std::string & source_topic) {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".noise_radius", rclcpp::ParameterValue(0.1));
  noise_radius_ = node->get_parameter(source_name_ + ".noise_radius").as_double();
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".noise_min_neighbors", rclcpp::ParameterValue(2));
  noise_min_neighbors_ = node->get_parameter(source_name_ + ".noise_min_neighbors").as_int();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  data_ = msg;
}

}  // namespace nav2_collision_monitor
