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

// ooii 去噪：放在 sensor_msgs 头文件之前，并禁用 PCL 的 min/max 预定义宏，
// 避免与 <algorithm> / sensor_msgs 冲突
#ifndef PCL_NO_PREDEFINED
#define PCL_NO_PREDEFINED
#endif
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

bool PointCloud::getSourceData(const rclcpp::Time & curr_time, std::vector<Point> & data) {
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr) {
    return false;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return false;
  }

  tf2::Transform tf_transform;
  if (!getTransform(curr_time, data_->header, tf_transform)) {
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  // Refill data array with PointCloud points in base frame
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);

    // Check range from sensor origin before transformation
    double range = p_v3_s.length();
    if (range < min_range_) {
      continue;
    }

    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    //ooii 车体过滤：同时要求点在高度区间内且不在车体矩形内
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_ &&
      (std::abs(p_v3_b.x()) > robot_half_length_ || std::abs(p_v3_b.y()) > robot_half_width_))
    {
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
  //ooii
  // 与外部实现一致：点云与此前来源的碰撞点一并进行半径过滤。
  denoise(data);
  return true;
}

// ooii
void PointCloud::denoise(std::vector<Point> & data) const
{
  if (data.empty() || noise_min_neighbors_ <= 0)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(data.size());
  for (size_t i = 0; i < data.size(); ++i)
  {
    (*cloud)[i].x = static_cast<float>(data[i].x);
    (*cloud)[i].y = static_cast<float>(data[i].y);
    (*cloud)[i].z = 0.0f;
  }
  cloud->is_dense = true;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  ror.setInputCloud(cloud);
  ror.setRadiusSearch(noise_radius_);
  ror.setMinNeighborsInRadius(noise_min_neighbors_);
  ror.setNegative(false);

  pcl::PointCloud<pcl::PointXYZ> filtered;
  ror.filter(filtered);
  data.clear();
  data.reserve(filtered.size());
  for (const auto & point : filtered.points)
  {
    data.push_back({point.x, point.y});
  }
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
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_range", rclcpp::ParameterValue(0.0));
  min_range_ = node->get_parameter(source_name_ + ".min_range").as_double();
  //ooii 去噪
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".noise_radius", rclcpp::ParameterValue(0.1));
  noise_radius_ = node->get_parameter(source_name_ + ".noise_radius").as_double();
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".noise_min_neighbors", rclcpp::ParameterValue(2));
  noise_min_neighbors_ = node->get_parameter(source_name_ + ".noise_min_neighbors").as_int();

  //ooii 车体过滤：复用 collision_monitor 节点级的 robot_half_length/width
  nav2_util::declare_parameter_if_not_declared(node, "robot_half_length", rclcpp::ParameterValue(0.265));
  robot_half_length_ = node->get_parameter("robot_half_length").as_double();
  nav2_util::declare_parameter_if_not_declared(node, "robot_half_width", rclcpp::ParameterValue(0.18));
  robot_half_width_ = node->get_parameter("robot_half_width").as_double();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  data_ = msg;
}

}  // namespace nav2_collision_monitor
