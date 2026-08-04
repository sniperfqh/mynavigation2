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

#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 中文：PointCloud 源把三维点云裁剪成二维安全点集；高度过滤必须在 TF 变换后进行，
// 中文：因为传感器坐标系的 z 轴可能与机器人 base frame 不同。
PointCloud::PointCloud(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const std::string & global_frame_id, const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout, const bool base_shift_correction) : Source(node, source_name, tf_buffer, base_frame_id, global_frame_id, transform_tolerance, source_timeout, base_shift_correction), data_(nullptr) {
  // 中文：data_ 为空表示尚未收到首帧点云，处理线程会忽略该源。
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud() {
  // 中文：释放订阅器，避免对象销毁后继续接收 PointCloud2 回调。
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

void PointCloud::configure() {
  // 中文：读取 Topic、高度上下限并按 SensorDataQoS 创建点云订阅器。
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
  // 中文：先完成消息时间有效性和 source->base TF 查询，再迭代点云字段，避免重复查询 TF。
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
    // 中文：时间对齐模式把点云采样时的传感器坐标变换到当前机器人基座坐标。
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    if (!nav2_util::getTransform(data_->header.frame_id, data_->header.stamp, base_frame_id_, curr_time, global_frame_id_, transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  } else {
    // 中文：快速模式使用当前帧变换，省略里程计／全局帧参与的时间补偿。
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
  // 中文：迭代器按 PointCloud2 的字段布局读取每个点；高度裁剪使用变换后的 z。
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
}

void PointCloud::getParameters(std::string & source_topic) {
  // 中文：min_height/max_height 定义投影到二维平面前保留的三维高度窗口。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // 中文：保存最新共享消息，不在订阅回调中做 TF 和大量点云遍历，避免阻塞 ROS 回调线程。
  data_ = msg;
}

}  // namespace nav2_collision_monitor
