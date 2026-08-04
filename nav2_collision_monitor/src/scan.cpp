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

#include "nav2_collision_monitor/scan.hpp"

#include <cmath>
#include <functional>

namespace nav2_collision_monitor
{

// 中文：LaserScan 源把极坐标量测转换成二维点，重点处理数据过期、量程过滤和时间对齐。
Scan::Scan(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const std::string & global_frame_id, const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout, const bool base_shift_correction) : Source(node, source_name, tf_buffer, base_frame_id, global_frame_id, transform_tolerance, source_timeout, base_shift_correction), data_(nullptr) {
  // 中文：data_ 为空表示尚未收到首帧，getData() 会在该状态下安全返回空结果。
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan() {
  // 中文：释放订阅器后，消息回调不会再写入当前数据快照。
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure() {
  // 中文：先注册公共动态参数回调，再创建 LaserScan 订阅器；SensorDataQoS 适配高频传感器。
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(source_topic, scan_qos, std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::getData(const rclcpp::Time & curr_time, std::vector<Point> & data) const {
  // 中文：本函数只追加当前有效射线端点，不改变调用方已有的其他 Source 点。
  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  tf2::Transform tf_transform;
  if (base_shift_correction_) {
    // 中文：查询“消息采样时刻的传感器帧 -> 当前时刻的机器人帧”，补偿机器人运动和传感器延迟。
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    if (!nav2_util::getTransform(data_->header.frame_id, data_->header.stamp, base_frame_id_, curr_time, global_frame_id_, transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  } else {
    // 中文：只查询当前时刻的 source -> base，少一次时间插值，适合对延迟不敏感的场景。
    // Obtaining the transform to get data from source frame to base frame without time shift
    // considered. Less accurate but much more faster option not dependent on state estimation
    // frames.
    if (!nav2_util::getTransform(data_->header.frame_id, base_frame_id_, transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  }

  // Calculate poses and refill data array
  float angle = data_->angle_min;
  // 中文：每条有效射线按 LaserScan 的离散角度计算，再通过 tf_transform 投影到 base frame。
  for (size_t i = 0; i < data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(data_->ranges[i] * std::cos(angle), data_->ranges[i] * std::sin(angle), 0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;
  }
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // 中文：只保存最新消息共享指针，实际转换延迟到 CollisionMonitor 的 process() 线程执行。
  data_ = msg;
}

}  // namespace nav2_collision_monitor
