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

#include "nav2_collision_monitor/circle.hpp"

#include <math.h>
#include <cmath>
#include <exception>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 中文：Circle 复用 Polygon 的动作、参数和发布接口，只替换几何判断与可视化顶点生成算法。
// 中文：圆形判断使用半径平方，避免对每个传感器点调用 sqrt，适合高频安全监控。
Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  // 中文：构造阶段只保存父类资源，半径在 configure() 读取参数后才有效。
  RCLCPP_INFO(logger_, "[%s]: Creating Circle", polygon_name_.c_str());
}

Circle::~Circle()
{
  // 中文：显式记录生命周期边界；父类析构负责清理公共参数回调和顶点缓存。
  RCLCPP_INFO(logger_, "[%s]: Destroying Circle", polygon_name_.c_str());
}

void Circle::getPolygon(std::vector<Point> & poly) const
{
  // 中文：发布消息需要 Polygon 顶点，因此用 16 条等角边对圆进行可视化近似；
  // 中文：该近似不会参与 getPointsInside() 的实际安全判断。
  // Number of polygon points. More edges means better approximation.
  const double polygon_edges = 16;
  // Increment of angle during points position calculation
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();

  // Making new polygon looks like a circle
  Point p;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    poly.push_back(p);
  }
}

int Circle::getPointsInside(const std::vector<Point> & points) const
{
  // 中文：逐点比较到圆心的平方距离，返回落入圆形区域的障碍点数量。
  int num = 0;
  for (Point point : points) {
    if (point.x * point.x + point.y * point.y < radius_squared_) {
      num++;
    }
  }

  return num;
}

bool Circle::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  // 中文：Circle 不订阅动态 Footprint；它只读取公共 Polygon 参数和 radius。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  // There is no footprint subscription for the Circle. Thus, set string as empty.
  // 中文：空 Topic 告诉 Polygon::configure() 不创建 FootprintSubscriber。
  footprint_topic.clear();

  try {
    // Leave it not initialized: the will cause an error if it will not set
    // 中文：不提供默认半径，让缺少 radius 的配置在 configure 阶段显式失败。
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".radius", rclcpp::PARAMETER_DOUBLE);
    radius_ = node->get_parameter(polygon_name_ + ".radius").as_double();
    radius_squared_ = radius_ * radius_;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting circle parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

}  // namespace nav2_collision_monitor
