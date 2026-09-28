// 碰撞边界可视化节点。订阅 Collision Monitor 的区域多边形，并转成 RViz MarkerArray；仅负责显示，不参与速度决策。

// Copyright (c) 2026 zpy
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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

// 标签相对多边形包围盒的摆放角，避免文字遮住轮廓。
enum class LabelCorner
{
  UpperLeft,
  UpperRight,
  LowerRight
};

// 每个区域独立保留名称、颜色和标签方位；Marker ID 由订阅索引确定。
struct BoundaryStyle
{
  std::string label;
  float red;
  float green;
  float blue;
  LabelCorner label_corner;
};

// 只负责显示，区域触发和速度裁剪仍由 Collision Monitor 完成。
class CollisionBoundaryVisualizer : public rclcpp::Node
{
  public:
  // 按参数化话题列表建立订阅；额外兼容实车已有的 Approach 轮廓话题。
  CollisionBoundaryVisualizer() : Node("collision_boundary_visualizer")
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("collision_monitor_boundaries", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    declare_parameter<std::vector<std::string>>("polygon_topics", {"collision_stop_zone", "collision_slowdown_zone", "collision_approach_footprint"});
    const auto topics = get_parameter("polygon_topics").as_string_array();
    polygons_.resize(topics.size());
    styles_.reserve(topics.size());
    polygon_subs_.reserve(topics.size());
    for (std::size_t index = 0; index < topics.size(); ++index)
    {
      const auto & topic = topics[index];
      styles_.push_back(styleForTopic(topic));
      polygon_subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(topic, rclcpp::SystemDefaultsQoS(), [this, index](geometry_msgs::msg::PolygonStamped::SharedPtr message)
      {
        handlePolygon(std::move(message), index);
      }));
      if (topic == "collision_approach_footprint")
      {
        approach_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), [this, index](geometry_msgs::msg::PolygonStamped::SharedPtr message)
        {
          handlePolygon(std::move(message), index);
        });
      }
    }
  }

private:
  // 用区域话题推断可读标签和颜色；未知区域使用中性样式。
  BoundaryStyle styleForTopic(const std::string & topic) const
  {
    if (topic.find("stop") != std::string::npos)
    {
      return {"STOP", 0.55F, 0.02F, 0.02F, LabelCorner::UpperLeft};
    }
    if (topic.find("approach") != std::string::npos)
    {
      return {"APPROACH", 0.02F, 0.45F, 0.18F, LabelCorner::LowerRight};
    }
    if (topic.find("slowdown_l1") != std::string::npos)
    {
      return {"SLOWDOWN L1", 0.02F, 0.15F, 0.55F, LabelCorner::UpperRight};
    }
    if (topic.find("slow") != std::string::npos)
    {
      return {"SLOWDOWN", 0.02F, 0.15F, 0.55F, LabelCorner::UpperRight};
    }
    return {"ZONE", 0.5F, 0.5F, 0.5F, LabelCorner::UpperRight};
  }

  // 更新单个区域缓存，再重发所有已收到的区域，保证 RViz 显示完整。
  void handlePolygon(geometry_msgs::msg::PolygonStamped::SharedPtr message, std::size_t index)
  {
    if (message->polygon.points.empty())
    {
      return;
    }
    polygons_[index] = std::move(message);
    visualization_msgs::msg::MarkerArray marker_array;
    for (std::size_t boundary_index = 0; boundary_index < polygons_.size(); ++boundary_index)
    {
      if (!polygons_[boundary_index] || polygons_[boundary_index]->polygon.points.empty())
      {
        continue;
      }
      marker_array.markers.push_back(makeLineMarker(*polygons_[boundary_index], boundary_index));
      marker_array.markers.push_back(makeTextMarker(*polygons_[boundary_index], boundary_index));
    }
    marker_pub_->publish(marker_array);
  }

  // 构造闭合轮廓线；使用区域索引作为稳定 Marker ID。
  visualization_msgs::msg::Marker makeLineMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = polygon.header;
    marker.ns = "collision_monitor_boundary_lines";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    setColor(marker, styles_[index]);
    marker.frame_locked = true;
    for (const auto & polygon_point : polygon.polygon.points)
    {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = polygon_point.x;
      marker_point.y = polygon_point.y;
      marker_point.z = 0.04;
      marker.points.push_back(marker_point);
    }
    marker.points.push_back(marker.points.front());
    return marker;
  }

  // 构造始终朝向观察者的区域标签，与线条共享相同坐标系。
  visualization_msgs::msg::Marker makeTextMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = polygon.header;
    marker.ns = "collision_monitor_boundary_labels";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position = labelPosition(polygon, styles_[index].label_corner);
    marker.scale.z = 0.18;
    marker.text = styles_[index].label;
    setColor(marker, styles_[index]);
    marker.frame_locked = true;
    return marker;
  }

  // 从多边形包围盒计算标签位置，避免依赖顶点排列顺序。
  geometry_msgs::msg::Point labelPosition(const geometry_msgs::msg::PolygonStamped & polygon, LabelCorner corner) const
  {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto & point : polygon.polygon.points)
    {
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    geometry_msgs::msg::Point position;
    position.x = corner == LabelCorner::UpperLeft ? min_x - 0.10 : max_x + 0.10;
    position.y = corner == LabelCorner::LowerRight ? min_y - 0.10 : max_y + 0.10;
    position.z = 0.08;
    return position;
  }

  // 为轮廓线与文字统一设置区域配色及不透明度。
  void setColor(visualization_msgs::msg::Marker & marker, const BoundaryStyle & style) const
  {
    marker.color.r = style.red;
    marker.color.g = style.green;
    marker.color.b = style.blue;
    marker.color.a = 1.0;
  }

  std::vector<BoundaryStyle> styles_;
  std::vector<geometry_msgs::msg::PolygonStamped::SharedPtr> polygons_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> polygon_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr approach_footprint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}
// namespace nav2_regulated_modules

// 启动纯可视化 ROS 节点；退出时释放 ROS 上下文。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_regulated_modules::CollisionBoundaryVisualizer>());
  rclcpp::shutdown();
  return 0;
}
