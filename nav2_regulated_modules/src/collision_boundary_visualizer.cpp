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
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

enum class LabelCorner
{
  UpperLeft,
  UpperRight,
  LowerRight
};

struct BoundaryStyle
{
  std::string label;
  float red;
  float green;
  float blue;
  LabelCorner label_corner;
};

class CollisionBoundaryVisualizer : public rclcpp::Node
{
public:
  CollisionBoundaryVisualizer() : Node("collision_boundary_visualizer") {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("collision_monitor_boundaries", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    polygon_subs_[0] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_stop_zone", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message) {handlePolygon(std::move(message), 0);});
    polygon_subs_[1] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_slowdown_zone", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message) {handlePolygon(std::move(message), 1);});
    polygon_subs_[2] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_approach_footprint", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message) {handlePolygon(std::move(message), 2);});
    approach_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message) {handlePolygon(std::move(message), 2);});
  }

private:
  void handlePolygon(geometry_msgs::msg::PolygonStamped::SharedPtr message, std::size_t index) {
    if (message->polygon.points.empty()) {
      return;
    }
    polygons_[index] = std::move(message);
    visualization_msgs::msg::MarkerArray marker_array;
    for (std::size_t boundary_index = 0; boundary_index < polygons_.size(); ++boundary_index) {
      if (!polygons_[boundary_index] || polygons_[boundary_index]->polygon.points.empty()) {
        continue;
      }
      marker_array.markers.push_back(makeLineMarker(*polygons_[boundary_index], boundary_index));
      marker_array.markers.push_back(makeTextMarker(*polygons_[boundary_index], boundary_index));
    }
    marker_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker makeLineMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const {
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
    for (const auto & polygon_point : polygon.polygon.points) {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = polygon_point.x;
      marker_point.y = polygon_point.y;
      marker_point.z = 0.04;
      marker.points.push_back(marker_point);
    }
    marker.points.push_back(marker.points.front());
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const {
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

  geometry_msgs::msg::Point labelPosition(const geometry_msgs::msg::PolygonStamped & polygon, LabelCorner corner) const {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto & point : polygon.polygon.points) {
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

  void setColor(visualization_msgs::msg::Marker & marker, const BoundaryStyle & style) const {
    marker.color.r = style.red;
    marker.color.g = style.green;
    marker.color.b = style.blue;
    marker.color.a = 1.0;
  }

  const std::array<BoundaryStyle, 3> styles_{{{"STOP", 0.55F, 0.02F, 0.02F, LabelCorner::UpperLeft}, {"SLOWDOWN", 0.02F, 0.15F, 0.55F, LabelCorner::UpperRight}, {"APPROACH", 0.02F, 0.45F, 0.18F, LabelCorner::LowerRight}}};
  std::array<geometry_msgs::msg::PolygonStamped::SharedPtr, 3> polygons_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr, 3> polygon_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr approach_footprint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace nav2_regulated_modules

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_regulated_modules::CollisionBoundaryVisualizer>());
  rclcpp::shutdown();
  return 0;
}
