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

// 将多个 PolygonStamped 区域转为 RViz 线框和标签，只做可视化，不生成 collision 状态或运动命令。
// 按话题名缓存多边形，颜色由名称中的 stop／slow／approach 关键字推断；注释中的数组订阅代码不参与编译。
#include <algorithm>
#include <array>
//ooii 速度标注：makeLabelText() 用 fixed/setprecision 拼 v/w 与框尺寸
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
// 注释掉的代码不参与编译。
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

// 注释掉的代码不参与编译。
// enum class LabelCorner
// {
//   UpperLeft,
//   UpperRight,
//   LowerRight
// };

struct BoundaryStyle
{
  std::string label;
  float red;
  float green;
  float blue;
  //LabelCorner label_corner;
};

enum class ActionType
{
  Stop,
  Slowdown,
  Approach,
  Unknown
};


class CollisionBoundaryVisualizer : public rclcpp::Node
{
  public:
  // CollisionBoundaryVisualizer() : Node("collision_boundary_visualizer")
  // {
  //   marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("collision_monitor_boundaries", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  //   polygon_subs_[0] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_stop_zone", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message)
  //   {
  //     handlePolygon(std::move(message), 0);
  //   }
  //   );
  //   polygon_subs_[1] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_slowdown_zone", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message)
  //   {
  //     handlePolygon(std::move(message), 1);
  //   }
  //   );
  //   polygon_subs_[2] = create_subscription<geometry_msgs::msg::PolygonStamped>("collision_approach_footprint", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message)
  //   {
  //     handlePolygon(std::move(message), 2);
  //   }
  //   );
  //   approach_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), [this](geometry_msgs::msg::PolygonStamped::SharedPtr message)
  //   {
  //     handlePolygon(std::move(message), 2);
  //   }
  //   );
  // }
  // 注释掉的代码不参与编译。
  // 创建 Marker 输出和参数指定的多边形订阅；只在新多边形回调时重绘，没有独立碰撞判定。
  CollisionBoundaryVisualizer() : Node("collision_boundary_visualizer")
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "collision_monitor_boundaries",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    const std::vector<std::string> default_topics =
    {
      "collision_stop_zone",
      "collision_slowdown_zone",
      "collision_approach_footprint"
    };
    declare_parameter<std::vector<std::string>>("polygon_topics", default_topics);

    auto polygon_topics = get_parameter("polygon_topics").as_string_array();

    for (const auto & topic : polygon_topics)
    {
      //ooii 与发布端对齐：transient_local，启动/重连时立即拿到最后一帧碰撞框，
      // 避免"关闭界面再运行就不显示"
      auto subscription = create_subscription<geometry_msgs::msg::PolygonStamped>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [this, topic](geometry_msgs::msg::PolygonStamped::SharedPtr message)
        {
          handlePolygon(message, topic);
        });
      polygon_subs_.push_back(subscription);
    }

    //ooii 订阅 cmd_vel：获取当前速度(v/w)，标注在框旁边，方便对比 VP 跳变 vs box 线性变化
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::SharedPtr msg)
      {
        cmd_vel_linear_ = msg->linear.x;
        cmd_vel_angular_ = msg->angular.z;
      });
  }


private:
  // void handlePolygon(geometry_msgs::msg::PolygonStamped::SharedPtr message, std::size_t index)
  // {
  //   if (message->polygon.points.empty())
  //   {
  //     return;
  //   }
  //   polygons_[index] = std::move(message);
  //   visualization_msgs::msg::MarkerArray marker_array;
  //   for (std::size_t boundary_index = 0; boundary_index < polygons_.size(); ++boundary_index)
  //   {
  //     if (!polygons_[boundary_index] || polygons_[boundary_index]->polygon.points.empty())
  //     {
  //       continue;
  //     }
  //     marker_array.markers.push_back(makeLineMarker(*polygons_[boundary_index], boundary_index));
  //     marker_array.markers.push_back(makeTextMarker(*polygons_[boundary_index], boundary_index));
  //   }
  //   marker_pub_->publish(marker_array);
  // }

  // 注释掉的代码不参与编译。
  // 空多边形被忽略；非空消息按 topic 保存并触发全部已知边界重绘。
  // 缓存没有显式过期清理，不能用显示仍存在来证明输入源仍活跃。
  void handlePolygon(
    geometry_msgs::msg::PolygonStamped::SharedPtr message,
    const std::string & topic)
  {
    if (message->polygon.points.empty())
    {
      return;
    }
    polygons_[topic] = std::move(message);
    publishMarkers();
  }
  // 注释掉的代码不参与编译。
  // 遍历缓存，为每个有效区域生成一对线框与标签 Marker，再一次发布数组。
  // ID 由当前遍历顺序给出，不是外部区域固定编号。
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    std::size_t index = 0;
    for (const auto & [topic, polygon_ptr] : polygons_)
    {
      if (!polygon_ptr || polygon_ptr->polygon.points.empty())
      {
        continue;
      }
      const auto style = styleForTopic(topic);
      marker_array.markers.push_back(
        makeLineMarker(*polygon_ptr, static_cast<int>(index), style));
      marker_array.markers.push_back(
        makeTextMarker(*polygon_ptr, static_cast<int>(index), style));
      ++index;
    }
    marker_pub_->publish(marker_array);
  }

  // 注释掉的代码不参与编译。
  // 按不区分大小写的关键字推测显示类别，匹配优先级为 stop、approach、slow。
  // 这是显示约定，不读取 Collision Monitor 的实际 action_type 配置。
  ActionType typeForTopic(const std::string & topic) const
  {
    std::string lowered = topic;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);
    if (lowered.find("stop") != std::string::npos)
    {
      return ActionType::Stop;
    }
    if (lowered.find("approach") != std::string::npos)
    {
      return ActionType::Approach;
    }
    if (lowered.find("slow") != std::string::npos)
    {
      return ActionType::Slowdown;
    }
    return ActionType::Unknown;
  }

  // 把推测类别映射到标签与 RGB：停车红、减速蓝、接近绿、未知灰。
  // 颜色不会影响真正的速度过滤。
  BoundaryStyle styleForTopic(const std::string & topic) const
  {
    switch (typeForTopic(topic))
    {
      case ActionType::Stop:
        return BoundaryStyle
        {
          "STOP", 0.55F, 0.02F, 0.02F
        };
      case ActionType::Slowdown:
        return BoundaryStyle
        {
          "SLOWDOWN", 0.02F, 0.15F, 0.55F
        };
      case ActionType::Approach:
        return BoundaryStyle
        {
          "APPROACH", 0.02F, 0.45F, 0.18F
        };
      default:
        return BoundaryStyle
        {
          "ZONE", 0.5F, 0.5F, 0.5F
        };
    }
  }


  // visualization_msgs::msg::Marker makeLineMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const
  // {
  //   visualization_msgs::msg::Marker marker;
  //   marker.header = polygon.header;
  //   marker.ns = "collision_monitor_boundary_lines";
  //   marker.id = static                                                          _cast<int>(index);
  //   marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  //   marker.action = visualization_msgs::msg::Marker::ADD;
  //   marker.pose.orientation.w = 1.0;
  //   marker.scale.x = 0.025;
  //   setColor(marker, styles_[index]);
  //   marker.frame_locked = true;
  //   for (const auto & polygon_point : polygon.polygon.points)
  //   {
  //     geometry_msgs::msg::Point marker_point;
  //     marker_point.x = polygon_point.x;
  //     marker_point.y = polygon_point.y;
  //     marker_point.z = 0.04;
  //     marker.points.push_back(marker_point);
  //   }
  //   marker.points.push_back(marker.points.front());
  //   return marker;
  // }

  // 注释掉的代码不参与编译。
  // 沿多边形顶点画闭合 LINE_STRIP，显示高度固定为 0.04 m，线宽 0.025 m。
  // frame_locked 保持其跟随输入坐标系，Marker 不修改原始 Polygon。
  visualization_msgs::msg::Marker makeLineMarker(
    const geometry_msgs::msg::PolygonStamped & polygon, int index,
    const BoundaryStyle & style) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = polygon.header;
    marker.ns = "collision_monitor_boundary_lines";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    setColor(marker, style);
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


  // visualization_msgs::msg::Marker makeTextMarker(const geometry_msgs::msg::PolygonStamped & polygon, std::size_t index) const
  // {
  //   visualization_msgs::msg::Marker marker;
  //   marker.header = polygon.header;
  //   marker.ns = "collision_monitor_boundary_labels";
  //   marker.id = static_cast<int>(index);
  //   marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  //   marker.action = visualization_msgs::msg::Marker::ADD;
  //   marker.pose.orientation.w = 1.0;
  //   marker.pose.position = labelPosition(polygon, styles_[index].label_corner);
  //   marker.scale.z = 0.18;
  //   marker.text = styles_[index].label;
  //   setColor(marker, styles_[index]);
  //   marker.frame_locked = true;
  //   return marker;
  // }


  // 注释掉的代码不参与编译。
  // 在多边形外侧生成面向观察者的文字，使用独立命名空间避免与线框 ID 冲突。
  visualization_msgs::msg::Marker makeTextMarker(
    const geometry_msgs::msg::PolygonStamped & polygon, int index,
    const BoundaryStyle & style) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = polygon.header;
    marker.ns = "collision_monitor_boundary_labels";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position = labelPosition(polygon);
    marker.scale.z = 0.18;
    //ooii 标签带上速度和框尺寸，方便对比 VP(跳变) vs box(线性变化)
    marker.text = makeLabelText(polygon, style);
    setColor(marker, style);
    marker.frame_locked = true;
    return marker;
  }

  //ooii 生成标签文字：名称 + 速度 + 框尺寸(前/后/宽)，一眼看出跳变还是连续变化
  std::string makeLabelText(
    const geometry_msgs::msg::PolygonStamped & polygon,
    const BoundaryStyle & style) const
  {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto & p : polygon.polygon.points)
    {
      min_x = std::min(min_x, static_cast<double>(p.x));
      max_x = std::max(max_x, static_cast<double>(p.x));
      min_y = std::min(min_y, static_cast<double>(p.y));
      max_y = std::max(max_y, static_cast<double>(p.y));
    }
    const double front = max_x;   // 前方距离(base_footprint 原点在车体中心偏后)
    const double back = -min_x;   // 后方距离
    const double width = max_y - min_y;

    std::ostringstream oss;
    oss << style.label << "\n"
        << "v=" << std::fixed << std::setprecision(2) << cmd_vel_linear_
        << " w=" << std::fixed << std::setprecision(2) << cmd_vel_angular_ << "\n"
        << "F=" << std::fixed << std::setprecision(2) << front
        << " B=" << std::fixed << std::setprecision(2) << back
        << " W=" << std::fixed << std::setprecision(2) << width;
    return oss.str();
  }


  // geometry_msgs::msg::Point labelPosition(const geometry_msgs::msg::PolygonStamped & polygon, LabelCorner corner) const
  // {
  //   double min_x = std::numeric_limits<double>::max();
  //   double max_x = std::numeric_limits<double>::lowest();
  //   double min_y = std::numeric_limits<double>::max();
  //   double max_y = std::numeric_limits<double>::lowest();
  //   for (const auto & point : polygon.polygon.points)
  //   {
  //     min_x = std::min(min_x, static_cast<double>(point.x));
  //     max_x = std::max(max_x, static_cast<double>(point.x));
  //     min_y = std::min(min_y, static_cast<double>(point.y));
  //     max_y = std::max(max_y, static_cast<double>(point.y));
  //   }
  //   geometry_msgs::msg::Point position;
  //   position.x = corner == LabelCorner::UpperLeft ? min_x - 0.10 : max_x + 0.10;
  //   position.y = corner == LabelCorner::LowerRight ? min_y - 0.10 : max_y + 0.10;
  //   position.z = 0.08;
  //   return position;
  // }

  // 注释掉的代码不参与编译。
  // 取输入多边形的 XY 包围盒，在最大 x/y 外各偏移 0.10 m，高度 0.08 m。
  // 它不是几何中心，也不做地图坐标变换。
  geometry_msgs::msg::Point labelPosition(
    const geometry_msgs::msg::PolygonStamped & polygon) const
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
    position.x = max_x + 0.10;
    position.y = max_y + 0.10;
    position.z = 0.08;
    return position;
  }


  // 仅复制显示样式的 RGB 并将透明度设为 1，不改变输入边界几何。
  void setColor(visualization_msgs::msg::Marker & marker, const BoundaryStyle & style) const
  {
    marker.color.r = style.red;
    marker.color.g = style.green;
    marker.color.b = style.blue;
    marker.color.a = 1.0;
  }

  // 注释掉的代码不参与编译。
  // const std::array<BoundaryStyle, 3> styles_
  // {
  //   {
  //     {
  //       "STOP", 0.55F, 0.02F, 0.02F, LabelCorner::UpperLeft
  //     }
  //     ,
  //     {
  //       "SLOWDOWN", 0.02F, 0.15F, 0.55F, LabelCorner::UpperRight
  //     }
  //     ,
  //     {
  //       "APPROACH", 0.02F, 0.45F, 0.18F, LabelCorner::LowerRight
  //     }
  //   }
  // };
  // std::array<geometry_msgs::msg::PolygonStamped::SharedPtr, 3> polygons_;
  // std::array<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr, 3> polygon_subs_;
  // rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr approach_footprint_sub_;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  // 注释掉的代码不参与编译。
  std::unordered_map<std::string, geometry_msgs::msg::PolygonStamped::SharedPtr>
    polygons_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr>
    polygon_subs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  //ooii cmd_vel 缓存：标注速度用
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  double cmd_vel_linear_ = 0.0;
  double cmd_vel_angular_ = 0.0;

};

}
// namespace nav2_regulated_modules

// 将多个 PolygonStamped 区域转为 RViz 线框和标签，只做可视化，不生成 collision 状态或运动命令。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_regulated_modules::CollisionBoundaryVisualizer>());
  rclcpp::shutdown();
  return 0;
}
