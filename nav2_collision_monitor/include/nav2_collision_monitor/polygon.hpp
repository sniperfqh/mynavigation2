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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic polygon shape class.
 * For STOP/SLOWDOWN model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 * 中文：多边形是 Collision Monitor 的通用安全区域抽象，保存动作阈值、可视化发布器和可选动态 Footprint。
 * 中文：STOP／SLOWDOWN 使用静态顶点，APPROACH 可以从 FootprintSubscriber 每次更新机器人轮廓。
 */
class Polygon
{
public:
  /**
   * @brief Polygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of polygon
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   */
  Polygon(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & polygon_name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id, const tf2::Duration & transform_tolerance);
  /**
   * @brief Polygon destructor
   */
  virtual ~Polygon();

  /**
   * @brief Shape configuration routine. Obtains ROS-parameters related to shape object
   * and creates polygon lifecycle publisher.
   * @return True in case of everything is configured correctly, or false otherwise
   * 中文：按 action_type 读取阈值和顶点／Footprint 参数；需要可视化时创建 Lifecycle Publisher。
   */
  bool configure();
  /**
   * @brief Activates polygon lifecycle publisher
   */
  void activate();
  /**
   * @brief Deactivates polygon lifecycle publisher
   */
  void deactivate();

  /**
   * @brief Returns the name of polygon
   * @return Polygon name
   */
  std::string getName() const;
  /**
   * @brief Obtains polygon action type
   * @return Action type for current polygon
   */
  ActionType getActionType() const;
  /**
  * @brief Obtains polygon enabled state
  * @return Whether polygon is enabled
  */
  bool getEnabled() const;
  /**
   * @brief Obtains polygon maximum points to enter inside polygon causing no action
   * @return Maximum points to enter to current polygon and take no action
   */
  int getMaxPoints() const;
  /**
   * @brief Obtains speed slowdown ratio for current polygon.
   * Applicable for SLOWDOWN model.
   * @return Speed slowdown ratio
   */
  double getSlowdownRatio() const;
  /**
   * @brief Obtains required time before collision for current polygon.
   * Applicable for APPROACH model.
   * @return Time before collision in seconds
   */
  double getTimeBeforeCollision() const;

  /**
   * @brief Gets polygon points
   * @param poly Output polygon points (vertices)
   * 中文：返回当前机器人坐标系下的顶点副本，供测试、可视化和碰撞预测使用。
   */
  virtual void getPolygon(std::vector<Point> & poly) const;

  /**
   * @brief Updates polygon from footprint subscriber (if any)
   * 中文：APPROACH 多边形从最新 Footprint 消息更新；静态 polygon 和 Circle 不会被修改。
   */
  void updatePolygon();

  /**
   * @brief Gets number of points inside given polygon
   * @param points Input array of points to be checked
   * @return Number of points inside polygon. If there are no points,
   * returns zero value.
   * 中文：逐点调用射线交叉算法统计区域内点数，阈值比较由 CollisionMonitor 决定。
   */
  virtual int getPointsInside(const std::vector<Point> & points) const;

  /**
   * @brief Obtains estimated (simulated) time before a collision.
   * Applicable for APPROACH model.
   * @param collision_points Array of 2D obstacle points
   * @param velocity Simulated robot velocity
   * @return Estimated time before a collision. If there is no collision,
   * return value will be negative.
   * 中文：从当前 base frame 原点开始按输入速度离散投影机器人，并把障碍点变换到每个预测位姿下；
   * 中文：返回首次超过 max_points_ 的模拟时刻，静态已碰撞返回 0，无碰撞返回负数。
   */
  double getCollisionTime(const std::vector<Point> & collision_points, const Velocity & velocity) const;

  /**
   * @brief Publishes polygon message into a its own topic
   * 中文：在 base_frame_id_ 下发布 PolygonStamped；仅当 visualize_ 为 true 时执行。
   */
  void publish() const;

protected:
  /**
   * @brief Supporting routine obtaining ROS-parameters common for all shapes
   * @param polygon_pub_topic Output name of polygon publishing topic
   * 中文：解析 stop／slowdown／approach 动作类型、enabled、max_points、减速比和可视化配置。
   * @return True if all parameters were obtained or false in failure case
   */
  bool getCommonParameters(std::string & polygon_pub_topic);

  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @param polygon_pub_topic Output name of polygon publishing topic
   * @param footprint_topic Output name of footprint topic. Empty, if no footprint subscription
   * @return True if all parameters were obtained or false in failure case
   * 中文：APPROACH 读取 Footprint Topic；其他动作读取 points 数组并构造静态多边形。
   */
  virtual bool getParameters(std::string & polygon_pub_topic, std::string & footprint_topic);

  /**
   * @brief Checks if point is inside polygon
   * @param point Given point to check
   * @return True if given point is inside polygon, otherwise false
   * 中文：使用水平射线与边界交点奇偶规则判断点内外，边界处理遵循 Shimrat Algorithm 112。
   */

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  bool isPointInside(const Point & point) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};
  /// @brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Basic parameters
  /// @brief Name of polygon
  std::string polygon_name_;
  /// @brief Action type for the polygon
  ActionType action_type_;
  /// @brief Maximum number of data readings within a zone to not trigger the action
  int max_points_;
  /// @brief Robot slowdown (share of its actual speed)
  double slowdown_ratio_;
  /// @brief Time before collision in seconds
  double time_before_collision_;
  /// @brief Time step for robot movement simulation
  double simulation_time_step_;
  /// @brief Footprint subscriber
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  /// @brief Whether polygon is enabled
  // 中文：可通过 `<polygon_name>.enabled` 动态切换，禁用后主处理循环跳过该区域。
  bool enabled_;

  // Global variables
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Base frame ID
  std::string base_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;

  // Visualization
  /// @brief Whether to publish the polygon
  bool visualize_;
  /// @brief Polygon points stored for later publishing
  geometry_msgs::msg::Polygon polygon_;
  /// @brief Polygon publisher for visualization purposes
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;

  /// @brief Polygon points (vertices)
  std::vector<Point> poly_;
};  // class Polygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_HPP_
