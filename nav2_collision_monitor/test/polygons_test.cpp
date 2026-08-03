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

#include <gtest/gtest.h>

#include <math.h>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"

using namespace std::chrono_literals;

// 中文：本文件覆盖 Polygon 和 Circle 的参数装配、动态 Footprint、点内判断、碰撞时间预测及可视化。
// 中文：测试通过 Lifecycle 节点、TF Listener 和 transient-local Footprint Topic 模拟生产运行环境。
static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char BASE_FRAME_ID[]{"base_link"};
static const char FOOTPRINT_TOPIC[]{"footprint"};
static const char POLYGON_PUB_TOPIC[]{"polygon"};
static const char POLYGON_NAME[]{"TestPolygon"};
static const char CIRCLE_NAME[]{"TestCircle"};
static const std::vector<double> SQUARE_POLYGON {
  0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, 0.5};
static const std::vector<double> ARBITRARY_POLYGON {
  1.0, 1.0, 1.0, 0.0, 2.0, 0.0, 2.0, -1.0, -1.0, -1.0, -1.0, 1.0};
static const double CIRCLE_RADIUS{0.5};
static const int MAX_POINTS{1};
static const double SLOWDOWN_RATIO{0.7};
static const double TIME_BEFORE_COLLISION{1.0};
static const double SIMULATION_TIME_STEP{0.01};
static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.1)};

class TestNode : public nav2_util::LifecycleNode
{
public:
  // 中文：测试节点同时发布 Footprint，并订阅 Polygon 的可视化输出。
  TestNode()
  : nav2_util::LifecycleNode("test_node"), polygon_received_(nullptr)
  {
    polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      POLYGON_PUB_TOPIC, rclcpp::SystemDefaultsQoS(),
      std::bind(&TestNode::polygonCallback, this, std::placeholders::_1));
  }

  ~TestNode()
  {
    footprint_pub_.reset();
  }

  void publishFootprint()
  {
    // 中文：发布一个方形机器人轮廓，验证 APPROACH Polygon 能动态读取并替换自身顶点。
    footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      FOOTPRINT_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<geometry_msgs::msg::PolygonStamped> msg =
      std::make_unique<geometry_msgs::msg::PolygonStamped>();

    msg->header.frame_id = BASE_FRAME_ID;
    msg->header.stamp = this->now();

    geometry_msgs::msg::Point32 p;
    for (unsigned int i = 0; i < SQUARE_POLYGON.size(); i = i + 2) {
      p.x = SQUARE_POLYGON[i];
      p.y = SQUARE_POLYGON[i + 1];
      msg->polygon.points.push_back(p);
    }

    footprint_pub_->publish(std::move(msg));
  }

  void polygonCallback(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    // 中文：保存最新可视化消息，供测试检查 frame、顶点数量和坐标。
    polygon_received_ = msg;
  }

  geometry_msgs::msg::PolygonStamped::SharedPtr waitPolygonReceived(
    const std::chrono::nanoseconds & timeout)
  {
    // 中文：在有限时间内泵送回调，等待 Polygon Publisher 的 transient-local 消息。
    rclcpp::Time start_time = this->now();
    while (rclcpp::ok() && this->now() - start_time <= rclcpp::Duration(timeout)) {
      if (polygon_received_) {
        return polygon_received_;
      }
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
    return nullptr;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;

  geometry_msgs::msg::PolygonStamped::SharedPtr polygon_received_;
};  // TestNode

class PolygonWrapper : public nav2_collision_monitor::Polygon
{
public:
  PolygonWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance)
  : nav2_collision_monitor::Polygon(
      node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
  {
  }

  double getSimulationTimeStep() const
  {
    // 中文：暴露保护成员，验证 action_type=approach 的模拟步长参数。
    return simulation_time_step_;
  }

  double isVisualize() const
  {
    // 中文：暴露可视化开关，验证默认值和显式配置。
    return visualize_;
  }
};  // PolygonWrapper

class CircleWrapper : public nav2_collision_monitor::Circle
{
public:
  CircleWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance)
  : nav2_collision_monitor::Circle(
      node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
  {
  }

  double getRadius() const
  {
    // 中文：暴露圆半径，验证 Circle 专属参数读取。
    return radius_;
  }

  double getRadiusSquared() const
  {
    // 中文：暴露缓存的半径平方，验证运行时优化值与 radius 一致。
    return radius_squared_;
  }
};  // CircleWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

protected:
  // Working with parameters
  void setCommonParameters(const std::string & polygon_name, const std::string & action_type);
  void setPolygonParameters(const std::vector<double> & points);
  void setCircleParameters(const double radius);
  bool checkUndeclaredParameter(const std::string & polygon_name, const std::string & param);
  // Creating routines
  void createPolygon(const std::string & action_type);
  void createCircle(const std::string & action_type);

  // Wait until footprint will be received
  bool waitFootprint(
    const std::chrono::nanoseconds & timeout,
    std::vector<nav2_collision_monitor::Point> & footprint);

  std::shared_ptr<TestNode> test_node_;

  std::shared_ptr<PolygonWrapper> polygon_;
  std::shared_ptr<CircleWrapper> circle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};  // Tester

Tester::Tester()
{
  // 中文：建立测试节点和 TF Listener；Polygon 的 FootprintSubscriber 将复用该 TF 缓存。
  test_node_ = std::make_shared<TestNode>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  // 中文：先销毁形状对象，再释放节点和 TF，避免回调访问已销毁资源。
  polygon_.reset();
  circle_.reset();

  test_node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::setCommonParameters(const std::string & polygon_name, const std::string & action_type)
{
  // 中文：为每个测试形状写入共享行为参数，测试随后只覆盖其专属 points 或 radius。
  test_node_->declare_parameter(
    polygon_name + ".action_type", rclcpp::ParameterValue(action_type));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".action_type", action_type));

  test_node_->declare_parameter(
    polygon_name + ".max_points", rclcpp::ParameterValue(MAX_POINTS));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".max_points", MAX_POINTS));

  test_node_->declare_parameter(
    polygon_name + ".slowdown_ratio", rclcpp::ParameterValue(SLOWDOWN_RATIO));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".slowdown_ratio", SLOWDOWN_RATIO));

  test_node_->declare_parameter(
    polygon_name + ".time_before_collision",
    rclcpp::ParameterValue(TIME_BEFORE_COLLISION));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".time_before_collision", TIME_BEFORE_COLLISION));

  test_node_->declare_parameter(
    polygon_name + ".simulation_time_step", rclcpp::ParameterValue(SIMULATION_TIME_STEP));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".simulation_time_step", SIMULATION_TIME_STEP));

  test_node_->declare_parameter(
    polygon_name + ".visualize", rclcpp::ParameterValue(true));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".visualize", true));

  test_node_->declare_parameter(
    polygon_name + ".polygon_pub_topic", rclcpp::ParameterValue(POLYGON_PUB_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".polygon_pub_topic", POLYGON_PUB_TOPIC));
}

void Tester::setPolygonParameters(const std::vector<double> & points)
{
  // 中文：配置 Footprint Topic 和静态 points；APPROACH 分支只使用前者，其他动作使用后者。
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".footprint_topic", rclcpp::ParameterValue(FOOTPRINT_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".footprint_topic", FOOTPRINT_TOPIC));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".points", rclcpp::ParameterValue(points));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".points", points));
}

void Tester::setCircleParameters(const double radius)
{
  // 中文：写入 Circle 的 radius 参数，验证基类公共参数和子类参数分离。
  test_node_->declare_parameter(
    std::string(CIRCLE_NAME) + ".radius", rclcpp::ParameterValue(radius));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".radius", radius));
}

bool Tester::checkUndeclaredParameter(const std::string & polygon_name, const std::string & param)
{
  // 中文：确认不适用的参数没有被错误声明，例如 Circle 不应创建 footprint_topic。
  bool ret = false;

  // Check that parameter is not set after configuring
  try {
    test_node_->get_parameter(polygon_name + "." + param);
  } catch (std::exception & ex) {
    std::string message = ex.what();
    if (message.find("." + param) != std::string::npos &&
      message.find("is not initialized") != std::string::npos)
    {
      ret = true;
    }
  }
  return ret;
}

void Tester::createPolygon(const std::string & action_type)
{
  // 中文：创建并激活普通 Polygon，后续测试直接调用其几何和碰撞接口。
  setCommonParameters(POLYGON_NAME, action_type);
  setPolygonParameters(SQUARE_POLYGON);

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(polygon_->configure());
  polygon_->activate();
}

void Tester::createCircle(const std::string & action_type)
{
  // 中文：创建并激活 Circle，验证圆形区域与 Polygon 公共生命周期一致。
  setCommonParameters(CIRCLE_NAME, action_type);
  setCircleParameters(CIRCLE_RADIUS);

  circle_ = std::make_shared<CircleWrapper>(
    test_node_, CIRCLE_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(circle_->configure());
  circle_->activate();
}

bool Tester::waitFootprint(
  const std::chrono::nanoseconds & timeout,
  std::vector<nav2_collision_monitor::Point> & footprint)
{
  // 中文：轮询动态 Footprint 更新，直到 Polygon 顶点非空或超时。
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    polygon_->updatePolygon();
    polygon_->getPolygon(footprint);
    if (footprint.size() > 0) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

TEST_F(Tester, testPolygonGetStopParameters)
{
  // 中文：验证 STOP Polygon 的名称、动作类型、阈值、静态顶点和可视化开关。
  createPolygon("stop");

  // Check that common parameters set correctly
  EXPECT_EQ(polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(polygon_->getActionType(), nav2_collision_monitor::STOP);
  EXPECT_EQ(polygon_->getMaxPoints(), MAX_POINTS);
  EXPECT_EQ(polygon_->isVisualize(), true);

  // Check that polygon set correctly
  std::vector<nav2_collision_monitor::Point> poly;
  polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, SQUARE_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, SQUARE_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, SQUARE_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, SQUARE_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, SQUARE_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, SQUARE_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, SQUARE_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, SQUARE_POLYGON[7], EPSILON);
}

TEST_F(Tester, testPolygonGetSlowdownParameters)
{
  // 中文：验证 SLOWDOWN Polygon 的减速比例以及公共区域参数。
  createPolygon("slowdown");

  // Check that common parameters set correctly
  EXPECT_EQ(polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(polygon_->getActionType(), nav2_collision_monitor::SLOWDOWN);
  EXPECT_EQ(polygon_->getMaxPoints(), MAX_POINTS);
  EXPECT_EQ(polygon_->isVisualize(), true);
  // Check that slowdown_ratio is correct
  EXPECT_NEAR(polygon_->getSlowdownRatio(), SLOWDOWN_RATIO, EPSILON);
}

TEST_F(Tester, testPolygonGetApproachParameters)
{
  // 中文：验证 APPROACH Polygon 读取 Footprint Topic、碰撞时间和模拟步长，并动态获得顶点。
  createPolygon("approach");

  // Check that common parameters set correctly
  EXPECT_EQ(polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(polygon_->getActionType(), nav2_collision_monitor::APPROACH);
  EXPECT_EQ(polygon_->getMaxPoints(), MAX_POINTS);
  EXPECT_EQ(polygon_->isVisualize(), true);
  // Check that time_before_collision and simulation_time_step are correct
  EXPECT_NEAR(polygon_->getTimeBeforeCollision(), TIME_BEFORE_COLLISION, EPSILON);
  EXPECT_NEAR(polygon_->getSimulationTimeStep(), SIMULATION_TIME_STEP, EPSILON);
}

TEST_F(Tester, testCircleGetParameters)
{
  // 中文：验证 Circle 半径及半径平方缓存，同时确认它不声明 Footprint 参数。
  createCircle("approach");

  // Check that common parameters set correctly
  EXPECT_EQ(circle_->getName(), CIRCLE_NAME);
  EXPECT_EQ(circle_->getActionType(), nav2_collision_monitor::APPROACH);
  EXPECT_EQ(circle_->getMaxPoints(), MAX_POINTS);

  // Check that Circle-specific parameters were set correctly
  EXPECT_NEAR(circle_->getRadius(), CIRCLE_RADIUS, EPSILON);
  EXPECT_NEAR(circle_->getRadiusSquared(), CIRCLE_RADIUS * CIRCLE_RADIUS, EPSILON);
}

TEST_F(Tester, testPolygonUndeclaredActionType)
{
  // 中文：缺少 action_type 时 configure 应失败，防止未初始化动作进入运行态。
  // "action_type" parameter is not initialized
  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(polygon_->configure());
  // Check that "action_type" parameter is not set after configuring
  ASSERT_TRUE(checkUndeclaredParameter(POLYGON_NAME, "action_type"));
}

TEST_F(Tester, testPolygonUndeclaredPoints)
{
  // 中文：静态 Polygon 缺少 points 时 configure 应失败。
  // "points" parameter is not initialized
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".action_type", rclcpp::ParameterValue("stop"));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".action_type", "stop"));
  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(polygon_->configure());
  // Check that "points" parameter is not set after configuring
  ASSERT_TRUE(checkUndeclaredParameter(POLYGON_NAME, "points"));
}

TEST_F(Tester, testPolygonIncorrectActionType)
{
  // 中文：未知 action_type 字符串必须被拒绝，而不是默认为某种安全动作。
  setCommonParameters(POLYGON_NAME, "incorrect_action_type");
  setPolygonParameters(SQUARE_POLYGON);

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(polygon_->configure());
}

TEST_F(Tester, testPolygonIncorrectPoints1)
{
  // 中文：奇数或过短的 points 数组必须被拒绝。
  setCommonParameters(POLYGON_NAME, "stop");

  std::vector<double> incorrect_points = SQUARE_POLYGON;
  incorrect_points.resize(6);  // Not enough for triangle
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".points", rclcpp::ParameterValue(incorrect_points));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".points", incorrect_points));

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(polygon_->configure());
}

TEST_F(Tester, testPolygonIncorrectPoints2)
{
  // 中文：再次覆盖非法顶点数量边界，确保参数格式校验不会误接受退化区域。
  setCommonParameters(POLYGON_NAME, "stop");

  std::vector<double> incorrect_points = SQUARE_POLYGON;
  incorrect_points.resize(9);  // Odd number of points
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".points", rclcpp::ParameterValue(incorrect_points));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".points", incorrect_points));

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(polygon_->configure());
}

TEST_F(Tester, testCircleUndeclaredRadius)
{
  // 中文：Circle 缺少 radius 时 configure 应失败。
  setCommonParameters(CIRCLE_NAME, "stop");

  circle_ = std::make_shared<CircleWrapper>(
    test_node_, CIRCLE_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_FALSE(circle_->configure());

  // Check that "radius" parameter is not set after configuring
  ASSERT_TRUE(checkUndeclaredParameter(CIRCLE_NAME, "radius"));
}

TEST_F(Tester, testPolygonUpdate)
{
  // 中文：验证 Footprint 消息到 Polygon 内部顶点和可视化点的动态同步。
  createPolygon("approach");

  std::vector<nav2_collision_monitor::Point> poly;
  polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);

  test_node_->publishFootprint();

  std::vector<nav2_collision_monitor::Point> footprint;
  ASSERT_TRUE(waitFootprint(500ms, footprint));

  ASSERT_EQ(footprint.size(), 4u);
  EXPECT_NEAR(footprint[0].x, SQUARE_POLYGON[0], EPSILON);
  EXPECT_NEAR(footprint[0].y, SQUARE_POLYGON[1], EPSILON);
  EXPECT_NEAR(footprint[1].x, SQUARE_POLYGON[2], EPSILON);
  EXPECT_NEAR(footprint[1].y, SQUARE_POLYGON[3], EPSILON);
  EXPECT_NEAR(footprint[2].x, SQUARE_POLYGON[4], EPSILON);
  EXPECT_NEAR(footprint[2].y, SQUARE_POLYGON[5], EPSILON);
  EXPECT_NEAR(footprint[3].x, SQUARE_POLYGON[6], EPSILON);
  EXPECT_NEAR(footprint[3].y, SQUARE_POLYGON[7], EPSILON);
}

TEST_F(Tester, testPolygonGetPointsInside)
{
  // 中文：验证普通多边形对内部、外部点的计数结果。
  createPolygon("stop");

  std::vector<nav2_collision_monitor::Point> points;

  // Out of boundaries points
  points.push_back({1.0, 0.0});
  points.push_back({0.0, 1.0});
  points.push_back({-1.0, 0.0});
  points.push_back({0.0, -1.0});
  ASSERT_EQ(polygon_->getPointsInside(points), 0);

  // Add one point inside
  points.push_back({-0.1, 0.3});
  ASSERT_EQ(polygon_->getPointsInside(points), 1);
}

TEST_F(Tester, testPolygonGetPointsInsideEdge)
{
  // 中文：验证点位于边界附近时的射线交叉算法行为，固定边界比较语义。
  // Test for checking edge cases in raytracing algorithm.
  // All points are lie on the edge lines parallel to OX, where the raytracing takes place.
  setCommonParameters(POLYGON_NAME, "stop");
  setPolygonParameters(ARBITRARY_POLYGON);

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(polygon_->configure());

  std::vector<nav2_collision_monitor::Point> points;

  // Out of boundaries points
  points.push_back({-2.0, -1.0});
  points.push_back({-2.0, 0.0});
  points.push_back({-2.0, 1.0});
  points.push_back({3.0, -1.0});
  points.push_back({3.0, 0.0});
  points.push_back({3.0, 1.0});
  ASSERT_EQ(polygon_->getPointsInside(points), 0);

  // Add one point inside
  points.push_back({0.0, 0.0});
  ASSERT_EQ(polygon_->getPointsInside(points), 1);
}

TEST_F(Tester, testCircleGetPointsInside)
{
  // 中文：验证 Circle 的平方距离判断和边界点计数。
  createCircle("stop");

  std::vector<nav2_collision_monitor::Point> points;
  // Point out of radius
  points.push_back({1.0, 0.0});
  ASSERT_EQ(circle_->getPointsInside(points), 0);

  // Add one point inside
  points.push_back({-0.1, 0.3});
  ASSERT_EQ(circle_->getPointsInside(points), 1);
}

TEST_F(Tester, testPolygonGetCollisionTime)
{
  // 中文：验证 APPROACH 预测中的立即碰撞、运动中碰撞和无碰撞三类结果。
  createPolygon("approach");

  // Set footprint for Polygon
  test_node_->publishFootprint();
  std::vector<nav2_collision_monitor::Point> footprint;
  ASSERT_TRUE(waitFootprint(500ms, footprint));
  ASSERT_EQ(footprint.size(), 4u);

  // Forward movement check
  nav2_collision_monitor::Velocity vel{0.5, 0.0, 0.0};  // 0.5 m/s forward movement
  // Two points 0.2 m ahead the footprint (0.5 m)
  std::vector<nav2_collision_monitor::Point> points{{0.7, -0.01}, {0.7, 0.01}};
  // Collision is expected to be ~= 0.2 m / 0.5 m/s seconds
  EXPECT_NEAR(polygon_->getCollisionTime(points, vel), 0.4, SIMULATION_TIME_STEP);

  // Backward movement check
  vel = {-0.5, 0.0, 0.0};  // 0.5 m/s backward movement
  // Two points 0.2 m behind the footprint (0.5 m)
  points.clear();
  points = {{-0.7, -0.01}, {-0.7, 0.01}};
  // Collision is expected to be in ~= 0.2 m / 0.5 m/s seconds
  EXPECT_NEAR(polygon_->getCollisionTime(points, vel), 0.4, SIMULATION_TIME_STEP);

  // Sideway movement check
  vel = {0.0, 0.5, 0.0};  // 0.5 m/s sideway movement
  // Two points 0.1 m ahead the footprint (0.5 m)
  points.clear();
  points = {{-0.01, 0.6}, {0.01, 0.6}};
  // Collision is expected to be in ~= 0.1 m / 0.5 m/s seconds
  EXPECT_NEAR(polygon_->getCollisionTime(points, vel), 0.2, SIMULATION_TIME_STEP);

  // Rotation check
  vel = {0.0, 0.0, 1.0};  // 1.0 rad/s rotation
  //          ^ OX
  //          '
  //         x'x <- 2 collision points
  //          '
  //     ----------- <- robot footprint
  // OY  |    '    |
  // <...|....o....|...
  //     |    '    |
  //     -----------
  //          '
  points.clear();
  points = {{0.49, -0.01}, {0.49, 0.01}};
  // Collision is expected to be in ~= 45 degrees * M_PI / (180 degrees * 1.0 rad/s) seconds
  double exp_res = 45 / 180 * M_PI;
  EXPECT_NEAR(polygon_->getCollisionTime(points, vel), exp_res, EPSILON);

  // Two points are already inside footprint
  vel = {0.5, 0.0, 0.0};  // 0.5 m/s forward movement
  // Two points inside
  points.clear();
  points = {{0.1, -0.01}, {0.1, 0.01}};
  // Collision already appeared: collision time should be 0
  EXPECT_NEAR(polygon_->getCollisionTime(points, vel), 0.0, EPSILON);

  // All points are out of simulation prediction
  vel = {0.5, 0.0, 0.0};  // 0.5 m/s forward movement
  // Two points 0.6 m ahead the footprint (0.5 m)
  points.clear();
  points = {{1.1, -0.01}, {1.1, 0.01}};
  // There is no collision: return value should be negative
  EXPECT_LT(polygon_->getCollisionTime(points, vel), 0.0);
}

TEST_F(Tester, testPolygonPublish)
{
  // 中文：验证可视化消息的 frame、顶点数量和坐标已经从内部 Polygon 正确发布。
  createPolygon("stop");
  polygon_->publish();
  geometry_msgs::msg::PolygonStamped::SharedPtr polygon_received =
    test_node_->waitPolygonReceived(500ms);

  ASSERT_NE(polygon_received, nullptr);
  ASSERT_EQ(polygon_received->polygon.points.size(), 4u);
  EXPECT_NEAR(polygon_received->polygon.points[0].x, SQUARE_POLYGON[0], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[0].y, SQUARE_POLYGON[1], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[1].x, SQUARE_POLYGON[2], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[1].y, SQUARE_POLYGON[3], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[2].x, SQUARE_POLYGON[4], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[2].y, SQUARE_POLYGON[5], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[3].x, SQUARE_POLYGON[6], EPSILON);
  EXPECT_NEAR(polygon_received->polygon.points[3].y, SQUARE_POLYGON[7], EPSILON);

  polygon_->deactivate();
}

TEST_F(Tester, testPolygonDefaultVisualize)
{
  // 中文：验证未显式配置 visualize 时默认关闭可视化，避免不必要的 Topic。
  // Use default parameters, visualize should be false by-default
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".action_type", rclcpp::ParameterValue("stop"));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".action_type", "stop"));
  setPolygonParameters(SQUARE_POLYGON);

  // Create new polygon
  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(polygon_->configure());
  polygon_->activate();

  // Try to publish polygon
  polygon_->publish();

  // Wait for polygon: it should not be published
  ASSERT_EQ(test_node_->waitPolygonReceived(100ms), nullptr);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
