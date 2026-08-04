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
#include <cmath>
#include <chrono>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/kinematics.hpp"

using namespace std::chrono_literals;

// 中文：本文件验证两个无 ROS 业务状态的二维运动工具：坐标系逆变换和离散位姿投影。
// 中文：测试使用几何解析值与 EXPECT_NEAR 比较，重点覆盖旋转、平移、角速度和线速度同步更新。
static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

class RclCppFixture
{
public:
  // 中文：为使用 rclcpp::Time 和基础 ROS 类型建立一次全局初始化环境。
  RclCppFixture() {rclcpp::init(0, nullptr);}
  // 中文：测试结束后关闭全局 ROS 上下文，避免影响其他 GTest。
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(KinematicsTest, testTransformPoints) {
  // 中文：验证先平移再逆旋转的点坐标变换，覆盖位于原点前后两侧的两个点。
  // Transform: move frame to (2.0, 1.0) coordinate and rotate it on 30 degrees
  const nav2_collision_monitor::Pose tf{2.0, 1.0, M_PI / 6.0};
  // Add two points in the basic frame
  std::vector<nav2_collision_monitor::Point> points;
  points.push_back({3.0, 2.0});
  points.push_back({0.0, 0.0});

  // Transform points from basic frame to the new frame
  nav2_collision_monitor::transformPoints(tf, points);

  // Check that all points were transformed correctly
  // Distance to point in a new frame
  double new_point_distance = std::sqrt(1.0 + 1.0);
  // Angle of point in a new frame. Calculated as:
  // angle of point in a moved frame - frame rotation.
  double new_point_angle = M_PI / 4.0 - M_PI / 6.0;
  EXPECT_NEAR(points[0].x, new_point_distance * std::cos(new_point_angle), EPSILON);
  EXPECT_NEAR(points[0].y, new_point_distance * std::sin(new_point_angle), EPSILON);

  new_point_distance = std::sqrt(1.0 + 4.0);
  new_point_angle = M_PI + std::atan(1.0 / 2.0) - M_PI / 6.0;
  EXPECT_NEAR(points[1].x, new_point_distance * std::cos(new_point_angle), EPSILON);
  EXPECT_NEAR(points[1].y, new_point_distance * std::sin(new_point_angle), EPSILON);
}

TEST(KinematicsTest, testProjectState) {
  // 中文：验证单位时间内的位置平移、朝向旋转以及线速度随角度同步旋转。
  //     Y                                Y
  //     ^                                ^
  //     '                                '
  //     '                        ==>     '   *
  //     '     * <- robot's nose       2.0'   o <- moved robot
  //  1.0'   o <- robot's back            '
  //     ..........>X                     ..........>X
  //         2.0                              2.0

  // Initial pose of robot
  nav2_collision_monitor::Pose pose{2.0, 1.0, M_PI / 4.0};
  // Initial velocity of robot
  nav2_collision_monitor::Velocity vel{0.0, 1.0, M_PI / 4.0};
  const double dt = 1.0;

  // Moving robot and rotating velocity
  nav2_collision_monitor::projectState(dt, pose, vel);

  // Check pose of moved and rotated robot
  EXPECT_NEAR(pose.x, 2.0, EPSILON);
  EXPECT_NEAR(pose.y, 2.0, EPSILON);
  EXPECT_NEAR(pose.theta, M_PI / 2, EPSILON);

  // Check rotated velocity
  // Rotated velocity angle is an initial velocity angle + rotation
  const double rotated_vel_angle = M_PI / 2.0 + M_PI / 4.0;
  EXPECT_NEAR(vel.x, std::cos(rotated_vel_angle), EPSILON);
  EXPECT_NEAR(vel.y, std::sin(rotated_vel_angle), EPSILON);
  EXPECT_NEAR(vel.tw, M_PI / 4.0, EPSILON);  // should be the same
}
