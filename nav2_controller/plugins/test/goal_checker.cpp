/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"

using nav2_controller::SimpleGoalChecker;
using nav2_controller::StoppedGoalChecker;

// 中文说明：该测试文件对比 SimpleGoalChecker 与 StoppedGoalChecker 的公共行为和差异。
// 两者都必须满足位置与朝向容差；只有 StoppedGoalChecker 额外检查平面线速度和 Z 轴角速度。
// 测试还覆盖 reset()、容差查询、动态参数更新、容差边界以及对称朝向模式。

// 中文说明：统一执行一个 GoalChecker 场景。
// pose0 表示当前位姿，pose1 表示目标位姿，xv／yv／thetav 表示当前速度；
// 每次先 reset() 清除 stateful 阶段状态，再把二维数据转换为 GoalChecker 使用的三维消息并断言结果。
void checkMacro(nav2_core::GoalChecker & gc, double x0, double y0, double theta0, double x1, double y1, double theta1, double xv, double yv, double thetav, bool expected_result) {
  gc.reset();
  geometry_msgs::msg::Pose2D pose0, pose1;
  pose0.x = x0;
  pose0.y = y0;
  pose0.theta = theta0;
  pose1.x = x1;
  pose1.y = y1;
  pose1.theta = theta1;
  nav_2d_msgs::msg::Twist2D v;
  v.x = xv;
  v.y = yv;
  v.theta = thetav;
  if (expected_result) {
    EXPECT_TRUE(gc.isGoalReached(nav_2d_utils::pose2DToPose(pose0), nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v)));
  } else {
    EXPECT_FALSE(gc.isGoalReached(nav_2d_utils::pose2DToPose(pose0), nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v)));
  }
}

// 中文说明：要求两个 GoalChecker 在同一输入下返回相同结果。
// 该辅助函数用于证明不涉及速度超限时，StoppedGoalChecker 与父类 SimpleGoalChecker 的几何语义一致。
void sameResult(nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1, double x0, double y0, double theta0, double x1, double y1, double theta1, double xv, double yv, double thetav, bool expected_result) {
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
}

// 中文说明：验证两个插件在速度约束上的预期差异：SimpleGoalChecker 返回 true，
// StoppedGoalChecker 因速度未停稳返回 false。参数顺序与 checkMacro 保持一致。
void trueFalse(nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1, double x0, double y0, double theta0, double x1, double y1, double theta1, double xv, double yv, double thetav) {
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, true);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, false);
}

// 中文说明：测试专用 LifecycleNode 提供插件初始化与动态参数服务所需接口。
// 生命周期回调全部直接成功，本文件不测试 Controller Server 的生命周期资源管理。
class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name) : nav2_util::LifecycleNode(name) {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &) {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

// 中文说明：验证 SimpleGoalChecker 能通过 nav2_core::GoalChecker 基类安全 reset 和析构。
TEST(VelocityIterator, goal_checker_reset) {
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  nav2_core::GoalChecker * gc = new SimpleGoalChecker;
  gc->reset();
  delete gc;
  EXPECT_TRUE(true);
}

// 中文说明：验证 StoppedGoalChecker 继承的 reset() 能通过基类接口安全调用和析构。
TEST(VelocityIterator, stopped_goal_checker_reset) {
  auto x = std::make_shared<TestLifecycleNode>("stopped_goal_checker");

  nav2_core::GoalChecker * sgc = new StoppedGoalChecker;
  sgc->reset();
  delete sgc;
  EXPECT_TRUE(true);
}

// 中文说明：对比两个插件的共同几何条件和停稳速度差异。
// 零速度时二者对位置、朝向和 ±π 环绕给出相同结果；速度非零时只允许 SimpleGoalChecker 成功。
TEST(VelocityIterator, two_checks) {
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  gc.initialize(x, "nav2_controller", costmap);
  sgc.initialize(x, "nav2_controller", costmap);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, true);
  sameResult(gc, sgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, true);
  trueFalse(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 1, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 1);
}

// 中文说明：验证 StoppedGoalChecker 的容差输出和两个插件的动态参数回调。
// 参数通过 AsyncParametersClient 原子设置，既检查节点参数值，也检查插件 getTolerances() 的实际输出。
TEST(StoppedGoalChecker, get_tol_and_dynamic_params) {
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  gc.initialize(x, "test2", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test stopped goal checker's tolerance API
  // 中文：默认停稳阈值应通过速度容差接口返回为 0.25。
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 0.25);
  EXPECT_EQ(vel_tol.linear.y, 0.25);
  EXPECT_EQ(vel_tol.angular.z, 0.25);

  // Test Stopped goal checker's dynamic parameters
  // 中文：将平移和旋转停稳阈值同时改为 100，验证本类动态参数回调处理批量原子更新。
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(x->get_node_base_interface(), x->get_node_topics_interface(), x->get_node_graph_interface(), x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically({rclcpp::Parameter("test.rot_stopped_velocity", 100.0), rclcpp::Parameter("test.trans_stopped_velocity", 100.0)});

  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  EXPECT_EQ(x->get_parameter("test.rot_stopped_velocity").as_double(), 100.0);
  EXPECT_EQ(x->get_parameter("test.trans_stopped_velocity").as_double(), 100.0);

  // Test normal goal checker's dynamic parameters
  // 中文：更新普通 GoalChecker 的 XY、Yaw、stateful 和对称朝向参数，覆盖 double 与 bool 两种类型分支。
  results = rec_param->set_parameters_atomically({rclcpp::Parameter("test2.xy_goal_tolerance", 200.0), rclcpp::Parameter("test2.yaw_goal_tolerance", 200.0), rclcpp::Parameter("test2.stateful", true), rclcpp::Parameter("test2.symmetric_yaw_tolerance", true)});

  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  EXPECT_EQ(x->get_parameter("test2.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.yaw_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.stateful").as_bool(), true);
  EXPECT_EQ(x->get_parameter("test2.symmetric_yaw_tolerance").as_bool(), true);

  // Test the dynamic parameters impacted the tolerances
  // 中文：再次调用容差接口，确认更新不仅写入节点参数，也已经改变插件内部实际阈值。
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 100.0);
  EXPECT_EQ(vel_tol.linear.y, 100.0);
  EXPECT_EQ(vel_tol.angular.z, 100.0);

  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);
}

// 中文说明：验证目标容差和停稳速度容差在等于阈值、略超阈值及二维合成速度下的边界行为。
// 同时验证普通 GoalChecker 忽略速度，而 StoppedGoalChecker 对线速度和角速度都实施额外约束。
TEST(StoppedGoalChecker, is_reached) {
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  gc.initialize(x, "test2", costmap);
  geometry_msgs::msg::Pose goal_pose;
  geometry_msgs::msg::Twist velocity;
  geometry_msgs::msg::Pose current_pose;

  // Current linear x position is tolerance away from goal
  // 中文：X 位置误差和 X 线速度都恰好等于 0.25，使用 <= 比较，因此两个插件都应成功。
  current_pose.position.x = 0.25;
  velocity.linear.x = 0.25;
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  // Current linear x speed exceeds tolerance
  // 中文：只把 X 速度增加一个 epsilon；几何仍合格，普通插件成功，停稳插件失败。
  velocity.linear.x = 0.25 + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  // Current linear x position is further than tolerance away from goal
  // 中文：只把 X 位置误差增加一个 epsilon；位置条件失败，因此两个插件都失败。
  current_pose.position.x = 0.25 + std::numeric_limits<double>::epsilon();
  velocity.linear.x = 0.25;
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();
  current_pose.position.x = 0.0;
  velocity.linear.x = 0.0;

  // Current linear position is tolerance away from goal
  // 中文：X、Y 各取 0.25／sqrt(2)，二维位置和速度模长都恰好为 0.25，两个插件都应成功。
  current_pose.position.x = 0.25 / std::sqrt(2);
  current_pose.position.y = 0.25 / std::sqrt(2);
  velocity.linear.x = 0.25 / std::sqrt(2);
  velocity.linear.y = 0.25 / std::sqrt(2);
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  // Current linear speed exceeds tolerance
  // 中文：两个速度分量各增加 epsilon，使平面速度模长略超阈值；只有停稳插件失败。
  velocity.linear.x = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  velocity.linear.y = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  // Current linear position is further than tolerance away from goal
  // 中文：两个位置分量各增加 epsilon，使二维位置误差略超阈值；两个插件都失败。
  current_pose.position.x = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  current_pose.position.y = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  velocity.linear.x = 0.25 / std::sqrt(2);
  velocity.linear.y = 0.25 / std::sqrt(2);
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  current_pose.position.x = 0.0;
  velocity.linear.x = 0.0;

  // Current angular speed exceeds tolerance
  // 中文：角速度略高于 0.25；普通插件忽略速度而成功，停稳插件因角速度超限失败。
  velocity.angular.z = 0.25 + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
  sgc.reset();
  gc.reset();

  // 中文：把当前朝向设为目标朝向加 π；默认非对称模式下，两个插件都因朝向误差失败。
  current_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.25 + M_PI);
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity));

  // 中文：运行期为两个插件开启 symmetric_yaw_tolerance，允许目标朝向加 π 作为等价朝向。
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(x->get_node_base_interface(), x->get_node_topics_interface(), x->get_node_graph_interface(), x->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically({rclcpp::Parameter("test2.symmetric_yaw_tolerance", true), rclcpp::Parameter("test.symmetric_yaw_tolerance", true)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  velocity.angular.z = 0.0;
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity));
}

// 中文说明：初始化 ROS 与 GoogleTest，执行本文件注册的全部目标检查器测试。
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
