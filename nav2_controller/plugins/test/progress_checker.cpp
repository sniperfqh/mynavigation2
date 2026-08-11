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
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"

using nav2_controller::SimpleProgressChecker;
using nav2_controller::PoseProgressChecker;

// 中文说明：该测试文件验证两种 ProgressChecker 的基准重置、时间窗口、平移阈值、
// 旋转阈值和动态参数行为。测试通过真实 ROS 时钟等待构造“允许时间以内／以外”场景，
// 重点确认 check() 何时继续返回 true、何时把机器人判定为没有进展。

// 中文说明：测试专用 LifecycleNode 只提供插件初始化和参数服务所需的节点接口。
// 所有生命周期回调直接返回 SUCCESS，因为本文件不测试节点资源的创建、激活或清理逻辑。
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

// 中文说明：统一执行一个进展检查场景。
// pc 是被测插件；pose0 是首次检查时建立的基准；pose1 是等待 delay 毫秒后的当前位姿；
// expected_result 是第二次 check() 的期望值。每个场景先 reset()，避免前一场景的基准污染结果。
void checkMacro(nav2_core::ProgressChecker & pc, double x0, double y0, double theta0, double x1, double y1, double theta1, int delay, bool expected_result) {
  pc.reset();
  geometry_msgs::msg::PoseStamped pose0, pose1;
  pose0.pose.position.x = x0;
  pose0.pose.position.y = y0;
  pose0.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta0);
  pose1.pose.position.x = x1;
  pose1.pose.position.y = y1;
  pose1.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta1);
  // 中文：第一次检查必须成功，并把 pose0 与当前时刻保存为新的基准。
  EXPECT_TRUE(pc.check(pose0));
  // 中文：真实等待用于让插件内部 clock_ 观察到指定的时间差。
  rclcpp::sleep_for(std::chrono::milliseconds(delay));
  if (expected_result) {
    EXPECT_TRUE(pc.check(pose1));
  } else {
    EXPECT_FALSE(pc.check(pose1));
  }
}

// 中文说明：验证 SimpleProgressChecker 可以通过 nav2_core::ProgressChecker 基类安全 reset 和析构。
// 该用例主要覆盖接口生命周期，不检查具体进展阈值。
TEST(SimpleProgressChecker, progress_checker_reset) {
  auto x = std::make_shared<TestLifecycleNode>("progress_checker");

  nav2_core::ProgressChecker * pc = new SimpleProgressChecker;
  pc->reset();
  delete pc;
  EXPECT_TRUE(true);
}

// 中文说明：验证 SimpleProgressChecker 的时间与平移组合逻辑，并验证动态修改允许时间。
// 时间窗口以内，无论位移是否达到阈值都应继续运行；时间窗口以外，只有足够平移才能刷新基准并返回 true。
TEST(SimpleProgressChecker, unit_tests) {
  auto x = std::make_shared<TestLifecycleNode>("progress_checker");

  SimpleProgressChecker pc;
  pc.initialize(x, "nav2_controller");

  double time_allowance = 0.5;
  // 中文：分别构造允许时间的一半和两倍，避开临界时刻调度抖动对结果的影响。
  int half_time_allowance_ms = static_cast<int>(time_allowance * 0.5 * 1000);
  int twice_time_allowance_ms = static_cast<int>(time_allowance * 2.0 * 1000);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(x->get_node_base_interface(), x->get_node_topics_interface(), x->get_node_graph_interface(), x->get_node_services_interface());

  // 中文：通过参数服务原子更新 movement_time_allowance，实际触发插件注册的动态参数回调。
  auto results = rec_param->set_parameters_atomically({rclcpp::Parameter("nav2_controller.movement_time_allowance", time_allowance)});

  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  EXPECT_EQ(x->get_parameter("nav2_controller.movement_time_allowance").as_double(), time_allowance);

  // BELOW time allowance (set to time_allowance)
  // 中文：以下场景都发生在允许时间以内，因此即使没有达到最小位移也不应判定卡滞。
  // no movement
  // 中文：完全静止，但尚未超时，期望 true。
  checkMacro(pc, 0, 0, 0, 0, 0, 0, half_time_allowance_ms, true);
  // translation below required_movement_radius (default 0.5)
  // 中文：X 或 Y 平移 0.25 m，小于默认 0.5 m，尚未超时仍期望 true。
  checkMacro(pc, 0, 0, 0, 0.25, 0, 0, half_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 0.25, 0, half_time_allowance_ms, true);
  // translation above required_movement_radius (default 0.5)
  // 中文：X 或 Y 平移 1 m，超过默认半径，会刷新基准并期望 true。
  checkMacro(pc, 0, 0, 0, 1, 0, 0, half_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 1, 0, half_time_allowance_ms, true);

  // ABOVE time allowance (set to time_allowance)
  // 中文：以下场景都发生在允许时间以外，必须达到最小平移距离才能继续返回 true。
  // no movement
  // 中文：完全静止且已经超时，期望 false。
  checkMacro(pc, 0, 0, 0, 0, 0, 0, twice_time_allowance_ms, false);
  // translation below required_movement_radius (default 0.5)
  // 中文：位移只有 0.25 m，未达到阈值且已经超时，期望 false。
  checkMacro(pc, 0, 0, 0, 0.25, 0, 0, twice_time_allowance_ms, false);
  checkMacro(pc, 0, 0, 0, 0, 0.25, 0, twice_time_allowance_ms, false);
  // translation above required_movement_radius (default 0.5)
  // 中文：虽然等待时间超限，但 1 m 平移构成新进展，因此期望 true。
  checkMacro(pc, 0, 0, 0, 1, 0, 0, twice_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 1, 0, twice_time_allowance_ms, true);
}

// 中文说明：验证 PoseProgressChecker 继承的 reset() 能通过基类接口安全调用和析构。
TEST(PoseProgressChecker, pose_progress_checker_reset) {
  auto x = std::make_shared<TestLifecycleNode>("pose_progress_checker");

  PoseProgressChecker * rpc = new PoseProgressChecker;
  rpc->reset();
  delete rpc;
  EXPECT_TRUE(true);
}

// 中文说明：验证 PoseProgressChecker 将“平移超过阈值”或“旋转超过阈值”都视为有效进展。
// 同时复用动态参数服务把 movement_time_allowance 改为 0.5 s，覆盖超时与未超时两组场景。
TEST(PoseProgressChecker, unit_tests) {
  auto x = std::make_shared<TestLifecycleNode>("pose_progress_checker");

  PoseProgressChecker rpc;
  rpc.initialize(x, "nav2_controller");

  double time_allowance = 0.5;
  // 中文：使用 0.25 s 与 1.0 s 形成清晰的时间窗口内外样本。
  int half_time_allowance_ms = static_cast<int>(time_allowance * 0.5 * 1000);
  int twice_time_allowance_ms = static_cast<int>(time_allowance * 2.0 * 1000);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(x->get_node_base_interface(), x->get_node_topics_interface(), x->get_node_graph_interface(), x->get_node_services_interface());

  // 中文：该参数由父类 SimpleProgressChecker 注册的动态回调消费。
  auto results = rec_param->set_parameters_atomically({rclcpp::Parameter("nav2_controller.movement_time_allowance", time_allowance)});

  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  EXPECT_EQ(x->get_parameter("nav2_controller.movement_time_allowance").as_double(), time_allowance);

  // BELOW time allowance (set to time_allowance)
  // 中文：允许时间以内，所有场景都应保持 true，不论是否达到平移或旋转阈值。
  // no movement
  // 中文：无平移、无旋转，尚未超时。
  checkMacro(rpc, 0, 0, 0, 0, 0, 0, half_time_allowance_ms, true);
  // translation below required_movement_radius (default 0.5)
  // 中文：0.25 m 平移低于默认阈值，尚未超时。
  checkMacro(rpc, 0, 0, 0, 0.25, 0, 0, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0.25, 0, half_time_allowance_ms, true);
  // rotation below required_movement_angle (default 0.5)
  // 中文：正负 0.25 rad 旋转低于默认阈值，尚未超时。
  checkMacro(rpc, 0, 0, 0, 0, 0, 0.25, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -0.25, half_time_allowance_ms, true);
  // translation above required_movement_radius (default 0.5)
  // 中文：1 m 平移超过阈值，构成有效进展。
  checkMacro(rpc, 0, 0, 0, 1, 0, 0, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 1, 0, half_time_allowance_ms, true);
  // rotation above required_movement_angle (default 0.5)
  // 中文：正负 1 rad 旋转超过阈值，构成有效进展。
  checkMacro(rpc, 0, 0, 0, 0, 0, 1, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -1, half_time_allowance_ms, true);

  // ABOVE time allowance (set to time_allowance)
  // 中文：允许时间以外，平移和旋转都不足时应返回 false，任一超过阈值则返回 true。
  // no movement
  // 中文：完全静止并超时，期望 false。
  checkMacro(rpc, 0, 0, 0, 0, 0, 0, twice_time_allowance_ms, false);
  // translation below required_movement_radius (default 0.5)
  // 中文：0.25 m 平移不足并超时，期望 false。
  checkMacro(rpc, 0, 0, 0, 0.25, 0, 0, twice_time_allowance_ms, false);
  checkMacro(rpc, 0, 0, 0, 0, 0.25, 0, twice_time_allowance_ms, false);
  // rotation below required_movement_angle (default 0.5)
  // 中文：正负 0.25 rad 旋转不足并超时，期望 false。
  checkMacro(rpc, 0, 0, 0, 0, 0, 0.25, twice_time_allowance_ms, false);
  checkMacro(rpc, 0, 0, 0, 0, 0, -0.25, twice_time_allowance_ms, false);
  // translation above required_movement_radius (default 0.5)
  // 中文：1 m 平移足够，即使等待时间超限也会刷新基准并返回 true。
  checkMacro(rpc, 0, 0, 0, 1, 0, 0, twice_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 1, 0, twice_time_allowance_ms, true);
  // rotation above required_movement_angle (default 0.5)
  // 中文：正负 1 rad 旋转足够，即使等待时间超限也返回 true。
  checkMacro(rpc, 0, 0, 0, 0, 0, 1, twice_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -1, twice_time_allowance_ms, true);
}

// 中文说明：初始化 ROS 与 GoogleTest，执行本文件注册的全部测试用例。
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
