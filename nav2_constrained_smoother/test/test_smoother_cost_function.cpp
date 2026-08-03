// Copyright (c) 2021 RoboTech Vision
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_constrained_smoother/smoother_cost_function.hpp"

// 中文：测试子类公开受保护的曲率残差接口，便于直接验证几何边界而不改变生产类可见性。
class TestableSmootherCostFunction : nav2_constrained_smoother::SmootherCostFunction
{
public:
  TestableSmootherCostFunction(const Eigen::Vector2d & original_pos, double next_to_last_length_ratio, bool reversing, const nav2_costmap_2d::Costmap2D * costmap, const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator, const nav2_constrained_smoother::SmootherParams & params, double costmap_weight) : SmootherCostFunction(original_pos, next_to_last_length_ratio, reversing, costmap, costmap_interpolator, params, costmap_weight) {
    // 中文：测试构造函数只转发生产代价函数所需的原始点、方向比例、Costmap 和参数。
  }

  inline double getCurvatureResidual(const double & weight, const Eigen::Vector2d & pt, const Eigen::Vector2d & pt_next, const Eigen::Vector2d & pt_prev) const {
    // 中文：计算单个点的曲率残差，供测试用例比较直线、无曲率上限等特殊输入。
    double r = 0.0;
    addCurvatureResidual<double>(weight, pt, pt_next, pt_prev, r);
    return r;
  }
};

class Test : public ::testing::Test
{
protected:
  void SetUp() {
    // 中文：本组轻量单元测试不需要创建 ROS 节点，保留空 SetUp 以匹配 GoogleTest fixture 结构。
  }
};

TEST_F(Test, testingCurvatureResidual) {
  // 中文：验证曲率残差在零权重、退化点和无最小转弯半径时不会产生非预期惩罚。
  nav2_costmap_2d::Costmap2D costmap;
  TestableSmootherCostFunction fn(Eigen::Vector2d(1.0, 0.0), 1.0, false, &costmap, std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(), nav2_constrained_smoother::SmootherParams(), 0.0);

  // test for edge values
  // 中文：两点重合形成退化几何，预期曲率残差为零。
  Eigen::Vector2d pt(1.0, 0.0);
  Eigen::Vector2d pt_other(0.0, 0.0);
  EXPECT_EQ(fn.getCurvatureResidual(0.0, pt, pt_other, pt_other), 0.0);

  nav2_constrained_smoother::SmootherParams params_no_min_turning_radius;
  // 中文：把最大曲率设为无穷，模拟关闭最小转弯半径约束的配置。
  params_no_min_turning_radius.max_curvature = 1.0f / 0.0;
  TestableSmootherCostFunction fn_no_min_turning_radius(Eigen::Vector2d(1.0, 0.0), 1.0, false, &costmap, std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(), params_no_min_turning_radius, 0.0);
  EXPECT_EQ(fn_no_min_turning_radius.getCurvatureResidual(1.0, pt, pt_other, pt_other), 0.0);
}

TEST_F(Test, testingUtils) {
  // 中文：验证 arcCenter() 和 tangentDir() 对直线、cusp 及退化方向的几何回退逻辑。
  Eigen::Vector2d pt(1.0, 0.0);
  Eigen::Vector2d pt_prev(0.0, 0.0);
  Eigen::Vector2d pt_next(0.0, 0.0);

  // test for intermediate values
  // 中文：该输入的首尾点重合，当前实现将其视为无法稳定求圆心的退化情况。
  auto center = nav2_constrained_smoother::arcCenter(pt_prev, pt, pt_next, false);
  // although in this situation the center would be at (0.5, 0.0),
  // cases where pt_prev == pt_next are very rare and thus unhandled
  // during the smoothing points will be separated (and thus made valid) by smoothness cost anyways
  EXPECT_EQ(center[0], std::numeric_limits<double>::infinity());
  EXPECT_EQ(center[1], std::numeric_limits<double>::infinity());

  auto tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, false).normalized();
  EXPECT_NEAR(tangent[0], 0, 1e-10);
  EXPECT_NEAR(std::abs(tangent[1]), 1, 1e-10);

  // no rotation when mid point is a cusp
  // 中文：cusp 会翻转后段运动方向，切向量应沿连续运动方向而不是产生额外旋转。
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);

  pt_prev[0] = -1.0;
  // rotation is mathematically invalid, picking direction of a shorter segment
  // 中文：圆弧计算退化时选择较短有效线段的法向方向，确保结果仍为单位轴向。
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);

  pt_prev[0] = 0.0;
  pt_next[0] = -1.0;
  // rotation is mathematically invalid, picking direction of a shorter segment
  // 中文：再次覆盖相反方向的退化输入，确认回退逻辑不依赖固定的线段朝向。
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  // 中文：虽然测试主体是几何函数，但 Costmap 和 Ceres 类型依赖 ROS 初始化环境。
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  // 中文：GoogleTest 完成后关闭 ROS，避免测试进程退出时遗留上下文资源。
  rclcpp::shutdown();

  return all_successful;
}
