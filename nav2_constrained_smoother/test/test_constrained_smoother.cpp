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
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "angles/angles.h"

#include "nav2_constrained_smoother/constrained_smoother.hpp"

#include "geometry_msgs/msg/pose_array.hpp"

// 中文：用内存中的 nav2_msgs/Costmap 模拟真实 Costmap 订阅器，为平滑测试提供可重复障碍物场景。
class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(nav2_util::LifecycleNode::SharedPtr node, const std::string & topic_name) : CostmapSubscriber(node, topic_name) {
    // 中文：创建 100×100、0.1 米分辨率的地图，世界坐标原点位于 (-5, -5)。
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);

    // create an obstacle in rect (1.0, -1.0) -> (3.0, -2.0)
    // with inflation of radius 2.0
    // 中文：在指定矩形内写入致命障碍和指数衰减的膨胀代价，模拟 Costmap Inflation Layer 输出。
    double cost_scaling_factor = 1.6;
    double inscribed_radius = 0.3;
    for (int i = 10; i < 60; ++i) {
      for (int j = 40; j < 100; ++j) {
        int dist_x = std::max(0, std::max(60 - j, j - 80));
        int dist_y = std::max(0, std::max(30 - i, i - 40));
        double dist = sqrt(dist_x * dist_x + dist_y * dist_y) * costmap->metadata.resolution;
        unsigned char cost;
        if (dist == 0) {
          // 中文：障碍物内部使用 LETHAL_OBSTACLE，表示机器人足迹不可进入。
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else if (dist < inscribed_radius) {
          // 中文：内切半径内使用 INSCRIBED_INFLATED_OBSTACLE，表示足迹边界已经接近障碍。
          cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        } else {
          // 中文：外部膨胀区按照距离指数衰减，形成平滑的避障代价梯度。
          double factor = exp(-1.0 * cost_scaling_factor * (dist - inscribed_radius));
          cost = static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        costmap->data[i * costmap->metadata.size_x + j] = cost;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg) {
    // 中文：直接注入测试地图并标记已收到，使父类 getCostmap() 可以立即返回该快照。
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

geometry_msgs::msg::Point pointMsg(double x, double y) {
  // 中文：把二维坐标转换为 Costmap 足迹使用的 geometry_msgs::msg::Point。
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

class SmootherTest : public ::testing::Test
{
protected:
  // 中文：构造时主动执行 SetUp，保证每个测试在创建插件前拥有独立的生命周期节点和 Costmap。
  SmootherTest() {SetUp();}
  // 中文：资源释放由 TearDown 和成员智能指针共同完成。
  ~SmootherTest() {}

  void SetUp() override {
    // 中文：创建 LifecycleNode，为插件参数声明、日志、时钟和生命周期发布器提供运行环境。
    node_lifecycle_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ConstrainedSmootherTestNode", rclcpp::NodeOptions());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    // 中文：绑定 TF 定时器接口，满足 Costmap 和插件构造阶段对 TF 缓存计时器的依赖。
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(node_lifecycle_->get_node_base_interface(), node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    costmap_sub_ = std::make_shared<DummyCostmapSubscriber>(node_lifecycle_, "costmap_topic");
    // 中文：每个测试使用独立的内存 Costmap，避免依赖外部 Topic 或仿真环境。

    path_poses_pub_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>("/plan_poses_optimized", 100);
    path_poses_pub_cmp_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>("/plan_poses_optimized_cmp", 100);
    path_poses_pub_orig_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>("/plan_poses_original", 100);
    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(node_lifecycle_, costmap_sub_->getCostmap().get(), "map", "/costmap", true);
    // 中文：发布 Costmap 和可视化 PoseArray 只用于测试观察，不参与平滑器核心计算。

    node_lifecycle_->configure();
    node_lifecycle_->activate();
    // 中文：先让 LifecycleNode 进入激活态，再激活测试发布器和待测插件。
    path_poses_pub_->on_activate();
    path_poses_pub_cmp_->on_activate();
    path_poses_pub_orig_->on_activate();
    costmap_pub_->on_activate();


    smoother_ = std::make_shared<nav2_constrained_smoother::ConstrainedSmoother>();

    // 中文：以插件名 SmoothPath 配置待测对象，后续所有参数均使用该前缀。
    smoother_->configure(node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_, std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();

    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 2000000.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.minimum_turning_radius", 0.4));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 30.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 0.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", -1.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 1.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_downsampling_factor", 1));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 1));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", true));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_start_orientation", true));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_goal_orientation", true));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.linear_solver_type", "SPARSE_NORMAL_CHOLESKY"));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>()));
    // 中文：初始化默认优化器、曲率、Costmap 和采样参数，保证各测试从明确基线开始。
    reloadParams();
  }

  void TearDown() override {
    // 中文：按生命周期顺序停用插件、发布器和节点，避免测试进程退出时访问已释放资源。
    smoother_->deactivate();
    smoother_->cleanup();
    path_poses_pub_->on_deactivate();
    path_poses_pub_cmp_->on_deactivate();
    path_poses_pub_orig_->on_deactivate();
    costmap_pub_->on_deactivate();
    node_lifecycle_->deactivate();
  }

  void reloadParams() {
    // 中文：参数修改后重新执行 cleanup→configure→activate，使新参数进入插件内部缓存。
    smoother_->deactivate();
    smoother_->cleanup();
    smoother_->configure(node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_, std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();
  }

  bool smoothPath(const std::vector<Eigen::Vector3d> & input, std::vector<Eigen::Vector3d> & output, bool publish = false, bool cmp = false) {
    // 中文：把测试用的 x、y、yaw 向量转换成 nav_msgs/Path，调用插件后再转换回便于断言的向量。
    nav_msgs::msg::Path path;
    path.poses.reserve(input.size());
    for (auto & xya : input) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = xya.x();
      pose.pose.position.y = xya.y();
      pose.pose.position.z = 0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(xya.z());
      path.poses.push_back(pose);
    }

    if (publish && !path.poses.empty()) {
      // 中文：可选发布原始路径和 Costmap，便于调试障碍物避让结果。
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = "map";
      poses.header.stamp = node_lifecycle_->get_clock()->now();
      for (auto & p : path.poses) {
        poses.poses.push_back(p.pose);
      }
      path_poses_pub_orig_->publish(poses);
      costmap_pub_->publishCostmap();
    }

    bool result = smoother_->smooth(path, rclcpp::Duration::from_seconds(10.0));
    // 中文：为每次测试提供 10 秒求解预算，并保存插件返回的成功标志。

    if (publish && !path.poses.empty()) {
      // 中文：可选发布平滑结果；cmp 参数选择普通结果或对比专用 Topic。
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = "map";
      poses.header.stamp = node_lifecycle_->get_clock()->now();
      for (auto & p : path.poses) {
        poses.poses.push_back(p.pose);
      }
      auto & pub = cmp ? path_poses_pub_cmp_ : path_poses_pub_;
      pub->publish(poses);
    }

    output.clear();
    output.reserve(path.poses.size());
    for (auto & pose : path.poses) {
      // 中文：从四元数提取 yaw，形成测试质量指标使用的 x、y、yaw 三元组。
      Eigen::Vector3d xya;
      xya.x() = pose.pose.position.x;
      xya.y() = pose.pose.position.y;
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      xya.z() = q.getAngle();
      output.push_back(xya);
    }
    return result;
  }

  typedef std::function<double (int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p, const Eigen::Vector3d & next_p)> QualityCriterion3;
  // 中文：三点指标用于评估运动平滑性、局部曲率等依赖前后邻居的路径质量。
  typedef std::function<double (int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p)> QualityCriterion2;
  // 中文：两点指标用于评估相邻姿态差异或相邻线段长度等局部质量。
  typedef std::function<double (int i, const Eigen::Vector3d & p)> QualityCriterion1;
  // 中文：单点指标用于评估 Costmap 足迹代价、点到原路径距离等逐点质量。
  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   * 中文：对输入和输出路径计算三点指标的均方根，返回输出相对输入的改善百分比。
   */
  double assessPathImprovement(const std::vector<Eigen::Vector3d> & input, const std::vector<Eigen::Vector3d> & output, const QualityCriterion3 & criterion, const QualityCriterion3 * criterion_out = nullptr) {
    // 中文：未显式提供输出指标时复用输入指标，适合输入输出结构一致的平滑质量比较。
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size() - 1; i++) {
      double input_crit = criterion(i, input[i - 1], input[i], input[i + 1]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / (input.size() - 2));
    // 中文：三点指标跳过首尾点，因为它们缺少完整的前后邻居。

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size() - 1; i++) {
      double output_crit = (*criterion_out)(i, output[i - 1], output[i], output[i + 1]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / (input.size() - 2));
    // 中文：沿用输入路径内部点数量作为归一化基准，保持不同采样输出之间的可比性。

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   * 中文：对相邻点组成的两点指标计算均方根，适用于方向变化或段长连续性评价。
   */
  double assessPathImprovement(const std::vector<Eigen::Vector3d> & input, const std::vector<Eigen::Vector3d> & output, const QualityCriterion2 & criterion, const QualityCriterion2 * criterion_out = nullptr) {
    // 中文：允许为输出路径提供不同的索引解释，例如下采样路径与上采样路径比较时的 cusp 位置不同。
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size(); i++) {
      double input_crit = criterion(i, input[i - 1], input[i]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / (input.size() - 1));

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size(); i++) {
      double output_crit = (*criterion_out)(i, output[i - 1], output[i]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / (output.size() - 1));

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   * 中文：对每个点独立计算指标并用均方根汇总，适合逐点 Costmap 或原路径距离评价。
   */
  double assessPathImprovement(const std::vector<Eigen::Vector3d> & input, const std::vector<Eigen::Vector3d> & output, const QualityCriterion1 & criterion, const QualityCriterion1 * criterion_out = nullptr) {
    // 中文：输出指标可以独立传入，以处理输入输出路径点数不同的情况。
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 0; i < input.size(); i++) {
      double input_crit = criterion(i, input[i]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / input.size());

    double total_output_crit = 0.0;
    for (size_t i = 0; i < output.size(); i++) {
      double output_crit = (*criterion_out)(i, output[i]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / output.size());

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Worst pose improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = max(criterion(data[i]))
   * @return Percentage of improvement (relative to input path quality)
   * 中文：只比较输入和输出的最差点代价，用于验证 cusp 附近避障是否改善了峰值风险。
   */
  double assessWorstPoseImprovement(const std::vector<Eigen::Vector3d> & input, const std::vector<Eigen::Vector3d> & output, const QualityCriterion1 & criterion) {
    // 中文：分别扫描两条路径的最大指标值，再计算最坏点层面的改善百分比。
    double max_input_crit = 0.0;
    for (size_t i = 0; i < input.size(); i++) {
      double input_crit = criterion(i, input[i]);
      max_input_crit = std::max(max_input_crit, input_crit);
    }

    double max_output_crit = 0.0;
    for (size_t i = 0; i < output.size(); i++) {
      double output_crit = criterion(i, output[i]);
      max_output_crit = std::max(max_output_crit, output_crit);
    }

    return (1.0 - max_output_crit / max_input_crit) * 100;
  }

  std::vector<Eigen::Vector3d> zigZaggedPath(const std::vector<Eigen::Vector3d> & input, double offset) {
    // 中文：在路径法向两侧交替施加偏移，构造一个比原路径更不平滑的上界基准。
    auto output = input;
    for (size_t i = 1; i < input.size() - 1; i++) {
      // add offset prependicular to path
      // 中文：使用相邻点连线的法向量交替偏移，避免测试只对某一固定方向有效。
      Eigen::Vector2d direction = (input[i + 1].block<2, 1>(0, 0) - input[i - 1].block<2, 1>(0, 0)).normalized();
      output[i].block<2, 1>(0, 0) += Eigen::Vector2d(direction[1], -direction[0]) * offset * (i % 2 == 0 ? 1.0 : -1.0);
    }
    return output;
  }

  // 中文：测试所用生命周期节点，负责参数、时钟、日志和发布器状态。
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  // 中文：被测 ConstrainedSmoother 插件实例。
  std::shared_ptr<nav2_constrained_smoother::ConstrainedSmoother> smoother_;
  // 中文：传给插件的 TF 缓存，测试中主要用于满足接口契约。
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // 中文：提供人工 Costmap 快照的订阅器。
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  // 中文：保留足迹订阅器句柄，当前测试使用空指针以聚焦路径平滑。
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_poses_pub_orig_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_poses_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_poses_pub_cmp_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;

  // 中文：当前测试路径的 cusp 索引，运动平滑指标在该点翻转后段位移方向。
  int cusp_i_ = -1;
  // 中文：衡量相邻运动向量变化幅度，数值越小表示路径运动越连续。
  QualityCriterion3 mvmt_smoothness_criterion_ = [this](int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p, const Eigen::Vector3d & next_p) { Eigen::Vector2d prev_mvmt = p.block<2, 1>(0, 0) - prev_p.block<2, 1>(0, 0); Eigen::Vector2d next_mvmt = next_p.block<2, 1>(0, 0) - p.block<2, 1>(0, 0); if (i == cusp_i_) { next_mvmt = -next_mvmt; } return (next_mvmt - prev_mvmt).norm(); };
};

TEST_F(SmootherTest, testingSmoothness) {
  // 中文：验证 w_smooth 主导时，普通急转弯和带前进／倒退 cusp 的路径都能降低运动与姿态突变。
  // set w_curve to 0.0 so that the whole job is upon w_smooth
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, M_PI / 4}, {0.3, 0.1, M_PI / 2}, {0.3, 0.2, M_PI / 2}, {0.3, 0.3, M_PI / 2} };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  double mvmt_smoothness_improvement = assessPathImprovement(sharp_turn_90, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 55.3, 1.0);

  auto orientation_smoothness_criterion = [](int, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p) { return angles::normalize_angle(p.z() - prev_p.z()); };
  double orientation_smoothness_improvement = assessPathImprovement(sharp_turn_90, smoothed_path, orientation_smoothness_criterion);
  EXPECT_GT(orientation_smoothness_improvement, 0.0);
  EXPECT_NEAR(orientation_smoothness_improvement, 38.7, 1.0);

  // path with a cusp
  // 中文：第二段路径在索引 6 附近发生方向反转，用于验证 cusp 不会被普通平滑跨越。
  std::vector<Eigen::Vector3d> sharp_turn_90_then_reverse = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, 0}, {0.4, 0, 0}, {0.5, 0, 0}, {0.6, 0, M_PI / 4}, {0.6, -0.1, M_PI / 2}, {0.6, -0.2, M_PI / 2}, {0.6, -0.3, M_PI / 2}, {0.6, -0.4, M_PI / 2}, {0.6, -0.5, M_PI / 2}, {0.6, -0.6, M_PI / 2} };
  cusp_i_ = 6;

  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));

  mvmt_smoothness_improvement = assessPathImprovement(sharp_turn_90_then_reverse, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 37.2, 1.0);

  orientation_smoothness_improvement = assessPathImprovement(sharp_turn_90_then_reverse, smoothed_path, orientation_smoothness_criterion);
  EXPECT_GT(orientation_smoothness_improvement, 0.0);
  EXPECT_NEAR(orientation_smoothness_improvement, 28.5, 1.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingAnchoringToOriginalPath) {
  // 中文：先关闭距离约束得到偏移更大的平滑路径，再提高 w_dist 验证结果回归原始路径。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 30.0));
  // set w_curve to 0.0, we don't care about turning radius in this test...
  // 中文：本测试隔离 w_dist 的作用，不让曲率残差影响路径偏离比较。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  // first keep w_dist at 0.0 to generate an unanchored smoothed path
  // 中文：第一轮不约束原路径，作为后续锚定效果的对照组。
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, M_PI / 4}, {0.3, 0.1, M_PI / 2}, {0.3, 0.2, M_PI / 2}, {0.3, 0.3, M_PI / 2} };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  // then update w_dist and compare the results
  // 中文：第二轮提高原路径距离权重，期望输出更接近原始急转弯路径。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 30.0));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_anchored;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path_anchored));

  auto origin_similarity_criterion = [&sharp_turn_90](int i, const Eigen::Vector3d & p) { return (p.block<2, 1>(0, 0) - sharp_turn_90[i].block<2, 1>(0, 0)).norm(); };
  double origin_similarity_improvement = assessPathImprovement(smoothed_path, smoothed_path_anchored, origin_similarity_criterion);
  EXPECT_GT(origin_similarity_improvement, 0.0);
  EXPECT_NEAR(origin_similarity_improvement, 45.5, 1.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingMaxCurvature) {
  // 中文：验证 w_curve 能把半径 0.3 的不可行转弯推向配置要求的最小半径 0.4。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 30.0));
  // set w_smooth to a small value so that the whole job is upon w_curve
  // 中文：降低平滑权重，使测试主要观察最大曲率约束，而不是一般几何平滑效果。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 0.3));
  // let's give the smoother more time since w_smooth is so small
  // 中文：曲率约束主导时收敛较慢，因此提高最大迭代次数避免过早结束。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 500));
  reloadParams();

  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  // 中文：第一条圆弧半径小于限制，期望优化后的中间点落在半径约 0.4 的圆上。
  std::vector<Eigen::Vector3d> radius_0_3_turn_90 = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.2 + 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), 0}, {0.2 + 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), 0}, {0.2 + 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), 0}, {0.2 + 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), 0}, {0.2 + 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), 0}, {0.5, 0.3, M_PI / 2}, {0.5, 0.4, M_PI / 2}, {0.5, 0.5, M_PI / 2} };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(radius_0_3_turn_90, smoothed_path));

  // we don't expect result to be smoother than original as w_smooth is too low
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(radius_0_3_turn_90, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  // smoothed path points should form a circle with radius 0.4
  for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
    auto & p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(0.1, 0.4)).norm();
    EXPECT_NEAR(r, 0.4, 0.01);
  }

  // path with a cusp
  // 中文：第二个场景把两个转弯通过 cusp 连接，验证前进和倒退两侧都满足曲率半径。
  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  std::vector<Eigen::Vector3d> radius_0_3_turn_90_then_reverse_turn_90 = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.2 + 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), M_PI / 12}, {0.2 + 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), M_PI *2 / 12}, {0.2 + 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), M_PI *3 / 12}, {0.2 + 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), M_PI *4 / 12}, {0.2 + 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), M_PI *5 / 12}, {0.5, 0.3, M_PI / 2}, {0.8 - 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), M_PI *7 / 12}, {0.8 - 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), M_PI *8 / 12}, {0.8 - 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), M_PI *9 / 12}, {0.8 - 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), M_PI *10 / 12}, {0.8 - 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), M_PI *11 / 12}, {0.8, 0, M_PI}, {0.9, 0, M_PI}, {1.0, 0, M_PI} };

  EXPECT_TRUE(smoothPath(radius_0_3_turn_90_then_reverse_turn_90, smoothed_path));

  // we don't expect result to be smoother than original as w_smooth is too low
  // but let's check for large discontinuities using a well chosen upper bound
  cusp_i_ = 8;
  upper_bound = zigZaggedPath(radius_0_3_turn_90_then_reverse_turn_90, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  // smoothed path points should form a circle with radius 0.4
  // for both forward and reverse movements
  // 中文：根据 cusp 前后使用不同圆心检查半径，确保运动方向切换不会破坏约束。
  for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
    auto & p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(i <= 8 ? 0.1 : 0.9, 0.4)).norm();
    EXPECT_NEAR(r, 0.4, 0.01);
  }

  SUCCEED();
}

TEST_F(SmootherTest, testingObstacleAvoidance) {
  // 中文：验证普通 Costmap 权重能够让一条靠近障碍物的直线路径偏离高代价区域。
  auto costmap = costmap_sub_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker collision_checker(costmap);
  nav2_costmap_2d::Footprint footprint;

  auto cost_avoidance_criterion = [&collision_checker, &footprint](int, const Eigen::Vector3d & p) { return collision_checker.footprintCostAtPose(p[0], p[1], p[2], footprint); };

  // a symmetric footprint (diff-drive with 4 actuated wheels)
  // 中文：使用对称矩形足迹，Costmap 代价只需围绕机器人参考点均匀评估。
  footprint.push_back(pointMsg(0.4, 0.25));
  footprint.push_back(pointMsg(-0.4, 0.25));
  footprint.push_back(pointMsg(-0.4, -0.25));
  footprint.push_back(pointMsg(0.4, -0.25));

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 2000000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  reloadParams();

  std::vector<Eigen::Vector3d> straight_near_obstacle = {{0.05, 0.05, 0}, {0.45, 0.05, 0}, {0.85, 0.05, 0}, {1.25, 0.05, 0}, {1.65, 0.05, 0}, {2.05, 0.05, 0}, {2.45, 0.05, 0}, {2.85, 0.05, 0}, {3.25, 0.05, 0}, {3.65, 0.05, 0}, {4.05, 0.05, 0} };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(straight_near_obstacle, smoothed_path));

  // we don't expect result to be smoother than original as original straight line was 100% smooth
  // 中文：原始路径已经是直线，测试只用轻微锯齿路径作为平滑性回归上界。
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(straight_near_obstacle, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  double cost_avoidance_improvement = assessPathImprovement(straight_near_obstacle, smoothed_path, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement, 9.4, 1.0);
}

TEST_F(SmootherTest, testingObstacleAvoidanceNearCusps) {
  // 中文：验证 cusp 附近提高 Costmap 权重后，机器人在方向切换危险区获得更强避障效果。
  auto costmap = costmap_sub_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker collision_checker(costmap);
  nav2_costmap_2d::Footprint footprint;

  auto cost_avoidance_criterion = [&collision_checker, &footprint](int, const Eigen::Vector3d & p) { return collision_checker.footprintCostAtPose(p[0], p[1], p[2], footprint); };

  // path with a cusp
  // 中文：构造靠近障碍物的前进→旋转→倒退路径，并记录 cusp 索引供质量指标识别方向反转。
  std::vector<Eigen::Vector3d> cusp_near_obstacle = {{0.05, 0.05, 0}, {0.15, 0.05, 0}, {0.25, 0.05, 0}, {0.35, 0.05, 0}, {0.45, 0.05, 0}, {0.55, 0.05, 0}, {0.65, 0.05, 0}, {0.75, 0.05, 0}, {0.85, 0.05, 0}, {0.95, 0.05, 0}, {1.05, 0.05, 0}, {1.15, 0.05, 0}, {1.25, 0.05, 0}, {1.25 + 0.4 * sin(M_PI / 12), 0.4 * (1 - cos(M_PI / 12)) + 0.05, M_PI / 12}, {1.25 + 0.4 * sin(M_PI * 2 / 12), 0.4 * (1 - cos(M_PI * 2 / 12)) + 0.05, M_PI *2 / 12}, {1.25 + 0.4 * sin(M_PI * 3 / 12), 0.4 * (1 - cos(M_PI * 3 / 12)) + 0.05, M_PI *3 / 12}, {1.25 + 0.4 * sin(M_PI * 4 / 12), 0.4 * (1 - cos(M_PI * 4 / 12)) + 0.05, M_PI *4 / 12}, {1.25 + 0.4 * sin(M_PI * 5 / 12), 0.4 * (1 - cos(M_PI * 5 / 12)) + 0.05, M_PI *5 / 12}, {1.65, 0.45, M_PI / 2}, {2.05 - 0.4 * sin(M_PI * 5 / 12), 0.4 * (1 - cos(M_PI * 5 / 12)) + 0.05, M_PI *7 / 12}, {2.05 - 0.4 * sin(M_PI * 4 / 12), 0.4 * (1 - cos(M_PI * 4 / 12)) + 0.05, M_PI *8 / 12}, {2.05 - 0.4 * sin(M_PI * 3 / 12), 0.4 * (1 - cos(M_PI * 3 / 12)) + 0.05, M_PI *9 / 12}, {2.05 - 0.4 * sin(M_PI * 2 / 12), 0.4 * (1 - cos(M_PI * 2 / 12)) + 0.05, M_PI *10 / 12}, {2.05 - 0.4 * sin(M_PI / 12), 0.4 * (1 - cos(M_PI / 12)) + 0.05, M_PI *11 / 12}, {2.05, 0.05, M_PI}, {2.15, 0.05, M_PI}, {2.25, 0.05, M_PI}, {2.35, 0.05, M_PI}, {2.45, 0.05, M_PI}, {2.55, 0.05, M_PI}, {2.65, 0.05, M_PI}, {2.75, 0.05, M_PI}, {2.85, 0.05, M_PI}, {2.95, 0.05, M_PI}, {3.05, 0.05, M_PI}, {3.15, 0.05, M_PI}, {3.25, 0.05, M_PI}, {3.35, 0.05, M_PI}, {3.45, 0.05, M_PI}, {3.55, 0.05, M_PI}, {3.65, 0.05, M_PI}, {3.75, 0.05, M_PI}, {3.85, 0.05, M_PI}, {3.95, 0.05, M_PI}, {4.05, 0.05, M_PI} };
  cusp_i_ = 18;

  // we don't expect result to be smoother than original
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(cusp_near_obstacle, 0.01);

  /////////////////////////////////////////////////////
  // testing option to pay extra attention near cusps
  // 中文：第一组使用全程一致的 w_cost，作为 cusp 专项加权的基线。

  // extra attention near cusps option is more significant with a long footprint
  // 中文：加长足迹扩大旋转时的碰撞风险，使 cusp 权重差异更容易被测试观察。
  footprint.clear();
  footprint.push_back(pointMsg(0.4, 0.2));
  footprint.push_back(pointMsg(-0.4, 0.2));
  footprint.push_back(pointMsg(-0.4, -0.2));
  footprint.push_back(pointMsg(0.4, -0.2));

  // first smooth with homogeneous w_cost to compare
  // 中文：先用均匀 Costmap 权重生成对照结果。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  // higher w_curve significantly decreases convergence speed here
  // path feasibility can be restored by subsequent resmoothing with higher w_curve
  // TODO(afrixs): tune ceres optimizer to "converge" faster,
  //               see http://ceres-solver.org/nnls_solving.html
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 1.0));
  // let's have more iterations so that the improvement is more significant
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 500));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path, true, true));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_simple = assessPathImprovement(cusp_near_obstacle, smoothed_path, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_simple, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_simple, 42.6, 1.0);
  double worst_cost_improvement_simple = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path, cost_avoidance_criterion);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp, simple): %lf, %lf", cost_avoidance_improvement_simple, worst_cost_improvement_simple);
  EXPECT_GE(worst_cost_improvement_simple, 0.0);


  // then update parameters so that robot is not so afraid of obstacles
  // 中文：降低普通行驶区域权重，同时提高 cusp 区域权重，让避障重点集中在危险旋转附近。
  // during simple movement but pays extra attention during rotations near cusps
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 0.027 / 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", 2.5));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.fn_tol", 1e-15));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_ecc;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_ecc, true, false));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path_ecc, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_extra_careful_cusp = assessPathImprovement(cusp_near_obstacle, smoothed_path_ecc, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_extra_careful_cusp, 44.2, 1.0);
  double worst_cost_improvement_extra_careful_cusp = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path_ecc, cost_avoidance_criterion);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp, ecc): %lf, %lf", cost_avoidance_improvement_extra_careful_cusp, worst_cost_improvement_extra_careful_cusp);
  EXPECT_GE(worst_cost_improvement_extra_careful_cusp, 0.0);
  EXPECT_GE(worst_cost_improvement_extra_careful_cusp, worst_cost_improvement_simple);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, cost_avoidance_improvement_simple);

  // although extra careful cusp optimization avoids cost better than simple one,
  // 中文：专项 cusp 优化应降低最坏障碍代价，同时因普通区域权重降低而减少整体路径偏移。
  // overall the path doesn't need to deflect so much from original, since w_cost is smaller
  // and thus the obstacles are avoided mostly in dangerous zones around cusps
  auto origin_similarity_criterion = [&cusp_near_obstacle](int i, const Eigen::Vector3d & p) { return (p.block<2, 1>(0, 0) - cusp_near_obstacle[i].block<2, 1>(0, 0)).norm(); };
  double origin_similarity_improvement = assessPathImprovement(smoothed_path, smoothed_path_ecc, origin_similarity_criterion);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Original similarity improvement (cusp, ecc vs. simple): %lf", origin_similarity_improvement);
  EXPECT_GT(origin_similarity_improvement, 0.0);
  EXPECT_NEAR(origin_similarity_improvement, 0.43, 0.02);


  /////////////////////////////////////////////////////
  // testing asymmetric footprint options
  // 中文：切换为前后不对称足迹，验证自定义 Costmap 采样点能够表达真实机器人外形。

  // (diff-drive with 2 actuated wheels and 2 caster wheels)
  footprint.clear();
  footprint.push_back(pointMsg(0.15, 0.2));
  footprint.push_back(pointMsg(-0.65, 0.2));
  footprint.push_back(pointMsg(-0.65, -0.2));
  footprint.push_back(pointMsg(0.15, -0.2));

  // reset parameters back to homogeneous and shift cost check point to the center of the footprint
  // 中文：恢复基础权重，并将两个采样点放到机器人前后部位，权重总和随后由参数读取逻辑归一化。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 1.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", -1.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>({-0.05, 0.0, 0.5, -0.45, 0.0, 0.5})  /* x1, y1, weight1, x2, y2, weight2*/));
  reloadParams();

  // cost improvement is different for path smoothed by original optimizer
  // since the footprint has changed
  cost_avoidance_improvement_simple = assessPathImprovement(cusp_near_obstacle, smoothed_path, cost_avoidance_criterion);
  worst_cost_improvement_simple = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_simple, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp_shifted, simple): %lf, %lf", cost_avoidance_improvement_simple, worst_cost_improvement_simple);
  EXPECT_NEAR(cost_avoidance_improvement_simple, 40.2, 1.0);

  // now smooth using the new optimizer with cost check point shifted
  // 中文：启用非对称足迹采样重新平滑，比较相对只查询参考点的避障改善。
  std::vector<Eigen::Vector3d> smoothed_path_scc;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scc));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path_scc, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_shifted_cost_check = assessPathImprovement(cusp_near_obstacle, smoothed_path_scc, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_shifted_cost_check, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_shifted_cost_check, 42.0, 1.0);
  double worst_cost_improvement_shifted_cost_check = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path_scc, cost_avoidance_criterion);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp_shifted, scc): %lf, %lf", cost_avoidance_improvement_shifted_cost_check, worst_cost_improvement_shifted_cost_check);
  EXPECT_GE(worst_cost_improvement_shifted_cost_check, 0.0);
  EXPECT_GE(worst_cost_improvement_shifted_cost_check, worst_cost_improvement_simple);
  EXPECT_GT(cost_avoidance_improvement_shifted_cost_check, cost_avoidance_improvement_simple);

  // same results should be achieved with unnormalized weights
  // 中文：输入权重总和从 1 改为 2，预期归一化后结果完全一致。
  // (testing automatic weights normalization, i.e. using avg instead of sum)
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>({-0.05, 0.0, 1.0, -0.45, 0.0, 1.0})  /* x1, y1, weight1, x2, y2, weight2*/));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path_scc_unnormalized;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scc_unnormalized));
  EXPECT_EQ(smoothed_path_scc, smoothed_path_scc_unnormalized);

  ////////////////////////////////////////
  // compare also with extra careful cusp
  // 中文：在非对称足迹下再次启用 cusp 专项权重，验证更复杂 Costmap 采样组合的收敛结果。

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 0.027 / 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", 2.5));
  // we need much more iterations here since it's a more complicated problem
  // TODO(afrixs): tune ceres optimizer to "converge" faster
  //               see http://ceres-solver.org/nnls_solving.html
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 1500));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_scce;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scce));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path_scce, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_shifted_extra = assessPathImprovement(cusp_near_obstacle, smoothed_path_scce, cost_avoidance_criterion);
  double worst_cost_improvement_shifted_extra = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path_scce, cost_avoidance_criterion);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp_shifted, scce): %lf, %lf", cost_avoidance_improvement_shifted_extra, worst_cost_improvement_shifted_extra);
  EXPECT_NEAR(cost_avoidance_improvement_shifted_extra, 51.0, 1.0);
  EXPECT_GE(worst_cost_improvement_shifted_extra, 0.0);

  // resmooth extra careful cusp with same conditions (higher max_iterations)
  // 中文：清空自定义采样点后复测默认参考点模式，确保两种代价查询方式可以独立切换。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>()));
  reloadParams();

  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_ecc));
  cost_avoidance_improvement_extra_careful_cusp = assessPathImprovement(cusp_near_obstacle, smoothed_path_ecc, cost_avoidance_criterion);
  worst_cost_improvement_extra_careful_cusp = assessWorstPoseImprovement(cusp_near_obstacle, smoothed_path_ecc, cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp_shifted, ecc): %lf, %lf", cost_avoidance_improvement_extra_careful_cusp, worst_cost_improvement_extra_careful_cusp);
  EXPECT_NEAR(cost_avoidance_improvement_extra_careful_cusp, 48.5, 1.0);
  EXPECT_GT(cost_avoidance_improvement_shifted_extra, cost_avoidance_improvement_extra_careful_cusp);
  // worst cost improvement is a bit lower but only by 5% so it's not a big deal
  EXPECT_GE(worst_cost_improvement_shifted_extra, worst_cost_improvement_extra_careful_cusp - 6.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingDownsamplingUpsampling) {
  // 中文：验证路径下采样、cusp 处重置采样和三次贝塞尔上采样对点数与平滑性的影响。
  // path with a cusp
  std::vector<Eigen::Vector3d> sharp_turn_90_then_reverse = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, 0}, {0.4, 0, 0}, {0.5, 0, 0}, {0.6, 0, M_PI / 4}, {0.6, -0.1, M_PI / 2}, {0.6, -0.2, M_PI / 2}, {0.6, -0.3, M_PI / 2}, {0.6, -0.4, M_PI / 2}, {0.6, -0.5, M_PI / 2}, {0.6, -0.6, M_PI / 2} };
  cusp_i_ = 6;

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_downsampling_factor", 2));
  // downsample only
  // 中文：只下采样时保留边界点，并按因子 2 跳过中间点。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", false));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path_downsampled;
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path_downsampled));
  // first two, last two and every 2nd pose between them
  EXPECT_EQ(smoothed_path_downsampled.size(), 8u);

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", true));
  reloadParams();
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path_downsampled));
  // same but downsampling is reset on cusp
  EXPECT_EQ(smoothed_path_downsampled.size(), 9u);

  // upsample to original size
  // 中文：上采样因子为 1 时恢复到原始点数，同时重新计算插值点朝向。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 1));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));
  EXPECT_EQ(smoothed_path.size(), sharp_turn_90_then_reverse.size());

  cusp_i_ = 4;  // for downsampled path
  int cusp_i_out = 6;  // for upsampled path
  QualityCriterion3 mvmt_smoothness_criterion_out = [&cusp_i_out](int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p, const Eigen::Vector3d & next_p) { Eigen::Vector2d prev_mvmt = p.block<2, 1>(0, 0) - prev_p.block<2, 1>(0, 0); Eigen::Vector2d next_mvmt = next_p.block<2, 1>(0, 0) - p.block<2, 1>(0, 0); if (i == cusp_i_out) { next_mvmt = -next_mvmt; } return (next_mvmt - prev_mvmt).norm(); };

  double smoothness_improvement = assessPathImprovement(smoothed_path_downsampled, smoothed_path, mvmt_smoothness_criterion_, &mvmt_smoothness_criterion_out);
  // more poses -> smoother path
  EXPECT_GT(smoothness_improvement, 0.0);
  EXPECT_NEAR(smoothness_improvement, 63.9, 1.0);

  // upsample above original size
  // 中文：上采样因子为 2 时每个原始区间插入更多点，期望路径更连续且点数翻倍减一。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 2));
  reloadParams();
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));
  // every pose except last produces 2 poses
  EXPECT_EQ(smoothed_path.size(), sharp_turn_90_then_reverse.size() * 2 - 1);
  cusp_i_out = 12;  // for upsampled path
  smoothness_improvement = assessPathImprovement(smoothed_path_downsampled, smoothed_path, mvmt_smoothness_criterion_, &mvmt_smoothness_criterion_out);
  // even more poses -> even smoother path
  EXPECT_GT(smoothness_improvement, 0.0);
  EXPECT_NEAR(smoothness_improvement, 82.2, 1.0);
}

TEST_F(SmootherTest, testingStartGoalOrientations) {
  // 中文：验证起点和终点朝向默认保持，以及关闭保护后沿路径切线自动调整的行为。
  std::vector<Eigen::Vector3d> sharp_turn_90 = {{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, M_PI / 4}, {0.3, 0.1, M_PI / 2}, {0.3, 0.2, M_PI / 2}, {0.3, 0.3, M_PI / 2} };

  // Keep start and goal orientations (by default)
  // 中文：默认配置把起终点位置及其朝向支撑点固定在 Ceres 问题中。
  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  double mvmt_smoothness_improvement = assessPathImprovement(sharp_turn_90, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 55.2, 1.0);
  // no change in orientations
  EXPECT_NEAR(smoothed_path.front()[2], 0, 0.001);
  EXPECT_NEAR(smoothed_path.back()[2], M_PI / 2, 0.001);

  // Overwrite start/goal orientations
  // 中文：关闭两个保护参数后，起终点朝向可以随优化路径重新计算。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_start_orientation", false));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_goal_orientation", false));
  reloadParams();

  sharp_turn_90[0][2] = M_PI;  // forward/reverse of start pose should not matter
  std::vector<Eigen::Vector3d> smoothed_path_sg_overwritten;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path_sg_overwritten));

  mvmt_smoothness_improvement = assessPathImprovement(smoothed_path, smoothed_path_sg_overwritten, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 58.9, 1.0);
  // orientations adjusted to follow the path
  EXPECT_NEAR(smoothed_path_sg_overwritten.front()[2], M_PI / 8, 0.1);
  EXPECT_NEAR(smoothed_path_sg_overwritten.back()[2], 3 * M_PI / 8, 0.1);

  // test short paths
  // 中文：测试两点路径和单点路径，确认无内部优化点时仍能稳定生成合理朝向。
  std::vector<Eigen::Vector3d> short_screwed_path = {{0, 0, M_PI * 0.25}, {0.1, 0, -M_PI * 0.25} };

  std::vector<Eigen::Vector3d> adjusted_path;
  EXPECT_TRUE(smoothPath(short_screwed_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], 0, 0.001);
  EXPECT_NEAR(adjusted_path.back()[2], 0, 0.001);

  short_screwed_path[0][2] = M_PI * 0.75;  // start is stronger than goal
  EXPECT_TRUE(smoothPath(short_screwed_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], M_PI, 0.001);
  EXPECT_NEAR(adjusted_path.back()[2], M_PI, 0.001);

  std::vector<Eigen::Vector3d> one_pose_path = {{0, 0, M_PI * 0.75}};
  EXPECT_TRUE(smoothPath(one_pose_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], M_PI * 0.75, 0.001);
}

TEST_F(SmootherTest, testingCostCheckPointsParamValidity) {
  // 中文：验证 Costmap 采样点参数必须按 x、y、weight 三元组组织，非法长度应在配置阶段抛错。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>()));
  reloadParams();

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>({0, 0, 0, 0, 0, 0, 0, 0, 0})));  // multiple of 3 is ok
  // 中文：长度为 3 的倍数时可以通过参数校验，即使权重随后会被归一化。
  reloadParams();

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cost_check_points", std::vector<double>({0, 0})));
  EXPECT_THROW(reloadParams(), std::runtime_error);
}

TEST_F(SmootherTest, testingLinearSolverTypeParamValidity) {
  // 中文：验证两个受支持的 Ceres 线性求解器名称可以配置，未知名称必须拒绝。
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.linear_solver_type", "SPARSE_NORMAL_CHOLESKY"));
  reloadParams();

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.linear_solver_type", "DENSE_QR"));
  reloadParams();

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.linear_solver_type", "INVALID_SOLVER"));
  EXPECT_THROW(reloadParams(), std::runtime_error);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  // 中文：测试使用 LifecycleNode、Costmap Publisher 和 TF Buffer，因此先初始化 ROS 上下文。
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  // 中文：所有 GoogleTest 用例结束后关闭 ROS，保证进程退出前释放节点相关资源。
  rclcpp::shutdown();

  return all_successful;
}
