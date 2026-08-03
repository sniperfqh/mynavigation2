// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/smoother.hpp"
#include "nav2_constrained_smoother/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_constrained_smoother
{

/**
 * @class nav2_constrained_smoother::ConstrainedSmoother
 * @brief Regulated pure pursuit controller plugin
 * 中文：该类实现 nav2_core::Smoother 插件接口，负责把 ROS 2 的 nav_msgs/Path 转换为内部优化路径，
 *      调用 Ceres 约束平滑器后再写回新的路径姿态。
 */
class ConstrainedSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for nav2_constrained_smoother::ConstrainedSmoother
   * 中文：构造函数不创建节点资源；插件实例化后由 Smoother Server 调用 configure() 完成实际初始化。
   */
  ConstrainedSmoother() = default;

  /**
   * @brief Destrructor for nav2_constrained_smoother::ConstrainedSmoother
   * 中文：析构阶段释放唯一指针管理的内部平滑器，生命周期资源的主动收口由 cleanup() 负责。
   */
  ~ConstrainedSmoother() override = default;

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   * 中文：保存父节点、插件名、TF 和 Costmap 订阅器，并读取插件参数、创建内部 Ceres 平滑器。
   */
  void configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub, std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Cleanup controller state machine
   * 中文：清理插件生命周期状态；当前实现主要输出日志，内部平滑器由成员智能指针管理。
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   * 中文：进入可处理平滑请求的激活状态；当前实现只记录激活边界。
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   * 中文：停止接受激活态业务请求；当前实现只记录停用边界。
   */
  void deactivate() override;

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be optimized
   * @param max_time Maximum duration smoothing should take
   * @return Smoothed path
   * 中文：输入路径按二维位置和前进／倒退方向编码，平滑成功后原地改写 path，返回是否完成。
   */
  bool smooth( nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:
  // 中文：TF 缓存由服务器传入并保存，为插件接口保持与其他 Nav2 平滑器一致的资源边界。
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 中文：插件在参数树中的实例名，例如 SmoothPath，日志和参数读取都使用该名称。
  std::string plugin_name_;
  // 中文：Costmap 订阅器提供优化阶段读取环境代价地图的入口。
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  // 中文：生命周期节点日志器，用于记录配置、激活、停用和平滑失败等边界事件。
  rclcpp::Logger logger_ {rclcpp::get_logger("ConstrainedSmoother")};

  // 中文：内部纯算法平滑器负责构建 Ceres 问题、求解残差并恢复路径密度与朝向。
  std::unique_ptr<nav2_constrained_smoother::Smoother> smoother_;
  // 中文：路径平滑权重、方向、下采样和起终点朝向策略。
  SmootherParams smoother_params_;
  // 中文：Ceres 线性求解器、收敛阈值、最大迭代次数和调试输出配置。
  OptimizerParams optimizer_params_;
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
