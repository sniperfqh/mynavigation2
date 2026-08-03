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

#include <algorithm>
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "nav2_constrained_smoother/constrained_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
// 中文：复用 Nav2 的参数声明、几何距离和 Costmap 类型，避免插件重复封装
// 中文：公共基础能力。

namespace nav2_constrained_smoother
{

void ConstrainedSmoother::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub, std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) {
  // 中文：Lifecycle 配置回调只接收弱引用，先锁定父节点，防止节点已经
  // 中文：销毁时继续初始化插件。
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  // 中文：保存服务器传入的共享资源和插件实例名，后续参数与日志都以
  // 中文：该名称为前缀。

  smoother_ = std::make_unique<nav2_constrained_smoother::Smoother>();
  // 中文：先读取 Ceres 求解器参数，再读取路径代价参数，最后用已解析的
  // 中文：OptimizerParams 初始化内部算法。
  optimizer_params_.get(node.get(), name);
  smoother_params_.get(node.get(), name);
  smoother_->initialize(optimizer_params_);
}

void ConstrainedSmoother::cleanup() {
  // 中文：服务器进入 cleanup 生命周期时调用；当前没有订阅或发布资源
  // 中文：需要额外销毁。
  RCLCPP_INFO(logger_, "Cleaning up smoother: %s of type" " nav2_constrained_smoother::ConstrainedSmoother", plugin_name_.c_str());
}

void ConstrainedSmoother::activate() {
  // 中文：激活阶段只记录插件可用，实际平滑请求仍由 Smoother Server 的
  // 中文：Action 回调触发。
  RCLCPP_INFO(logger_, "Activating smoother: %s of type " "nav2_constrained_smoother::ConstrainedSmoother", plugin_name_.c_str());
}

void ConstrainedSmoother::deactivate() {
  // 中文：停用阶段只记录状态；服务器负责阻止新的业务调用并协调其他
  // 中文：生命周期节点。
  RCLCPP_INFO(logger_, "Deactivating smoother: %s of type " "nav2_constrained_smoother::ConstrainedSmoother", plugin_name_.c_str());
}

bool ConstrainedSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) {
  // 中文：这是 nav2_core::Smoother 的同步入口，输入输出使用同一个 Path
  // 中文：引用，max_time 传给 Ceres。
  if (path.poses.size() < 2) {
    // 中文：单点或空路径没有可优化的线段，保持原路径并视为成功，避免
    // 中文：无意义地抛出异常。
    return true;
  }

  // populate smoother input with (x, y, forward/reverse dir)
  // 中文：内部第三维不是 yaw，而是该点之后运动段的符号：+1 前进，-1
  // 中文：倒退。
  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());
  // smoother keeps record of start/end orientations so that it
  // can use them in the final path, preventing degradation of these (often important) values
  // 中文：单独保存起点和终点单位方向，后处理阶段可按参数决定是否
  // 中文：恢复原始边界朝向。
  Eigen::Vector2d start_dir;
  Eigen::Vector2d end_dir;
  for (size_t i = 0; i < path.poses.size(); i++) {
    // 中文：逐个 Pose 提取位置和 yaw，并根据相邻位置判断机器人是否在
    // 中文：倒退。
    auto & pose = path.poses[i].pose;
    double angle = tf2::getYaw(pose.orientation);
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    if (i == path.poses.size() - 1) {
      // Note: `reversing` indicates the direction of the segment after the point and
      // there is no segment after the last point. Most probably the value is irrelevant, but
      // copying it from the last but one point, just to make it defined...
      path_world.emplace_back(pose.position.x, pose.position.y, path_world.back()[2]);
      // 中文：终点没有后继线段，沿用前一个点的方向符号仅用于保持数据
      // 中文：定义完整。
      end_dir = orientation;
    } else {
      auto & pos_next = path.poses[i + 1].pose.position;
      Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);
      // robot is considered reversing when angle between its orientation and movement direction
      // is more than 90 degrees (i.e. dot product is less than 0)
      // 中文：姿态方向与位移方向点积为负时认为车辆倒退；
      // 中文：reversing_enabled=false 会关闭该判断。
      bool reversing = smoother_params_.reversing_enabled && orientation.dot(mvmt) < 0;
      // we transform boolean value of "reversing" into sign of movement direction (+1 or -1)
      // to simplify further computations
      // 中文：把布尔方向编码为数值符号，供 cusp 检测、切线选择和 Costmap
      // 中文：足迹变换复用。
      path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
      if (i == 0) {
        start_dir = orientation;
      } else if (i == 1 && !smoother_params_.keep_start_orientation) {
        // overwrite start forward/reverse when orientation was set to be ignored
        // note: start_dir is overwritten inside Smoother::upsampleAndPopulate() method
        path_world[0][2] = path_world.back()[2];
        // 中文：不保留起点朝向时，让起点方向符号与第一段实际运动方向
        // 中文：保持一致。
      }
    }
  }

  smoother_params_.max_time = max_time.seconds();
  // 中文：将上层 Action 剩余时间预算转换为秒，限制内部 Ceres 求解时长。

  // Smooth plan
  // 中文：从订阅器读取当前 Costmap 快照，算法只在本次同步调用中使用该
  // 中文：快照。
  auto costmap = costmap_sub_->getCostmap();
  if (!smoother_->smooth(path_world, start_dir, end_dir, costmap.get(), smoother_params_)) {
    // 中文：Ceres 无法得到可用解时抛出 Nav2 异常，由上层 Smoother Server
    // 中文：将其转为 Action 失败。
    RCLCPP_WARN(logger_, "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.", plugin_name_.c_str());
    throw new nav2_core::PlannerException("Failed to smooth plan, Ceres could not find a usable solution.");
  }

  // populate final path
  // 中文：把内部 x、y、yaw 重新编码为 PoseStamped，保留原路径首个 Pose
  // 中文：的 Header。
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  path.poses.clear();
  path.poses.reserve(path_world.size());
  for (auto & pw : path_world) {
    pose.pose.position.x = pw[0];
    pose.pose.position.y = pw[1];
    pose.pose.orientation.z = sin(pw[2] / 2);
    pose.pose.orientation.w = cos(pw[2] / 2);

    path.poses.push_back(pose);
  }

  // 中文：插件接口返回 true 表示路径已经完成改写；具体 Action 成功语义由
  // 中文：包外服务器决定。
  return true;
}

}  // namespace nav2_constrained_smoother

// Register this smoother as a nav2_core plugin
// 中文：将类名、插件类型和 nav2_core::Smoother 基类写入 Pluginlib 注册表，
// 中文：供 Smoother Server 动态加载。
PLUGINLIB_EXPORT_CLASS(nav2_constrained_smoother::ConstrainedSmoother, nav2_core::Smoother)
