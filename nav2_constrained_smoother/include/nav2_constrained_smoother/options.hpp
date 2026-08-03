// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020, Samsung Research America
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

#ifndef NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_

#include <map>
#include <string>
#include <vector>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "ceres/ceres.h"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_smac_planner::SmootherParams
 * @brief Parameters for the smoother cost function
 * 中文：集中保存路径平滑、曲率约束、Costmap 避障、cusp 区域和路径采样相关的运行参数。
 */
struct SmootherParams
{
  /**
   * @brief A constructor for nav2_smac_planner::SmootherParams
   * 中文：使用成员默认值构造参数对象，随后由 get() 从生命周期节点读取配置。
   */
  SmootherParams() {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   * 中文：按插件名称拼接参数前缀，声明缺省值并把 ROS 参数写入内部优化字段。
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name) {
    std::string local_name = name + std::string(".");

    // Smoother params
    // 中文：先读取最小转弯半径，再转换为最大允许曲率，后续残差只惩罚超出该曲率的部分。
    double minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared( node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);
    max_curvature = 1.0f / minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared( node, local_name + "w_curve", rclcpp::ParameterValue(30.0));
    node->get_parameter(local_name + "w_curve", curvature_weight);
    // 中文：w_curve 控制违反最大曲率约束的惩罚强度。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "w_cost", rclcpp::ParameterValue(0.015));
    node->get_parameter(local_name + "w_cost", costmap_weight);
    // 中文：w_cost 控制路径远离 Costmap 高代价区域的基础权重。
    double cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared( node, local_name + "w_cost_cusp_multiplier", rclcpp::ParameterValue(3.0));
    node->get_parameter(local_name + "w_cost_cusp_multiplier", cost_cusp_multiplier);
    // 中文：cusp 附近的避障权重由基础权重乘以该倍数得到。
    cusp_costmap_weight = costmap_weight * cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared( node, local_name + "cusp_zone_length", rclcpp::ParameterValue(2.5));
    node->get_parameter(local_name + "cusp_zone_length", cusp_zone_length);
    // 中文：cusp_zone_length 定义方向切换点两侧需要提高避障关注度的总区间长度。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_dist", distance_weight);
    // 中文：w_dist 将优化后的点约束在原始路径附近，避免平滑过度偏离原规划结果。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "w_smooth", rclcpp::ParameterValue(2000000.0));
    node->get_parameter(local_name + "w_smooth", smooth_weight);
    // 中文：w_smooth 惩罚相邻线段方向变化，是连续平滑效果的主要权重。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "cost_check_points", rclcpp::ParameterValue(std::vector<double>()));
    node->get_parameter(local_name + "cost_check_points", cost_check_points);
    // 中文：可选的机器人坐标系采样点按 x、y、weight 三元组描述非对称足迹的代价检查位置。
    if (cost_check_points.size() % 3 != 0) {
      RCLCPP_ERROR( rclcpp::get_logger( "constrained_smoother"), "cost_check_points parameter must contain values as follows: " "[x1, y1, weight1, x2, y2, weight2, ...]");
      throw std::runtime_error("Invalid parameter: cost_check_points");
    }
    // normalize check point weights so that their sum == 1.0
    // 中文：归一化采样点权重，保证增加采样点数量不会无意放大整体 Costmap 残差。
    double check_point_weights_sum = 0.0;
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      check_point_weights_sum += cost_check_points[i];
    }
    // 中文：实现只校验三元组长度，不额外拒绝权重总和为零的输入；配置采样点时应提供非零总权重。
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      cost_check_points[i] /= check_point_weights_sum;
    }
    // 中文：下采样与上采样参数共同决定 Ceres 优化节点数量和最终输出路径密度。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "path_downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_downsampling_factor", path_downsampling_factor);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "path_upsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_upsampling_factor", path_upsampling_factor);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "reversing_enabled", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "reversing_enabled", reversing_enabled);
    // 中文：决定是否根据姿态与相邻位移的夹角识别前进／倒退方向和 cusp。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "keep_goal_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_goal_orientation", keep_goal_orientation);
    // 中文：控制终点姿态是否作为优化边界固定下来。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "keep_start_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_start_orientation", keep_start_orientation);
    // 中文：控制起点姿态是否作为优化边界固定下来。
  }

  // 中文：平滑残差、Costmap 残差、cusp 权重、距离约束和曲率约束的运行时数值。
  double smooth_weight{0.0};
  double costmap_weight{0.0};
  double cusp_costmap_weight{0.0};
  double cusp_zone_length{0.0};
  double distance_weight{0.0};
  double curvature_weight{0.0};
  double max_curvature{0.0};
  double max_time{10.0};  // adjusted by action goal, not by parameters
  // 中文：最大求解时间由 SmoothPath Action 的 max_duration 传入，不从参数服务器读取。
  int path_downsampling_factor{1};
  int path_upsampling_factor{1};
  bool reversing_enabled{true};
  bool keep_goal_orientation{true};
  bool keep_start_orientation{true};
  std::vector<double> cost_check_points{};
};

/**
 * @struct nav2_smac_planner::OptimizerParams
 * @brief Parameters for the ceres optimizer
 * 中文：保存 Ceres 求解器类型、迭代上限、收敛阈值和调试输出开关。
 */
struct OptimizerParams
{
  // 中文：构造 Ceres 参数的默认值，保持生产环境默认不输出逐迭代调试信息。
  OptimizerParams() : debug(false), max_iterations(50), param_tol(1e-8), fn_tol(1e-6), gradient_tol(1e-10) {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   * 中文：从 `<plugin_name>.optimizer.*` 参数命名空间读取 Ceres 配置，并校验求解器类型。
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name) {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    // 中文：线性求解器名称必须位于 solver_types 映射中，非法值直接阻止插件配置完成。
    nav2_util::declare_parameter_if_not_declared( node, local_name + "linear_solver_type", rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));
    node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
    if (solver_types.find(linear_solver_type) == solver_types.end()) {
      std::stringstream valid_types_str;
      for (auto type = solver_types.begin(); type != solver_types.end(); type++) {
        if (type != solver_types.begin()) {
          valid_types_str << ", ";
        }
        valid_types_str << type->first;
      }
      RCLCPP_ERROR( rclcpp::get_logger("constrained_smoother"), "Invalid linear_solver_type. Valid values are %s", valid_types_str.str().c_str());
      throw std::runtime_error("Invalid parameter: linear_solver_type");
    }
    nav2_util::declare_parameter_if_not_declared( node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
    node->get_parameter(local_name + "param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
    node->get_parameter(local_name + "fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "max_iterations", rclcpp::ParameterValue(100));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared( node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);
    // 中文：其余参数控制 Ceres 的最大迭代次数和三类收敛阈值。
  }

  // 中文：当前实现只开放稠密 QR 和稀疏正规方程 Cholesky 两种线性求解器。
  const std::map<std::string, ceres::LinearSolverType> solver_types = {{"DENSE_QR", ceres::DENSE_QR}, {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY}};

  // 中文：是否让内部 Smoother 输出 Ceres 的求解进度和完整报告。
  bool debug;
  // 中文：参数服务器中的求解器名称，同时用于查找 solver_types 中的 Ceres 枚举。
  std::string linear_solver_type;
  // 中文：Ceres 最大迭代次数；超过后即使未达到阈值也会结束求解。
  int max_iterations;  // Ceres default: 50

  // 中文：参数、目标函数和梯度的收敛阈值，数值越小通常意味着更严格的停止条件。
  double param_tol;  // Ceres default: 1e-8
  double fn_tol;  // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
