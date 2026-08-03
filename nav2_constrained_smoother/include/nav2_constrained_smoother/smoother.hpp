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

#ifndef NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <deque>
#include <limits>
#include <algorithm>

#include "nav2_constrained_smoother/smoother_cost_function.hpp"
#include "nav2_constrained_smoother/utils.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_constrained_smoother
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 * 中文：内部算法层把带方向的二维路径建模为 Ceres 非线性最小二乘问题，输出平滑且满足曲率、
 *      原路径距离和 Costmap 避障约束的路径。
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   * 中文：构造函数保持算法对象为空配置状态，实际求解器选项由 initialize() 设置。
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   * 中文：释放 Ceres 选项和 Costmap 网格智能指针，不拥有外部生命周期节点或 Costmap。
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   * 中文：把 ROS 参数层的求解器名称和收敛阈值转换为 Ceres::Solver::Options。
   */
  void initialize(const OptimizerParams params) {
    // 中文：保存调试开关，后续 smooth() 据此决定是否打印 Ceres 完整报告。
    debug_ = params.debug;

    // 中文：用参数中的字符串查找 Ceres 线性求解器枚举；无效名称已在参数读取阶段拒绝。
    options_.linear_solver_type = params.solver_types.at(params.linear_solver_type);

    // 中文：限制单次优化的最大迭代次数，防止复杂路径无限求解。
    options_.max_num_iterations = params.max_iterations;

    // 中文：设置参数、目标函数和梯度三个维度的停止阈值。
    options_.function_tolerance = params.fn_tol;
    options_.gradient_tolerance = params.gradient_tol;
    options_.parameter_tolerance = params.param_tol;

    if (debug_) {
      // 中文：调试模式把每次迭代的进度输出到标准输出，便于分析收敛速度和残差变化。
      options_.minimizer_progress_to_stdout = true;
      options_.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      // 中文：生产模式关闭 Ceres 自身日志，避免高频优化污染 Nav2 日志。
      options_.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param costmap Pointer to minimal costmap
   * @param params parameters weights
   * @return If smoothing was successful
   * 中文：先构建残差问题，再按时间上限调用 Ceres，最后把优化点和插值点恢复为输出路径。
   */
  bool smooth( std::vector<Eigen::Vector3d> & path, const Eigen::Vector2d & start_dir, const Eigen::Vector2d & end_dir, const nav2_costmap_2d::Costmap2D * costmap, const SmootherParams & params) {
    // Path has always at least 2 points
    // 中文：内部算法至少需要起点和终点；上层插件通常会提前处理更短路径。
    if (path.size() < 2) {
      throw std::runtime_error("Constrained smoother: Path must have at least 2 points");
    }

    options_.max_solver_time_in_seconds = params.max_time;
    // 中文：max_time 来自 Action 目标的剩余时间预算，直接限制 Ceres 求解器运行时长。

    ceres::Problem problem;
    std::vector<Eigen::Vector3d> path_optim;
    std::vector<bool> optimized;
    if (buildProblem(path, costmap, params, problem, path_optim, optimized)) {
      // solve the problem
      // 中文：buildProblem() 返回 true 表示至少存在一个可优化的内部点和有效残差块。
      ceres::Solver::Summary summary;
      ceres::Solve(options_, &problem, &summary);
      if (debug_) {
        // 中文：完整报告包含初始／最终代价、迭代次数和终止原因，仅在显式调试时输出。
        RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "%s", summary.FullReport().c_str());
      }
      if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost < 0.0) {
        // 中文：不可用或最终代价没有下降时拒绝结果，避免把异常优化结果写回导航路径。
        return false;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "Path too short to optimize");
      // 中文：没有可优化内部点时保留原始几何点，但仍由后处理统一生成姿态和采样密度。
    }

    upsampleAndPopulate(path_optim, optimized, start_dir, end_dir, params, path);
    // 中文：Ceres 只优化选中的位置点，后处理负责补回下采样点并恢复每个点的 yaw。

    return true;
  }

private:
  /**
   * @brief Build problem method
   * @param path Reference to path
   * @param costmap Pointer to costmap
   * @param params Smoother parameters
   * @param problem Output problem to solve
   * @param path_optim Output path on which the problem will be solved
   * @param optimized False for points skipped by downsampling
   * @return If there is a problem to solve
   * 中文：把输入路径转换成 Ceres 参数块，识别 cusp、建立四项残差，并固定起终点边界。
   */
  bool buildProblem( const std::vector<Eigen::Vector3d> & path, const nav2_costmap_2d::Costmap2D * costmap, const SmootherParams & params, ceres::Problem & problem, std::vector<Eigen::Vector3d> & path_optim, std::vector<bool> & optimized) {
    // Create costmap grid
    // 中文：直接引用 Costmap 的字符数组构造 Ceres 网格，再用双三次插值支持连续坐标查询。
    costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>( costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0, costmap->getSizeInCellsX()); auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>( *costmap_grid_);

    // Create residual blocks
    // 中文：cusp_half_length 用于计算方向切换点两侧 Costmap 权重的线性过渡范围。
    const double cusp_half_length = params.cusp_zone_length / 2;
    ceres::LossFunction * loss_function = NULL;
    path_optim = path;
    optimized = std::vector<bool>(path.size());
    optimized[0] = true;
    int prelast_i = -1;
    int last_i = 0;
    double last_direction = path_optim[0][2];
    bool last_was_cusp = false;
    bool last_is_reversing = false;
    std::deque<std::pair<double, SmootherCostFunction *>> potential_cusp_funcs;
    double last_segment_len = EPSILON;
    double potential_cusp_funcs_len = 0;
    double len_since_cusp = std::numeric_limits<double>::infinity();

    for (size_t i = 1; i < path_optim.size(); i++) {
      // 中文：逐点扫描路径；path_optim[i][2] 保存该点之后线段的前进／倒退符号。
      auto & pt = path_optim[i];
      bool is_cusp = false;
      if (i != path_optim.size() - 1) {
        // 中文：相邻方向符号相乘小于零表示经过 cusp，需要保留该点并重置附近代价权重。
        is_cusp = pt[2] * last_direction < 0;
        last_direction = pt[2];

        // skip to downsample if can be skipped (no forward/reverse direction change)
        // 中文：无方向变化且未触及起终点保护区时，按 path_downsampling_factor 跳过部分优化节点。
        if (!is_cusp &&
          i > (params.keep_start_orientation ? 1 : 0) &&
          i < path_optim.size() - (params.keep_goal_orientation ? 2 : 1) &&
          static_cast<int>(i - last_i) < params.path_downsampling_factor)
        {
          continue;
        }
      }

      // keep distance inequalities between poses
      // (some might have been downsampled while others might not)
      // 中文：即使前一段被下采样，也通过实际几何距离更新残差中的长度比例。
      double current_segment_len = (path_optim[i] - path_optim[last_i]).block<2, 1>(0, 0).norm();

      // forget cost functions which don't have chance to be part of a cusp zone
      // 中文：只保留仍可能落入 cusp 区域的残差指针，避免无限增长并限制后续权重回溯范围。
      potential_cusp_funcs_len += current_segment_len;
      while (!potential_cusp_funcs.empty() && potential_cusp_funcs_len > cusp_half_length) {
        potential_cusp_funcs_len -= potential_cusp_funcs.front().first;
        potential_cusp_funcs.pop_front();
      }

      // update cusp zone costmap weights
      if (is_cusp) {
        // 中文：从 cusp 向前回溯，在区间内把普通 w_cost 与 cusp_costmap_weight 线性混合。
        double len_to_cusp = current_segment_len;
        for (int i = potential_cusp_funcs.size() - 1; i >= 0; i--) {
          auto & f = potential_cusp_funcs[i];
          double new_weight = params.cusp_costmap_weight * (1.0 - len_to_cusp / cusp_half_length) + params.costmap_weight * len_to_cusp / cusp_half_length;
          if (std::abs(new_weight - params.cusp_costmap_weight) <
            std::abs(f.second->getCostmapWeight() - params.cusp_costmap_weight))
          {
            // 中文：只在新权重更接近 cusp 强权重时更新，保证每个残差得到单调合理的区域权重。
            f.second->setCostmapWeight(new_weight);
          }
          len_to_cusp += f.first;
        }
        potential_cusp_funcs_len = 0;
        potential_cusp_funcs.clear();
        len_since_cusp = 0;
      }

      // add cost function
      // 中文：当前点成为新的优化参数块；从第三个有效点开始，为前一、当前、后一位置创建残差。
      optimized[i] = true;
      if (prelast_i != -1) {
        double costmap_weight = params.costmap_weight;
        if (len_since_cusp <= cusp_half_length) {
          // 中文：当前点仍处于 cusp 后半区时，按离 cusp 的距离逐渐恢复普通 Costmap 权重。
          costmap_weight = params.cusp_costmap_weight * (1.0 - len_since_cusp / cusp_half_length) + params.costmap_weight * len_since_cusp / cusp_half_length;
        }
        SmootherCostFunction * cost_function = new SmootherCostFunction( path[last_i].template block<2, 1>( 0, 0), (last_was_cusp ? -1 : 1) * last_segment_len / current_segment_len, last_is_reversing, costmap, costmap_interpolator, params, costmap_weight ); problem.AddResidualBlock( /* 中文：三个二维参数块分别连接前一点、当前点和后一点，残差函数据此计算局部几何代价。 */ cost_function->AutoDiff(), loss_function, path_optim[last_i].data(), pt.data(), path_optim[prelast_i].data());

        potential_cusp_funcs.emplace_back(current_segment_len, cost_function);
      }

      // shift current to last and last to pre-last
      // 中文：推进滑动窗口状态，供下一次循环判断方向变化、长度比例和 cusp 距离。
      last_was_cusp = is_cusp;
      last_is_reversing = last_direction < 0;
      prelast_i = last_i;
      last_i = i;
      len_since_cusp += current_segment_len;
      last_segment_len = std::max(EPSILON, current_segment_len);
    }

    int posesToOptimize = problem.NumParameterBlocks() - 2;  // minus start and goal
    // 中文：先排除起点和终点，再根据起终点朝向保护策略排除额外的边界点。
    if (params.keep_goal_orientation) {
      posesToOptimize -= 1;  // minus goal orientation holder
    }
    if (params.keep_start_orientation) {
      posesToOptimize -= 1;  // minus start orientation holder
    }
    if (posesToOptimize <= 0) {
      // 中文：没有自由内部点时不调用 Ceres，交给后处理直接恢复路径。
      return false;  // nothing to optimize
    }
    // first two and last two points are constant (to keep start and end direction)
    // 中文：把边界参数块固定，避免优化改变用户要求保留的起点／终点位置和朝向支撑点。
    problem.SetParameterBlockConstant(path_optim.front().data());
    if (params.keep_start_orientation) {
      problem.SetParameterBlockConstant(path_optim[1].data());
    }
    if (params.keep_goal_orientation) {
      problem.SetParameterBlockConstant(path_optim[path_optim.size() - 2].data());
    }
    problem.SetParameterBlockConstant(path_optim.back().data());
    return true;
  }

  /**
   * @brief Populate optimized points to path, assigning orientations and upsampling poses using cubic bezier
   * @param path_optim Path with optimized points
   * @param optimized False for points skipped by downsampling
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param params Smoother parameters
   * @param path Output path with upsampled optimized points
   * 中文：遍历优化点，按三次贝塞尔曲线插入被下采样或额外上采样的点，并为所有点计算 yaw。
   */
  void upsampleAndPopulate( const std::vector<Eigen::Vector3d> & path_optim, const std::vector<bool> & optimized, const Eigen::Vector2d & start_dir, const Eigen::Vector2d & end_dir, const SmootherParams & params, std::vector<Eigen::Vector3d> & path) {
    // Populate path, assign orientations, interpolate skipped/upsampled poses
    // 中文：输出路径重新生成，不复用输入容器中的旧点，保证点数和姿态与当前参数一致。
    path.clear();
    if (params.path_upsampling_factor > 1) {
      path.reserve(params.path_upsampling_factor * (path_optim.size() - 1) + 1);
    }
    int last_i = 0;
    int prelast_i = -1;
    Eigen::Vector2d prelast_dir;
    for (int i = 1; i <= static_cast<int>(path_optim.size()); i++) {
      // 中文：`optimized` 标记保留点；循环额外访问 path_optim.size() 用于强制收尾最后一段。
      if (i == static_cast<int>(path_optim.size()) || optimized[i]) {
        if (prelast_i != -1) {
          Eigen::Vector2d last_dir;
          auto & prelast = path_optim[prelast_i];
          auto & last = path_optim[last_i];

          // Compute orientation of last
          // 中文：优先使用局部圆弧切线；终点可选择保留原朝向，否则使用最后一段运动方向。
          if (i < static_cast<int>(path_optim.size())) {
            auto & current = path_optim[i];
            Eigen::Vector2d tangent_dir = tangentDir<double>( prelast.block<2, 1>(0, 0), last.block<2, 1>(0, 0), current.block<2, 1>(0, 0), prelast[2] * last[2] < 0);

            last_dir = tangent_dir.dot((current - last).block<2, 1>(0, 0) * last[2]) >= 0 ? tangent_dir : -tangent_dir;
            last_dir.normalize();
          } else if (params.keep_goal_orientation) {
            // 中文：保护终点朝向时，直接采用插件入口保存的 end_dir。
            last_dir = end_dir;
          } else {
            // 中文：不保护终点朝向时，根据最后一段位移和运动方向计算终点切线。
            last_dir = (last - prelast).block<2, 1>(0, 0) * last[2];
            last_dir.normalize();
          }
          double last_angle = atan2(last_dir[1], last_dir[0]);

          // Interpolate poses between prelast and last
          // 中文：上采样因子决定每个保留点区间内需要插入多少个贝塞尔中间点。
          int interp_cnt = (last_i - prelast_i) * params.path_upsampling_factor - 1;
          if (interp_cnt > 0) {
            Eigen::Vector2d last_pt = last.block<2, 1>(0, 0);
            Eigen::Vector2d prelast_pt = prelast.block<2, 1>(0, 0);
            double dist = (last_pt - prelast_pt).norm();
            Eigen::Vector2d pt1 = prelast_pt + prelast_dir * dist * 0.4 * prelast[2];
            Eigen::Vector2d pt2 = last_pt - last_dir * dist * 0.4 * prelast[2];
            // 中文：控制点沿前后切线放置，0.4 倍距离用于在连续切线与不过度弯折之间取折中。
            for (int j = 1; j <= interp_cnt; j++) {
              double interp = j / static_cast<double>(interp_cnt + 1);
              Eigen::Vector2d pt = cubicBezier(prelast_pt, pt1, pt2, last_pt, interp);
              path.emplace_back(pt[0], pt[1], 0.0);
            }
          }
          path.emplace_back(last[0], last[1], last_angle);

          // Assign orientations to interpolated points
          // 中文：位置插值完成后重新计算中间点切线，避免所有新点错误继承同一个 yaw。
          for (size_t j = path.size() - 1 - interp_cnt; j < path.size() - 1; j++) {
            Eigen::Vector2d tangent_dir = tangentDir<double>( path[j - 1].block<2, 1>(0, 0), path[j].block<2, 1>(0, 0), path[j + 1].block<2, 1>(0, 0), false); tangent_dir = tangent_dir.dot((path[j + 1] - path[j]).block<2, 1>(0, 0) * prelast[2]) >= 0 ? tangent_dir : -tangent_dir;
            path[j][2] = atan2(tangent_dir[1], tangent_dir[0]);
          }

          prelast_dir = last_dir;
        } else {  // start pose
          // 中文：第一次进入时写入起点；根据参数决定保留入口朝向或沿第一段运动方向对齐。
          auto & start = path_optim[0];
          Eigen::Vector2d dir = params.keep_start_orientation ? start_dir : ((path_optim[i] - start).block<2, 1>(0, 0) * start[2]).normalized();
          path.emplace_back(start[0], start[1], atan2(dir[1], dir[0]));
          prelast_dir = dir;
        }
        prelast_i = last_i;
        last_i = i;
      }
    }
  }

  /*
    Piecewise cubic bezier curve as defined by Adobe in Postscript
    The two end points are pt0 and pt3
    Their associated control points are pt1 and pt2
    * 中文：使用两个端点和两个控制点计算三次贝塞尔曲线上的参数位置。
  */
  static Eigen::Vector2d cubicBezier( Eigen::Vector2d & pt0, Eigen::Vector2d & pt1, Eigen::Vector2d & pt2, Eigen::Vector2d & pt3, double mu) {
    // 中文：把控制点转换为多项式系数，再按 mu∈[0,1] 求出插值坐标。
    Eigen::Vector2d a, b, c, pt;

    c[0] = 3 * (pt1[0] - pt0[0]);
    c[1] = 3 * (pt1[1] - pt0[1]);
    b[0] = 3 * (pt2[0] - pt1[0]) - c[0];
    b[1] = 3 * (pt2[1] - pt1[1]) - c[1];
    a[0] = pt3[0] - pt0[0] - c[0] - b[0];
    a[1] = pt3[1] - pt0[1] - c[1] - b[1];

    pt[0] = a[0] * mu * mu * mu + b[0] * mu * mu + c[0] * mu + pt0[0];
    pt[1] = a[1] * mu * mu * mu + b[1] * mu * mu + c[1] * mu + pt0[1];

    return pt;
  }

  // 中文：调试开关决定 Ceres 是否输出每次迭代的进度。
  bool debug_;
  // 中文：保存初始化阶段配置好的求解器类型、阈值、迭代和时间上限。
  ceres::Solver::Options options_;
  // 中文：持有当前 Costmap 网格，保证插值器在整个优化问题生命周期内引用有效数据。
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_
