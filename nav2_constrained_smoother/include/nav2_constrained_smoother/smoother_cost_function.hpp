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

#ifndef NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Core"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_constrained_smoother/options.hpp"
#include "nav2_constrained_smoother/utils.hpp"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_constrained_smoother::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, distance from original and obstacle avoidance.
 * 中文：每个 Ceres 残差块同时评估平滑性、最大曲率、原路径距离和 Costmap 避障四项代价。
 */
class SmootherCostFunction
{
public:
  /**
   * @brief A constructor for nav2_constrained_smoother::SmootherCostFunction
   * @param original_path Original position of the path node
   * @param next_to_last_length_ratio Ratio of next path segment compared to previous.
   *  Negative if one of them represents reversing motion.
   * @param reversing Whether the path segment after this node represents reversing motion.
   * @param costmap A costmap to get values for collision and obstacle avoidance
   * @param params Optimization weights and parameters
   * @param costmap_weight Costmap cost weight. Can be params.costmap_weight or params.cusp_costmap_weight
   * 中文：构造函数缓存原始点、前后段长度比例、运动方向、Costmap 原点／分辨率和双三次插值器，
   *      避免 Ceres 每次求值时重复读取静态环境信息。
   */
  SmootherCostFunction( const Eigen::Vector2d & original_pos, double next_to_last_length_ratio, bool reversing, const nav2_costmap_2d::Costmap2D * costmap, const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator, const SmootherParams & params, double costmap_weight) : original_pos_(original_pos), next_to_last_length_ratio_(next_to_last_length_ratio), reversing_(reversing), params_(params), costmap_weight_(costmap_weight), costmap_origin_(costmap->getOriginX(), costmap->getOriginY()), costmap_resolution_(costmap->getResolution()), costmap_interpolator_(costmap_interpolator) {
  }

  ceres::CostFunction * AutoDiff() {
    // 中文：把当前四维残差模型包装成 Ceres 自动求导代价函数，三个参数块分别是前、当前、后位置。
    return new ceres::AutoDiffCostFunction<SmootherCostFunction, 4, 2, 2, 2>(this);
  }

  void setCostmapWeight(double costmap_weight) {
    // 中文：cusp 区域识别后动态调整该残差的权重，使方向切换附近更重视避障。
    costmap_weight_ = costmap_weight;
  }

  double getCostmapWeight() {
    // 中文：返回当前代价地图权重，供 Smoother 更新 cusp 区域的线性过渡权重。
    return costmap_weight_;
  }

  /**
   * @brief Smoother cost function evaluation
   * @param pt X, Y coords of current point
   * @param pt_next X, Y coords of next point
   * @param pt_prev X, Y coords of previous point
   * @param pt_residual array of output residuals (smoothing, curvature, distance, cost)
   * @return if successful in computing values
   * 中文：Ceres 每次改变三个二维位置参数时调用该函数，输出四个可微残差并始终返回成功。
   */
  template<typename T>
  bool operator()( const T * const pt, const T * const pt_next, const T * const pt_prev, T * pt_residual) const {
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi(pt);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(pt_next);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(pt_prev);
    Eigen::Map<Eigen::Matrix<T, 4, 1>> residual(pt_residual);
    residual.setZero();

    // compute cost
    // 中文：残差顺序固定为平滑、曲率、原路径距离、Costmap 代价，必须与 AutoDiff 的维度一致。
    addSmoothingResidual<T>(params_.smooth_weight, xi, xi_next, xi_prev, residual[0]);
    addCurvatureResidual<T>(params_.curvature_weight, xi, xi_next, xi_prev, residual[1]);
    addDistanceResidual<T>( params_.distance_weight, xi, original_pos_.template cast<T>(), residual[2]);
    addCostResidual<T>(costmap_weight_, xi, xi_next, xi_prev, residual[3]);

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   * 中文：通过前后位移与长度比例的差异惩罚方向突变，鼓励相邻线段形成连续切向。
   */
  template<typename T> inline void addSmoothingResidual( const double & weight, const Eigen::Matrix<T, 2, 1> & pt, const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev, T & r) const {
    Eigen::Matrix<T, 2, 1> d_next = pt_next - pt;
    Eigen::Matrix<T, 2, 1> d_prev = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d_diff = next_to_last_length_ratio_ * d_next - d_prev;
    // 中文：长度比例补偿不同采样间距，避免仅按点坐标差把不等距路径误判为不平滑。
    r += (T)weight * d_diff.dot(d_diff);    // objective function value
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   * 中文：使用三点圆弧半径计算当前曲率，只惩罚超过 max_curvature 的部分。
   */
  template<typename T> inline void addCurvatureResidual( const double & weight, const Eigen::Matrix<T, 2, 1> & pt, const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev, T & r) const { Eigen::Matrix<T, 2, 1> center = arcCenter( pt_prev, pt, pt_next, next_to_last_length_ratio_ < 0);
    if (CERES_ISINF(center[0])) {
      // 中文：直线或退化转向没有有限圆心，不产生曲率惩罚。
      return;
    }
    T turning_rad = (pt - center).norm();
    T ki_minus_kmax = (T)1.0 / turning_rad - params_.max_curvature;

    if (ki_minus_kmax <= (T)EPSILON) {
      // 中文：当前曲率未超过上限时残差为零，避免对可行路径施加额外代价。
      return;
    }

    r += (T)weight * ki_minus_kmax * ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   * 中文：惩罚优化点偏离原始规划路径的距离，权重为零时允许路径完全由其他代价决定。
   */
  template<typename T> inline void addDistanceResidual( const double & weight, const Eigen::Matrix<T, 2, 1> & xi, const Eigen::Matrix<T, 2, 1> & xi_original, T & r) const {
    // 中文：使用平方欧氏距离而不是距离本身，保持残差对坐标连续可导并便于自动求导。
    r += (T)weight * (xi - xi_original).squaredNorm();  // objective function value
  }

  /**
   * @brief Cost function term for steering away from costs
   * @param weight Weight to apply to function
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param r Residual (cost) of term
   * 中文：从 Costmap 插值获取当前位置或机器人足迹采样点的代价，构成障碍物远离约束。
   */
  template<typename T> inline void addCostResidual( const double & weight, const Eigen::Matrix<T, 2, 1> & pt, const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev, T & r) const {
    if (params_.cost_check_points.empty()) {
      // 中文：没有自定义采样点时只查询机器人参考点的 Costmap 代价。
      Eigen::Matrix<T, 2, 1> interp_pos = (pt - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
      T value;
      costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);
      r += (T)weight * value * value;  // objective function value
    } else {
      // 中文：有采样点时先建立机器人局部坐标到世界坐标的刚体变换，再逐点加权查询代价。
      Eigen::Matrix<T, 2, 1> dir = tangentDir( pt_prev, pt, pt_next, next_to_last_length_ratio_ < 0);
      dir.normalize();
      if (((pt_next - pt).dot(dir) < (T)0) != reversing_) {
        // 中文：切向量的正负必须与当前前进／倒退运动方向一致，否则足迹采样会朝向相反一侧。
        dir = -dir;
      }
      Eigen::Matrix<T, 3, 3> transform;
      transform << dir[0], -dir[1], pt[0], dir[1], dir[0], pt[1], (T)0, (T)0, (T)1;
      for (size_t i = 0; i < params_.cost_check_points.size(); i += 3) {
        // 中文：每组三元组依次是机器人坐标 x、y 和该采样点的归一化权重。
        Eigen::Matrix<T, 3, 1> ccpt((T)params_.cost_check_points[i], (T)params_.cost_check_points[i + 1], (T)1);
        auto ccpt_world = (transform * ccpt).template block<2, 1>(0, 0);
        Eigen::Matrix<T, 2, 1> interp_pos = (ccpt_world - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
        T value;
        costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);

        r += (T)weight * (T)params_.cost_check_points[i + 2] * value * value;
      }
    }
  }

  // 中文：该残差点对应的原始二维位置，用于计算距离约束。
  const Eigen::Vector2d original_pos_;
  // 中文：前后线段长度比例，负号表示经过 cusp 后运动方向发生反转。
  double next_to_last_length_ratio_;
  // 中文：当前点之后的线段是否按倒退方向运动。
  bool reversing_;
  // 中文：复制保存本次优化使用的权重、曲率上限和足迹采样参数。
  SmootherParams params_;
  // 中文：当前点的 Costmap 权重，cusp 区域会在构建问题时动态调整。
  double costmap_weight_;
  // 中文：Costmap 世界坐标原点，用于把世界点转换到插值网格坐标。
  Eigen::Vector2d costmap_origin_;
  // 中文：Costmap 单元分辨率，用于世界坐标到栅格索引的换算。
  double costmap_resolution_;
  // 中文：Ceres 双三次插值器，允许优化点在栅格单元之间连续读取代价。
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> costmap_interpolator_;
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
