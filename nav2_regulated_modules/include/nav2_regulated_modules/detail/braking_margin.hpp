// 固定路径制动裕量的纯函数。low/high 是距离（m），transition 是速度（m/s），speed 保留符号但插值使用其绝对值。
// 输出只是制动距离预留，不是速度命令；调用方负责参数校验与实际停车判定。
#ifndef NAV2_REGULATED_MODULES__DETAIL__BRAKING_MARGIN_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__BRAKING_MARGIN_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_regulated_modules
{
namespace detail
{
// 校验 0≤low≤high、transition>0 且全部有限；调用者据此拒绝非法配置。
inline bool validBrakingMargin(double low, double high, double transition)
{
  return std::isfinite(low) && std::isfinite(high) && std::isfinite(transition) && low >= 0.0 && high >= low && transition > 0.0;
}

// 裕量 = low + (high−low) × clamp(abs(speed)÷transition, 0, 1)。
// abs(speed) 为速度绝对值；未启用、选自适应分支或速度无效时返回 high，保留更大预留。
inline double brakingMargin(bool enabled, bool adaptive, double speed, double low, double high, double transition)
{
  if (!enabled || adaptive || !std::isfinite(speed))
  {
    return high;
  }
  return low + (high - low) * std::clamp(std::abs(speed) / transition, 0.0, 1.0);
}

// 返回 previous 与 target 的较小值，使既有制动阶段命令幅值不能重新上升。
// 两者按非负速度幅值传入，方向符号由上层另加。
inline double boundedBrakingCommand(double previous, double target)
{
  return std::min(previous, target);
}
}
}

#endif
