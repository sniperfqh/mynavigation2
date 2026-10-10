#ifndef NAV2_REGULATED_MODULES__DETAIL__BRAKING_MARGIN_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__BRAKING_MARGIN_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_regulated_modules
{
namespace detail
{
inline bool validBrakingMargin(double low, double high, double transition)
{
  return std::isfinite(low) && std::isfinite(high) && std::isfinite(transition) && low >= 0.0 && high >= low && transition > 0.0;
}

inline double brakingMargin(bool enabled, bool adaptive, double speed, double low, double high, double transition)
{
  if (!enabled || adaptive || !std::isfinite(speed))
  {
    return high;
  }
  return low + (high - low) * std::clamp(std::abs(speed) / transition, 0.0, 1.0);
}

inline double boundedBrakingCommand(double previous, double target)
{
  return std::min(previous, target);
}
}
}

#endif
