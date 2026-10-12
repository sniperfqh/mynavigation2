#ifndef NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#include <algorithm>
namespace nav2_regulated_modules
{
namespace detail
{
inline double terminalLookahead(double normal, double terminal, double distance)
{
  const double blend = std::clamp((0.5 - distance) / 0.4, 0.0, 1.0);
  return normal + blend * (terminal - normal);
}
}
}
#endif
