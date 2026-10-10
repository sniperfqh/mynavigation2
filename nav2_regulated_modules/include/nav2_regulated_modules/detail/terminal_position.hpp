#ifndef NAV2_REGULATED_MODULES__DETAIL__TERMINAL_POSITION_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TERMINAL_POSITION_HPP_

namespace nav2_regulated_modules
{
namespace detail
{
struct TerminalPosition
{
  bool stopped;
  bool accurate;
};

// 入口半径用于预留停后变化空间，验收半径仍由 GoalChecker 提供。
inline TerminalPosition terminalPosition(double distance, double tolerance, double entry, bool crossed, bool was_stopped, bool was_accurate)
{
  const double stop_radius = entry > 0.0 ? entry : tolerance;
  TerminalPosition result;
  result.stopped = was_stopped || distance <= stop_radius || crossed;
  result.accurate = result.stopped && (distance <= tolerance || (entry == 0.0 && was_stopped && was_accurate));
  return result;
}
}
}

#endif
