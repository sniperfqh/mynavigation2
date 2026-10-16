// 将“要求停车”与“位置准确”分开的纯位置判据。距离、验收容差和可选停车入口均为 m。
// 越过终点平面可触发停车，但不能据此证明精度合格；速度停稳和持续时间由调用方及控制服务器另行判断。
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

// entry>0 时以入口半径触发停车，否则采用 tolerance；crossed 与既有停止状态也可锁停。
// accurate 按 tolerance 判断；entry=0 且此前已锁停并准确时，继续保持准确标志。
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
