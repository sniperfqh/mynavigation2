// 终点前视距离混合的纯函数。normal、terminal、distance 均为 m。
// 在距终点 0.5 m 到 0.1 m 区间内线性过渡；输入合法性由调用方保证，不在此处判断到点成功。
#ifndef NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#include <algorithm>
namespace nav2_regulated_modules
{
namespace detail
{
// 混合比例 = clamp((0.5−distance)÷0.4, 0, 1)。
// 输出前视 = normal + 混合比例 × (terminal−normal)，所有距离单位为 m。
inline double terminalLookahead(double normal, double terminal, double distance)
{
  const double blend = std::clamp((0.5 - distance) / 0.4, 0.0, 1.0);
  return normal + blend * (terminal - normal);
}
}
}
#endif
