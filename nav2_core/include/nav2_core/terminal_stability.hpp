#ifndef NAV2_CORE__TERMINAL_STABILITY_HPP_
#define NAV2_CORE__TERMINAL_STABILITY_HPP_
#include <cmath>
namespace nav2_core
{
class TerminalStability
{
public:
  void reset()
  {
    active_ = false;
  }
  bool observe(bool valid, double now, double duration)
  {
    if (!valid || !std::isfinite(now) || !std::isfinite(duration) || duration <= 0.0)
    {
      reset();
      return false;
    }
    if (!active_ || now < last_ || now - last_ > 0.2)
    {
      start_ = now;
      active_ = true;
    }
    last_ = now;
    return now - start_ >= duration;
  }
private:
  bool active_ = false;
  double start_ = 0.0;
  double last_ = 0.0;
};
}
#endif
