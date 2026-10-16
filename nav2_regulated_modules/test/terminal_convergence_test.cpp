// 终点稳定计时、停稳等待兜底与末段前视混合的纯回归。
// 时间推进、采样中断和回跳均构造为数值输入，不是执行机器人运动。
#include <cmath>
#include <iostream>
#include "nav2_core/terminal_stability.hpp"
#include "nav2_core/terminal_aware_controller.hpp"
#include "nav2_regulated_modules/detail/terminal_lookahead.hpp"

// 终点稳定计时、停稳等待兜底与末段前视混合的纯回归。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
int main()
{
  nav2_core::TerminalStability stable;
  for (int i = 0; i < 20; ++i)
  {
    if (stable.observe(true, i * 0.05, 1.0))
    {
      return 1;
    }
  }
  if (!stable.observe(true, 1.01, 1.0))
  {
    return 2;
  }
  stable.observe(false, 1.02, 1.0);
  if (stable.observe(true, 1.03, 1.0))
  {
    return 3;
  }
  if (stable.observe(true, 10.0, 1.0) || stable.observe(true, 0.0, 1.0))
  {
    return 4;
  }
  stable.reset();
  if (stable.observe(true, 20.0, 1.0))
  {
    return 5;
  }
  nav2_core::TerminalStopWait wait;
  // 未锁停、仍运动或速度反馈失效都不能开始成功兜底计时。
  if (wait.expired(false, 0.0) || wait.expired(false, 100.0))
  {
    return 8;
  }
  if (wait.expired(true, 100.0) || wait.expired(true, 101.999) || !wait.expired(true, 102.0))
  {
    return 9;
  }
  // 精度复核中断不影响独立等待；停稳条件丢失则重新计时。
  stable.observe(false, 102.0, 1.0);
  if (!wait.expired(true, 102.01) || wait.expired(false, 102.02) || wait.expired(true, 103.0) || !wait.expired(true, 105.0))
  {
    return 10;
  }
  wait.reset();
  if (wait.expired(true, 200.0) || wait.expired(true, 199.0) || wait.expired(true, 200.999) || !wait.expired(true, 201.0))
  {
    return 11;
  }
  using nav2_regulated_modules::detail::terminalLookahead;
  if (terminalLookahead(0.45, 0.05, 0.5) != 0.45 || std::abs(terminalLookahead(0.45, 0.05, 0.1) - 0.05) > 1e-12)
  {
    return 6;
  }
  double previous = 0.45;
  for (int i = 0; i <= 100; ++i)
  {
    const double value = terminalLookahead(0.45, 0.05, 0.5 - i * 0.005);
    if (value > previous + 1e-12 || value < 0.05 - 1e-12)
    {
      return 7;
    }
    previous = value;
  }
  return 0;
}
