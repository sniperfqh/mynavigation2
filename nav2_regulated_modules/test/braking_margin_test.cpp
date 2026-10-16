// 制动裕量纯函数回归：参数非法值、边界插值、正负速度对称和制动命令单调性。
// 测试抛异常或返回非零表示失败，不建立 ROS 节点或物理停车证据。
#include "nav2_regulated_modules/detail/braking_margin.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

// 测试辅助函数：将断言失败显式转成 false／异常，或构造最小采样结构；不依赖 ROS。
void require(bool value)
{
  if (!value)
  {
    throw std::runtime_error("braking margin regression");
  }
}

// 制动裕量纯函数回归：参数非法值、边界插值、正负速度对称和制动命令单调性。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
int main()
{
  using namespace nav2_regulated_modules::detail;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  require(validBrakingMargin(.03, .1, .3));
  require(!validBrakingMargin(-.01, .1, .3));
  require(!validBrakingMargin(.2, .1, .3));
  require(!validBrakingMargin(.03, .1, 0.));
  require(!validBrakingMargin(nan, .1, .3));
  require(!validBrakingMargin(.03, infinity, .3));
  require(!validBrakingMargin(.03, .1, nan));
  require(brakingMargin(false, false, 0., .03, .1, .3) == .1);
  require(brakingMargin(true, true, 0., .03, .1, .3) == .1);
  require(brakingMargin(true, false, nan, .03, .1, .3) == .1);
  require(brakingMargin(true, false, infinity, .03, .1, .3) == .1);
  require(brakingMargin(true, false, 0., .03, .1, .3) == .03);
  require(std::abs(brakingMargin(true, false, .3, .03, .1, .3) - .1) < 1e-12);
  require(std::abs(brakingMargin(true, false, .299999, .03, .1, .3) - .1) < 1e-6);
  require(std::abs(brakingMargin(true, false, 1.5, .03, .1, .3) - .1) < 1e-12);
  double previous_margin = .03;
  for (int i = 0; i <= 150; ++i)
  {
    const double speed = i * .01;
    const double margin = brakingMargin(true, false, speed, .03, .1, .3);
    require(margin >= .03 && margin <= .1 + 1e-12 && margin >= previous_margin);
    require(margin == brakingMargin(true, false, -speed, .03, .1, .3));
    previous_margin = margin;
  }
  double command = .01;
  for (int i = 30; i >= 0; --i)
  {
    const double margin = brakingMargin(true, false, i * .01, .03, .1, .3);
    const double target = std::sqrt(2. * .25 * std::max(.08 - margin, 0.));
    const double next = boundedBrakingCommand(command, std::max(.01, target));
    require(next <= command && next == .01);
    command = next;
  }
  return 0;
}
