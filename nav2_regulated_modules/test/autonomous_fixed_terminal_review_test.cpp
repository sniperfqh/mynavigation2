// 确定性状态回归；12 mm 为构造超差值，不是上轮未记录的失败帧测量。
#include "nav2_regulated_modules/detail/autonomous_fixed_terminal_review.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_terminal_trim.hpp"
#include <iostream>
#include <limits>

namespace
{
void require(const bool value, const char * message)
{
  if (!value)
  {
    throw std::runtime_error(message);
  }
}
}

int main()
{
  using namespace nav2_regulated_modules::detail;
  using Phase = FixedReviewPhase;
  FixedTerminalReview state;
  const auto tick = [&state](const double time, const double xy, const bool stopped = true, const double yaw = 0.0)
  {
    return reviewFixedTerminal(state, xy, yaw, stopped, time, 0.01, 0.08726646259971647);
  };
  require(tick(0.0, 0.0075, false) == Phase::BRAKE, "lock is a brake request, not stopped acceptance");
  require(tick(0.06, 0.012, false) == Phase::BRAKE && !state.failed(), "transient overshoot while braking must not reproduce immediate FAILED");
  for (int i = 12; i <= 31; ++i)
  {
    require(tick(i * 0.01, 0.006) == Phase::BRAKE, "actual stopped feedback must hold for 0.2s first");
  }
  for (int i = 33; i <= 82; ++i)
  {
    require(tick(i * 0.01, 0.006) != Phase::READY, "arrival also needs a separate 0.5s pose hold");
  }
  require(tick(0.84, 0.006) == Phase::READY, "recovered 6mm position can pass unchanged 1cm tolerance");

  state.reset();
  for (int i = 0; i <= 25; ++i)
  {
    tick(i * 0.01, 0.0075);
  }
  require(tick(0.26, 0.012) == Phase::WAIT_POSITION, "stopped transient resets pose acceptance, not the task");
  for (int i = 27; i <= 40; ++i)
  {
    require(tick(i * 0.01, 0.012) == Phase::WAIT_POSITION, "capture margin is only zero-speed waiting");
  }
  for (int i = 41; i <= 90; ++i)
  {
    require(tick(i * 0.01, 0.006) != Phase::READY, "position recovery restarts continuous hold");
  }
  require(tick(0.93, 0.006) == Phase::READY, "settling resumes after a transient position excursion");

  state.reset();
  for (int i = 0; i <= 219; ++i)
  {
    const auto phase = tick(i * 0.01, 0.015);
    require(phase != Phase::READY && !state.failed(), "2s position review starts after confirmed braking");
  }
  require(tick(2.25, 0.015) == Phase::FAILED_POSITION_TIMEOUT, "persistent 15mm error fails instead of timeout success");
  require(tick(2.26, 0.006) == Phase::FAILED_POSITION_TIMEOUT, "irrecoverable error stays latched until a new plan");

  state.reset();
  bool ready = false;
  for (int i = 0; i <= 240 && !state.failed(); ++i)
  {
    const double xy = i < 25 || i % 15 < 5 ? 0.012 : 0.006;
    ready = ready || tick(i * 0.01, xy) == Phase::READY;
  }
  require(state.failed() && !ready, "repeated excursions cannot extend review forever or borrow disconnected valid samples");

  state.reset();
  for (int i = 0; i < 20; ++i)
  {
    require(tick(i * 0.01, 0.006, true, 1.0) == Phase::BRAKE, "rotation cannot start before braking completes");
  }
  require(tick(0.21, 0.006, true, 1.0) == Phase::ALIGN_GOAL, "rotation starts after continuous stopped feedback");
  for (int i = 22; i <= 300; ++i)
  {
    require(tick(i * 0.01, 0.006, false, 1.0) == Phase::ALIGN_GOAL, "2s position timeout does not cut off a valid long rotation");
  }
  require(tick(3.01, 0.006, false) == Phase::BRAKE, "rotation must stop before final pose acceptance");
  for (int i = 302; i <= 373; ++i)
  {
    const auto phase = tick(i * 0.01, 0.006);
    if (i < 372)
    {
      require(phase != Phase::READY, "post-rotation brake and settle intervals are independent");
    }
  }
  require(tick(3.76, 0.006) == Phase::READY, "aligned stationary pose eventually completes");

  state.reset();
  require(tick(0.0, 0.031, false) == Phase::FAILED_CAPTURE, "large deviation still stops and fails immediately");
  state.reset();
  tick(1.0, 0.006);
  tick(1.1, 0.006);
  require(tick(0.0, 0.006) == Phase::BRAKE, "clock rollback restarts continuous validation");
  state.reset();
  for (int i = 0; i < 65; ++i)
  {
    tick(i * 0.01, 0.006);
  }
  require(tick(1.0, 0.006) == Phase::BRAKE, "sample gap cannot supply missing acceptance time");
  state.reset();
  require(tick(2.0, 0.006) == Phase::BRAKE, "freshness loss reset also requires braking again");

  bool invalid = false;
  try
  {
    tick(0.0, std::numeric_limits<double>::quiet_NaN());
  }
  catch (const std::invalid_argument &)
  {
    invalid = true;
  }
  require(invalid, "NaN must not turn into a valid stopped pose");
  FixedTrimState trim;
  FixedTrimLimits limits;
  require(beginFixedTrim(trim, limits, 1.0), "a stopped persistent error can begin bounded trim");
  require(accountFixedTrim(trim, 1.1, 0.01) && std::abs(trim.distance - 0.001) < 1e-9, "physical feedback consumes distance without counting localization jumps");
  const double forward = fixedTrimLinear(trim, limits, 0.013904, 0.01, 0.0, 0.0, 0.0, 1, 0.02, 0.01);
  const double reverse = fixedTrimLinear(trim, limits, 0.013904, 0.01, 0.0, 0.0, 0.0, -1, 0.02, 0.01);
  require(forward > 0.0 && reverse < 0.0, "same frozen endpoint converges in its original driving direction");
  require(fixedTrimLinear(trim, limits, 0.013904, 0.01, 0.8, 0.0, 0.0, 1, 0.02, 0.01) == 0.0, "large lateral bearing needs stationary heading alignment first");
  require(fixedTrimLinear(trim, limits, 0.004, 0.01, 0.0, 0.0, 0.0, 1, 0.02, 0.01) == 0.0, "interior capture stops rather than driving across the endpoint");
  require(fixedTrimLinear(trim, limits, 0.02, 0.01, 0.0, 0.01, 0.0, 1, 0.002, 0.01) <= 0.002, "external speed limit is never raised to the nominal micro speed");
  require(!fixedTrimWithinBudget(trim, limits, 6.0), "five seconds is a failure bound, never a success fallback");
  trim.active = false;
  require(beginFixedTrim(trim, limits, 1.2), "one retry retains the original absolute deadline");
  trim.active = false;
  require(!beginFixedTrim(trim, limits, 1.3), "third trim attempt is refused");
  trim.reset();
  require(beginFixedTrim(trim, limits, 0.0), "new execution resets the trim budget");
  trim.distance = 0.03;
  require(!fixedTrimWithinBudget(trim, limits, 0.1), "three centimeter total displacement is a hard bound");
  trim.reset();
  beginFixedTrim(trim, limits, 0.0);
  trim.step_distance = 0.02;
  require(!fixedTrimWithinBudget(trim, limits, 0.1), "single attempt also has a two centimeter bound");
  trim.reset();
  beginFixedTrim(trim, limits, 1.0);
  require(!accountFixedTrim(trim, 0.9, 0.0) && !accountFixedTrim(trim, 1.3, 0.0), "rollback and feedback gaps cannot renew or invent trim budget");
  std::cout << "Terminal review: transient excursion, strict arrival, timeout failure, rotation, capture and timing checks passed\n";
}
