#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"

namespace
{

void require(const bool condition, const char * message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main()
{
  using namespace nav2_regulated_modules::detail;
  require(freshAutonomousTransform(0.3, 0.3), "fresh boundary");
  require(!freshAutonomousTransform(0.31, 0.3), "old source stamp rejected");
  require(!freshAutonomousTransform(-0.1, 0.3), "future source stamp rejected");
  require(!freshAutonomousTransform(std::numeric_limits<double>::quiet_NaN(), 0.3), "NaN rejected");
  require(autonomousFeedbackDistance(0.0, 0.12) == 0.12, "nearest end point cannot erase remaining XY gap");
  require(autonomousFeedbackDistance(4.0, 2.0) == 4.0, "longer curved-path distance retained");
  require(autonomousFeedbackDistance(std::numeric_limits<double>::quiet_NaN(), 0.2) == 0.2, "invalid child distance replaced with valid geometric lower bound");
  require(autonomousFeedbackDistance(0.0, 0.0) == 0.0, "true zero XY remains zero even when yaw still pending");
  require(autonomousFeedbackEta(0.0, 0.6, 0.0) > 2.0, "rotation-only ETA is not zero");
  require(autonomousFeedbackEta(0.1, 0.0, 0.01) > 10.0, "slow approach ETA uses observed speed");
  require(autonomousFeedbackEta(0.0, 0.0, 0.0) == 0.5, "stationary goal retains settle estimate");
  require(autonomousFeedbackEta(std::numeric_limits<double>::max(), 0.0, 0.0) < std::numeric_limits<int32_t>::max(), "ETA fits ROS Duration seconds");
  require(autonomousFeedbackRecoveries(1, 3) == 4, "localization recoveries included independently");
  require(autonomousFeedbackRecoveries(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()) == std::numeric_limits<int16_t>::max(), "feedback recovery count saturates safely");
  require(std::string(autonomousArrivalBlocker(false, true, true, false)) == "POSITION", "arrival reports pending position");
  require(std::string(autonomousArrivalBlocker(true, false, true, false)) == "HEADING", "arrival reports pending heading");
  require(std::string(autonomousArrivalBlocker(true, true, false, false)) == "VELOCITY", "arrival reports pending velocity");
  require(std::string(autonomousArrivalBlocker(true, true, true, false)) == "SETTLING", "arrival reports pending continuous hold");
  require(std::string(autonomousArrivalBlocker(true, true, true, true)) == "READY", "arrival ready requires all gates");
  require(autonomousProgress(0.06, 0.0, 0.0, 2.0, 0.05, 0.2, 0.01), "translation progress");
  require(autonomousProgress(0.0, 0.21, 0.0, 2.0, 0.05, 0.2, 0.01), "turning toward path counts");
  require(!autonomousProgress(0.0, -0.21, 0.0, 2.0, 0.05, 0.2, 0.01), "turning away is not progress");
  require(autonomousProgress(0.0, 0.0, 0.011, 0.08, 0.05, 0.2, 0.01), "terminal improvement counts");
  require(!autonomousProgress(0.0, 0.0, 0.011, 2.0, 0.05, 0.2, 0.01), "small distant drift does not count");
  double previous = 0.0;
  for (int index = 0; index <= 1200; ++index)
  {
    const double limit = autonomousApproachLimit(index / 1000.0, 0.03, 1.5, 1.0);
    require(limit >= previous - 1e-9 && limit <= 1.5, "approach constraint is bounded and continuous");
    previous = limit;
  }
  require(std::abs(autonomousApproachLimit(0.1, 0.03, 1.5, 1.0) - 0.05) < 1e-9, "last 10cm is slow");
  require(autonomousApproachLimit(0.02, 0.03, 0.02, 1.0) <= 0.02, "lower configured speed respected");
  require(autonomousHeadingCommand(0.05, 0.0, 0.5, 0.8, 0.01, 0.15) == 0.0, "terminal angular deadband stops");
  require(std::abs(autonomousHeadingCommand(1.0, 0.0, 0.5, 0.8, 0.01, 0.15) - 0.008) < 1e-9, "angular acceleration bounded");
  require(autonomousHeadingCommand(-1.0, 0.0, 0.5, 0.8, 0.01, 0.15) < 0.0, "turn direction respected");
  require(autonomousHeadingCommand(-0.3, 0.1, 0.5, 0.8, 0.02, 0.15) == 0.0, "overshoot brakes before reversing");
  require(autonomousHeadingCommand(0.11, 0.2, 0.5, 0.8, 0.02, 0.15) == 0.0, "reaction margin starts braking early");
  require(autonomousHeadingCommand(-0.3, 0.0, 0.5, 0.8, 0.02, 0.15) < 0.0, "reacquire after angular velocity settles");
  std::optional<double> since;
  require(!autonomousSettled(true, 1.0, 0.5, since), "settle begins");
  require(!autonomousSettled(true, 1.4, 0.5, since), "settle must be continuous");
  require(!autonomousSettled(false, 1.45, 0.5, since), "drift resets settle");
  require(!autonomousSettled(true, 2.0, 0.5, since), "new settle begins");
  require(autonomousSettled(true, 2.5, 0.5, since), "settle completes after 0.5s");
  require(!autonomousSettled(true, 1.0, 0.5, since), "clock rollback resets settle");
  using Phase = AutonomousTerminalPhase;
  AutonomousTerminalState terminal;
  const auto step = [&terminal](const double distance, const double bearing, const double goal_yaw, const double v, const double w, const double now, const double linear_cap = 0.05, const double angular_cap = 0.3)
  {
    return autonomousTerminalCommand(terminal, distance, bearing, goal_yaw, v, w, 0.03, 0.15, 0.01, 0.05, now, 0.15, linear_cap, angular_cap, 0.8, 0.02);
  };
  step(0.16, 0.0, 1.0, 0.0, 0.0, 1.0);
  require(terminal.phase == Phase::TRACK, "MPPI retained outside terminal entry");
  auto cmd = step(0.15, 1.0, 0.0, 0.0, 0.0, 1.02);
  require(terminal.phase == Phase::ALIGN_POSITION && cmd.linear == 0.0 && cmd.angular > 0.0, "position bearing aligned before translation");
  step(0.18, 0.2, 0.0, 0.0, 0.0, 1.04);
  require(terminal.phase == Phase::ALIGN_POSITION, "terminal exit has hysteresis");
  cmd = step(0.1, 0.1, 0.0, 0.0, 0.0, 1.06);
  require(terminal.phase == Phase::POSITION && cmd.linear > 0.0 && cmd.linear <= 0.05, "slow translation enabled after alignment");
  step(0.1, 0.3, 0.0, 0.0, 0.0, 1.08);
  require(terminal.phase == Phase::POSITION, "bearing noise does not chatter");
  step(0.1, 0.36, 0.0, 0.0, 0.0, 1.10);
  require(terminal.phase == Phase::ALIGN_POSITION, "large bearing error stops translation");
  cmd = step(0.014, 0.0, 1.0, 0.03, 0.0, 2.0);
  require(terminal.phase == Phase::BRAKE && cmd.linear == 0.0 && cmd.angular == 0.0, "capture brakes before final rotation");
  step(0.018, 0.0, 1.0, 0.0, 0.0, 2.1);
  step(0.018, 0.0, 1.0, 0.02, 0.0, 2.29);
  step(0.018, 0.0, 1.0, 0.0, 0.0, 2.3);
  require(terminal.phase == Phase::BRAKE, "velocity violation resets brake stability");
  cmd = step(0.018, 0.0, 1.0, 0.0, 0.0, 2.51);
  require(terminal.phase == Phase::ALIGN_GOAL && cmd.linear == 0.0 && cmd.angular > 0.0, "rotation starts after 0.2s stopped");
  cmd = step(0.019, 0.0, 0.08, 0.0, 0.04, 2.53);
  require(terminal.phase == Phase::SETTLE && cmd.angular == 0.0, "inner yaw band commands zero");
  step(0.022, 0.0, 0.14, 0.0, 0.0, 2.55);
  require(terminal.phase == Phase::SETTLE, "yaw within outer tolerance keeps zero output");
  step(0.022, 0.0, 0.16, 0.0, 0.0, 2.57);
  require(terminal.phase == Phase::ALIGN_GOAL, "yaw tolerance violation realigns");
  cmd = step(0.031, 0.0, 0.0, 0.0, 0.0, 2.59, 0.003, 0.02);
  require(terminal.phase == Phase::POSITION && cmd.linear > 0.0 && cmd.linear <= 0.003 && std::abs(cmd.angular) <= 0.02, "position drift reacquired and external low cap preserved");
  step(0.251, 0.0, 0.0, 0.0, 0.0, 2.61);
  require(terminal.phase == Phase::TRACK, "larger displacement returns to MPPI");
  require(!autonomousTerminalProgressHeading(0.05, 0.03, false), "position heading used while approaching");
  require(autonomousTerminalProgressHeading(0.014, 0.03, false), "goal heading used once captured");
  require(autonomousTerminalProgressHeading(0.029, 0.03, true), "goal progress phase has hysteresis");
  require(!autonomousTerminalProgressHeading(0.031, 0.03, true), "position violation restores bearing progress");
  bool rejected = false;
  try
  {
    step(0.1, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 3.0);
  }
  catch (const std::invalid_argument &)
  {
    rejected = true;
  }
  require(rejected, "invalid feedback cannot drive terminal controller");
  // 数值机器人只有低通执行器和位姿反馈；不是导航仿真或实车精度证明。
  const double initial_headings[] =
  {
    0.0, 1.5, -2.5
  };
  for (const double initial_heading : initial_headings)
  {
    terminal = AutonomousTerminalState();
    double x = -0.12;
    double y = 0.02;
    double heading = initial_heading;
    double v = 0.0;
    double w = 0.0;
    std::optional<double> settled;
    bool reached = false;
    for (int tick = 0; tick < 3000; ++tick)
    {
      const double t = tick * 0.02;
      const double distance = std::hypot(x, y);
      const double bearing = std::atan2(-y, -x) - heading;
      const double error = std::atan2(std::sin(bearing), std::cos(bearing));
      const double yaw_error = std::atan2(std::sin(0.7 - heading), std::cos(0.7 - heading));
      cmd = step(distance, error, yaw_error, v, w, t);
      v += (cmd.linear - v) * 0.1;
      w += (cmd.angular - w) * 0.1;
      x += v * std::cos(heading) * 0.02;
      y += v * std::sin(heading) * 0.02;
      heading += w * 0.02;
      if (autonomousSettled(distance <= 0.03 && std::abs(yaw_error) <= 0.15 && std::abs(v) <= 0.01 && std::abs(w) <= 0.05, t, 0.5, settled))
      {
        reached = true;
        break;
      }
    }
    require(reached, "terminal policy converges with actuator lag from multiple headings");
  }
  std::cout << "Autonomous TF/progress/terminal state, drift, limits and numerical convergence checks passed\n";
}
