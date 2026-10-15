// collision 状态、接收超时和速度目标门控；不改变任务速度基准或 S 曲线历史。
#ifndef NAV2_UTIL__COLLISION_STATE_HPP_
#define NAV2_UTIL__COLLISION_STATE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav2_util
{

struct CollisionStateSnapshot
{
  uint8_t raw = 0;
  uint8_t effective = 0;
  double age = -1.0;
  bool received = false;
  bool expired = false;

  bool stopped() const
  {
    return effective <= 1;
  }

  double ratio() const
  {
    return effective == 4 ? 1.0 : effective == 3 ? 0.65 : effective == 2 ? 0.35 : 0.0;
  }

  const char * reason() const
  {
    return !received ? "initial" : raw > 4 ? "invalid" : expired ? "timeout" : "state";
  }
};

class CollisionState
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void configure(double timeout)
  {
    if (!std::isfinite(timeout) || timeout <= 0.0)
    {
      throw std::invalid_argument("collision_state_timeout must be finite and positive");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_ = timeout;
    received_ = false;
    raw_ = 0;
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_ = false;
    raw_ = 0;
  }

  void update(uint8_t state, TimePoint stamp = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_ = state;
    stamp_ = stamp;
    received_ = true;
  }

  CollisionStateSnapshot snapshot(TimePoint now = Clock::now()) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CollisionStateSnapshot result;
    result.raw = raw_;
    result.received = received_;
    if (received_)
    {
      result.age = std::chrono::duration<double>(now - stamp_).count();
      result.expired = result.age < 0.0 || result.age >= timeout_;
      result.effective = raw_ <= 4 && !result.expired ? raw_ : 0;
    }
    return result;
  }

private:
  mutable std::mutex mutex_;
  double timeout_ = 0.5;
  uint8_t raw_ = 0;
  bool received_ = false;
  TimePoint stamp_;
};

inline bool isCollisionStateParameter(const std::string & name)
{
  return name == "collision_state_enabled" || name == "collision_state_topic" || name == "collision_state_timeout";
}

// 按任务上限取小，保留方向；同步缩放转向，避免重复乘小已处于蠕动的指令。
inline std::array<double, 3> collisionVelocityTarget(const std::array<double, 3> & input, double task_speed, const CollisionStateSnapshot & state, const std::vector<double> & minimum, const std::vector<double> & maximum)
{
  std::array<double, 3> target{0.0, 0.0, 0.0};
  if (!std::isfinite(task_speed) || task_speed <= 0.0 || state.stopped())
  {
    return target;
  }
  const double hardware_limit = input[0] >= 0.0 ? maximum[0] : -minimum[0];
  const double cap = std::max(0.0, std::min(task_speed * state.ratio(), hardware_limit));
  const double scale = std::abs(input[0]) > 1e-9 ? std::min(1.0, cap / std::abs(input[0])) : state.ratio();
  for (std::size_t axis = 0; axis < target.size(); ++axis)
  {
    target[axis] = std::clamp(input[axis] * scale, minimum[axis], maximum[axis]);
  }
  return target;
}

// 闭环补偿不得突破门控后的输入目标；零目标始终为零。
inline double collisionFeedbackTarget(double corrected, double selected)
{
  return selected >= 0.0 ? std::clamp(corrected, 0.0, selected) : std::clamp(corrected, selected, 0.0);
}

}

#endif  // NAV2_UTIL__COLLISION_STATE_HPP_
