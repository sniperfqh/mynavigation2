#ifndef NAV2_REGULATED_MODULES__DETAIL__TF_TIMING_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TF_TIMING_HPP_

#include <algorithm>
#include <cstdint>
#include <limits>

namespace nav2_regulated_modules::detail
{
// 时间戳差只代表观察端时钟下的年龄，不能单凭它区分发布延迟与跨机时钟偏差。
// 接收间隔使用单调时钟；统计为观测窗口累计，静态 TF 不应用动态新鲜度判据。
struct TfTiming
{
  std::uint64_t samples = 0;
  std::uint64_t repeats = 0;
  std::uint64_t backwards = 0;
  std::uint64_t zero_stamps = 0;
  std::int64_t stamp_ns = 0;
  std::int64_t ros_ns = 0;
  std::int64_t wall_ns = 0;
  std::int64_t steady_ns = 0;
  double stamp_step_s = 0.0;
  double max_gap_s = 0.0;
  double min_arrival_age_s = std::numeric_limits<double>::infinity();
  double max_arrival_age_s = -std::numeric_limits<double>::infinity();

  void observe(const std::int64_t stamp, const std::int64_t ros, const std::int64_t wall, const std::int64_t steady)
  {
    if (samples > 0)
    {
      repeats += stamp == stamp_ns;
      backwards += stamp < stamp_ns;
      stamp_step_s = static_cast<double>(stamp - stamp_ns) * 1e-9;
      max_gap_s = std::max(max_gap_s, static_cast<double>(steady - steady_ns) * 1e-9);
    }
    ++samples;
    zero_stamps += stamp == 0;
    stamp_ns = stamp;
    ros_ns = ros;
    wall_ns = wall;
    steady_ns = steady;
    const double age = static_cast<double>(ros - stamp) * 1e-9;
    min_arrival_age_s = std::min(min_arrival_age_s, age);
    max_arrival_age_s = std::max(max_arrival_age_s, age);
  }

  void resetWindow()
  {
    max_gap_s = 0.0;
    min_arrival_age_s = std::numeric_limits<double>::infinity();
    max_arrival_age_s = -std::numeric_limits<double>::infinity();
  }
};
}

#endif
