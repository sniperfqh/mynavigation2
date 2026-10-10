#ifndef NAV2_REGULATED_MODULES__DETAIL__PATH_SAMPLE_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__PATH_SAMPLE_HPP_

#include <cmath>
#include <utility>

namespace nav2_regulated_modules
{
namespace detail
{
// 与切线判定采用相同容差；末点取新样本，保留精确业务终点。
template<typename Sequence, typename Pose>
void appendPathSample(Sequence & samples, Pose pose)
{
  if (!samples.empty())
  {
    const auto & previous = samples.back().pose.position;
    const auto & next = pose.pose.position;
    if (std::hypot(next.x - previous.x, next.y - previous.y) <= 1e-9)
    {
      samples.back() = std::move(pose);
      return;
    }
  }
  samples.push_back(std::move(pose));
}
}
}

#endif
