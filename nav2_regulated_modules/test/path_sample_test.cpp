#include <cmath>
#include <vector>

#include "nav2_regulated_modules/detail/path_sample.hpp"

struct Sample
{
  struct Pose
  {
    struct Point
    {
      double x;
      double y;
    } position;
  } pose;
};

Sample makeSample(double x, double y)
{
  Sample sample;
  sample.pose.position.x = x;
  sample.pose.position.y = y;
  return sample;
}

int main()
{
  std::vector<Sample> samples;
  nav2_regulated_modules::detail::appendPathSample(samples, makeSample(0.0, 0.0));
  nav2_regulated_modules::detail::appendPathSample(samples, makeSample(2.0000000000000004, 0.0));
  nav2_regulated_modules::detail::appendPathSample(samples, makeSample(2.0, 0.0));
  if (samples.size() != 2 || samples.back().pose.position.x != 2.0)
  {
    return 1;
  }
  nav2_regulated_modules::detail::appendPathSample(samples, makeSample(2.0, 1e-8));
  if (samples.size() != 3)
  {
    return 2;
  }
  for (std::size_t i = 1; i < samples.size(); ++i)
  {
    if (std::hypot(samples[i].pose.position.x - samples[i - 1].pose.position.x, samples[i].pose.position.y - samples[i - 1].pose.position.y) <= 1e-9)
    {
      return 3;
    }
  }
  return 0;
}
