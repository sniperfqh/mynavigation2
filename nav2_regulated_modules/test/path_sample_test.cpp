// 最小 Pose 结构上的采样去重回归，检查浮点近重复终点被精确新终点替换。
// 小于等于 1e-9 m 的邻点会合并，仍相隔 1e-8 m 的点必须保留，以避免误删有效路径段。
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

// 测试辅助函数：将断言失败显式转成 false／异常，或构造最小采样结构；不依赖 ROS。
Sample makeSample(double x, double y)
{
  Sample sample;
  sample.pose.position.x = x;
  sample.pose.position.y = y;
  return sample;
}

// 最小 Pose 结构上的采样去重回归，检查浮点近重复终点被精确新终点替换。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
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
