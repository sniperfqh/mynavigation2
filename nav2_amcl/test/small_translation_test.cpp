#include <cmath>
#include <iostream>

#include "nav2_amcl/motion_model/differential_motion_model.hpp"

bool checkStep(double translation, double yaw, double rotation)
{
  nav2_amcl::DifferentialMotionModel model;
  model.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  pf_sample_t sample;
  sample.pose = pf_vector_zero();
  sample.pose.v[2] = yaw;
  pf_t filter;
  filter.current_set = 0;
  filter.sets[0].sample_count = 1;
  filter.sets[0].samples = &sample;
  pf_vector_t delta = pf_vector_zero();
  delta.v[0] = translation * std::cos(yaw);
  delta.v[1] = translation * std::sin(yaw);
  delta.v[2] = rotation;
  pf_vector_t pose = delta;
  pose.v[2] = yaw + rotation;
  model.odometryUpdate(&filter, pose, delta);
  const bool passed = std::hypot(sample.pose.v[0] - delta.v[0], sample.pose.v[1] - delta.v[1]) < 1e-9 && std::abs(nav2_amcl::angleutils::angle_diff(sample.pose.v[2], pose.v[2])) < 1e-9;
  if (!passed)
  {
    std::cerr << "translation=" << translation << ", yaw=" << yaw << ", predicted=" << sample.pose.v[0] << "," << sample.pose.v[1] << ", expected=" << delta.v[0] << "," << delta.v[1] << std::endl;
  }
  return passed;
}

int main()
{
  for (double yaw : {0.0, M_PI / 2.0, 2.557722257927678})
  {
    for (double translation : {-0.0101, -0.01, -0.0099, -0.001, 0.0, 0.001, 0.0099, 0.01, 0.0101})
    {
      if (!checkStep(translation, yaw, 0.0) || !checkStep(translation, yaw, 0.02))
      {
        return 1;
      }
    }
  }
  return 0;
}
