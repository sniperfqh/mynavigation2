#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"

namespace
{

void require(const bool valid, const char * reason)
{
  if (!valid)
  {
    throw std::runtime_error(reason);
  }
}

}  // namespace

int main()
{
  using namespace nav2_costmap_2d;
  Costmap2D map(400, 400, 0.005, -1.0, -1.0, FREE_SPACE);
  FootprintCollisionChecker<Costmap2D *> checker(&map);
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.2;
  footprint[0].y = 0.05;
  footprint[1].x = 0.2;
  footprint[1].y = -0.05;
  footprint[2].x = -0.2;
  footprint[2].y = -0.05;
  footprint[3].x = -0.2;
  footprint[3].y = 0.05;
  const auto free = [&checker, &footprint](const double x, const double y, const double heading)
  {
    const double cost = checker.footprintCostAtPose(x, y, heading, footprint);
    return cost >= 0.0 && cost < LETHAL_OBSTACLE;
  };
  const auto obstacle = [&map](const double x0, const double x1, const double y0, const double y1, const unsigned char cost)
  {
    for (double x = x0; x <= x1; x += 0.004)
    {
      for (double y = y0; y <= y1; y += 0.004)
      {
        unsigned int mx = 0;
        unsigned int my = 0;
        require(map.worldToMap(x, y, mx, my), "obstacle is within test map");
        map.setCost(mx, my, cost);
      }
    }
  };
  const auto preview = [&free](const double v, const double w, const double command_v, const double command_w)
  {
    return nav2_regulated_modules::detail::autonomousTerminalTrajectory(0.0, 0.0, 0.0, v, w, command_v, command_w, 0.8, 0.02, free);
  };
  require(preview(0.0, 0.0, 0.05, 0.0), "free translation allowed");
  require(preview(0.0, 0.0, 0.0, 0.3), "free rotation allowed");
  obstacle(0.225, 0.24, -0.08, 0.08, LETHAL_OBSTACLE);
  require(free(0.0, 0.0, 0.0), "translation obstacle outside initial footprint");
  require(!preview(0.0, 0.0, 0.05, 0.0), "forward trajectory obstacle rejected");
  std::fill_n(map.getCharMap(), 400 * 400, FREE_SPACE);
  obstacle(0.12, 0.25, 0.075, 0.20, LETHAL_OBSTACLE);
  require(free(0.0, 0.0, 0.0), "rotation obstacle outside initial footprint");
  require(!preview(0.0, 0.3, 0.0, 0.3), "rotating corner obstacle rejected");
  std::fill_n(map.getCharMap(), 400 * 400, FREE_SPACE);
  obstacle(0.28, 0.30, -0.08, 0.08, LETHAL_OBSTACLE);
  require(!preview(0.2, 0.0, 0.0, 0.0), "zero command still checks measured-speed braking path");
  std::fill_n(map.getCharMap(), 400 * 400, FREE_SPACE);
  obstacle(0.225, 0.24, -0.08, 0.08, NO_INFORMATION);
  require(!preview(0.0, 0.0, 0.05, 0.0), "unknown footprint rejected when tracking unknown");
  require(!nav2_regulated_modules::detail::autonomousTerminalTrajectory(0.85, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.8, 0.02, free), "map boundary rejected");
  std::cout << "Autonomous real costmap footprint translation/rotation/braking/unknown/boundary checks passed; no ROS nodes created\n";
}
