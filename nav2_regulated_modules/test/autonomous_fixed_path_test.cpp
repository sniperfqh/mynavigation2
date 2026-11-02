#include "nav2_regulated_modules/detail/autonomous_fixed_path.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
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

geometry_msgs::msg::PoseStamped point(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(yaw * 0.5);
  result.pose.orientation.w = std::cos(yaw * 0.5);
  return result;
}
}

int main()
{
  using namespace nav2_regulated_modules::detail;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses =
  {
    point(0, 0), point(0.1, 0), point(0.2, 0), point(0.1, 0), point(0, 0), point(0.1, 0)
  };
  const auto segments = splitFixedPath(path);
  require(segments.size() == 3, "two cusps split into three fixed segments");
  require(segments[0].direction == 1 && segments[1].direction == -1 && segments[2].direction == 1, "forward reverse forward");
  require(segments[0].last == segments[1].first && segments[1].last == segments[2].first, "cusps shared without a connector");
  require(advanceFixedCursor(path, segments[0], 0, point(0.1, 0)) == 1, "cursor remains in active segment");
  require(advanceFixedCursor(path, segments[0], 1, point(0, 0)) >= 1, "cursor never goes backwards");
  require(advanceFixedCursor(path, segments[0], 0, point(0.2, 0), 0.11) == 1, "arc window prevents skipping ahead");
  const auto hash = fixedPathFingerprint(path);
  path.header.stamp.sec = 100;
  require(hash == fixedPathFingerprint(path), "publication time does not change frozen geometry identity");
  path.poses[0].pose.orientation.z = 0.01;
  require(hash != fixedPathFingerprint(path), "pose orientation is part of frozen geometry");
  path.poses =
  {
    point(0, 0), point(0, 0), point(0.1, 0)
  };
  require(splitFixedPath(path).size() == 1, "duplicate point is not a cusp");
  path.poses =
  {
    point(0, 0), point(0, 0, 1.0)
  };
  require(splitFixedPath(path)[0].rotation, "in place orientation change retained");
  path.poses =
  {
    point(0, 0)
  };
  require(splitFixedPath(path)[0].rotation, "one point is stationary acceptance only");
  for (int invalid = 0; invalid < 4; ++invalid)
  {
    path.poses =
    {
      point(0, 0), point(0.1, 0)
    };
    if (invalid == 0)
    {
      path.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    }
    if (invalid == 1)
    {
      path.poses[0].header.frame_id = "odom";
    }
    if (invalid == 2)
    {
      path.poses[0] = point(0, 0, 1.5707963267948966);
    }
    if (invalid == 3)
    {
      path.poses[0].pose.orientation.w = 0.0;
    }
    bool rejected = false;
    try
    {
      splitFixedPath(path);
    }
    catch (const std::invalid_argument &)
    {
      rejected = true;
    }
    require(rejected, "invalid geometry rejected before motion");
  }
  std::optional<double> since;
  require(!autonomousSettled(true, 1.0, 0.5, since), "strict acceptance starts timer");
  require(!autonomousSettled(false, 1.4, 0.5, since), "drift resets timer");
  require(!autonomousSettled(true, 2.0, 0.5, since), "no two second fallback success");
  require(autonomousSettled(true, 2.5, 0.5, since), "continuous acceptance completes");
  std::cout << "Frozen path segmentation, direction, cursor, validation and identity checks passed\n";
}
