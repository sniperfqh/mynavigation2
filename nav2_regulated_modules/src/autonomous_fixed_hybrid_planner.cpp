// 仅 autonomous_fixed 使用：一次 Hybrid A*，冻结前消除格心偏差；不追加连接线或重新搜索。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{
class AutonomousFixedHybridPlanner : public nav2_smac_planner::SmacPlannerHybrid
{
public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override
  {
    const auto began = std::chrono::steady_clock::now();
    auto plan = nav2_smac_planner::SmacPlannerHybrid::createPlan(start, goal);
    if (plan.poses.empty())
    {
      return plan;
    }
    // Base search releases these locks before returning. Keep validation and correction atomic
    // with respect to costmap updates and the inherited planner's parameter callback.
    std::lock_guard<std::mutex> planner_lock(_mutex);
    auto * costmap = _costmap_ros->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getMutex());
    if (!_costmap_ros->isCurrent())
    {
      throw nav2_core::PlannerException("Fixed hybrid correction requires a current costmap");
    }
    const double dx = goal.pose.position.x - plan.poses.back().pose.position.x;
    const double dy = goal.pose.position.y - plan.poses.back().pose.position.y;
    const double offset = std::hypot(dx, dy);
    const double resolution = costmap->getResolution();
    const double limit = std::min(0.03, std::sqrt(0.5) * resolution + _tolerance + 1e-6);
    if (!std::isfinite(offset) || !std::isfinite(resolution) || resolution <= 0.0 || offset > limit)
    {
      throw nav2_core::PlannerException("Fixed hybrid grid-center correction exceeds its bounded offset");
    }
    if (plan.poses.size() == 1 && offset > 1e-6)
    {
      throw nav2_core::PlannerException("Same-cell hybrid result has no translation segment for precise arrival");
    }
    // Rigid translation preserves curvature, yaw, path length and every direction-change cusp.
    for (auto & pose : plan.poses)
    {
      pose.pose.position.x += dx;
      pose.pose.position.y += dy;
    }
    const auto footprint = _costmap_ros->getRobotFootprint();
    if (footprint.size() < 3)
    {
      throw nav2_core::PlannerException("Fixed hybrid correction requires a polygon footprint");
    }
    double radius = 0.0;
    for (const auto & vertex : footprint)
    {
      radius = std::max(radius, std::hypot(vertex.x, vertex.y));
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
    const auto validate = [&](const double x, const double y, const double yaw)
    {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      if (elapsed >= _max_planning_time)
      {
        throw nav2_core::PlannerException("Fixed hybrid search and correction exceeded the planning deadline");
      }
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !costmap->worldToMap(x, y, mx, my) || costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        throw nav2_core::PlannerException("Fixed hybrid corrected center is blocked, unknown or outside the costmap");
      }
      const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
      if (cost < 0.0 || !std::isfinite(cost) || cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        throw nav2_core::PlannerException("Fixed hybrid corrected footprint is blocked or outside the costmap");
      }
    };
    // Validate the actual start and swept alignment to the shifted first sample; do not add
    // that check interval to the returned path or create any new tracking geometry.
    auto previous = start;
    validate(previous.pose.position.x, previous.pose.position.y, tf2::getYaw(previous.pose.orientation));
    const double sample_step = std::min(0.005, resolution * 0.5);
    for (const auto & pose : plan.poses)
    {
      const double x0 = previous.pose.position.x;
      const double y0 = previous.pose.position.y;
      const double yaw0 = tf2::getYaw(previous.pose.orientation);
      const double yaw_delta = std::remainder(tf2::getYaw(pose.pose.orientation) - yaw0, 2.0 * M_PI);
      const double distance = std::hypot(pose.pose.position.x - x0, pose.pose.position.y - y0);
      const double swept_distance = distance + radius * std::abs(yaw_delta);
      if (!std::isfinite(swept_distance))
      {
        throw nav2_core::PlannerException("Fixed hybrid path contains a non-finite pose");
      }
      const auto steps = static_cast<std::size_t>(std::max(1.0, std::ceil(swept_distance / sample_step)));
      for (std::size_t i = 1; i <= steps; ++i)
      {
        const double t = static_cast<double>(i) / steps;
        validate(x0 + t * (pose.pose.position.x - x0), y0 + t * (pose.pose.position.y - y0), yaw0 + t * yaw_delta);
      }
      previous = pose;
    }
    RCLCPP_INFO(_logger, "Fixed hybrid goal correction before freeze: dx=%.6f dy=%.6f m, poses=%zu, searches=1", dx, dy, plan.poses.size());
    return plan;
  }
};
}  // namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousFixedHybridPlanner, nav2_core::GlobalPlanner)
