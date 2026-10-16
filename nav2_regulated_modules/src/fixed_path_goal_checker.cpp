// 固定路径终点检查实现。先判断位置容差或末段终点平面，再累计有限速度门限内的连续周期。
// “位置曾触发终点”与“本周期速度已足够小”是两层条件；本插件不执行独立一秒定位覆盖率验收。

#include "nav2_regulated_modules/fixed_path_goal_checker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

// 从插件参数读取位置／速度阈值及连续周期；位置容差须为有限正数，停稳速度须有限非负，周期至少为 1。
// position_stable_cycles 计的是连续控制检查次数，不是固定秒数。
void FixedPathGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::invalid_argument("FixedPathGoalChecker cannot lock lifecycle node");
  }
  plugin_name_ = plugin_name;
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".trans_stopped_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rot_stopped_velocity", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".position_stable_cycles", rclcpp::ParameterValue(10));
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name_ + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name_ + ".position_stable_cycles", position_stable_cycles_);
  if (!std::isfinite(xy_goal_tolerance_) || !std::isfinite(trans_stopped_velocity_) || !std::isfinite(rot_stopped_velocity_) || xy_goal_tolerance_ <= 0.0 || trans_stopped_velocity_ < 0.0 || rot_stopped_velocity_ < 0.0 || position_stable_cycles_ < 1)
  {
    throw std::invalid_argument("FixedPathGoalChecker parameters are invalid");
  }
  LOG_INFO("固定路径位置 GoalChecker 配置完成，plugin={}，位置容差={:.4f}m，停止线速度={:.3f}m/s，停止角速度={:.3f}rad/s，位置稳定周期={}", plugin_name_, xy_goal_tolerance_, trans_stopped_velocity_, rot_stopped_velocity_, position_stable_cycles_);
}

// 丢弃旧路径、末段切线、终点锁存和停稳计数，新任务不能继承这些历史。
void FixedPathGoalChecker::reset()
{
  path_ = nav_msgs::msg::Path();
  goal_tangent_index_ = 0;
  terminal_tangent_x_ = 0.0;
  terminal_tangent_y_ = 0.0;
  path_valid_ = false;
  terminal_reached_ = false;
  stopped_cycles_ = 0;
}

// 从最后一个有效非重合线段提取归一化切线，并清空先前到点状态。
// 重复末点会向前回退；全路径没有有效末段切线时拒绝。
void FixedPathGoalChecker::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2)
  {
    throw std::invalid_argument("FixedPathGoalChecker requires at least two path poses");
  }
  std::size_t goal_tangent_index = path.poses.size() - 2;
  const auto & goal = path.poses.back().pose.position;
  while (goal_tangent_index > 0)
  {
    const auto & candidate = path.poses[goal_tangent_index].pose.position;
    if (std::hypot(goal.x - candidate.x, goal.y - candidate.y) > 1e-6)
    {
      break;
    }
    --goal_tangent_index;
  }
  const auto & tangent_start = path.poses[goal_tangent_index].pose.position;
  const double tangent_x = goal.x - tangent_start.x;
  const double tangent_y = goal.y - tangent_start.y;
  const double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length <= 1e-6)
  {
    throw std::invalid_argument("FixedPathGoalChecker path has no valid terminal tangent");
  }
  path_ = path;
  goal_tangent_index_ = goal_tangent_index;
  terminal_tangent_x_ = tangent_x / tangent_length;
  terminal_tangent_y_ = tangent_y / tangent_length;
  path_valid_ = true;
  terminal_reached_ = false;
  stopped_cycles_ = 0;
}

// 位置在容差内，或已经靠近末段并越过终点切平面时，单向锁存 terminal_reached。
// 未检查航向与速度，返回 true 仍不等于最终任务完成。
bool FixedPathGoalChecker::isTerminalPositionReached(const geometry_msgs::msg::Pose & query_pose)
{
  if (!path_valid_)
  {
    return false;
  }
  const auto & goal = path_.poses.back().pose.position;
  const double delta_x = query_pose.position.x - goal.x;
  const double delta_y = query_pose.position.y - goal.y;
  const double goal_distance = std::hypot(delta_x, delta_y);
  const double terminal_projection = delta_x * terminal_tangent_x_ + delta_y * terminal_tangent_y_;
  const bool goal_plane_crossed = findNearestIndex(query_pose) >= goal_tangent_index_ && terminal_projection >= 0.0;
  if (goal_distance <= xy_goal_tolerance_ || goal_plane_crossed)
  {
    terminal_reached_ = true;
  }
  return terminal_reached_;
}

// 先更新终点几何锁存，再在平面线速度及角速度都低于门限时累计连续周期。
// 任何运动周期清零停稳计数；满足配置周期才返回 true。
bool FixedPathGoalChecker::isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist & velocity)
{
  isTerminalPositionReached(query_pose);
  if (!terminal_reached_)
  {
    stopped_cycles_ = 0;
    return false;
  }
  if (std::hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_ && std::abs(velocity.angular.z) <= rot_stopped_velocity_)
  {
    ++stopped_cycles_;
  }
  else
  {
    stopped_cycles_ = 0;
  }
  return stopped_cycles_ >= position_stable_cycles_;
}

// 通过 Nav2 接口返回 XY 半径与停止速度阈值，未使用的维度写入约定占位。
// 输出是供控制器参考的阈值，不是当前机器人误差。
bool FixedPathGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & velocity_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;
  velocity_tolerance.linear.x = trans_stopped_velocity_;
  velocity_tolerance.linear.y = trans_stopped_velocity_;
  velocity_tolerance.linear.z = invalid_field;
  velocity_tolerance.angular.x = invalid_field;
  velocity_tolerance.angular.y = invalid_field;
  velocity_tolerance.angular.z = rot_stopped_velocity_;
  return true;
}

// 在整条保存路径中搜索离 query_pose 最近的点，用于末段越界判据。
// 与控制器只向前搜索的最近点函数不同，此处不维护递增进度。
std::size_t FixedPathGoalChecker::findNearestIndex(const geometry_msgs::msg::Pose & query_pose) const
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < path_.poses.size(); ++index)
  {
    const auto & position = path_.poses[index].pose.position;
    const double distance = std::hypot(query_pose.position.x - position.x, query_pose.position.y - position.y);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::FixedPathGoalChecker, nav2_core::GoalChecker)
