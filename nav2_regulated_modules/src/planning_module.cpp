// 规划模块参数实现。配置规划器和平滑器及重规划间隔。

#include "nav2_regulated_modules/planning_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

// 保存规划器、平滑器、重规划周期和失败上限；本模块不发起 Action。
void PlanningModule::configure(std::string planner_id, std::string smoother_id, const bool use_smoother, const double replan_frequency, const int max_failures)
{
  if (replan_frequency <= 0.0 || max_failures < 1)
  {
    throw std::invalid_argument("规划频率必须大于零，最大连续失败次数必须大于零");
  }
  planner_id_ = std::move(planner_id);
  smoother_id_ = std::move(smoother_id);
  use_smoother_ = use_smoother;
  replan_frequency_ = replan_frequency;
  max_failures_ = max_failures;
}

// 返回当前规划器插件标识。
const std::string & PlanningModule::plannerId() const
{
  return planner_id_;
}

// 返回当前平滑器插件标识。
const std::string & PlanningModule::smootherId() const
{
  return smoother_id_;
}

// 指出当前任务是否需调用路径平滑。
bool PlanningModule::useSmoother() const
{
  return use_smoother_;
}

// 返回普通导航的规划周期。
double PlanningModule::replanPeriod() const
{
  return 1.0 / replan_frequency_;
}

// 返回连续规划失败允许次数。
int PlanningModule::maxFailures() const
{
  return max_failures_;
}

}
// namespace nav2_regulated_modules
