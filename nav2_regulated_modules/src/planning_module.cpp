// 规划模块的配置归一化与只读访问器。重规划输入是 Hz，保存的是秒周期。
// 这里不提交 ComputePath 或 SmoothPath Goal，真正调度位于 navigator_planning.cpp。

#include "nav2_regulated_modules/planning_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

// 保存规划／平滑标识和开关，并将有效重规划频率 Hz 转换为秒周期。
// 失败次数按实现设下限；这里不检查对应 Action Server 是否可用。
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

// 返回 Planner Server 所配置插件的选择 ID，不是 C++ 导出类名。
const std::string & PlanningModule::plannerId() const
{
  return planner_id_;
}

// 返回 Smoother Server 的实例 ID，只有 useSmoother 为 true 才需要调用。
const std::string & PlanningModule::smootherId() const
{
  return smoother_id_;
}

// 读取路径几何平滑开关，与底盘速度平滑不是同一个功能。
bool PlanningModule::useSmoother() const
{
  return use_smoother_;
}

// 返回重规划周期 s，不是频率 Hz；monitorTask 用它比较 elapsed 时间。
double PlanningModule::replanPeriod() const
{
  return 1.0 / replan_frequency_;
}

// 返回连续规划失败的允许上限，不是总恢复轮次。
int PlanningModule::maxFailures() const
{
  return max_failures_;
}

}
// namespace nav2_regulated_modules
