#include "nav2_regulated_modules/planning_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

// 中文注释：校验重规划频率和失败上限，再保存规划／平滑插件选择及平滑开关。
void PlanningModule::configure(std::string planner_id, std::string smoother_id, const bool use_smoother, const double replan_frequency, const int max_failures) {
  if (replan_frequency <= 0.0 || max_failures < 1) {
    throw std::invalid_argument("规划频率必须大于零，最大连续失败次数必须大于零");
  }
  planner_id_ = std::move(planner_id);
  smoother_id_ = std::move(smoother_id);
  use_smoother_ = use_smoother;
  replan_frequency_ = replan_frequency;
  max_failures_ = max_failures;
}

// 中文注释：返回全局规划 Action Goal 使用的插件 ID。
const std::string & PlanningModule::plannerId() const {return planner_id_;}

// 中文注释：返回路径平滑 Action Goal 使用的插件 ID。
const std::string & PlanningModule::smootherId() const {return smoother_id_;}

// 中文注释：返回是否启用规划后平滑步骤。
bool PlanningModule::useSmoother() const {return use_smoother_;}

// 中文注释：把赫兹频率换算为监控循环比较使用的秒周期。
double PlanningModule::replanPeriod() const {return 1.0 / replan_frequency_;}

// 中文注释：返回连续规划失败上限，超过后进入恢复链。
int PlanningModule::maxFailures() const {return max_failures_;}

}  // namespace nav2_regulated_modules
