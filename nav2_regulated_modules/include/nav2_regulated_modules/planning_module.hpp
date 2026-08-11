#ifndef NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
#define NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

// 中文注释：集中保存规划和平滑策略，节点只负责状态迁移和 Action 编排。
class PlanningModule
{
public:
  // 中文注释：写入规划／平滑插件选择、平滑开关、重规划频率和连续失败上限。
  void configure(std::string planner_id, std::string smoother_id, bool use_smoother, double replan_frequency, int max_failures);

  // 中文注释：返回 ComputePath Goal 使用的规划插件 ID。
  const std::string & plannerId() const;
  // 中文注释：返回 SmoothPath Goal 使用的平滑插件 ID。
  const std::string & smootherId() const;
  // 中文注释：说明规划结果是否需要经过 Smoother Server。
  bool useSmoother() const;
  // 中文注释：把重规划频率转换为监控循环使用的周期秒数。
  double replanPeriod() const;
  // 中文注释：返回允许连续规划失败的最大次数。
  int maxFailures() const;

private:
  // 中文注释：以下成员是 configure 后供状态机读取的规划策略快照。
  std::string planner_id_;
  std::string smoother_id_;
  bool use_smoother_{true};
  double replan_frequency_{1.0};
  int max_failures_{3};
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
