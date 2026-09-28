// 路径规划参数接口。统一保存规划器和平滑器选择及重规划周期，供导航任务复用。

#ifndef NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
#define NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class PlanningModule
{
  public:
  // 保存规划器、平滑器、重规划周期和失败上限；本模块不发起 Action。
  void configure(std::string planner_id, std::string smoother_id, bool use_smoother, double replan_frequency, int max_failures);

  // 返回当前规划器插件标识。
  const std::string & plannerId() const;
  // 返回当前平滑器插件标识。
  const std::string & smootherId() const;
  // 指出当前任务是否需调用路径平滑。
  bool useSmoother() const;
  // 返回普通导航的规划周期。
  double replanPeriod() const;
  // 返回连续规划失败允许次数。
  int maxFailures() const;

private:
  std::string planner_id_;
  std::string smoother_id_;
  bool use_smoother_ { true };
  double replan_frequency_ { 1.0 };
  int max_failures_ { 3 };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
