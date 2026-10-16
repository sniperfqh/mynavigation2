// 规划配置对象：保存 Planner／Smoother 标识、平滑开关、重规划周期与连续失败上限。
// 频率在 configure 时转换为周期；Action 调度、异步结果和恢复状态由 RegulatedNavigator 管理。

#ifndef NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
#define NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class PlanningModule
{
  public:
  // 保存规划／平滑标识和开关，并将有效重规划频率 Hz 转换为秒周期。
  // 失败次数按实现设下限；这里不检查对应 Action Server 是否可用。
  void configure(std::string planner_id, std::string smoother_id, bool use_smoother, double replan_frequency, int max_failures);

  // 返回 Planner Server 所配置插件的选择 ID，不是 C++ 导出类名。
  const std::string & plannerId() const;
  // 返回 Smoother Server 的实例 ID，只有 useSmoother 为 true 才需要调用。
  const std::string & smootherId() const;
  // 读取路径几何平滑开关，与底盘速度平滑不是同一个功能。
  bool useSmoother() const;
  // 返回重规划周期 s，不是频率 Hz；monitorTask 用它比较 elapsed 时间。
  double replanPeriod() const;
  // 返回连续规划失败的允许上限，不是总恢复轮次。
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
