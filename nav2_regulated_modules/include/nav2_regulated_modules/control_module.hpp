// FollowPath 配置快照：集中保存普通导航控制器、目标检查器与无进展超时。
// 该类不运行控制循环；固定路径任务在协调器中另选专用插件，不能把这里的普通导航标识当作固定路径默认。

#ifndef NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
#define NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class ControlModule
{
  public:
  // 保存传入控制器与 GoalChecker ID，对进度超时做实现中的下限归一化。
  // 该配置对象不会动态修改宿主插件参数。
  void configure(std::string controller_id, std::string goal_checker_id, double progress_timeout);

  // 返回保存的普通导航控制器 ID 常量引用，引用生命周期依赖本对象。
  const std::string & controllerId() const;
  // 返回普通导航目标检查器 ID，固定路径另由协调器选专用标识。
  const std::string & goalCheckerId() const;
  // 返回保存的无进展阈值，单位 s；实际计时在任务监控中进行。
  double progressTimeout() const;

private:
  std::string controller_id_;
  std::string goal_checker_id_;
  double progress_timeout_ { 10.0 };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
