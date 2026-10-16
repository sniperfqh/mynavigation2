// 普通导航跟踪配置的实现。configure 归一化超时后保存标识，访问器返回该快照。
// 本模块没有 ROS 发布器、Action 客户端或独立线程；修改成员不等于重新配置 Controller Server。

#include "nav2_regulated_modules/control_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

// 保存传入控制器与 GoalChecker ID，对进度超时做实现中的下限归一化。
// 该配置对象不会动态修改宿主插件参数。
void ControlModule::configure(std::string controller_id, std::string goal_checker_id, const double progress_timeout)
{
  if (progress_timeout <= 0.0)
  {
    throw std::invalid_argument("控制无进展超时必须大于零");
  }
  controller_id_ = std::move(controller_id);
  goal_checker_id_ = std::move(goal_checker_id);
  progress_timeout_ = progress_timeout;
}

// 返回保存的普通导航控制器 ID 常量引用，引用生命周期依赖本对象。
const std::string & ControlModule::controllerId() const
{
  return controller_id_;
}

// 返回普通导航目标检查器 ID，固定路径另由协调器选专用标识。
const std::string & ControlModule::goalCheckerId() const
{
  return goal_checker_id_;
}

// 返回保存的无进展阈值，单位 s；实际计时在任务监控中进行。
double ControlModule::progressTimeout() const
{
  return progress_timeout_;
}

}
// namespace nav2_regulated_modules
