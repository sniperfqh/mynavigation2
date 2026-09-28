// 跟踪控制配置接口。集中保存 FollowPath 使用的控制器、目标检查器和进度超时，供不同任务模式选择。

#ifndef NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
#define NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class ControlModule
{
  public:
  // 保存控制器、目标检查器标识和进度超时；本模块不创建 ROS 实体。
  void configure(std::string controller_id, std::string goal_checker_id, double progress_timeout);

  // 返回当前控制器插件标识。
  const std::string & controllerId() const;
  // 返回当前目标检查器插件标识。
  const std::string & goalCheckerId() const;
  // 返回当前模式下无进展超时阈值，单位秒。
  double progressTimeout() const;

private:
  std::string controller_id_;
  std::string goal_checker_id_;
  double progress_timeout_ { 10.0 };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
