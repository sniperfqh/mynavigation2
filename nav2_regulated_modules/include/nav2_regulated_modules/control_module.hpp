#ifndef NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
#define NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

// 中文注释：集中保存控制器、目标检查器和控制超时策略。
class ControlModule
{
public:
  // 中文注释：写入控制插件 ID、目标检查器 ID 和无进展超时，并校验超时必须为正数。
  void configure(std::string controller_id, std::string goal_checker_id, double progress_timeout);

  // 中文注释：返回 FollowPath Goal 使用的控制器插件 ID。
  const std::string & controllerId() const;
  // 中文注释：返回 FollowPath Goal 使用的目标检查器插件 ID。
  const std::string & goalCheckerId() const;
  // 中文注释：返回导航器基于真实 TF 判断无进展时采用的超时秒数。
  double progressTimeout() const;

private:
  // 中文注释：以下成员是 configure 后只读的控制策略快照。
  std::string controller_id_;
  std::string goal_checker_id_;
  double progress_timeout_{10.0};
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
