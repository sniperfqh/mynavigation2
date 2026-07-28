#include "nav2_regulated_modules/control_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

// 中文注释：校验并保存控制器插件、目标检查器和基于 TF 的无进展超时配置。
void ControlModule::configure(std::string controller_id, std::string goal_checker_id, const double progress_timeout) {
  if (progress_timeout <= 0.0) {
    throw std::invalid_argument("控制无进展超时必须大于零");
  }
  controller_id_ = std::move(controller_id);
  goal_checker_id_ = std::move(goal_checker_id);
  progress_timeout_ = progress_timeout;
}

// 中文注释：向 FollowPath Goal 提供控制器插件 ID。
const std::string & ControlModule::controllerId() const {return controller_id_;}

// 中文注释：向 FollowPath Goal 提供目标检查器插件 ID。
const std::string & ControlModule::goalCheckerId() const {return goal_checker_id_;}

// 中文注释：向导航监控循环提供允许车辆无位移的最长秒数。
double ControlModule::progressTimeout() const {return progress_timeout_;}

}  // namespace nav2_regulated_modules
