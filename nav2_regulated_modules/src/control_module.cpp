#include "nav2_regulated_modules/control_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

void ControlModule::configure(
  std::string controller_id, std::string goal_checker_id, const double progress_timeout)
{
  if (progress_timeout <= 0.0) {
    throw std::invalid_argument("控制无进展超时必须大于零");
  }
  controller_id_ = std::move(controller_id);
  goal_checker_id_ = std::move(goal_checker_id);
  progress_timeout_ = progress_timeout;
}

const std::string & ControlModule::controllerId() const {return controller_id_;}

const std::string & ControlModule::goalCheckerId() const {return goal_checker_id_;}

double ControlModule::progressTimeout() const {return progress_timeout_;}

}  // namespace nav2_regulated_modules
