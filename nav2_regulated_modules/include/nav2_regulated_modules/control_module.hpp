#ifndef NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
#define NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class ControlModule
{
public:
  void configure(std::string controller_id, std::string goal_checker_id, double progress_timeout);

  const std::string & controllerId() const;
  const std::string & goalCheckerId() const;
  double progressTimeout() const;

private:
  std::string controller_id_;
  std::string goal_checker_id_;
  double progress_timeout_{10.0};
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CONTROL_MODULE_HPP_
