#ifndef NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
#define NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_

#include <string>

namespace nav2_regulated_modules
{

class PlanningModule
{
public:
  void configure(std::string planner_id, std::string smoother_id, bool use_smoother, double replan_frequency, int max_failures);

  const std::string & plannerId() const;
  const std::string & smootherId() const;
  bool useSmoother() const;
  double replanPeriod() const;
  int maxFailures() const;

private:
  std::string planner_id_;
  std::string smoother_id_;
  bool use_smoother_{true};
  double replan_frequency_{1.0};
  int max_failures_{3};
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__PLANNING_MODULE_HPP_
