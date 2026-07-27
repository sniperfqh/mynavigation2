#include "nav2_regulated_modules/planning_module.hpp"

#include <stdexcept>
#include <utility>

namespace nav2_regulated_modules
{

void PlanningModule::configure(std::string planner_id, std::string smoother_id, const bool use_smoother, const double replan_frequency, const int max_failures) {
  if (replan_frequency <= 0.0 || max_failures < 1) {
    throw std::invalid_argument("规划频率必须大于零，最大连续失败次数必须大于零");
  }
  planner_id_ = std::move(planner_id);
  smoother_id_ = std::move(smoother_id);
  use_smoother_ = use_smoother;
  replan_frequency_ = replan_frequency;
  max_failures_ = max_failures;
}

const std::string & PlanningModule::plannerId() const {return planner_id_;}

const std::string & PlanningModule::smootherId() const {return smoother_id_;}

bool PlanningModule::useSmoother() const {return use_smoother_;}

double PlanningModule::replanPeriod() const {return 1.0 / replan_frequency_;}

int PlanningModule::maxFailures() const {return max_failures_;}

}  // namespace nav2_regulated_modules
