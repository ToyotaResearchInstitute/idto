#include "optimizer/trajectory_optimizer_solution.h"

#include <drake/common/default_scalars.h>

namespace idto {
namespace optimizer {

std::string DecodeConvergenceReasons(ConvergenceReason reason) {
  if (reason == ConvergenceReason::kNoConvergenceCriteriaSatisfied) {
    return "no convergence criterion satisfied";
  }
  std::string reasons;
  if ((reason & ConvergenceReason::kCostReductionCriterionSatisfied) != 0)
    reasons = "cost reduction";
  if ((reason & ConvergenceReason::kGradientCriterionSatisfied) != 0) {
    if (!reasons.empty()) reasons += ", ";
    reasons += "gradient";
  }
  if ((reason & ConvergenceReason::kSateCriterionSatisfied) != 0) {
    if (!reasons.empty()) reasons += ", ";
    reasons += "state change";
  }
  return reasons;
}

}  // namespace optimizer
}  // namespace idto

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::idto::optimizer::TrajectoryOptimizerSolution);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::idto::optimizer::TrajectoryOptimizerStats);
