#include "traj_opt/trajectory_optimizer_state.h"

#include "drake/common/default_scalars.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::idto::traj_opt::TrajectoryOptimizerCache)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::idto::traj_opt::ProximalOperatorData)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::idto::traj_opt::TrajectoryOptimizerState)
