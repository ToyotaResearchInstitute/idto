#pragma once

#include <string>
#include <vector>

#include "optimizer/trajectory_optimizer_state.h"

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>

namespace idto {
namespace optimizer {

using Eigen::VectorXd;

/**
 * A container for holding variables that are re-used between MPC solves. Using
 * this container allows us to perform a full warm-start (including trust region
 * radius) for MPC, and allows us to avoid expensive state allocations between
 * MPC re-solves.
 */
class WarmStart {
 public:
  /**
   * The constructor allocates state variables.
   *
   * @param num_steps Number of steps in the trajectory optimization problem
   * @param diagram Overall system diagram containing the plant
   * @param plant MultibodyPlant model to optimize over
   * @param num_eq_constraints Number of equality constraints
   * @param q_guess Initial guess of the sequence of generalized positions
   * @param Delta0 Initial trust region radius
   */
  WarmStart(const int num_steps, const Diagram<double>& diagram,
            const MultibodyPlant<double>& plant, const int num_eq_constraints,
            const std::vector<VectorXd> q_guess, const double Delta0)
      : state(num_steps, diagram, plant, num_eq_constraints),
        scratch_state(num_steps, diagram, plant, num_eq_constraints),
        Delta(Delta0) {
    // Set the initial guess
    state.set_q(q_guess);

    // Make sure the update vector is the right size
    const int num_vars = plant.num_positions() + (num_steps + 1);
    dq.resize(num_vars);
    dqH.resize(num_vars);
  }

  /**
   * Set the initial guess of the sequence of generalized positions.
   *
   * @param q_guess Initial guess of the sequence of generalized positions
   */
  void set_q(const std::vector<VectorXd>& q_guess) { state.set_q(q_guess); }

  /**
   * Get the initial guess of the sequence of generalized positions.
   */
  const std::vector<VectorXd>& get_q() const { return state.q(); }

  // A state variable to store q and everything that is computed from q
  TrajectoryOptimizerState<double> state;

  // A separate state variable for computations like L(q + dq)
  TrajectoryOptimizerState<double> scratch_state;

  // Trust region size
  double Delta;

  // The update vector q_{k+1} = q_k + dq
  VectorXd dq;

  // The full Newton step H * dqH = -g
  VectorXd dqH;
};

}  // namespace optimizer
}  // namespace idto
