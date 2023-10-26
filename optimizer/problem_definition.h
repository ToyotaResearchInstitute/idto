#pragma once

#include <vector>

#include <drake/common/eigen_types.h>

namespace idto {
namespace optimizer {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * A struct for specifying the optimization problem
 *
 *    min x_err(T)'*Qf*x_err(T) + dt*sum{ x_err(t)'*Q*x_err(t) + u(t)'*R*u(t) }
 *    s.t. x(0) = x0
 *         multibody dynamics with contact
 *
 *  where x(t) = [q(t); v(t)], x_err(t) = x(t)-x_nom(t), and
 *  Q = [ Qq  0  ]
 *      [ 0   Qv ].
 */
struct ProblemDefinition {
  // Time horizon (number of steps) for the optimization problem
  int num_steps;

  // Initial generalized positions
  VectorXd q_init;

  // Initial generalized velocities
  VectorXd v_init;

  // Joint Position Limits
  MatrixXd q_min;
  MatrixXd q_max;

  // Running cost coefficients for generalized positions
  // N.B. these weights are per unit of time
  // TODO(vincekurtz): consider storing these as VectorXd, assuming they are
  // diagonal, and using X.asDiagonal() when multiplying.
  MatrixXd Qq;

  // Running cost coefficients for generalized velocities
  // N.B. these weights are per unit of time
  MatrixXd Qv;

  // Running cost coefficients for joint limit violations
  // N.B. these weights are per unit of time
  MatrixXd Qlq;

  // Terminal cost coefficients for generalized positions
  MatrixXd Qf_q;

  // Terminal cost coefficients for generalized velocities
  MatrixXd Qf_v;

  // Control cost coefficients
  // N.B. these weights are per unit of time
  MatrixXd R;

  // Target generalized positions at each time step
  std::vector<VectorXd> q_nom;

  // Target generalized velocities at each time step
  std::vector<VectorXd> v_nom;

  // Contact signed distance cost
  double signed_distance_penalty{0};

  double signed_distance_threshold;
};

}  // namespace optimizer
}  // namespace idto
