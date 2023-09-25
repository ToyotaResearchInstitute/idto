#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace pd_plus {

using systems::BasicVector;
using systems::Context;
using systems::InputPort;
using systems::LeafSystem;
using systems::OutputPort;

using Eigen::MatrixXd;

/// A simple PD+ controller that outputs control torques computed by
///
///      u = u_nom + Kp*(q_nom - q) + Kd*(v_nom - v)
///
/// where u_nom, q_nom, and v_nom are nominal inputs, generalized positions, and
/// generalized velocities. Including the feed-forward input u_nom is optional.
class PdPlusController : public LeafSystem<double> {
 public:
  /**
   * Construct a PD+ controller
   *
   * @param Kp Proportional gains (including unactuated DoFs)
   * @param Kd Derivative gains (including unactuated DoFs)
   * @param feed_forward flag for including the feed_forward torques u_nom
   */
  PdPlusController(const MatrixXd& Kp, const MatrixXd& Kd,
                   const bool feed_forward);

  const InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const InputPort<double>& get_nominal_state_input_port() const {
    return this->get_input_port(nominal_state_input_port_);
  }

  const InputPort<double>& get_nominal_control_input_port() const {
    return this->get_input_port(nominal_control_input_port_);
  }

  const OutputPort<double>& get_control_output_port() const {
    return this->get_output_port(control_output_port_);
  }

 private:
  /**
   * Compute and send output torque commands
   *
   *     u = u_nom + Kp*(q_nom - q) + Kd*(v_nom - v)
   *
   * @param context system context containing u, q_nom, q, v_nom, and v
   * @param output output u that we'll set
   */
  void CalcOutput(const Context<double>& context,
                  BasicVector<double>* output) const;

  // Input and output port indices
  int state_input_port_;
  int nominal_state_input_port_;
  int nominal_control_input_port_;
  int control_output_port_;

  // System dimensions (positions, velocities, inputs)
  int nq_;
  int nv_;
  int nu_;

  // Feedback gains
  const MatrixXd Kp_;
  const MatrixXd Kd_;
  const bool feed_forward_;
};

}  // namespace pd_plus
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
