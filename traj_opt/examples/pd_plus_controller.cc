#include "drake/traj_opt/examples/pd_plus_controller.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace pd_plus {

using Eigen::VectorXd;

PdPlusController::PdPlusController(const MatrixXd& Kp, const MatrixXd& Kd,
                                   const bool feed_forward)
    : Kp_(Kp), Kd_(Kd), feed_forward_(feed_forward) {
  // Size sanity checks
  DRAKE_DEMAND(Kp.rows() == Kd.rows());
  nq_ = Kp.cols();
  nv_ = Kd.cols();
  nu_ = Kp.rows();

  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(nq_ + nv_))
          .get_index();

  nominal_state_input_port_ =
      this->DeclareVectorInputPort("nominal_state",
                                   BasicVector<double>(nq_ + nv_))
          .get_index();

  nominal_control_input_port_ =
      this->DeclareVectorInputPort("nominal_control", BasicVector<double>(nu_))
          .get_index();

  control_output_port_ =
      this->DeclareVectorOutputPort("control", BasicVector<double>(nu_),
                                    &PdPlusController::CalcOutput)
          .get_index();
}

void PdPlusController::CalcOutput(const Context<double>& context,
                                  BasicVector<double>* output) const {
  // Current state
  const VectorXd& x = EvalVectorInput(context, state_input_port_)->value();
  const auto& q = x.topRows(nq_);
  const auto& v = x.bottomRows(nv_);

  // Desired state
  const VectorXd& x_nom =
      EvalVectorInput(context, nominal_state_input_port_)->value();
  const auto& q_nom = x_nom.topRows(nq_);
  const auto& v_nom = x_nom.bottomRows(nv_);

  // Set output control value
  auto u = output->get_mutable_value();
  u = Kp_ * (q_nom - q) + Kd_ * (v_nom - v);

  // Feedforward nominal control
  if (feed_forward_) {
    const VectorXd& u_nom =
        EvalVectorInput(context, nominal_control_input_port_)->value();
    u += u_nom;
  }
}

}  // namespace pd_plus
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
