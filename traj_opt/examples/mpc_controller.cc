#include "drake/traj_opt/examples/mpc_controller.h"

#include <iostream>

namespace drake {
namespace traj_opt {
namespace examples {
namespace mpc {

ModelPredictiveController::ModelPredictiveController(
    const Diagram<double>* diagram, const MultibodyPlant<double>* plant,
    const ProblemDefinition& prob,
    const TrajectoryOptimizerSolution<double>& warm_start_solution,
    const SolverParameters& params, const double replan_period)
    : time_step_(plant->time_step()),
      num_steps_(prob.num_steps + 1),
      nq_(plant->num_positions()),
      nv_(plant->num_velocities()),
      nu_(plant->num_actuators()),
      B_(plant->MakeActuationMatrix()),
      optimizer_(diagram, plant, prob, params),
      warm_start_(optimizer_.num_steps(), optimizer_.diagram(),
                  optimizer_.plant(), optimizer_.num_equality_constraints(),
                  warm_start_solution.q, params.Delta0) {
  // Abstract-valued discrete state stores an optimal trajectory
  StoredTrajectory initial_guess_traj;
  StoreOptimizerSolution(warm_start_solution, 0.0, &initial_guess_traj);
  stored_trajectory_ = this->DeclareAbstractState(Value(initial_guess_traj));
  this->DeclarePeriodicUnrestrictedUpdateEvent(
      replan_period, 0, &ModelPredictiveController::UpdateAbstractState);

  // Input port recieves state estimates
  state_input_port_ = this->DeclareVectorInputPort(
                              "state_estimate", BasicVector<double>(nq_ + nv_))
                          .get_index();

  // Output port sends the optimal trajectory
  trajectory_output_port_ =
      this->DeclareStateOutputPort("optimal_trajectory", stored_trajectory_)
          .get_index();
}

EventStatus ModelPredictiveController::UpdateAbstractState(
    const Context<double>& context, State<double>* state) const {
  std::cout << "Resolving at t=" << context.get_time() << std::endl;

  // Get the latest initial condition
  const VectorXd& x0 = EvalVectorInput(context, state_input_port_)->value();
  const auto& q0 = x0.topRows(nq_);
  const auto& v0 = x0.bottomRows(nv_);

  // Get a reference to the previous solution stored in the discrete state
  StoredTrajectory& stored_trajectory =
      state->get_mutable_abstract_state<StoredTrajectory>(stored_trajectory_);

  // Set the initial guess from the stored solution
  std::vector<VectorXd> q_guess(num_steps_, VectorXd(nq_));
  UpdateInitialGuess(stored_trajectory, context.get_time(), &q_guess);
  q_guess[0] = q0;  // guess must be consistent with the initial condition
  warm_start_.state.set_q(q_guess);

  // Solve the trajectory optimization problem from the new initial condition
  optimizer_.ResetInitialConditions(q0, v0);
  TrajectoryOptimizerStats<double> stats;
  TrajectoryOptimizerSolution<double> solution;
  optimizer_.SolveFromWarmStart(&warm_start_, &solution, &stats);

  // Store the result in the discrete state
  StoreOptimizerSolution(solution, context.get_time(), &stored_trajectory);

  return EventStatus::Succeeded();
}

void ModelPredictiveController::UpdateInitialGuess(
    const StoredTrajectory& stored_trajectory, const double current_time,
    std::vector<VectorXd>* q_guess) const {
  DRAKE_DEMAND(static_cast<int>(q_guess->size()) == num_steps_);

  const double start_time = current_time - stored_trajectory.start_time;
  for (int i = 0; i < num_steps_; ++i) {
    const double t = start_time + i * time_step_;
    q_guess->at(i) = stored_trajectory.q.value(t);
  }
}

void ModelPredictiveController::StoreOptimizerSolution(
    const TrajectoryOptimizerSolution<double>& solution,
    const double start_time, StoredTrajectory* stored_trajectory) const {
  // Set up knot points for a polynomial interpolation
  // N.B. PiecewisePolynomial requires std::vector<MatrixXd> rather than
  // std::vector<VectorXd>
  std::vector<double> time_steps;
  std::vector<MatrixXd> q_knots(num_steps_, VectorXd(nq_));
  std::vector<MatrixXd> v_knots(num_steps_, VectorXd(nq_));
  std::vector<MatrixXd> u_knots(num_steps_, VectorXd(nq_));

  for (int i = 0; i < num_steps_; ++i) {
    // Time steps
    time_steps.push_back(i * time_step_);

    // Generalized positions and velocities
    q_knots[i] = solution.q[i];
    v_knots[i] = solution.v[i];

    // Control inputs, which are undefined at the last time step
    if (i == num_steps_ - 1) {
      u_knots[i] = B_.transpose() * solution.tau[i - 1];
    } else {
      u_knots[i] = B_.transpose() * solution.tau[i];
    }
  }

  // Perform polynomial interpolation and store the result in our struct
  // TODO(vincekurz): set interpolation with some parameter
  stored_trajectory->start_time = start_time;
  stored_trajectory->q =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          time_steps, q_knots);
  stored_trajectory->v =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          time_steps, v_knots);
  stored_trajectory->u =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          time_steps, u_knots);
}

Interpolator::Interpolator(const int nq, const int nv, const int nu)
    : nq_(nq), nv_(nv), nu_(nu) {
  // Input port recieves a StoredTrajectory
  trajectory_input_port_ =
      this->DeclareAbstractInputPort("trajectory", Value(StoredTrajectory()))
          .get_index();

  // First output port sends interpolated state values x(t)
  state_output_port_ =
      this->DeclareVectorOutputPort("state", BasicVector<double>(nq_ + nv_),
                                    &Interpolator::SendState)
          .get_index();

  // First output port sends interpolated control values u(t)
  control_output_port_ =
      this->DeclareVectorOutputPort("control", BasicVector<double>(nu_),
                                    &Interpolator::SendControl)
          .get_index();
}

void Interpolator::SendState(const Context<double>& context,
                             BasicVector<double>* output) const {
  const StoredTrajectory& traj =
      EvalAbstractInput(context, trajectory_input_port_)
          ->get_value<StoredTrajectory>();
  auto x = output->get_mutable_value();
  x.topRows(nq_) = traj.q.value(context.get_time() - traj.start_time);
  x.bottomRows(nv_) = traj.v.value(context.get_time() - traj.start_time);
}

void Interpolator::SendControl(const Context<double>& context,
                               BasicVector<double>* output) const {
  const StoredTrajectory& traj =
      EvalAbstractInput(context, trajectory_input_port_)
          ->get_value<StoredTrajectory>();
  auto u = output->get_mutable_value();
  u = traj.u.value(context.get_time() - traj.start_time);
}

}  // namespace mpc
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
