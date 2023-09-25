#include "drake/traj_opt/examples/example_base.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/traj_opt/examples/mpc_controller.h"
#include "drake/traj_opt/examples/pd_plus_controller.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace traj_opt {
namespace examples {

using Eigen::Matrix4d;
using geometry::MeshcatAnimation;
using geometry::MeshcatVisualizerd;
using math::RigidTransformd;
using mpc::Interpolator;
using mpc::ModelPredictiveController;
using pd_plus::PdPlusController;
using systems::DiscreteTimeDelay;

void TrajOptExample::RunExample(const std::string options_file) const {
  // Load parameters from file
  TrajOptExampleParams default_options;
  TrajOptExampleParams options = yaml::LoadYamlFile<TrajOptExampleParams>(
      FindResourceOrThrow(options_file), {}, default_options);

  if (options.mpc) {
    // Run a simulation that uses the optimizer as a model predictive controller
    RunModelPredictiveControl(options);
  } else {
    // Solve a single instance of the optimization problem and play back the
    // result on the visualizer
    SolveTrajectoryOptimization(options);
  }
}

void TrajOptExample::RunModelPredictiveControl(
    const TrajOptExampleParams& options) const {
  // Perform a full solve to convergence (as defined by YAML parameters) to
  // warm-start the first MPC iteration. Subsequent MPC iterations will be
  // warm-started based on the prior MPC iteration.
  TrajectoryOptimizerSolution<double> initial_solution =
      SolveTrajectoryOptimization(options);

  // Set up the system diagram for the simulator
  DiagramBuilder<double> builder;

  // Construct the multibody plant system model
  MultibodyPlantConfig config;
  config.time_step = options.sim_time_step;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  CreatePlantModelForSimulation(&plant);
  plant.Finalize();

  const int nq = plant.num_positions();
  const int nv = plant.num_velocities();
  const int nu = plant.num_actuators();

  // Connect to the Meshcat visualizer
  auto& visualizer =
      MeshcatVisualizerd::AddToBuilder(&builder, scene_graph, meshcat_);

  // Create a system model for the controller
  DiagramBuilder<double> ctrl_builder;
  MultibodyPlantConfig ctrl_config;
  ctrl_config.time_step = options.time_step;
  auto [ctrl_plant, ctrl_scene_graph] =
      AddMultibodyPlant(ctrl_config, &ctrl_builder);
  CreatePlantModel(&ctrl_plant);
  ctrl_plant.Finalize();
  auto ctrl_diagram = ctrl_builder.Build();

  // Define the optimization problem
  ProblemDefinition opt_prob;
  SetProblemDefinition(options, &opt_prob);
  NormalizeQuaternions(ctrl_plant, &opt_prob.q_nom);

  // Set MPC-specific solver parameters
  SolverParameters solver_params;
  SetSolverParameters(options, &solver_params);
  solver_params.max_iterations = options.mpc_iters;

  // Set up the MPC system
  const double replan_period = 1. / options.controller_frequency;
  auto controller = builder.AddSystem<ModelPredictiveController>(
      ctrl_diagram.get(), &ctrl_plant, opt_prob, initial_solution,
      solver_params, replan_period);

  // Create an interpolator to send samples from the optimal trajectory at a
  // faster rate
  auto interpolator = builder.AddSystem<Interpolator>(nq, nv, nu);

  // Connect the MPC controller to the interpolator
  // N.B. We place a delay block between the MPC controller and the interpolator
  // to simulate the fact that the system continues to evolve over time as the
  // optimizer solves the trajectory optimization problem.
  mpc::StoredTrajectory placeholder_trajectory;
  controller->StoreOptimizerSolution(initial_solution, 0.0,
                                     &placeholder_trajectory);

  auto delay = builder.AddSystem<DiscreteTimeDelay>(
      replan_period, 1, Value(placeholder_trajectory));
  builder.Connect(controller->get_trajectory_output_port(),
                  delay->get_input_port());
  builder.Connect(delay->get_output_port(),
                  interpolator->get_trajectory_input_port());

  // Connect the interpolator to a low-level PD controller
  const MatrixXd B = plant.MakeActuationMatrix();
  auto dummy_context = plant.CreateDefaultContext();
  MatrixXd N(nq, nv);
  plant.CalcNMatrix(*dummy_context, &N);
  MatrixXd Bq = N * B;

  const MatrixXd Kp =
      (options.Kp.size() == 0)
          ? MatrixXd::Zero(nu, nq)
          : static_cast<MatrixXd>(Bq.transpose() * options.Kp.asDiagonal());
  const MatrixXd Kd =
      (options.Kd.size() == 0)
          ? MatrixXd::Zero(nu, nv)
          : static_cast<MatrixXd>(B.transpose() * options.Kd.asDiagonal());

  auto pd = builder.AddSystem<PdPlusController>(Kp, Kd, options.feed_forward);
  builder.Connect(interpolator->get_state_output_port(),
                  pd->get_nominal_state_input_port());
  builder.Connect(interpolator->get_control_output_port(),
                  pd->get_nominal_control_input_port());
  builder.Connect(plant.get_state_output_port(), pd->get_state_input_port());
  builder.Connect(pd->get_control_output_port(),
                  plant.get_actuation_input_port());

  // Connect the plant's state estimate to the MPC planner
  builder.Connect(plant.get_state_output_port(),
                  controller->get_state_input_port());

  // Compile the diagram
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set up the simulation
  plant.SetPositions(&plant_context, options.q_init);
  plant.SetVelocities(&plant_context, options.v_init);
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(options.sim_realtime_rate);
  simulator.Initialize();

  // Run the simulation, recording the result for later playback in MeshCat
  MeshcatAnimation* animation = visualizer.StartRecording();
  animation->set_autoplay(false);
  simulator.AdvanceTo(options.sim_time);
  visualizer.StopRecording();
  visualizer.PublishRecording();

  if (options.save_mpc_result_as_static_html) {
    std::ofstream data_file;
    data_file.open(options.static_html_filename);
    data_file << meshcat_->StaticHtml();
    data_file.close();
  }

  // Print profiling info
  std::cout << TableOfAverages() << std::endl;
}

TrajectoryOptimizerSolution<double> TrajOptExample::SolveTrajectoryOptimization(
    const TrajOptExampleParams& options) const {
  // Create a system model
  // N.B. we need a whole diagram, including scene_graph, to handle contact
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = options.time_step;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  CreatePlantModel(&plant);
  plant.Finalize();
  const int nv = plant.num_velocities();

  auto diagram = builder.Build();

  // Define the optimization problem
  ProblemDefinition opt_prob;
  SetProblemDefinition(options, &opt_prob);

  // Normalize quaternions in the reference
  // TODO(vincekurtz): consider moving this to SetProblemDefinition
  NormalizeQuaternions(plant, &opt_prob.q_nom);

  // Set our solver parameters
  SolverParameters solver_params;
  SetSolverParameters(options, &solver_params);

  // Establish an initial guess
  std::vector<VectorXd> q_guess = MakeLinearInterpolation(
      opt_prob.q_init, options.q_guess, opt_prob.num_steps + 1);
  NormalizeQuaternions(plant, &q_guess);

  // N.B. This should always be the case, and is checked by the solver. However,
  // sometimes floating point + normalization stuff makes q_guess != q_init, so
  // we'll just doubly enforce that here
  DRAKE_DEMAND((q_guess[0] - opt_prob.q_init).norm() < 1e-8);
  q_guess[0] = opt_prob.q_init;

  // Visualize the target trajectory and initial guess, if requested
  if (options.play_target_trajectory) {
    PlayBackTrajectory(opt_prob.q_nom, options.time_step);
  }
  if (options.play_initial_guess) {
    PlayBackTrajectory(q_guess, options.time_step);
  }

  // Solve the optimzation problem
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        solver_params);
  TrajectoryOptimizerSolution<double> solution;
  TrajectoryOptimizerStats<double> stats;
  ConvergenceReason reason;
  SolverFlag status = optimizer.Solve(q_guess, &solution, &stats, &reason);
  if (status == SolverFlag::kSuccess) {
    std::cout << "Solved in " << stats.solve_time << " seconds." << std::endl;
  } else if (status == SolverFlag::kMaxIterationsReached) {
    std::cout << "Maximum iterations reached in " << stats.solve_time
              << " seconds." << std::endl;
  } else {
    std::cout << "Solver failed!" << std::endl;
  }

  std::cout << "Convergence reason: "
            << DecodeConvergenceReasons(reason) + ".\n";

  // Report maximum torques on all DoFs
  VectorXd tau_max = VectorXd::Zero(nv);
  VectorXd abs_tau_t = VectorXd::Zero(nv);
  for (int t = 0; t < options.num_steps; ++t) {
    abs_tau_t = solution.tau[t].cwiseAbs();
    for (int i = 0; i < nv; ++i) {
      if (abs_tau_t(i) > tau_max(i)) {
        tau_max(i) = abs_tau_t(i);
      }
    }
  }
  std::cout << std::endl;
  std::cout << fmt::format("Max torques: {}", fmt_eigen(tau_max.transpose()))
            << std::endl;

  // Report maximum actuated and unactuated torques
  // TODO(vincekurtz): deal with the fact that B is not well-defined for some
  // systems, such as the block pusher and floating box examples.
  const MatrixXd B = plant.MakeActuationMatrix();
  double tau_max_unactuated = 0;
  double tau_max_actuated = 0;
  for (int i = 0; i < nv; ++i) {
    if (B.row(i).sum() == 0) {
      if (tau_max(i) > tau_max_unactuated) {
        tau_max_unactuated = tau_max(i);
      }
    } else {
      if (tau_max(i) > tau_max_actuated) {
        tau_max_actuated = tau_max(i);
      }
    }
  }

  std::cout << std::endl;
  std::cout << "Max actuated torque   : " << tau_max_actuated << std::endl;
  std::cout << "Max unactuated torque : " << tau_max_unactuated << std::endl;

  // Report desired and final state
  std::cout << std::endl;
  std::cout << fmt::format(
                   "q_nom[T] : {}",
                   fmt_eigen(opt_prob.q_nom[options.num_steps].transpose()))
            << std::endl;
  std::cout << fmt::format("q[T]     : {}",
                           fmt_eigen(solution.q[options.num_steps].transpose()))
            << std::endl;
  std::cout << std::endl;
  std::cout << fmt::format(
                   "v_nom[T] : {}",
                   fmt_eigen(opt_prob.v_nom[options.num_steps].transpose()))
            << std::endl;
  std::cout << fmt::format("v[T]     : {}",
                           fmt_eigen(solution.v[options.num_steps].transpose()))
            << std::endl;

  // Print speed profiling info
  std::cout << std::endl;
  std::cout << TableOfAverages() << std::endl;

  // Save stats to CSV for later plotting
  if (options.save_solver_stats_csv) {
    stats.SaveToCsv("solver_stats.csv");
  }

  // Play back the result on the visualizer
  if (options.play_optimal_trajectory) {
    PlayBackTrajectory(solution.q, options.time_step);
  }

  return solution;
}

void TrajOptExample::PlayBackTrajectory(const std::vector<VectorXd>& q,
                                        const double time_step) const {
  // Create a system diagram that includes the plant and is connected to
  // the meshcat visualizer
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = time_step;

  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  CreatePlantModel(&plant);
  plant.Finalize();

  auto& visualizer =
      MeshcatVisualizerd::AddToBuilder(&builder, scene_graph, meshcat_);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const VectorXd u = VectorXd::Zero(plant.num_actuators());
  plant.get_actuation_input_port().FixValue(&plant_context, u);

  // Set up a recording for later playback in Meshcat
  MeshcatAnimation* animation = visualizer.StartRecording();
  animation->set_autoplay(false);

  // Step through q, setting the plant positions at each step accordingly
  const int N = q.size();
  for (int t = 0; t < N; ++t) {
    diagram_context->SetTime(t * time_step);
    plant.SetPositions(&plant_context, q[t]);
    diagram->ForcedPublish(*diagram_context);

    // Hack to make the playback roughly realtime
    // TODO(vincekurtz): add realtime rate option?
    std::this_thread::sleep_for(std::chrono::duration<double>(time_step));
  }
  visualizer.StopRecording();
  visualizer.PublishRecording();
}

void TrajOptExample::SetProblemDefinition(const TrajOptExampleParams& options,
                                          ProblemDefinition* opt_prob) const {
  opt_prob->num_steps = options.num_steps;

  // Initial state
  opt_prob->q_init = options.q_init;
  opt_prob->v_init = options.v_init;

  // Cost weights
  opt_prob->Qq = options.Qq.asDiagonal();
  opt_prob->Qv = options.Qv.asDiagonal();
  opt_prob->Qf_q = options.Qfq.asDiagonal();
  opt_prob->Qf_v = options.Qfv.asDiagonal();
  opt_prob->R = options.R.asDiagonal();

  // Check which DoFs the cost is updated relative to the initial condition for
  VectorX<bool> q_nom_relative = options.q_nom_relative_to_q_init;
  if (q_nom_relative.size() == 0) {
    // If not specified, assume the nominal trajectory is not relative to the
    // initial conditions.
    q_nom_relative = VectorX<bool>::Constant(options.q_init.size(), false);
  }

  // Target state at each timestep
  VectorXd q_nom_start = options.q_nom_start;
  VectorXd q_nom_end = options.q_nom_end;
  q_nom_start += q_nom_relative.cast<double>().cwiseProduct(options.q_init);
  q_nom_end += q_nom_relative.cast<double>().cwiseProduct(options.q_init);
  opt_prob->q_nom =
      MakeLinearInterpolation(q_nom_start, q_nom_end, options.num_steps + 1);

  opt_prob->v_nom.push_back(opt_prob->v_init);
  for (int t = 1; t <= options.num_steps; ++t) {
    if (options.q_init.size() == options.v_init.size()) {
      // No quaternion DoFs, so compute v_nom from q_nom
      opt_prob->v_nom.push_back((opt_prob->q_nom[t] - opt_prob->q_nom[t - 1]) /
                                options.time_step);
    } else {
      // Set v_nom = v_init for systems with quaternion DoFs
      // TODO(vincekurtz): enable better specification of v_nom for
      // floating-base systems
      opt_prob->v_nom.push_back(opt_prob->v_init);
    }
  }
}

void TrajOptExample::SetSolverParameters(
    const TrajOptExampleParams& options,
    SolverParameters* solver_params) const {
  if (options.linesearch == "backtracking") {
    solver_params->linesearch_method = LinesearchMethod::kBacktracking;
  } else if (options.linesearch == "armijo") {
    solver_params->linesearch_method = LinesearchMethod::kArmijo;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown linesearch method '{}'", options.linesearch));
  }

  if (options.gradients_method == "forward_differences") {
    solver_params->gradients_method = GradientsMethod::kForwardDifferences;
  } else if (options.gradients_method == "central_differences") {
    solver_params->gradients_method = GradientsMethod::kCentralDifferences;
  } else if (options.gradients_method == "central_differences4") {
    solver_params->gradients_method = GradientsMethod::kCentralDifferences4;
  } else if (options.gradients_method == "autodiff") {
    solver_params->gradients_method = GradientsMethod::kAutoDiff;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown gradient method '{}'", options.gradients_method));
  }

  if (options.method == "linesearch") {
    solver_params->method = SolverMethod::kLinesearch;
  } else if (options.method == "trust_region") {
    solver_params->method = SolverMethod::kTrustRegion;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown solver method '{}'", options.method));
  }

  if (options.linear_solver == "pentadiagonal_lu") {
    solver_params->linear_solver =
        SolverParameters::LinearSolverType::kPentaDiagonalLu;
  } else if (options.linear_solver == "dense_ldlt") {
    solver_params->linear_solver =
        SolverParameters::LinearSolverType::kDenseLdlt;
  } else if (options.linear_solver == "petsc") {
    solver_params->linear_solver = SolverParameters::LinearSolverType::kPetsc;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown linear solver '{}'", options.linear_solver));
  }

  solver_params->petsc_parameters.relative_tolerance =
      options.petsc_rel_tolerance;

  // PETSc solver type.
  if (options.petsc_solver == "cg") {
    solver_params->petsc_parameters.solver_type =
        SolverParameters::PetscSolverPatameters::SolverType::kConjugateGradient;
  } else if (options.petsc_solver == "direct") {
    solver_params->petsc_parameters.solver_type =
        SolverParameters::PetscSolverPatameters::SolverType::kDirect;
  } else if (options.petsc_solver == "minres") {
    solver_params->petsc_parameters.solver_type =
        SolverParameters::PetscSolverPatameters::SolverType::kMINRES;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown PETSc solver '{}'", options.petsc_solver));
  }

  // PETSc preconditioner.
  if (options.petsc_preconditioner == "none") {
    solver_params->petsc_parameters.preconditioner_type =
        SolverParameters::PetscSolverPatameters::PreconditionerType::kNone;
  } else if (options.petsc_preconditioner == "chol") {
    solver_params->petsc_parameters.preconditioner_type =
        SolverParameters::PetscSolverPatameters::PreconditionerType::kCholesky;
  } else if (options.petsc_preconditioner == "ichol") {
    solver_params->petsc_parameters.preconditioner_type = SolverParameters::
        PetscSolverPatameters::PreconditionerType::kIncompleteCholesky;
  } else {
    throw std::runtime_error(fmt::format("Unknown PETSc preconditioner '{}'",
                                         options.petsc_preconditioner));
  }

  solver_params->max_iterations = options.max_iters;
  solver_params->max_linesearch_iterations = 60;
  solver_params->print_debug_data = options.print_debug_data;
  solver_params->linesearch_plot_every_iteration =
      options.linesearch_plot_every_iteration;

  solver_params->convergence_tolerances = options.tolerances;

  solver_params->proximal_operator = options.proximal_operator;
  solver_params->rho_proximal = options.rho_proximal;

  // Set contact parameters
  solver_params->contact_stiffness = options.contact_stiffness;
  solver_params->dissipation_velocity = options.dissipation_velocity;
  solver_params->friction_coefficient = options.friction_coefficient;
  solver_params->stiction_velocity = options.stiction_velocity;
  solver_params->smoothing_factor = options.smoothing_factor;

  // Set parameters for making contour plot of the first two variables
  solver_params->save_contour_data = options.save_contour_data;
  solver_params->contour_q1_min = options.contour_q1_min;
  solver_params->contour_q1_max = options.contour_q1_max;
  solver_params->contour_q2_min = options.contour_q2_min;
  solver_params->contour_q2_max = options.contour_q2_max;

  // Parameters for making line plots of the first variable
  solver_params->save_lineplot_data = options.save_lineplot_data;
  solver_params->lineplot_q_min = options.lineplot_q_min;
  solver_params->lineplot_q_max = options.lineplot_q_max;

  // Flag for printing iteration data
  solver_params->verbose = options.verbose;

  // Whether to normalize quaterions between iterations
  solver_params->normalize_quaternions = options.normalize_quaternions;

  // Type of Hessian approximation
  solver_params->exact_hessian = options.exact_hessian;

  // Hessian rescaling
  solver_params->scaling = options.scaling;

  if (options.scaling_method == "sqrt") {
    solver_params->scaling_method = ScalingMethod::kSqrt;
  } else if (options.scaling_method == "adaptive_sqrt") {
    solver_params->scaling_method = ScalingMethod::kAdaptiveSqrt;
  } else if (options.scaling_method == "double_sqrt") {
    solver_params->scaling_method = ScalingMethod::kDoubleSqrt;
  } else if (options.scaling_method == "adaptive_double_sqrt") {
    solver_params->scaling_method = ScalingMethod::kAdaptiveDoubleSqrt;
  } else {
    throw std::runtime_error(
        fmt::format("Unknown scaling method '{}'", options.scaling_method));
  }

  // Equality constriant (unactuated DoF torques) enforcement
  solver_params->equality_constraints = options.equality_constraints;

  // Maximum and initial trust region radius
  solver_params->Delta0 = options.Delta0;
  solver_params->Delta_max = options.Delta_max;

  // Number of threads
  solver_params->num_threads = options.num_threads;

  // Check which DoFs the cost is updated relative to the initial condition for
  VectorX<bool> q_nom_relative = options.q_nom_relative_to_q_init;
  if (q_nom_relative.size() == 0) {
    // If not specified, assume the nominal trajectory is not relative to the
    // initial conditions.
    q_nom_relative = VectorX<bool>::Constant(options.q_init.size(), false);
  }
  solver_params->q_nom_relative_to_q_init = q_nom_relative;
}

void TrajOptExample::NormalizeQuaternions(const MultibodyPlant<double>& plant,
                                          std::vector<VectorXd>* q) const {
  const int num_steps = q->size() - 1;
  for (const multibody::BodyIndex& index : plant.GetFloatingBaseBodies()) {
    const multibody::Body<double>& body = plant.get_body(index);
    const int q_start = body.floating_positions_start();
    DRAKE_DEMAND(body.has_quaternion_dofs());
    for (int t = 0; t <= num_steps; ++t) {
      auto body_qs = q->at(t).segment<4>(q_start);
      body_qs.normalize();
    }
  }
}

}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
