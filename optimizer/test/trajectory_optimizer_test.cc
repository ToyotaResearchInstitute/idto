#include "optimizer/trajectory_optimizer.h"

#include <algorithm>

#include "optimizer/inverse_dynamics_partials.h"
#include "optimizer/penta_diagonal_matrix.h"
#include "optimizer/penta_diagonal_solver.h"
#include "optimizer/problem_definition.h"
#include "optimizer/trajectory_optimizer_state.h"
#include "optimizer/trajectory_optimizer_workspace.h"
#include "optimizer/velocity_partials.h"
#include "utils/eigen_matrix_compare.h"
#include "utils/find_resource.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/multibody/tree/planar_joint.h>
#include <drake/systems/framework/diagram_builder.h>
#include <gtest/gtest.h>

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace idto {

using utils::FindIdtoResource;

namespace optimizer {

class TrajectoryOptimizerTester {
 public:
  TrajectoryOptimizerTester() = delete;

  static double CalcCost(const TrajectoryOptimizer<double>& optimizer,
                         const std::vector<VectorXd>& q,
                         const std::vector<VectorXd>& v,
                         const std::vector<VectorXd>& tau,
                         TrajectoryOptimizerWorkspace<double>* workspace) {
    return optimizer.CalcCost(q, v, tau, workspace);
  }

  static void CalcVelocities(const TrajectoryOptimizer<double>& optimizer,
                             const std::vector<VectorXd>& q,
                             const std::vector<MatrixXd>& Nplus,
                             std::vector<VectorXd>* v) {
    optimizer.CalcVelocities(q, Nplus, v);
  }

  static void CalcAccelerations(const TrajectoryOptimizer<double>& optimizer,
                                const std::vector<VectorXd>& v,
                                std::vector<VectorXd>* a) {
    optimizer.CalcAccelerations(v, a);
  }

  static void CalcInverseDynamics(const TrajectoryOptimizer<double>& optimizer,
                                  const TrajectoryOptimizerState<double>& state,
                                  const std::vector<VectorXd>& a,
                                  std::vector<VectorXd>* tau) {
    optimizer.CalcInverseDynamics(state, a, tau);
  }

  static void CalcInverseDynamicsPartials(
      const TrajectoryOptimizer<double>& optimizer,
      const TrajectoryOptimizerState<double>& state,
      InverseDynamicsPartials<double>* id_partials) {
    optimizer.CalcInverseDynamicsPartials(state, id_partials);
  }

  static void CalcGradientFiniteDiff(
      const TrajectoryOptimizer<double>& optimizer,
      const TrajectoryOptimizerState<double>& state,
      drake::EigenPtr<VectorXd> g) {
    optimizer.CalcGradientFiniteDiff(state, g);
  }

  static double CalcTrustRatio(
      const TrajectoryOptimizer<double>& optimizer,
      const TrajectoryOptimizerState<double>& state, const VectorXd& dq,
      TrajectoryOptimizerState<double>* scratch_state) {
    return optimizer.CalcTrustRatio(state, dq, scratch_state);
  }

  static bool CalcDoglegPoint(const TrajectoryOptimizer<double>& optimizer,
                              const TrajectoryOptimizerState<double>& state,
                              const double Delta, VectorXd* dq, VectorXd* dqH) {
    return optimizer.CalcDoglegPoint(state, Delta, dq, dqH);
  }
};

namespace internal {

using drake::AutoDiffXd;
using drake::CompareMatrices;
using drake::MatrixCompareType;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::DiscreteContactSolver;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::PlanarJoint;
using drake::multibody::RigidBody;
using drake::systems::DiagramBuilder;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

/**
 * Test optimization for a simple system with quaternion DoFs.
 */
GTEST_TEST(TrajectoryOptimizerTest, QuaternionDofs) {
  // Set up a simple example system
  const double dt = 1e-2;
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const RigidBody<double>& body = plant.AddRigidBody(
      "body", drake::multibody::SpatialInertia<double>::MakeUnitary());
  plant.Finalize();
  auto diagram = builder.Build();

  const int nq = plant.num_positions();
  const int nv = plant.num_velocities();
  EXPECT_TRUE(body.is_floating());
  ASSERT_EQ(nq, 7);
  ASSERT_EQ(nv, 6);

  // Define a super simple optimization problem. We are only interested in
  // configurations.
  const int num_steps = 1;
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = VectorXd(7);
  opt_prob.q_init << 1, 0, 0, 0, 0, 0, 0;
  opt_prob.v_init = drake::Vector6d::Zero();
  opt_prob.q_nom.resize(num_steps + 1, VectorXd::Zero(nq));
  opt_prob.v_nom.resize(num_steps + 1, VectorXd::Zero(nv));

  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q;
  q.push_back(opt_prob.q_init);
  q.push_back(opt_prob.q_init);
  q[1](0) = 0.0;  // change orientation
  q[1](1) = 0.1;
  state.set_q(q);

  // Compute N+
  std::vector<MatrixXd> Nplus = optimizer.EvalNplus(state);
  for (int t = 0; t <= num_steps; ++t) {
    EXPECT_TRUE(Nplus[t].cols() == nq);
    EXPECT_TRUE(Nplus[t].rows() == nv);
  }

  // Compute velocities
  const std::vector<VectorXd>& v = optimizer.EvalV(state);
  for (int t = 0; t <= num_steps; ++t) {
    EXPECT_TRUE(v[t].size() == nv);
  }

  // Compute velocity partials
  const VelocityPartials<double>& v_partials =
      optimizer.EvalVelocityPartials(state);
  for (int t = 0; t <= num_steps; ++t) {
    EXPECT_TRUE(v_partials.dvt_dqt[t].cols() == nq);
    EXPECT_TRUE(v_partials.dvt_dqm[t].cols() == nq);
    EXPECT_TRUE(v_partials.dvt_dqt[t].rows() == nv);
    EXPECT_TRUE(v_partials.dvt_dqm[t].rows() == nv);
  }
}

/**
 * Test different methods of computing gradients through contact.
 */
GTEST_TEST(TrajectoryOptimizerTest, ContactGradientMethods) {
  // Set up an example system with sphere-sphere contact
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = 1.0;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  Parser(&plant).AddModels(FindIdtoResource("idto/models/spinner_sphere.urdf"));
  plant.Finalize();
  auto diagram = builder.Build();

  const int num_steps = 2;
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector3d(0.2, 1.5, 0.0);
  opt_prob.v_init = Vector3d(0.0, 0.0, 0.0);
  opt_prob.q_nom.resize(num_steps + 1, Vector3d::Zero());
  opt_prob.v_nom.resize(num_steps + 1, Vector3d::Zero());
  SolverParameters solver_params;
  solver_params.contact_stiffness = 100;
  solver_params.dissipation_velocity = 0.1;
  solver_params.friction_coefficient = 0.5;

  // Create optimizers for each potential gradient method
  solver_params.gradients_method = GradientsMethod::kForwardDifferences;
  TrajectoryOptimizer<double> optimizer_fd(diagram.get(), &plant, opt_prob,
                                           solver_params);
  TrajectoryOptimizerState<double> state_fd = optimizer_fd.CreateState();

  solver_params.gradients_method = GradientsMethod::kCentralDifferences;
  TrajectoryOptimizer<double> optimizer_cd(diagram.get(), &plant, opt_prob,
                                           solver_params);
  TrajectoryOptimizerState<double> state_cd = optimizer_cd.CreateState();

  solver_params.gradients_method = GradientsMethod::kAutoDiff;
  TrajectoryOptimizer<double> optimizer_ad(diagram.get(), &plant, opt_prob,
                                           solver_params);
  TrajectoryOptimizerState<double> state_ad = optimizer_ad.CreateState();

  // Make some fake data
  std::vector<VectorXd> q;
  q.push_back(opt_prob.q_init);
  q.push_back(Vector3d(0.4, 1.5, 0.0));
  q.push_back(Vector3d(0.3, 1.4, 0.0));
  state_cd.set_q(q);
  state_fd.set_q(q);
  state_ad.set_q(q);

  // Sanity check that forward dynamics match for all of the methods
  const std::vector<VectorXd> tau_fd = optimizer_fd.EvalTau(state_fd);
  const std::vector<VectorXd> tau_cd = optimizer_cd.EvalTau(state_cd);
  const std::vector<VectorXd> tau_ad = optimizer_ad.EvalTau(state_ad);

  const double kEpsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(tau_fd[0], tau_cd[0], kEpsilon,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(tau_fd[0], tau_ad[0], kEpsilon,
                              MatrixCompareType::relative));

  // Compute inverse dynamics partials for each method
  InverseDynamicsPartials<double> idp_fd(num_steps, 3, 3);
  TrajectoryOptimizerTester::CalcInverseDynamicsPartials(optimizer_fd, state_fd,
                                                         &idp_fd);

  InverseDynamicsPartials<double> idp_cd(num_steps, 3, 3);
  TrajectoryOptimizerTester::CalcInverseDynamicsPartials(optimizer_cd, state_cd,
                                                         &idp_cd);

  InverseDynamicsPartials<double> idp_ad(num_steps, 3, 3);
  TrajectoryOptimizerTester::CalcInverseDynamicsPartials(optimizer_ad, state_ad,
                                                         &idp_ad);

  // Verify that inverse dynamics partials match, at least roughly
  const double kToleranceForwardDifference = 100 * sqrt(kEpsilon);
  const double kToleranceCentralDifference = 10 * sqrt(kEpsilon);
  for (int t = 1; t < num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(idp_fd.dtau_dqm[t], idp_ad.dtau_dqm[t],
                                kToleranceForwardDifference,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(idp_cd.dtau_dqm[t], idp_ad.dtau_dqm[t],
                                kToleranceCentralDifference,
                                MatrixCompareType::relative));

    EXPECT_TRUE(CompareMatrices(idp_fd.dtau_dqt[t], idp_ad.dtau_dqt[t],
                                kToleranceForwardDifference,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(idp_cd.dtau_dqt[t], idp_ad.dtau_dqt[t],
                                kToleranceCentralDifference,
                                MatrixCompareType::relative));

    EXPECT_TRUE(CompareMatrices(idp_fd.dtau_dqp[t], idp_ad.dtau_dqp[t],
                                kToleranceForwardDifference,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(idp_cd.dtau_dqp[t], idp_ad.dtau_dqp[t],
                                kToleranceCentralDifference,
                                MatrixCompareType::relative));
  }
}

/**
 * Test our computation of the dogleg point for trust-region optimization
 */
GTEST_TEST(TrajectoryOptimizerTest, DoglegPoint) {
  // Define a super simple optimization problem, where we know the solution is
  // q = [0 0 0].
  const int num_steps = 2;
  const double dt = 5e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.0);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.R = 1.0 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(0.0));
    opt_prob.v_nom.push_back(drake::Vector1d(0.0));
  }

  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  SolverParameters solver_params;
  solver_params.scaling = false;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        solver_params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Choose a q that is away from the optimal solution
  std::vector<VectorXd> q;
  q.push_back(drake::Vector1d(0.0));
  q.push_back(drake::Vector1d(1.5));
  q.push_back(drake::Vector1d(1.5));
  state.set_q(q);

  // Allocate variables for small, medium, and large trust region sizes.
  VectorXd dq_small(num_steps + 1);
  VectorXd dq_medium(num_steps + 1);
  VectorXd dq_large(num_steps + 1);
  VectorXd dqH(num_steps + 1);  // dummy variable for newton step
  const double Delta_small = 1e-3;
  const double Delta_medium = 1.0;  // hand-chosen to intersect the second leg
  const double Delta_large = 1e3;
  bool trust_region_constraint_active;
  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;

  // Compute the dogleg point for a very small trust region
  trust_region_constraint_active = TrajectoryOptimizerTester::CalcDoglegPoint(
      optimizer, state, Delta_small, &dq_small, &dqH);

  EXPECT_TRUE(trust_region_constraint_active);
  EXPECT_NEAR(dq_small.norm(), Delta_small, kTolerance);

  // Compute the dogleg point for a very large trust region
  trust_region_constraint_active = TrajectoryOptimizerTester::CalcDoglegPoint(
      optimizer, state, Delta_large, &dq_large, &dqH);

  EXPECT_FALSE(trust_region_constraint_active);
  EXPECT_GT(dq_large.norm(), dq_small.norm());

  // Compute the dogleg point for a medium-sized trust region
  trust_region_constraint_active = TrajectoryOptimizerTester::CalcDoglegPoint(
      optimizer, state, Delta_medium, &dq_medium, &dqH);

  EXPECT_TRUE(trust_region_constraint_active);
  EXPECT_NEAR(dq_medium.norm(), Delta_medium, kTolerance);
  EXPECT_GT(dq_large.norm(), dq_medium.norm());
  EXPECT_GT(dq_medium.norm(), dq_small.norm());
}

/**
 * Test our computation of the trust ratio, which should be exactly 1
 * when our quadratic model of the cost matches the true cost. This is the case
 * for the simple pendulum without gravity.
 */
GTEST_TEST(TrajectoryOptimizerTest, TrustRatio) {
  // Define an optimization problem
  const int num_steps = 5;
  const double dt = 5e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 2.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 3.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 4.0 * MatrixXd::Identity(1, 1);
  opt_prob.R = 5.0 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(M_PI));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.3));
  }

  // Create a pendulum model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  SolverParameters solver_params;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        solver_params);

  // Create state, scratch state, and an initial guess
  std::vector<VectorXd> q_guess;
  for (int t = 0; t <= num_steps; ++t) {
    q_guess.push_back(opt_prob.q_init + 0.01 * t * VectorXd::Ones(1));
  }
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerState<double> scratch_state = optimizer.CreateState();
  state.set_q(q_guess);
  scratch_state.set_q(q_guess);

  // Solve for the search direction
  const PentaDiagonalMatrix<double>& H = optimizer.EvalHessian(state);
  const VectorXd& g = optimizer.EvalGradient(state);
  VectorXd dq = -g;
  PentaDiagonalFactorization Hchol(H);
  Hchol.SolveInPlace(&dq);

  // Compute the trust ratio, which should be 1
  double trust_ratio = TrajectoryOptimizerTester::CalcTrustRatio(
      optimizer, state, dq, &scratch_state);

  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(trust_ratio, 1.0, kTolerance);
}

/**
 * Test our optimizer with a simple pendulum swingup task.
 */
GTEST_TEST(TrajectoryOptimizerTest, PendulumSwingup) {
  // Define the optimization problem
  const int num_steps = 20;
  const double dt = 5e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 1000 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.01 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(M_PI));
    opt_prob.v_nom.push_back(drake::Vector1d(0));
  }

  // Create a pendulum model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  SolverParameters solver_params;
  solver_params.max_iterations = 20;
  solver_params.verbose = false;
  solver_params.check_convergence = true;
  solver_params.convergence_tolerances.rel_cost_reduction = 1e-5;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        solver_params);

  // Set an initial guess
  std::vector<VectorXd> q_guess;
  for (int t = 0; t <= num_steps; ++t) {
    q_guess.push_back(opt_prob.q_init);
  }

  // Solve the optimization problem
  TrajectoryOptimizerSolution<double> solution;
  TrajectoryOptimizerStats<double> stats;

  SolverFlag status = optimizer.Solve(q_guess, &solution, &stats);
  EXPECT_EQ(status, SolverFlag::kSuccess);

  // With such a large penalty on the final position, and such a low control
  // penalty, we should be close to the target position at the last timestep.
  EXPECT_NEAR(solution.q[num_steps][0], opt_prob.q_nom[num_steps][0], 1e-3);
}

/**
 * Test our computation of the Hessian on a system
 * with more than one DoF.
 */
GTEST_TEST(TrajectoryOptimizerTest, HessianAcrobot) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector2d(0.1, 0.2);
  opt_prob.v_init = Vector2d(-0.01, 0.03);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(2, 2);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(2, 2);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(2, 2);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(2, 2);
  opt_prob.R = 0.01 * MatrixXd::Identity(2, 2);

  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(Vector2d(1.5, -0.1));
    opt_prob.v_nom.push_back(Vector2d(0.2, 0.1));
  }

  // Create an acrobot model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      FindIdtoResource("idto/models/acrobot/acrobot.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + dt * opt_prob.v_init;
  }
  state.set_q(q);

  // Compute the Gauss-Newton Hessian approximation numerically
  const int nq = plant.num_positions();
  const int num_vars = nq * (num_steps + 1);
  PentaDiagonalMatrix<double> H_sparse(num_steps + 1, nq);
  optimizer.CalcHessian(state, &H_sparse);
  MatrixXd H = H_sparse.MakeDense();

  // Set up an autodiff copy of the optimizer and plant
  auto diagram_ad = drake::systems::System<double>::ToAutoDiffXd(*diagram);
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant.get_name()));
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(diagram_ad.get(), &plant_ad,
                                               opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1, VectorX<AutoDiffXd>(2));
  int ad_idx = 0;  // index for autodiff variables
  for (int t = 0; t <= num_steps; ++t) {
    for (int i = 0; i < nq; ++i) {
      q_ad[t].segment<1>(i) =
          drake::math::InitializeAutoDiff(q[t].segment<1>(i), num_vars, ad_idx);
      ++ad_idx;
    }
  }
  state_ad.set_q(q_ad);
  AutoDiffXd L_ad = optimizer_ad.EvalCost(state_ad);  // forces cache update

  // Formulate an equivalent least-squares problem, where
  //
  //    L(q) = 1/2 r(q)'*r(q)
  //    J = dr(q)/dq
  //
  // and the gradient and Gauss-Newton Hessian approximation are given by
  //
  //    g = J'r,
  //    H = J'J.
  //
  Matrix2d Qq_sqrt = (dt * 2 * opt_prob.Qq).cwiseSqrt();  // diagonal matrices
  Matrix2d Qv_sqrt = (dt * 2 * opt_prob.Qv).cwiseSqrt();
  Matrix2d R_sqrt = (dt * 2 * opt_prob.R).cwiseSqrt();
  Matrix2d Qfq_sqrt = (2 * opt_prob.Qf_q).cwiseSqrt();
  Matrix2d Qfv_sqrt = (2 * opt_prob.Qf_v).cwiseSqrt();

  const std::vector<VectorX<AutoDiffXd>>& v_ad = optimizer_ad.EvalV(state_ad);
  const std::vector<VectorX<AutoDiffXd>>& u_ad = optimizer_ad.EvalTau(state_ad);

  // Here we construct the residual
  //        [        ...           ]
  //        [ sqrt(Qq)*(q_t-q_nom) ]
  // r(q) = [ sqrt(Qv)*(v_t-v_nom) ]
  //        [ sqrt(R) * tau_t      ]
  //        [        ...           ]
  //        [ sqrt(Qfq)*(q_T-q_nom)]
  //        [ sqrt(Qfv)*(v_T-v_nom)]
  VectorX<AutoDiffXd> r(num_steps * 6 + 4);
  r.setZero();
  for (int t = 0; t < num_steps; ++t) {
    r.segment(t * 6, 2) = Qq_sqrt * (q_ad[t] - opt_prob.q_nom[t]);
    r.segment(t * 6 + 2, 2) = Qv_sqrt * (v_ad[t] - opt_prob.v_nom[t]);
    r.segment(t * 6 + 4, 2) = R_sqrt * u_ad[t];
  }
  r.segment(num_steps * 6, 2) =
      Qfq_sqrt * (q_ad[num_steps] - opt_prob.q_nom[num_steps]);
  r.segment(num_steps * 6 + 2, 2) =
      Qfv_sqrt * (v_ad[num_steps] - opt_prob.v_nom[num_steps]);

  MatrixXd J = drake::math::ExtractGradient(r);
  AutoDiffXd L_lsqr = 0.5 * r.transpose() * r;
  VectorXd g_lsqr = J.transpose() * drake::math::ExtractValue(r);
  MatrixXd H_lsqr = J.transpose() * J;

  // Check that the cost from our least-squares formulation is correct
  const double kToleranceCost = 10 * std::numeric_limits<double>::epsilon();
  double L = optimizer.EvalCost(state);
  EXPECT_NEAR(L, L_lsqr.value(), kToleranceCost);

  // Check that the gradient from our least-squares formulation matches what we
  // compute analytically. Primary source of error is our use of finite
  // differences to compute dtau/dq. We ignore the top block of the gradient,
  // since this is overwritten with zero, because q0 is fixed.
  const double kToleranceGrad =
      sqrt(std::numeric_limits<double>::epsilon()) / dt;
  VectorXd g(num_vars);
  optimizer.CalcGradient(state, &g);
  EXPECT_TRUE(CompareMatrices(g.bottomRows(num_steps * nq),
                              g_lsqr.bottomRows(num_steps * nq), kToleranceGrad,
                              MatrixCompareType::relative));

  // Check that the Hessian approximation from least-squares (Gauss-Newton)
  // matches what we compute analytically. Finite differences is again the
  // primary source of error. We ignore the rows and columns, since these are
  // overwritten to fix q0.
  const double kToleranceHess = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(
      CompareMatrices(H.bottomRightCorner(num_steps * nq, num_steps * nq),
                      H_lsqr.bottomRightCorner(num_steps * nq, num_steps * nq),
                      kToleranceHess, MatrixCompareType::relative));
}

/**
 * Compare the Gauss-Newton Hessian with the exact (autodiff) Hessian.
 */
GTEST_TEST(TrajectoryOptimizerTest, HessianPendulum) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.05 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(0.1));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.1));
  }

  // Create a pendulum model without gravity
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();
  auto diagram = builder.Build();

  // Create two optimizers: one that uses a Gauss-Newton Hessian approximation
  // and the other that uses an exact (autodiff) Hessian
  TrajectoryOptimizer<double> optimizer_gn(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state_gn = optimizer_gn.CreateState();

  SolverParameters params;
  params.exact_hessian = true;
  TrajectoryOptimizer<double> optimizer_exact(diagram.get(), &plant, opt_prob,
                                              params);
  TrajectoryOptimizerState<double> state_exact = optimizer_exact.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.3 * dt * VectorXd::Ones(1);
  }
  state_gn.set_q(q);
  state_exact.set_q(q);

  // Compare the Hessians
  const MatrixXd H_gn = optimizer_gn.EvalHessian(state_gn).MakeDense();
  const MatrixXd H_exact = optimizer_exact.EvalHessian(state_exact).MakeDense();

  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon()) / dt;
  EXPECT_TRUE(
      CompareMatrices(H_gn, H_exact, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, AutodiffGradient) {
  // Test our gradient computations using autodiff
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(0.1));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.1));
  }

  // Create a pendulum model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * VectorXd::Ones(1);
  }
  state.set_q(q);

  // Compute the gradient analytically
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Compute the gradient using autodiff
  auto diagram_ad = drake::systems::System<double>::ToAutoDiffXd(*diagram);
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant.get_name()));
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(diagram_ad.get(), &plant_ad,
                                               opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps, plant_ad);

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1);
  for (int t = 0; t <= num_steps; ++t) {
    q_ad[t] = drake::math::InitializeAutoDiff(q[t], num_steps + 1, t);
  }
  state_ad.set_q(q_ad);

  AutoDiffXd cost_ad = optimizer_ad.EvalCost(state_ad);
  VectorXd g_ad = cost_ad.derivatives();

  // We neglect the top row of the gradient, since we are setting it to zero.
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(g.bottomRows(num_steps),
                              g_ad.bottomRows(num_steps), kTolerance,
                              MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientKuka) {
  const int num_steps = 3;
  const double dt = 1e-2;

  // Create a robot model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  std::string url =
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_collision.urdf";
  Parser(&plant).AddModelsFromUrl(url);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();
  auto diagram = builder.Build();

  // Set up an optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = VectorXd(7);
  opt_prob.q_init.setConstant(0.1);
  opt_prob.v_init = VectorXd(7);
  opt_prob.v_init.setConstant(0.2);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(7, 7);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(7, 7);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(7, 7);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(7, 7);
  opt_prob.R = 0.5 * MatrixXd::Identity(7, 7);

  VectorXd q_nom(7);
  VectorXd v_nom(7);
  q_nom.setConstant(-0.2);
  v_nom.setConstant(-0.1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(q_nom);
    opt_prob.v_nom.push_back(v_nom);
  }

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * VectorXd::Ones(7);
  }
  state.set_q(q);

  // Compute the ("ground truth") gradient with finite differences
  VectorXd g_gt(plant.num_positions() * (num_steps + 1));
  TrajectoryOptimizerTester::CalcGradientFiniteDiff(optimizer, state, &g_gt);

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Looks like we're losing a lot of precision here, but I think that's because
  // it comes from several sources:
  //    1) finite differences give us eps^(1/2)
  //    2) computing v(q) gives us a factor of 1/dt
  //    3) computing tau(v(q)) gives us an additional factor of 1/dt
  const double kTolerance =
      pow(std::numeric_limits<double>::epsilon(), 1. / 2.) / dt / dt;
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientPendulumNoGravity) {
  const int num_steps = 10;
  const double dt = 5e-2;

  // Set up a system model: pendulum w/o gravity yields a linear system
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();
  auto diagram = builder.Build();

  // Set up a toy optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.0);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(M_PI));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.1));
  }

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Create some fake data
  std::vector<VectorXd> q;
  q.push_back(drake::Vector1d(0.0000000000000000000000000));
  q.push_back(drake::Vector1d(0.0950285641187840757204697));
  q.push_back(drake::Vector1d(0.2659896360172592788551071));
  q.push_back(drake::Vector1d(0.4941147113506765831125733));
  q.push_back(drake::Vector1d(0.7608818755930255584019051));
  q.push_back(drake::Vector1d(1.0479359055822168311777887));
  q.push_back(drake::Vector1d(1.3370090901260500704239575));
  q.push_back(drake::Vector1d(1.6098424281109515732168802));
  q.push_back(drake::Vector1d(1.8481068641834854648919872));
  q.push_back(drake::Vector1d(2.0333242222438583368671061));
  q.push_back(drake::Vector1d(2.1467874956452459578315484));
  state.set_q(q);

  // Compute the ground truth gradient with autodiff
  auto diagram_ad = drake::systems::System<double>::ToAutoDiffXd(*diagram);
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant.get_name()));
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(diagram_ad.get(), &plant_ad,
                                               opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps, plant_ad);

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1);
  for (int t = 0; t <= num_steps; ++t) {
    q_ad[t] = drake::math::InitializeAutoDiff(q[t], num_steps + 1, t);
  }
  state_ad.set_q(q_ad);

  AutoDiffXd cost_ad = optimizer_ad.EvalCost(state_ad);
  VectorXd g_gt = cost_ad.derivatives();
  g_gt[0] = 0;  // q0 is fixed

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Even without gravity (a.k.a. linear system), finite differences is only
  // accurate to sqrt(epsilon)
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));

  // Evaluate error in d(tau)/d(q), as compared to the ground truth pendulum
  // model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;

  InverseDynamicsPartials<double> id_partials_gt(num_steps, 1, 1);
  for (int t = 0; t < num_steps; ++t) {
    // dtau[t]/dq[t+1]
    id_partials_gt.dtau_dqp[t](0, 0) = 1 / dt / dt * m * l * l + 1 / dt * b;

    // dtau[t]/dq[t]
    if (t == 0) {
      // q[0] is constant
      id_partials_gt.dtau_dqt[t](0, 0) = 0.0;
    } else {
      id_partials_gt.dtau_dqt[t](0, 0) = -2 / dt / dt * m * l * l - 1 / dt * b;
    }

    // dtau[t]/dq[t-1]
    if (t == 0) {
      // Derivatives w.r.t. q[t-1] do not exist
      id_partials_gt.dtau_dqm[t](0, 0) = NAN;
    } else if (t == 1) {
      // q[0] is constant
      id_partials_gt.dtau_dqm[t](0, 0) = 0.0;
    } else {
      id_partials_gt.dtau_dqm[t](0, 0) = 1 / dt / dt * m * l * l;
    }
  }

  const InverseDynamicsPartials<double>& id_partials =
      optimizer.EvalInverseDynamicsPartials(state);
  for (int t = 1; t < num_steps; ++t) {
    EXPECT_NEAR(id_partials.dtau_dqm[t](0), id_partials_gt.dtau_dqm[t](0),
                10 * kTolerance);
    EXPECT_NEAR(id_partials.dtau_dqt[t](0), id_partials_gt.dtau_dqt[t](0),
                10 * kTolerance);
    EXPECT_NEAR(id_partials.dtau_dqp[t](0), id_partials_gt.dtau_dqp[t](0),
                10 * kTolerance);
  }

  // Check that mass matrix of the plant is truely constant
  auto plant_context = plant.CreateDefaultContext();
  MatrixXd M(1, 1);
  for (int t = 0; t < num_steps; ++t) {
    plant.SetPositions(plant_context.get(), q[t]);
    plant.SetVelocities(plant_context.get(), optimizer.EvalV(state)[t]);
    plant.CalcMassMatrix(*plant_context, &M);

    EXPECT_NEAR(M(0, 0), m * l * l, std::numeric_limits<double>::epsilon());
  }

  // Check our computation of tau(q)
  double tau;
  double tau_gt;
  const std::vector<VectorXd>& v = optimizer.EvalV(state);
  const std::vector<VectorXd>& a = optimizer.EvalA(state);
  const std::vector<VectorXd>& tau_comp = optimizer.EvalTau(state);
  const double kToleranceTau = 10 * std::numeric_limits<double>::epsilon();
  for (int t = 0; t < num_steps; ++t) {
    tau = tau_comp[t](0);
    tau_gt = m * l * l * a[t](0) + b * v[t + 1](0);
    EXPECT_NEAR(tau_gt, tau, kToleranceTau);
  }
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientPendulum) {
  // Set up an optimization problem
  const int num_steps = 5;
  const double dt = 1e-3;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(0.1));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.1));
  }

  // Create a pendulum model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * VectorXd::Ones(1);
  }
  state.set_q(q);

  // Compute the ("ground truth") gradient with finite differences
  VectorXd g_gt(plant.num_positions() * (num_steps + 1));
  TrajectoryOptimizerTester::CalcGradientFiniteDiff(optimizer, state, &g_gt);

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Compare the two
  const double kTolerance = pow(std::numeric_limits<double>::epsilon(), 0.5);
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, PendulumDtauDq) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Set up a system model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create a trajectory optimizer
  ProblemDefinition opt_prob;
  opt_prob.q_init = drake::Vector1d(0.0);
  opt_prob.v_init = drake::Vector1d(0.1);
  opt_prob.num_steps = num_steps;
  opt_prob.q_nom.resize(num_steps + 1, drake::Vector1d(0.0));
  opt_prob.v_nom.resize(num_steps + 1, drake::Vector1d(0.0));
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Create some fake data
  std::vector<VectorXd> q;
  q.push_back(opt_prob.q_init);
  for (int t = 1; t <= num_steps; ++t) {
    q.push_back(drake::Vector1d(0.0 + 0.6 * t));
  }
  state.set_q(q);

  // Compute inverse dynamics partials
  InverseDynamicsPartials<double> grad_data(num_steps, 1, 1);
  std::vector<VectorXd> v(num_steps + 1);
  std::vector<VectorXd> a(num_steps);
  std::vector<VectorXd> tau(num_steps);
  const std::vector<MatrixXd>& Nplus = optimizer.EvalNplus(state);
  TrajectoryOptimizerTester::CalcVelocities(optimizer, q, Nplus, &v);
  TrajectoryOptimizerTester::CalcAccelerations(optimizer, v, &a);
  TrajectoryOptimizerTester::CalcInverseDynamics(optimizer, state, a, &tau);
  TrajectoryOptimizerTester::CalcInverseDynamicsPartials(optimizer, state,
                                                         &grad_data);

  // Compute ground truth partials from the pendulum model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;
  const double g = 9.81;

  InverseDynamicsPartials<double> grad_data_gt(num_steps, 1, 1);
  for (int t = 0; t < num_steps; ++t) {
    // dtau[t]/dq[t+1]
    grad_data_gt.dtau_dqp[t](0, 0) =
        1 / dt / dt * m * l * l + 1 / dt * b + m * g * l * cos(q[t + 1](0));

    // dtau[t]/dq[t]
    grad_data_gt.dtau_dqt[t](0, 0) = -2 / dt / dt * m * l * l - 1 / dt * b;

    if (t == 0) {
      // q[0] is constant
      grad_data_gt.dtau_dqt[t](0, 0) = 0.0;
    }

    // dtau[t]/dq[t-1]
    if (t == 0) {
      // Derivatives w.r.t. q[t-1] do not exist
      grad_data_gt.dtau_dqm[t](0, 0) = NAN;
    } else if (t == 1) {
      grad_data_gt.dtau_dqm[t](0, 0) = 0.0;
    } else {
      grad_data_gt.dtau_dqm[t](0, 0) = 1 / dt / dt * m * l * l;
    }
  }

  // Compare the computed values and the analytical ground truth
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  for (int t = 1; t < num_steps; ++t) {
    // N.B. dX/dq0 is not used, sinze q0 is fixed.
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqm[t], grad_data_gt.dtau_dqm[t],
                                kTolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqt[t], grad_data_gt.dtau_dqt[t],
                                kTolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqp[t], grad_data_gt.dtau_dqp[t],
                                kTolerance, MatrixCompareType::relative));
  }
}

/**
 * Check the precision of our computation of costs using the state abstraction.
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcCostFromState) {
  const int num_steps = 10;
  const double dt = 5e-2;

  // Set up a system model: pendulum w/o gravity yields a linear system
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();
  auto diagram = builder.Build();

  // Set up a toy optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.0);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 10.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.R = 1.0 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(M_PI));
    opt_prob.v_nom.push_back(drake::Vector1d(-0.1));
  }

  // Create some fake data, which are very close to optimality.
  std::vector<VectorXd> q;
  q.push_back(drake::Vector1d(0.0000000000000000000000000));
  q.push_back(drake::Vector1d(0.0950285641187840757204697));
  q.push_back(drake::Vector1d(0.2659896360172592788551071));
  q.push_back(drake::Vector1d(0.4941147113506765831125733));
  q.push_back(drake::Vector1d(0.7608818755930255584019051));
  q.push_back(drake::Vector1d(1.0479359055822168311777887));
  q.push_back(drake::Vector1d(1.3370090901260500704239575));
  q.push_back(drake::Vector1d(1.6098424281109515732168802));
  q.push_back(drake::Vector1d(1.8481068641834854648919872));
  q.push_back(drake::Vector1d(2.0333242222438583368671061));
  q.push_back(drake::Vector1d(2.1467874956452459578315484));

  // Compute the cost as a function of state
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  state.set_q(q);
  double L = optimizer.EvalCost(state);

  // Compute the ground truth cost using an analytical model of the (linear)
  // pendulum dynamics.
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt, and g=0.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;

  double L_gt = 0;
  double qt;
  double vt = opt_prob.v_init[0];
  double vp;
  double ut;
  for (int t = 0; t < num_steps; ++t) {
    qt = q[t][0];
    if (t > 0) {
      vt = (q[t][0] - q[t - 1][0]) / dt;
    }
    vp = (q[t + 1][0] - q[t][0]) / dt;
    ut = m * l * l * (vp - vt) / dt + b * vp;

    L_gt += dt * (qt - opt_prob.q_nom[t](0)) * opt_prob.Qq(0) *
            (qt - opt_prob.q_nom[t](0));
    L_gt += dt * (vt - opt_prob.v_nom[t](0)) * opt_prob.Qv(0) *
            (vt - opt_prob.v_nom[t](0));
    L_gt += dt * ut * opt_prob.R(0) * ut;
  }

  qt = q[num_steps][0];
  vt = (q[num_steps][0] - q[num_steps - 1][0]) / dt;
  L_gt += (qt - opt_prob.q_nom[num_steps](0)) * opt_prob.Qf_q(0) *
          (qt - opt_prob.q_nom[num_steps](0));
  L_gt += (vt - opt_prob.v_nom[num_steps](0)) * opt_prob.Qf_v(0) *
          (vt - opt_prob.v_nom[num_steps](0));

  const double kTolerance = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(L, L_gt, kTolerance);
}

/**
 * Test our computation of the total cost L(q)
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcCost) {
  const int num_steps = 100;
  const double dt = 1e-2;

  // Set up a system model with 2 DoFs
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      FindIdtoResource("idto/models/acrobot/acrobot.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Set up the optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector2d(0.2, 0.1);
  opt_prob.v_init = Vector2d(-0.1, 0.0);
  opt_prob.Qq = 0.1 * Matrix2d::Identity();
  opt_prob.Qv = 0.2 * Matrix2d::Identity();
  opt_prob.Qf_q = 0.3 * Matrix2d::Identity();
  opt_prob.Qf_v = 0.4 * Matrix2d::Identity();
  opt_prob.R = 0.5 * Matrix2d::Identity();
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(Vector2d(1.2, 1.1));
    opt_prob.v_nom.push_back(Vector2d(-1.1, 1.0));
  }

  // Make some fake data
  std::vector<VectorXd> q;
  std::vector<VectorXd> v;
  std::vector<VectorXd> tau;
  for (int t = 0; t < opt_prob.num_steps; ++t) {
    q.push_back(Vector2d(0.2, 0.1));
    v.push_back(Vector2d(-0.1, 0.0));
    tau.push_back(Vector2d(-1.0, 1.0));
  }
  q.push_back(Vector2d(0.2, 0.1));
  v.push_back(Vector2d(-0.1, 0.0));

  // Compute the cost and compare with the true value
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);
  double L =
      TrajectoryOptimizerTester::CalcCost(optimizer, q, v, tau, &workspace);
  double L_gt =
      num_steps * dt * (2 * 0.1 + 2 * 0.2 + 2 * 0.5) + 2 * 0.3 + 2 * 0.4;

  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;
  EXPECT_NEAR(L, L_gt, kTolerance);
}

/**
 * Test our computation of generalized forces
 *
 *   tau_t = InverseDynamics(a_t, v_t, q_t)
 *
 * where a_t = (v_{t+1}-v_t)/dt.
 *
 */
GTEST_TEST(TrajectoryOptimizerTest, PendulumCalcInverseDynamics) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Set up the system model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Make some fake data
  std::vector<VectorXd> q;
  for (int t = 0; t <= num_steps; ++t) {
    q.push_back(drake::Vector1d(-0.2 + dt * 0.1 * t * t));
  }

  // Create a trajectory optimizer object
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.v_init = drake::Vector1d(-0.23);
  opt_prob.q_nom.resize(num_steps + 1, drake::Vector1d(0.0));
  opt_prob.v_nom.resize(num_steps + 1, drake::Vector1d(0.0));
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  state.set_q(q);

  const std::vector<VectorXd>& v = optimizer.EvalV(state);

  // Compute ground truth torque analytically using the
  // pendulum model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;
  const double g = 9.81;

  std::vector<VectorXd> tau_gt;
  for (int t = 0; t < num_steps; ++t) {
    drake::Vector1d a = (v[t + 1] - v[t]) / dt;
    drake::Vector1d sin_q = sin(q[t + 1](0)) * MatrixXd::Identity(1, 1);
    drake::Vector1d tau_t = m * l * l * a + m * g * l * sin_q + b * v[t + 1];
    tau_gt.push_back(tau_t);
  }

  // Compute tau from q and v
  std::vector<VectorXd> tau(num_steps, VectorXd(1));
  std::vector<VectorXd> a(num_steps, VectorXd(1));
  {
    // It appears, via trial and error, that CalcInverseDynamics makes exactly
    // 15 allocations for this example.
    // LimitMalloc guard({.max_num_allocations = 15});
    // TODO(vincekurtz): track down whatever extra allocations we got from
    // refactoring
    TrajectoryOptimizerTester::CalcAccelerations(optimizer, v, &a);
    TrajectoryOptimizerTester::CalcInverseDynamics(optimizer, state, a, &tau);
  }

  // Check that our computed values match the true (recorded) ones
  const double kToleranceTau = std::numeric_limits<double>::epsilon();
  for (int t = 0; t < num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(tau[t], tau_gt[t], kToleranceTau,
                                MatrixCompareType::relative));
  }
}

/**
 * Test our computation of generalized velocities
 *
 *   v_t = (q_t - q_{t-1})/dt
 *
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcVelocities) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Create a plant model with 2 DoFs
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      FindIdtoResource("idto/models/acrobot/acrobot.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create a TrajectoryOptimizer object
  ProblemDefinition opt_prob;
  opt_prob.q_init = Vector2d(0.1, 0.2);
  opt_prob.v_init = Vector2d(0.5 / dt, 1.5 / dt);
  opt_prob.num_steps = num_steps;
  opt_prob.q_nom.resize(num_steps + 1, Vector2d::Zero());
  opt_prob.v_nom.resize(num_steps + 1, Vector2d::Zero());
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob);

  // Construct a std::vector of generalized positions (q)
  // where q(t) = [0.1 + 0.5*t]
  //              [0.2 + 1.5*t]
  std::vector<VectorXd> q;
  for (int t = 0; t <= num_steps; ++t) {
    q.push_back(Vector2d(0.1 + 0.5 * t, 0.2 + 1.5 * t));
  }

  // Make a dummy N+ for this (nonexistant) system
  std::vector<MatrixXd> Nplus;
  for (int t = 0; t <= num_steps; ++t) {
    Nplus.push_back(MatrixXd::Identity(2, 2));
  }

  // Compute v from q
  std::vector<VectorXd> v(num_steps + 1);
  TrajectoryOptimizerTester::CalcVelocities(optimizer, q, Nplus, &v);

  // Check that our computed v is correct
  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;
  for (int t = 0; t <= num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(v[t], opt_prob.v_init, kTolerance,
                                MatrixCompareType::relative));
  }
}

// Test our computation of equality constraints (torques on unactuated DoFs)
// using the spinner example
GTEST_TEST(TrajectoryOptimizerTest, SpinnerEqualityConstraints) {
  // Define an optimization problem.
  const int num_steps = 3;
  const double dt = 0.05;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector3d(-0.1, 1.5, 0.0);
  opt_prob.v_init = Vector3d(0.0, 0.0, 0.0);
  opt_prob.Qq = Vector3d(0.0, 0.0, 0.1).asDiagonal();
  opt_prob.Qv = Vector3d(0.0, 0.0, 1.0).asDiagonal();
  opt_prob.Qf_q = Vector3d(0.0, 0.0, 10.0).asDiagonal();
  opt_prob.Qf_v = Vector3d(0.0, 0.0, 1.0).asDiagonal();
  opt_prob.R = Vector3d(1e0, 1e0, 1e1).asDiagonal();

  for (int t = 0; t <= num_steps; ++t) {
    VectorXd q_nom(3);
    VectorXd v_nom(3);
    q_nom << -0.1, 1.5, t;
    v_nom << 0.0, 0.0, dt * num_steps;
    opt_prob.q_nom.push_back(q_nom);
    opt_prob.v_nom.push_back(v_nom);
  }

  // Create a system model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      FindIdtoResource("idto/models/spinner_friction.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  SolverParameters params;
  params.scaling = false;
  params.gradients_method = GradientsMethod::kCentralDifferences;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + dt * opt_prob.v_init;
  }
  state.set_q(q);

  // Compute equality constraint violations
  VectorXd h = optimizer.EvalEqualityConstraintViolations(state);
  EXPECT_TRUE(h.size() == num_steps);

  // Set up an autodiff copy of the optimizer and plant
  auto diagram_ad = drake::systems::System<double>::ToAutoDiffXd(*diagram);
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant.get_name()));
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(diagram_ad.get(), &plant_ad,
                                               opt_prob, params);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1, VectorX<AutoDiffXd>(3));
  int ad_idx = 0;  // index for autodiff variables
  const int nq = plant.num_positions();
  const int num_vars = (num_steps + 1) * nq;
  for (int t = 0; t <= num_steps; ++t) {
    for (int i = 0; i < nq; ++i) {
      q_ad[t].segment<1>(i) =
          drake::math::InitializeAutoDiff(q[t].segment<1>(i), num_vars, ad_idx);
      ++ad_idx;
    }
  }
  state_ad.set_q(q_ad);

  VectorX<AutoDiffXd> h_ad =
      optimizer_ad.EvalEqualityConstraintViolations(state_ad);
  MatrixXd J_ad = drake::math::ExtractGradient(h_ad);
  MatrixXd J = optimizer.EvalEqualityConstraintJacobian(state);

  // We get a factor of sqrt(epsilon) since we're doing finite differences to
  // get inverse dynamics partials. The first column from autodiff is not
  // accurate since q0 is not a decision variable.
  const double kTolerance = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(J_ad.rightCols(num_steps * nq),
                              J.rightCols(num_steps * nq), kTolerance,
                              MatrixCompareType::relative));
}

// Test our computation of equality constraints (torques on unactuated DoFs)
// using the hopper example
GTEST_TEST(TrajectoryOptimizerTest, HopperEqualityConstraints) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init.resize(5);
  opt_prob.q_init << 0.0, 0.6, 0.3, -0.5, 0.2;
  opt_prob.v_init.resize(5);
  opt_prob.v_init << 1.0, -0.2, 0.1, -0.3, 0.4;
  opt_prob.Qq = 0.1 * MatrixXd::Identity(5, 5);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(5, 5);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(5, 5);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(5, 5);
  opt_prob.R = 0.01 * MatrixXd::Identity(5, 5);

  for (int t = 0; t <= num_steps; ++t) {
    VectorXd q_nom(5);
    VectorXd v_nom(5);
    q_nom << 0.5, 0.5, 0.3, -0.4, 0.1;
    v_nom << 0.01, 0.0, 0.2, 0.1, -0.1;
    opt_prob.q_nom.push_back(q_nom);
    opt_prob.v_nom.push_back(v_nom);
  }

  // Create a system
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      idto::FindIdtoResource("idto/models/hopper.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  SolverParameters params;
  params.scaling = false;
  params.gradients_method = GradientsMethod::kCentralDifferences;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + dt * opt_prob.v_init;
  }
  state.set_q(q);

  // Compute equality constraint violations
  VectorXd h = optimizer.EvalEqualityConstraintViolations(state);

  EXPECT_TRUE(h[0] != 0.0);
  EXPECT_TRUE(h.size() == num_steps * 3);

  // Set up an autodiff copy of the optimizer and plant
  auto diagram_ad = drake::systems::System<double>::ToAutoDiffXd(*diagram);
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant.get_name()));
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(diagram_ad.get(), &plant_ad,
                                               opt_prob, params);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1, VectorX<AutoDiffXd>(5));
  int ad_idx = 0;  // index for autodiff variables
  const int nq = plant.num_positions();
  const int num_vars = (num_steps + 1) * nq;
  for (int t = 0; t <= num_steps; ++t) {
    for (int i = 0; i < nq; ++i) {
      q_ad[t].segment<1>(i) =
          drake::math::InitializeAutoDiff(q[t].segment<1>(i), num_vars, ad_idx);
      ++ad_idx;
    }
  }
  state_ad.set_q(q_ad);

  VectorX<AutoDiffXd> h_ad =
      optimizer_ad.EvalEqualityConstraintViolations(state_ad);
  MatrixXd J_ad = drake::math::ExtractGradient(h_ad);
  MatrixXd J = optimizer.EvalEqualityConstraintJacobian(state);

  // We get a factor of sqrt(epsilon) since we're doing finite differences to
  // get inverse dynamics partials. The first column from autodiff is not
  // accurate since q0 is not a decision variable.
  const double kTolerance =
      10 * std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(J_ad.rightCols(num_steps * nq),
                              J.rightCols(num_steps * nq), kTolerance,
                              MatrixCompareType::relative));
}

// Test the combination of equality constraints and scaling using the hopper
GTEST_TEST(TrajectoryOptimizerTest, EqualityConstraintsAndScaling) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init.resize(5);
  opt_prob.q_init << 0.0, 0.6, 0.3, -0.5, 0.2;
  opt_prob.v_init.resize(5);
  opt_prob.v_init << 1.0, -0.2, 0.1, -0.3, 0.4;
  opt_prob.Qq = 0.1 * MatrixXd::Identity(5, 5);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(5, 5);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(5, 5);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(5, 5);
  opt_prob.R = 0.01 * MatrixXd::Identity(5, 5);

  for (int t = 0; t <= num_steps; ++t) {
    VectorXd q_nom(5);
    VectorXd v_nom(5);
    q_nom << 0.5, 0.5, 0.3, -0.4, 0.1;
    v_nom << 0.01, 0.0, 0.2, 0.1, -0.1;
    opt_prob.q_nom.push_back(q_nom);
    opt_prob.v_nom.push_back(v_nom);
  }

  // Create a system
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file = FindIdtoResource("idto/models/hopper.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create two optimizers: one with scaling and one without
  SolverParameters params;
  params.scaling = false;
  params.equality_constraints = true;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  SolverParameters params_scaled;
  params.scaling = true;
  params.equality_constraints = true;
  TrajectoryOptimizer<double> optimizer_scaled(diagram.get(), &plant, opt_prob,
                                               params);
  TrajectoryOptimizerState<double> state_scaled =
      optimizer_scaled.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + dt * opt_prob.v_init;
  }
  state.set_q(q);
  state_scaled.set_q(q);

  // The scaled constraint jacobian is given by J_scaled = J * D
  const VectorXd& D = optimizer_scaled.EvalScaleFactors(state_scaled);
  const MatrixXd& J = optimizer.EvalEqualityConstraintJacobian(state);
  const MatrixXd& J_scaled =
      optimizer_scaled.EvalEqualityConstraintJacobian(state_scaled);

  const double kEpsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(J * D.asDiagonal(), J_scaled, kEpsilon,
                              MatrixCompareType::relative));

  // Lagrange multipliers should be the same with and without scaling
  const MatrixXd Hinv = optimizer.EvalHessian(state).MakeDense().inverse();
  const VectorXd& h = optimizer.EvalEqualityConstraintViolations(state);
  const VectorXd& g = optimizer.EvalGradient(state);
  const VectorXd lambda_dense =
      (J * Hinv * J.transpose()).inverse() * (h - J * Hinv * g);
  const VectorXd& lambda = optimizer.EvalLagrangeMultipliers(state);
  const VectorXd& lambda_scaled =
      optimizer_scaled.EvalLagrangeMultipliers(state_scaled);

  const double kTolerance =
      100 * kEpsilon;  // N.B. we loose precision in H^{-1}
  EXPECT_TRUE(CompareMatrices(lambda_dense, lambda, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(lambda_dense, lambda_scaled, kTolerance,
                              MatrixCompareType::relative));

  // Merit function should be the same with and without scaling
  const double merit = optimizer.EvalMeritFunction(state);
  const double merit_scaled = optimizer_scaled.EvalMeritFunction(state_scaled);

  EXPECT_NEAR(merit, merit_scaled, kTolerance);

  // The scaled merit function gradient is given by gm_scaled = D * gm
  const VectorXd& gm = optimizer.EvalMeritFunctionGradient(state);
  const VectorXd& gm_scaled =
      optimizer_scaled.EvalMeritFunctionGradient(state_scaled);

  const double kGradTolerance = std::sqrt(kEpsilon);  // N.B. finite difference
  EXPECT_TRUE(CompareMatrices(D.asDiagonal() * gm, gm_scaled, kGradTolerance,
                              MatrixCompareType::relative));

  // Trust ratio should be the same with and without scaling
  const VectorXd dq = -Hinv * gm;
  TrajectoryOptimizerState<double> scratch_state = optimizer.CreateState();
  double trust_ratio = TrajectoryOptimizerTester::CalcTrustRatio(
      optimizer, state, dq, &scratch_state);
  double trust_ratio_scaled = TrajectoryOptimizerTester::CalcTrustRatio(
      optimizer_scaled, state_scaled, dq, &scratch_state);

  EXPECT_TRUE(trust_ratio > 0.6);  // avoid the trivial rho = 0.5 case
  EXPECT_NEAR(trust_ratio, trust_ratio_scaled, kTolerance);
}

// Test updating the nominal trajectory
GTEST_TEST(TrajectoryOptimizerTest, UpdateNominalTrajectory) {
  // Define an optimization problem for the pendulum
  const int num_steps = 20;
  const double dt = 5e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = drake::Vector1d(0.1);
  opt_prob.v_init = drake::Vector1d(0.0);
  opt_prob.Qq = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 1000 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.01 * MatrixXd::Identity(1, 1);
  for (int t = 0; t <= num_steps; ++t) {
    opt_prob.q_nom.push_back(drake::Vector1d(M_PI));
    opt_prob.v_nom.push_back(drake::Vector1d(0));
  }

  // Create a system model
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = dt;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(config, &builder);
  const std::string urdf_file =
      drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddModels(urdf_file);
  plant.Finalize();
  auto diagram = builder.Build();

  // Create an optimizer
  SolverParameters solver_params;
  solver_params.max_iterations = 20;
  solver_params.verbose = false;
  solver_params.check_convergence = true;
  solver_params.convergence_tolerances.rel_cost_reduction = 1e-5;
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, opt_prob,
                                        solver_params);

  // Set an initial guess
  std::vector<VectorXd> q_guess;
  for (int t = 0; t <= num_steps; ++t) {
    q_guess.push_back(opt_prob.q_init);
  }

  // Solve the optimization problem
  TrajectoryOptimizerSolution<double> solution;
  TrajectoryOptimizerStats<double> stats;
  SolverFlag status = optimizer.Solve(q_guess, &solution, &stats);
  EXPECT_EQ(status, SolverFlag::kSuccess);

  // Check that the final state is close to the target
  const VectorXd& q_final = solution.q[num_steps];
  EXPECT_NEAR(q_final(0), M_PI, 1e-3);

  // Update the nominal trajectory
  std::vector<VectorXd> q_nom_new;
  std::vector<VectorXd> v_nom_new;
  for (int t = 0; t <= num_steps; ++t) {
    q_nom_new.push_back(drake::Vector1d(-1.2));
    v_nom_new.push_back(drake::Vector1d(0.0));
  }
  optimizer.UpdateNominalTrajectory(q_nom_new, v_nom_new);

  // Re-solve the optimization problem
  stats = TrajectoryOptimizerStats<double>();  // clear solver stats
  status = optimizer.Solve(q_guess, &solution, &stats);
  EXPECT_EQ(status, SolverFlag::kSuccess);

  // Check that the final state is close to the new target
  const VectorXd& q_final_new = solution.q[num_steps];
  EXPECT_NEAR(q_final_new(0), -1.2, 1e-3);
}

}  // namespace internal
}  // namespace optimizer
}  // namespace idto

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
