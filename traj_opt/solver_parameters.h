#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/fem/petsc_symmetric_block_sparse_matrix.h"
#include "drake/traj_opt/convergence_criteria_tolerances.h"

namespace drake {
namespace traj_opt {

enum LinesearchMethod {
  // Simple backtracking linesearch with Armijo's condition
  kArmijo,

  // Backtracking linesearch that tries to find a local minimum
  kBacktracking
};

enum SolverMethod { kLinesearch, kTrustRegion };

enum GradientsMethod {
  // First order forward differences.
  kForwardDifferences,
  // Second order central differences.
  kCentralDifferences,
  // Fourth order central differences.
  kCentralDifferences4,
  // Automatic differentiation.
  kAutoDiff,
  // The optimizer will not be used for the computation of gradients. If
  // requested, an exception will be thrown.
  kNoGradients
};

enum ScalingMethod {
  // Method for setting the diagonal scaling matrix D at the k^th iteration
  // based on terms in the Hessian

  // The simplest scaling scheme, which attempts to set diagonal terms of the
  // scaled Hessian H̃ = DHD to approximately 1.
  // Dᵢᵢᵏ = min(1, 1/√Hᵏᵢᵢ).
  kSqrt,

  // Adaptive variant of the sqrt scaling recommended by Moré, "Recent
  // developments in algorithms and software for trust region methods."
  // Springer, 1983.
  // Dᵢᵢᵏ = min(Dᵢᵢᵏ⁻¹, 1/√Hᵢᵢᵏ)
  kAdaptiveSqrt,

  // A less extreme version of the sqrt scaling, which produces
  // closer-to-spherical trust regions. We found that this performs better than
  // sqrt scaling on some of our examples.
  // Dᵢᵢᵏ = min(1, 1/√√Hᵏᵢᵢ)
  kDoubleSqrt,

  // Adaptive version of the double sqrt scaling.
  // Dᵢᵢᵏ = min(Dᵢᵢᵏ⁻¹, 1/√√Hᵢᵢᵏ)
  kAdaptiveDoubleSqrt
};

struct SolverParameters {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverParameters);

  enum LinearSolverType {
    // Dense Eigen::LDLT solver.
    kDenseLdlt,
    // Pentadiagonal LU solver.
    kPentaDiagonalLu,
    // PETSc solver.
    kPetsc,
  };

  struct PetscSolverPatameters {
    using SolverType = drake::multibody::fem::internal::
        PetscSymmetricBlockSparseMatrix::SolverType;
    using PreconditionerType = drake::multibody::fem::internal::
        PetscSymmetricBlockSparseMatrix::PreconditionerType;
    double relative_tolerance{1.0e-12};
    SolverType solver_type{SolverType::kConjugateGradient};
    PreconditionerType preconditioner_type{
        PreconditionerType::kIncompleteCholesky};
  };

  // Flag for whether we should check for convergence, along with default
  // tolerances for the convergence check
  bool check_convergence = false;
  ConvergenceCriteriaTolerances convergence_tolerances;

  SolverParameters() = default;

  // Which overall optimization strategy to use - linesearch or trust region
  SolverMethod method{SolverMethod::kTrustRegion};

  // Which linesearch strategy to use
  LinesearchMethod linesearch_method{LinesearchMethod::kArmijo};

  // Maximum number of Gauss-Newton iterations
  int max_iterations{100};

  // Maximum number of linesearch iterations
  int max_linesearch_iterations{50};

  GradientsMethod gradients_method{kForwardDifferences};

  // Select the linear solver to be used in the Gauss-Newton step computation.
  LinearSolverType linear_solver{LinearSolverType::kPentaDiagonalLu};

  // Parameters for the PETSc solver. Ignored if linear_solver != kPetsc.
  PetscSolverPatameters petsc_parameters{};

  // Enable/disable quaternions' normalization at each iteration.
  bool normalize_quaternions{false};

  // Flag for whether to print out iteration data
  bool verbose{true};

  // Flag for whether to print (and compute) additional slow-to-compute
  // debugging info, like the condition number, at each iteration
  bool print_debug_data{false};

  // Only for debugging. When `true`, the computation with sparse algebra is
  // checked against a dense LDLT computation. This is an expensive check and
  // must be avoided unless we are trying to debug loss of precision due to
  // round-off errors or similar problems.
  bool debug_compare_against_dense{false};

  // Flag for whether to record linesearch data to a file at each iteration (for
  // later plotting). This saves a file called "linesearch_data_[k].csv" for
  // each iteration, where k is the iteration number. This file can then be
  // found somewhere in drake/bazel-out/.
  bool linesearch_plot_every_iteration{false};

  // Flag for whether to add a proximal operator term,
  //
  //      1/2 * rho * (q_k - q_{k-1})' * diag(H) * (q_k - q_{k-1})
  //
  // to the cost, where q_{k-1} are the decision variables at iteration {k-1}
  // and H_{k-1} is the Hessian at iteration k-1.
  bool proximal_operator{false};

  // Scale factor for the proximal operator cost
  double rho_proximal{1e-8};

  // Contact model parameters
  // TODO(vincekurtz): this is definitely the wrong place to specify the contact
  // model - figure out the right place and put these parameters there
  double contact_stiffness{100};     // normal force stiffness, N/m
  double dissipation_velocity{0.1};  // Hunt-Crossley velocity, in m/s.
  double stiction_velocity{0.05};    // Regularization of stiction, in m/s.
  double friction_coefficient{0.5};  // Coefficient of friction.
  double smoothing_factor{0.1};      // force at a distance smoothing

  // Flags for making a contour plot with the first two decision variables.
  bool save_contour_data{false};
  double contour_q1_min{0.0};
  double contour_q1_max{1.0};
  double contour_q2_min{0.0};
  double contour_q2_max{1.0};

  // Flags for making line plots with the first decision variable
  bool save_lineplot_data{false};
  double lineplot_q_min{0.0};
  double lineplot_q_max{1.0};

  // Flag for choosing between the exact Hessian (autodiff, very slow) and a
  // Gauss-Newton approximation
  bool exact_hessian{false};

  // Flag for rescaling the Hessian, for better numerical conditioning
  bool scaling{true};

  // Method to use for rescaling the Hessian (and thus reshaping the Hessian)
  ScalingMethod scaling_method{ScalingMethod::kDoubleSqrt};

  // Parameter for activating hard equality constraints on unactuated DoFs
  bool equality_constraints{true};

  // Initial and maximum trust region radius
  // N.B. these have very different units depending on whether scaling is used.
  // These defaults are reasonable when scaling=true: without scaling a smaller
  // trust region radius is more appropriate.
  double Delta0{1e-1};
  double Delta_max{1e5};

  // Number of cpu threads for parallel computation of derivatives
  int num_threads{1};

  // Indicator for which DoFs the nominal trajectory is defined as relative to
  // the initial condition. Useful for locomotion or continuous rotation tasks.
  VectorX<bool> q_nom_relative_to_q_init;
};

}  // namespace traj_opt
}  // namespace drake
