#pragma once

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/traj_opt/convergence_criteria_tolerances.h"

namespace drake {
namespace traj_opt {
namespace examples {

using Eigen::VectorXd;

/**
 * A simple object which stores parameters that define an optimization problem
 * and various options, and can be loaded from a YAML file.
 *
 * See, e.g., spinner.yaml for an explanation of each field, and
 * https://drake.mit.edu/doxygen_cxx/group__yaml__serialization.html for details
 * on loading options from YAML.
 */
struct TrajOptExampleParams {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q_init));
    a->Visit(DRAKE_NVP(v_init));
    a->Visit(DRAKE_NVP(q_nom_start));
    a->Visit(DRAKE_NVP(q_nom_end));
    a->Visit(DRAKE_NVP(q_guess));
    a->Visit(DRAKE_NVP(Qq));
    a->Visit(DRAKE_NVP(Qv));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(Qfq));
    a->Visit(DRAKE_NVP(Qfv));
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(num_steps));
    a->Visit(DRAKE_NVP(max_iters));
    a->Visit(DRAKE_NVP(linesearch));
    a->Visit(DRAKE_NVP(gradients_method));
    a->Visit(DRAKE_NVP(method));
    a->Visit(DRAKE_NVP(proximal_operator));
    a->Visit(DRAKE_NVP(rho_proximal));
    a->Visit(DRAKE_NVP(play_optimal_trajectory));
    a->Visit(DRAKE_NVP(play_initial_guess));
    a->Visit(DRAKE_NVP(play_target_trajectory));
    a->Visit(DRAKE_NVP(linesearch_plot_every_iteration));
    a->Visit(DRAKE_NVP(print_debug_data));
    a->Visit(DRAKE_NVP(save_solver_stats_csv));
    a->Visit(DRAKE_NVP(contact_stiffness));
    a->Visit(DRAKE_NVP(dissipation_velocity));
    a->Visit(DRAKE_NVP(stiction_velocity));
    a->Visit(DRAKE_NVP(friction_coefficient));
    a->Visit(DRAKE_NVP(smoothing_factor));
    a->Visit(DRAKE_NVP(save_contour_data));
    a->Visit(DRAKE_NVP(contour_q1_min));
    a->Visit(DRAKE_NVP(contour_q1_max));
    a->Visit(DRAKE_NVP(contour_q2_min));
    a->Visit(DRAKE_NVP(contour_q2_max));
    a->Visit(DRAKE_NVP(save_lineplot_data));
    a->Visit(DRAKE_NVP(lineplot_q_min));
    a->Visit(DRAKE_NVP(lineplot_q_max));
    a->Visit(DRAKE_NVP(tolerances));
    a->Visit(DRAKE_NVP(normalize_quaternions));
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(linear_solver));
    a->Visit(DRAKE_NVP(petsc_rel_tolerance));
    a->Visit(DRAKE_NVP(petsc_solver));
    a->Visit(DRAKE_NVP(petsc_preconditioner));
    a->Visit(DRAKE_NVP(exact_hessian));
    a->Visit(DRAKE_NVP(scaling));
    a->Visit(DRAKE_NVP(mpc));
    a->Visit(DRAKE_NVP(mpc_iters));
    a->Visit(DRAKE_NVP(controller_frequency));
    a->Visit(DRAKE_NVP(sim_time));
    a->Visit(DRAKE_NVP(sim_time_step));
    a->Visit(DRAKE_NVP(sim_realtime_rate));
    a->Visit(DRAKE_NVP(Kp));
    a->Visit(DRAKE_NVP(Kd));
    a->Visit(DRAKE_NVP(feed_forward));
    a->Visit(DRAKE_NVP(scaling_method));
    a->Visit(DRAKE_NVP(equality_constraints));
    a->Visit(DRAKE_NVP(Delta_max));
    a->Visit(DRAKE_NVP(Delta0));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(q_nom_relative_to_q_init));
    a->Visit(DRAKE_NVP(save_mpc_result_as_static_html));
    a->Visit(DRAKE_NVP(static_html_filename));
  }
  // Initial state
  VectorXd q_init;
  VectorXd v_init;

  // Nominal state at each timestep is defined by linear interpolation between
  // q_nom_start and q_nom_end
  VectorXd q_nom_start;
  VectorXd q_nom_end;

  // Initial guess is defined by linear interpolation between q_init and q_guess
  VectorXd q_guess;

  // Running cost weights (diagonal matrices)
  VectorXd Qq;
  VectorXd Qv;
  VectorXd R;

  // Terminal cost weights (diagonal matrices)
  VectorXd Qfq;
  VectorXd Qfv;

  // Time step size, in seconds
  double time_step;

  // Number of time steps in the optimization problem
  int num_steps;

  // Maximum number of iterations
  int max_iters;

  // Convergence tolerances
  ConvergenceCriteriaTolerances tolerances;

  // Linesearch method, "backtracking" or "armijo"
  std::string linesearch{"armijo"};

  // Optimization method, "linesearch" or "trust_region"
  std::string method{"trust_region"};

  // Method of computing gradients, "forward_differences",
  // "central_differences", "central_differences4" or "autodiff"
  std::string gradients_method{"forward_differences"};

  // Linear solve type: dense_ldlt, pentadiagonal_lu, petsc.
  std::string linear_solver{"pentadiagonal_lu"};

  double petsc_rel_tolerance{1.0e-12};

  // PETSc solver type: cg, direct, minres.
  // Note. "direct" only works with petsc_preconditioner = chol.
  std::string petsc_solver{"cg"};

  // PETSc preconditioner type: none, chol, ichol.
  // Note. "chol" only works with petsc_solver = direct.
  std::string petsc_preconditioner{"ichol"};

  // Whether to add a proximal operator term to the cost (essentially adds to
  // the diagonal of the Hessian)
  bool proximal_operator{false};
  double rho_proximal{1e-8};

  // Flags for playing back the target trajectory, initital guess, and optimal
  // trajectory on the visualizer
  bool play_optimal_trajectory{true};
  bool play_initial_guess{false};
  bool play_target_trajectory{false};

  // Save cost along the linesearch direction to linesearch_data.csv
  bool linesearch_plot_every_iteration{false};

  // Print additional debugging data
  bool print_debug_data{false};

  // Save convergence data to solver_stats.csv
  bool save_solver_stats_csv{true};

  // Contact model parameters
  double contact_stiffness{100.0};
  double dissipation_velocity{0.1};
  double smoothing_factor{1.0};
  double stiction_velocity{0.05};
  double friction_coefficient{0.5};

  // Save data for a 2d contour plot of cost/gradient/Hessian w.r.t. the first
  // two variables to contour_data.csv
  bool save_contour_data{false};
  double contour_q1_min{0};
  double contour_q1_max{1};
  double contour_q2_min{0};
  double contour_q2_max{1};

  // Save data for plotting the cost/gradient/Hessian w.r.t. the first variable
  // to lineplot_data.csv
  bool save_lineplot_data{false};
  double lineplot_q_min{0};
  double lineplot_q_max{1};

  // Whether to print iteration data to stdout
  bool verbose{true};

  // Whether to normalize quaternion DoFs between iterations
  bool normalize_quaternions{false};

  // Whether to use an exact (autodiff on the finite diff gradient) Hessian
  bool exact_hessian{false};

  // Whether to rescale the Hessian
  bool scaling{true};

  // MPC-related parameters
  bool mpc{false};                  // whether to do MPC
  int mpc_iters{1};                 // fixed number of optimizer iterations
  double controller_frequency{30};  // target control frequency
  double sim_time{10.0};            // Time to simulate for, in seconds
  double sim_time_step{1e-3};       // Simulator time step
  double sim_realtime_rate{1.0};    // Simulator realtime rate

  // Gains for the low-level PD+ controller that operates between MPC
  // iterations. Terms related to unactuated DoFs are ignored.
  VectorXd Kp;
  VectorXd Kd;
  bool feed_forward{true};

  // Method to use when rescaling the Hessian
  std::string scaling_method{"double_sqrt"};

  // Whether to enforce strict equality constraints
  bool equality_constraints{true};

  // Maximum trust region radius
  double Delta_max{1e5};

  // Initial trust region radius
  double Delta0{1e-1};

  // Number of cpu threads to use for parallel computation of derivatives
  int num_threads{1};

  // Indicator for which DoFs the nominal trajectory is defined as relative to
  // the initial condition. Useful for locomotion or continuous rotation tasks.
  VectorX<bool> q_nom_relative_to_q_init;

  // Flag for saving a recording of the closed-loop MPC trajectory as a static
  // HTML file that can be played back later using meshcat.
  bool save_mpc_result_as_static_html{false};

  // File name to save the meshcat recordint file to
  std::string static_html_filename{"/tmp/meshcat_recording.html"};
};

}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
