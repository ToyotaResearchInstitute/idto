#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "examples/yaml_config.h"
#include "optimizer/problem_definition.h"
#include "optimizer/trajectory_optimizer.h"
#include "utils/find_resource.h"
#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

namespace idto {
namespace examples {

using drake::geometry::Meshcat;
using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

using optimizer::ConvergenceReason;
using optimizer::DecodeConvergenceReasons;
using optimizer::GradientsMethod;
using optimizer::LinesearchMethod;
using optimizer::ProblemDefinition;
using optimizer::ScalingMethod;
using optimizer::SolverFlag;
using optimizer::SolverMethod;
using optimizer::SolverParameters;
using optimizer::TrajectoryOptimizer;
using optimizer::TrajectoryOptimizerSolution;
using optimizer::TrajectoryOptimizerStats;

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Abstract base class for trajectory optimization examples.
 */
class TrajOptExample {
 public:
  virtual ~TrajOptExample() = default;

  /**
   * Run the example, either solving a single trajectory optimization problem or
   * running MPC, as specified by the options in the YAML file.
   *
   * @param options_file YAML file containing cost function definition, solver
   * parameters, etc., with fields as defined in yaml_config.h.
   * @param test Flag for whether this is being run as a unit test. If set to
   * true, some of the options are overwritten for simplicity:
   *   - mpc = false
   *   - max_iters = 10
   *   - save_solver_stats_csv = false
   *   - play_target_trajectory = false
   *   - play_initial_guess = false
   *   - play_optimal_trajectory = false
   *   - num_threads = 1; */
  void RunExample(const std::string options_file,
                  const bool test = false) const;

  /**
   * Solve the optimization problem, as defined by the parameters in the given
   * YAML file.
   *
   * @param options YAML options, incluidng cost function definition, solver
   * parameters, etc.
   * @return TrajectoryOptimizerSolution<double> the optimal trajectory
   */
  TrajectoryOptimizerSolution<double> SolveTrajectoryOptimization(
      const TrajOptExampleParams& options) const;

  /**
   * Use the optimizer as an MPC controller in simulation.
   *
   * @param options YAML options, incluidng cost function definition, solver
   * parameters, etc.
   */
  void RunModelPredictiveControl(const TrajOptExampleParams& options) const;

  /**
   * Set an optimization problem from example options which were loaded from
   * YAML.
   *
   * @param options parameters loaded from yaml
   * @param plant model of the system that we're optimizing over
   * @param opt_prob the problem definition (cost, initital state, etc)
   */
  void SetProblemDefinition(const TrajOptExampleParams& options,
                            const MultibodyPlant<double>& plant,
                            ProblemDefinition* opt_prob) const;

  /**
   * Set solver parameters (used to pass options to the optimizer)
   * from example options (loaded from a YAML file).
   *
   * @param options parameters loaded from yaml
   * @param solver_params parameters for the optimizer that we'll set
   */
  void SetSolverParameters(const TrajOptExampleParams& options,
                           SolverParameters* solver_params) const;

  /**
   * Normalize quaternions in the given sequence of generalized positions. This
   * is useful for, for example, ensuring that the reference and initial guess
   * contain valid quaternions.
   *
   * @param plant model of the system that we're optimizing over
   * @param q sequence of generalized positions, including quaternion DoFs, that
   * we'll normalize
   */
  void NormalizeQuaternions(const MultibodyPlant<double>& plant,
                            std::vector<VectorXd>* q) const;

  /**
   * Normalize quaternion in the given vector of generalized positions.
   *
   * @param plant model of the system that we're optimizing over
   * @param q vector of generalized positions, including quaternion DoFs, that
   * we'll normalize
   */
  void NormalizeQuaternions(const MultibodyPlant<double>& plant,
                            VectorXd* q) const;

 protected:
  /**
   * Meshcat instance used for visualization.
   *
   * N.B. Derivived classes will need to access this to change the default
   * camera pose.
   */
  std::shared_ptr<Meshcat> meshcat_ = std::make_shared<Meshcat>();

 private:
  /**
   * Create a MultibodyPlant model of the system that we're optimizing over.
   * This is the only method that needs to be overwritten to specialize to
   * different systems.
   *
   * @param plant the MultibodyPlant that we'll add the system to.
   */
  virtual void CreatePlantModel(MultibodyPlant<double>*) const {}

  /**
   * Update any custom meshcat visualizations, such as a frame that illustrates
   * a target pose. In most cases this is not used.
   *
   * @param options Options that we read from YAML
   */
  virtual void UpdateCustomMeshcatElements(const TrajOptExampleParams&) const {}

  /**
   * Create a MultibodyPlant model of the system to use for simulation (i.e., to
   * test MPC). The default behavior is to use the same model that we use for
   * optimization.
   *
   * @param plant the MultibodyPlant that we'll add the system to.
   */
  virtual void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const {
    CreatePlantModel(plant);
  }

  /**
   * Play back the given trajectory on the Drake visualizer
   *
   * @param q sequence of generalized positions defining the trajectory
   * @param time_step time step (seconds) for the discretization
   */
  void PlayBackTrajectory(const std::vector<VectorXd>& q,
                          const double time_step) const;

  /**
   * Return a vector that interpolates linearly between q_start and q_end.
   * Useful for setting initial guesses and target trajectories.
   *
   * @param start initial vector
   * @param end final vector
   * @param N number of elements in the linear interpolation
   * @return std::vector<VectorXd> vector that interpolates between start and
   * end.
   */
  std::vector<VectorXd> MakeLinearInterpolation(const VectorXd& start,
                                                const VectorXd& end,
                                                int N) const {
    std::vector<VectorXd> result;
    double lambda = 0;
    for (int i = 0; i < N; ++i) {
      lambda = i / (N - 1.0);
      result.push_back((1 - lambda) * start + lambda * end);
    }
    return result;
  }
};

}  // namespace examples
}  // namespace idto
