#include "optimizer/trajectory_optimizer.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>

namespace py = pybind11;

using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using idto::optimizer::ProblemDefinition;
using idto::optimizer::SolverParameters;
using idto::optimizer::TrajectoryOptimizer;

TrajectoryOptimizer<double> MakeOptimizer(
    const std::string& model_file, const ProblemDefinition& problem,
    const SolverParameters& params, const double time_step) {
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = time_step;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  Parser(&plant).AddModels(model_file);
  plant.Finalize();
  auto diagram = builder.Build();

  return TrajectoryOptimizer<double>(diagram.get(), &plant, problem, params);
}

/**
 * A thin wrapper around TrajectoryOptimizer to expose it to Python.
 */
class TrajectoryOptimizerWrapper {
 public:
  /**
   * Constructor that uses a model file (SDF or URDF) to construct a
   * MultibodyPlant model internally. This allows us to avoid having to
   * interface with pydrake.
   */
  TrajectoryOptimizerWrapper(const std::string& model_file,
                             const ProblemDefinition& problem,
                             const SolverParameters& params,
                             const double time_step) 
      : builder_(),
      optimizer_(MakeOptimizer(model_file, problem, params, time_step)) {}

 private:
  DiagramBuilder<double> builder_;
  MultibodyPlantConfig config_;
  TrajectoryOptimizer<double> optimizer_;
};

PYBIND11_MODULE(trajectory_optimizer, m) {
  py::class_<TrajectoryOptimizerWrapper>(m, "TrajectoryOptimizer")
      .def(py::init<const std::string&, const ProblemDefinition&,
                    const SolverParameters&, const double>());
}
