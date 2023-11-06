#include <iostream>

#include "optimizer/trajectory_optimizer.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::VectorXd;
using idto::optimizer::ProblemDefinition;
using idto::optimizer::SolverParameters;
using idto::optimizer::TrajectoryOptimizer;
using idto::optimizer::TrajectoryOptimizerSolution;
using idto::optimizer::TrajectoryOptimizerStats;

TrajectoryOptimizer<double> MakeOptimizer(const std::string& model_file,
                                          const ProblemDefinition& problem,
                                          const SolverParameters& params,
                                          const double time_step) {
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
 * A python wrapper class for TrajectoryOptimizer. Has access to a limited
 * set of methods, and is initialized from a URDF or SDF file instead of a
 * Drake diagram and MultibodyPlant.
 */
class TrajectoryOptimizerPy {
 public:
  TrajectoryOptimizerPy(const std::string& model_file,
                        const ProblemDefinition& problem,
                        const SolverParameters& params,
                        const double time_step) {
    DiagramBuilder<double> builder;

    MultibodyPlantConfig config;
    config.time_step = time_step;
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(config, &builder);
    Parser(plant_).AddModels(model_file);
    plant_->Finalize();
    diagram_ = builder.Build();

    optimizer_ = std::make_unique<TrajectoryOptimizer<double>>(
        diagram_.get(), plant_, problem, params);
  }

  void Solve(const std::vector<VectorXd>& q_guess,
             TrajectoryOptimizerSolution<double>* solution,
             TrajectoryOptimizerStats<double>* stats) {
    optimizer_->Solve(q_guess, solution, stats);
  }

  double time_step() const { return optimizer_->time_step(); }

  int num_steps() const { return optimizer_->num_steps(); }

 private:
  // Plant and scene graph are owned by the diagram
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};

  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<TrajectoryOptimizer<double>> optimizer_;
};

PYBIND11_MODULE(trajectory_optimizer, m) {
  py::class_<TrajectoryOptimizerPy>(m, "TrajectoryOptimizer")
      .def(py::init<const std::string&, const ProblemDefinition&,
                    const SolverParameters&, const double>())
      .def("time_step", &TrajectoryOptimizerPy::time_step)
      .def("num_steps", &TrajectoryOptimizerPy::num_steps)
      .def("Solve", &TrajectoryOptimizerPy::Solve);
}
