#include "optimizer/trajectory_optimizer.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>

#include <iostream>

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
using idto::optimizer::TrajectoryOptimizerSolution;
using idto::optimizer::TrajectoryOptimizerStats;
using idto::optimizer::TrajectoryOptimizerStats;
using idto::optimizer::SolverFlag;
using Eigen::VectorXd;

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

SolverFlag TestFunction(const std::vector<VectorXd>* q_guess,
                  TrajectoryOptimizerSolution<double>* solution,
                  TrajectoryOptimizerStats<double>* stats) {
  (void)q_guess;
  (void)solution;
  (void)stats;
  std::cout << "Test function called!" << std::endl;
  return SolverFlag::kSuccess;
}

PYBIND11_MODULE(trajectory_optimizer, m) {
  m.def("MakeOptimizer", &MakeOptimizer,
        py::return_value_policy::take_ownership);
  //m.def("TestFunction", &TestFunction);
  py::class_<TrajectoryOptimizer<double>>(m, "TrajectoryOptimizer")
      .def("time_step", &TrajectoryOptimizer<double>::time_step)
      .def("num_steps", &TrajectoryOptimizer<double>::num_steps)
      // Solve, but without a return value using lambda function
      .def("Solve", [](TrajectoryOptimizer<double>& self,
                       const std::vector<VectorXd>& q_guess,
                       TrajectoryOptimizerSolution<double>* solution,
                       TrajectoryOptimizerStats<double>* stats) {
        self.Solve(q_guess, solution, stats);
      });

      //.def("Solve", &TrajectoryOptimizer<double>::Solve, 
      //     "Solve the optimization problem.", py::arg("q_guess"),
      //     py::arg("solution"), py::arg("stats"), py::arg("reason") = nullptr);
}
