#include <iostream>

#include "optimizer/trajectory_optimizer.h"
#include "optimizer/warm_start.h"
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
using idto::optimizer::WarmStart;

void bind_trajectory_optimizer(py::module_& m) {
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.systems.framework");

  py::class_<TrajectoryOptimizer<double>>(m, "TrajectoryOptimizer")
      .def(py::init<const Diagram<double>*, const MultibodyPlant<double>*,
                    const ProblemDefinition&, const SolverParameters&>())
      .def("time_step", &TrajectoryOptimizer<double>::time_step)
      .def("num_steps", &TrajectoryOptimizer<double>::num_steps)
      .def("Solve", &TrajectoryOptimizer<double>::Solve)
      .def("SolveFromWarmStart",
           &TrajectoryOptimizer<double>::SolveFromWarmStart)
      //.def("MakeWarmStart", &TrajectoryOptimizerPy::MakeWarmStart)
      .def("ResetInitialConditions",
           &TrajectoryOptimizer<double>::ResetInitialConditions)
      .def("UpdateNominalTrajectory",
           &TrajectoryOptimizer<double>::UpdateNominalTrajectory)
      .def("params", &TrajectoryOptimizer<double>::params)
      .def("prob", &TrajectoryOptimizer<double>::prob);
  py::class_<WarmStart>(m, "WarmStart")
      // Warm start is not default constructible: it should be created
      // in python using the TrajectoryOptimizer.MakeWarmStart method.
      .def("set_q", &WarmStart::set_q)
      .def("get_q", &WarmStart::get_q)
      .def_readonly("Delta", &WarmStart::Delta)
      .def_readonly("dq", &WarmStart::dq)
      .def_readonly("dqH", &WarmStart::dqH);
}
