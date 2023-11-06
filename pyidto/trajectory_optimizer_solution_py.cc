#include "optimizer/trajectory_optimizer_solution.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using idto::optimizer::TrajectoryOptimizerSolution;

PYBIND11_MODULE(trajectory_optimizer_solution, m) {
  py::class_<TrajectoryOptimizerSolution<double>>(m,
                                                  "TrajectoryOptimizerSolution")
      .def(py::init<>())
      // Traj-opt solution should only be written to by the solver.
      .def_readonly("q", &TrajectoryOptimizerSolution<double>::q)
      .def_readonly("v", &TrajectoryOptimizerSolution<double>::v)
      .def_readonly("tau", &TrajectoryOptimizerSolution<double>::tau);
}
