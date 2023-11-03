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
      .def_readwrite("q", &TrajectoryOptimizerSolution<double>::q)
      .def_readwrite("v", &TrajectoryOptimizerSolution<double>::v)
      .def_readwrite("tau", &TrajectoryOptimizerSolution<double>::tau);
}
