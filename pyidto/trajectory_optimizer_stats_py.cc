#include "optimizer/trajectory_optimizer_solution.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using idto::optimizer::TrajectoryOptimizerStats;

PYBIND11_MODULE(trajectory_optimizer_stats, m) {
  py::class_<TrajectoryOptimizerStats<double>>(m, "TrajectoryOptimizerStats")
      .def(py::init<>())
      // Optimizer stats should only be written to by the solver
      .def_readonly("solve_time", &TrajectoryOptimizerStats<double>::solve_time)
      .def_readonly("iteration_costs",
                    &TrajectoryOptimizerStats<double>::iteration_costs)
      .def_readonly("iteration_times",
                    &TrajectoryOptimizerStats<double>::iteration_times)
      .def_readonly("trust_region_radii",
                    &TrajectoryOptimizerStats<double>::trust_region_radii)
      .def_readonly("gradient_norms",
                    &TrajectoryOptimizerStats<double>::gradient_norms)
      .def_readonly("q_norms", &TrajectoryOptimizerStats<double>::q_norms)
      .def_readonly("dq_norms", &TrajectoryOptimizerStats<double>::dq_norms)
      .def_readonly("trust_ratios",
                    &TrajectoryOptimizerStats<double>::trust_ratios)
      .def_readonly("dL_dqs", &TrajectoryOptimizerStats<double>::dL_dqs)
      .def_readonly("h_norms", &TrajectoryOptimizerStats<double>::h_norms)
      .def_readonly("merits", &TrajectoryOptimizerStats<double>::merits)
      .def("is_empty", &TrajectoryOptimizerStats<double>::is_empty);
}
