#include "optimizer/solver_parameters.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using idto::optimizer::SolverParameters;

PYBIND11_MODULE(solver_parameters, m) {
  py::class_<SolverParameters>(m, "SolverParameters")
      .def(py::init<>())
      .def_readwrite("max_iterations", &SolverParameters::max_iterations)
      .def_readwrite("normalize_quaternions",
                     &SolverParameters::normalize_quaternions)
      .def_readwrite("verbose", &SolverParameters::verbose)
      .def_readwrite("contact_stiffness", &SolverParameters::contact_stiffness)
      .def_readwrite("dissipation_velocity",
                     &SolverParameters::dissipation_velocity)
      .def_readwrite("stiction_velocity", &SolverParameters::stiction_velocity)
      .def_readwrite("friction_coefficient",
                     &SolverParameters::friction_coefficient)
      .def_readwrite("smoothing_factor", &SolverParameters::smoothing_factor)
      .def_readwrite("scaling", &SolverParameters::scaling)
      .def_readwrite("equality_constraints",
                     &SolverParameters::equality_constraints)
      .def_readwrite("Delta0", &SolverParameters::Delta0)
      .def_readwrite("Delta_max", &SolverParameters::Delta_max)
      .def_readwrite("num_threads", &SolverParameters::num_threads);
}
