#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_problem_definition(py::module_&);
void bind_solver_parameters(py::module_&);
void bind_trajectory_optimizer(py::module_&);
void bind_trajectory_optimizer_solution(py::module_&);
void bind_trajectory_optimizer_stats(py::module_&);
void bind_find_resource(py::module_&);

PYBIND11_MODULE(pyidto, m) {
  m.doc() = "Inverse Dynamics Trajectory Optimization (IDTO) python bindings.";

  py::module::import("pydrake.multibody.plant");

  bind_problem_definition(m);
  bind_solver_parameters(m);
  bind_trajectory_optimizer(m);
  bind_trajectory_optimizer_solution(m);
  bind_trajectory_optimizer_stats(m);
  bind_find_resource(m);
}
