#include "toy_example/my_fun_library.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;
using idto::toy_example::MyFunLibrary;

void bind_problem_definition(py::module_&);

PYBIND11_MODULE(pyidto, m) {
  m.doc() = "Inverse Dynamics Trajectory Optimization (IDTO) python bindings.";

  py::module::import("pydrake.multibody.plant");

  py::class_<MyFunLibrary>(m, "MyFunLibrary")
      .def(py::init<>())
      .def("SquarePlantGeneralizedPositions",
           &MyFunLibrary::SquarePlantGeneralizedPositions);

  bind_problem_definition(m);
}
