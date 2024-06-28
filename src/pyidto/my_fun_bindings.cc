#include <pybind11/pybind11.h>

#include "toy_example/my_fun_library.h"

namespace py = pybind11;
using idto::toy_example::MyFunLibrary;

PYBIND11_MODULE(pyidto, m) {
  m.doc() = "Inverse Dynamics Trajectory Optimization (IDTO) python bindings.";

  py::module::import("pydrake.multibody.plant");

  py::class_<MyFunLibrary>(m, "MyFunLibrary")
      .def(py::init<>())
      .def("SquarePlantGeneralizedPositions",
           &MyFunLibrary::SquarePlantGeneralizedPositions);
}