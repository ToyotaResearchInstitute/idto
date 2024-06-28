#include "my_fun_library.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using idto::toy_example::MyFunLibrary;

PYBIND11_MODULE(my_fun_bindings, m) {
    m.doc() = "Example module interfacing with pydrake and Drake C++";

    py::module::import("pydrake.multibody.plant");
    py::module::import("pydrake.multibody");

    py::class_<MyFunLibrary>(m, "MyFunLibrary")
        .def(py::init<>())
        .def("SquarePlantGeneralizedPositions",
             &MyFunLibrary::SquarePlantGeneralizedPositions);
}