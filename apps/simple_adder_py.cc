// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides an example of binding a simple Drake C++ system in pybind11, to be
 * used with pydrake.
 */

#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"

#include "simple_adder.h"

namespace py = pybind11;

using drake::pydrake::CommonScalarPack;
using drake::pydrake::DefineTemplateClassWithDefault;
using drake::pydrake::GetPyParam;
using drake::systems::LeafSystem;

namespace drake_external_examples {
namespace {

PYBIND11_MODULE(simple_adder, m) {
  m.doc() = "Example module interfacing with pydrake and Drake C++";

  py::module::import("pydrake.systems.framework");

  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<SimpleAdder<T>, LeafSystem<T>>(
        m, "SimpleAdder", GetPyParam<T>())
      .def(py::init<double>(), py::arg("add"));
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace
}  // namespace drake_external_examples
