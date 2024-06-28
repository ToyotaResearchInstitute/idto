// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides an example of creating a simple Drake C++ system and binding it in
 * pybind11, to be used with pydrake.
 */

#include <pybind11/pybind11.h>

#include <drake/systems/framework/leaf_system.h>

namespace py = pybind11;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::kVectorValued;

namespace drake_external_examples {
namespace {

/// Adds a constant to an input.
template <typename T>
class SimpleAdder : public LeafSystem<T> {
 public:
  explicit SimpleAdder(T add)
      : add_(add) {
    this->DeclareInputPort("in", kVectorValued, 1);
    this->DeclareVectorOutputPort(
        "out", BasicVector<T>(1), &SimpleAdder::CalcOutput);
  }

 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const {
    auto u = this->get_input_port(0).Eval(context);
    auto&& y = output->get_mutable_value();
    y.array() = u.array() + add_;
  }

  const T add_{};
};

PYBIND11_MODULE(simple_bindings, m) {
  m.doc() = "Example module interfacing with pydrake and Drake C++";

  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<SimpleAdder<T>, LeafSystem<T>>(m, "SimpleAdder")
      .def(py::init<T>(), py::arg("add"));
}

}  // namespace
}  // namespace drake_external_examples
