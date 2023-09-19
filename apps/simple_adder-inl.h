// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides an example of creating a simple Drake C++ system that will later be
 * bound.
 *
 * This is the definition portion of the -inl.h pattern:
 * http://drake.mit.edu/cxx_inl.html#cxx-inl-files
 */

#include "simple_adder.h"

namespace drake_external_examples {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::kVectorValued;

template <typename T>
SimpleAdder<T>::SimpleAdder(T add)
      : add_(add) {
  this->DeclareInputPort("in", kVectorValued, 1);
  this->DeclareVectorOutputPort(
      "out", BasicVector<T>(1), &SimpleAdder::CalcOutput);
}

template <typename T>
void SimpleAdder<T>::CalcOutput(
    const Context<T>& context, BasicVector<T>* output) const {
  auto u = this->get_input_port(0).Eval(context);
  auto&& y = output->get_mutable_value();
  y.array() = u.array() + add_;
}

}  // namespace drake_external_examples
