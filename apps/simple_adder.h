// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides an example of creating a simple Drake C++ system that will later be
 * bound.
 *
 * This is the header portion of the -inl.h pattern:
 * http://drake.mit.edu/cxx_inl.html#cxx-inl-files
 */

#include <drake/systems/framework/leaf_system.h>

namespace drake_external_examples {

/// Adds a constant to an input.
template <typename T>
class SimpleAdder : public drake::systems::LeafSystem<T> {
 public:
  explicit SimpleAdder(T add);

 private:
  void CalcOutput(
      const drake::systems::Context<T>& context,
      drake::systems::BasicVector<T>* output) const;

  const T add_{};
};

}  // namespace drake_external_examples
