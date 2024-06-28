// SPDX-License-Identifier: MIT-0

///
/// @brief  A simple find_resource test for locating installed drake resources.
///

#include <iostream>
#include <stdexcept>

#include <drake/common/find_resource.h>

namespace drake_external_examples {
namespace {

int main() {
  drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");

  try {
    drake::FindResourceOrThrow("nobody_home.urdf");
    std::cerr << "Should have thrown" << std::endl;
    return 1;
  } catch (const std::runtime_error&) {
  }

  return 0;
}

}  // namespace
}  // namespace drake_external_examples

int main() { return drake_external_examples::main(); }
