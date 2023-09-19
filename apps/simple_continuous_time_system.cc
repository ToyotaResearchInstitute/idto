// SPDX-License-Identifier: MIT-0

// Simple Continuous Time System Example
//
// This is meant to be a sort of "hello world" example for the drake::system
// classes. It defines a very simple continuous time system and simulates it
// from a given initial condition.

#include <cmath>

#include <drake/common/drake_assert.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/leaf_system.h>

namespace drake_external_examples {
namespace systems {

// Simple Continuous Time System
//   xdot = -x + x³
//   y = x
class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    DeclareVectorOutputPort("y", drake::systems::BasicVector<double>(1),
                            &SimpleContinuousTimeSystem::CopyStateOut);
    DeclareContinuousState(1);  // One state variable.
  }

 private:
  // xdot = -x + x³
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    const double x = context.get_continuous_state()[0];
    const double xdot = -x + std::pow(x, 3.0);
    (*derivatives)[0] = xdot;
  }

  // y = x
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    const double x = context.get_continuous_state()[0];
    (*output)[0] = x;
  }
};

}  // namespace systems
}  // namespace drake_external_examples

int main() {
  // Create the simple system.
  drake_external_examples::systems::SimpleContinuousTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::ContinuousState<double>& state =
      simulator.get_mutable_context().get_mutable_continuous_state();
  state[0] = 0.9;

  // Simulate for 10 seconds.
  simulator.AdvanceTo(10);

  // Make sure the simulation converges to the stable fixed point at x = 0.
  DRAKE_DEMAND(state[0] < 1.0e-4);

  return 0;
}
