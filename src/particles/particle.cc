// SPDX-License-Identifier: MIT-0

#include "particle.h"

#include <drake/systems/framework/framework_common.h>
#include <drake/systems/framework/vector_base.h>

namespace drake_external_examples {
namespace particles {

template <typename T>
Particle<T>::Particle() {
  // A 1D input vector for acceleration.
  this->DeclareInputPort(drake::systems::kUseDefaultName,
                         drake::systems::kVectorValued, 1);
  // Adding one generalized position and one generalized velocity.
  this->DeclareContinuousState(1, 1, 0);
  // A 2D output vector for position and velocity.
  this->DeclareVectorOutputPort(drake::systems::kUseDefaultName,
                                drake::systems::BasicVector<T>(2),
                                &Particle::CopyStateOut);
}

template <typename T>
void Particle<T>::CopyStateOut(const drake::systems::Context<T>& context,
                               drake::systems::BasicVector<T>* output) const {
  // Get current state from context.
  const drake::systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Write system output.
  output->set_value(continuous_state_vector.CopyToVector());
}

template <typename T>
void Particle<T>::DoCalcTimeDerivatives(
    const drake::systems::Context<T>& context,
    drake::systems::ContinuousState<T>* derivatives) const {
  // Get current state from context.
  const drake::systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Obtain the structure we need to write into.
  drake::systems::VectorBase<T>& derivatives_vector =
      derivatives->get_mutable_vector();
  // Get current input acceleration value.
  const drake::systems::BasicVector<T>* input_vector =
      this->EvalVectorInput(context, 0);
  // Set the derivatives. The first one is
  // velocity and the second one is acceleration.
  derivatives_vector.SetAtIndex(0, continuous_state_vector.GetAtIndex(1));
  derivatives_vector.SetAtIndex(1, input_vector->GetAtIndex(0));
}

template class Particle<double>;

}  // namespace particles
}  // namespace drake_external_examples
