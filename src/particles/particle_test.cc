// SPDX-License-Identifier: MIT-0

#include "particle.h"  // IWYU pragma: associated

#include <memory>

#include <gtest/gtest.h>

#include <drake/common/eigen_types.h>
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/system_output.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/framework/vector_base.h>

namespace drake_external_examples {
namespace particles {
namespace {

///
/// A test fixture class for Particle systems.
///
/// @tparam T must be a valid Eigen ScalarType.
///
template <typename T>
class ParticleTest : public ::testing::Test {
 protected:
  /// Arrange a Particle as the Device Under Test.
  void SetUp() override {
    this->dut_ = std::make_unique<Particle<T>>();
    this->context_ = this->dut_->CreateDefaultContext();
    this->output_ = this->dut_->AllocateOutput();
    this->derivatives_ = this->dut_->AllocateTimeDerivatives();
  }

  /// System (aka Device Under Test) being tested.
  std::unique_ptr<drake::systems::System<T>> dut_;
  /// Context for the given @p dut_.
  std::unique_ptr<drake::systems::Context<T>> context_;
  /// Outputs of the given @p dut_.
  std::unique_ptr<drake::systems::SystemOutput<T>> output_;
  /// Derivatives of the given @p dut_.
  std::unique_ptr<drake::systems::ContinuousState<T>> derivatives_;
};

TYPED_TEST_SUITE_P(ParticleTest);

/// Makes sure a Particle output is consistent with its
/// state (position and velocity).
TYPED_TEST_P(ParticleTest, OutputTest) {
  // Initialize state.
  drake::systems::VectorBase<TypeParam>& continuous_state_vector =
      this->context_->get_mutable_continuous_state_vector();
  continuous_state_vector.SetAtIndex(
      0, static_cast<TypeParam>(10.0));  // x0 = 10 m.
  continuous_state_vector.SetAtIndex(
      1, static_cast<TypeParam>(1.0));  // x1 = 1 m/s.
  // Compute outputs.
  this->dut_->CalcOutput(*this->context_, this->output_.get());
  drake::systems::BasicVector<TypeParam>* output_vector =
      this->output_->GetMutableVectorData(0);
  // Check results.
  EXPECT_EQ(output_vector->GetAtIndex(0),
            static_cast<TypeParam>(10.0));  // y0 == x0
  EXPECT_EQ(output_vector->GetAtIndex(1),
            static_cast<TypeParam>(1.0));  // y1 == x1
}

/// Makes sure a Particle system state derivatives are
/// consistent with its state and input (velocity and acceleration).
TYPED_TEST_P(ParticleTest, DerivativesTest) {
  // Set input.
  const drake::systems::InputPort<TypeParam>& input_port =
      this->dut_->get_input_port(0);
  drake::VectorX<TypeParam> u0(input_port.size());
  u0 << 1.0;  // m/s^2
  input_port.FixValue(this->context_.get(), u0);
  // Set state.
  drake::systems::VectorBase<TypeParam>& continuous_state_vector =
      this->context_->get_mutable_continuous_state_vector();
  continuous_state_vector.SetAtIndex(
      0, static_cast<TypeParam>(0.0));  // x0 = 0 m
  continuous_state_vector.SetAtIndex(
      1, static_cast<TypeParam>(2.0));  // x1 = 2 m/s
  // Compute derivatives.
  this->dut_->CalcTimeDerivatives(*this->context_, this->derivatives_.get());
  const drake::systems::VectorBase<TypeParam>& derivatives_vector =
      this->derivatives_->get_vector();
  // Check results.
  EXPECT_EQ(derivatives_vector.GetAtIndex(0),
            static_cast<TypeParam>(2.0));  // x0dot == x1
  EXPECT_EQ(derivatives_vector.GetAtIndex(1),
            static_cast<TypeParam>(1.0));  // x1dot == u0
}

REGISTER_TYPED_TEST_SUITE_P(ParticleTest, OutputTest, DerivativesTest);

INSTANTIATE_TYPED_TEST_SUITE_P(WithDoubles, ParticleTest, double);

}  // namespace
}  // namespace particles
}  // namespace drake_external_examples

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
