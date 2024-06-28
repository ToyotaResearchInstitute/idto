// SPDX-License-Identifier: MIT-0

///
/// @brief  A simple 1DOF, constantly accelerated particle example.
///

#include <sys/stat.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>

#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "particle.h"
#include "particle_geometry.h"

DEFINE_double(initial_position, 0.0, "Particle initial x position");
DEFINE_double(initial_velocity, 0.0, "Particle initial x velocity");
DEFINE_double(acceleration, 1.0, "Particle constant x acceleration");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate the particle");

namespace drake_external_examples {
namespace particles {

/// Check if the specified file exists.
/// @param[in] name of the file
/// @return existence (true) or otherwise (false)
bool file_exists(const std::string& name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

/// A sample diagram for visualizing a 1DOF particle to which a
/// a constant acceleration is applied.
class UniformlyAcceleratedParticle : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformlyAcceleratedParticle);

  /// A constructor that wires up the whole diagram,
  /// taking a constant acceleration to be applied to
  /// the particle.
  ///
  /// @param[in] acceleration in m/s^2 units.
  explicit UniformlyAcceleratedParticle(double acceleration);

  /// Creates a context using AllocateContext() and sets
  /// state variables according to the initial conditions supplied.
  ///
  /// @param[in] position in m units.
  /// @param[in] velocity in m/s units.
  /// @return a newly created Context.
  std::unique_ptr<drake::systems::Context<double>> CreateContext(
      double position, double velocity) const;
};

UniformlyAcceleratedParticle::UniformlyAcceleratedParticle(
    const double acceleration) {
  // Building diagram.
  drake::systems::DiagramBuilder<double> builder;
  // Adding particle.
  auto particle = builder.AddSystem<Particle<double>>();
   // Adding constant acceleration source.
  auto constant_acceleration_vector_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(acceleration);
  builder.Connect(*constant_acceleration_vector_source, *particle);
  // Adding visualization.
  auto scene_graph = builder.AddSystem<drake::geometry::SceneGraph>();
  ParticleGeometry::AddToBuilder(
      &builder, particle->get_output_port(0), scene_graph);
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  builder.BuildInto(this);
}

std::unique_ptr<drake::systems::Context<double>>
UniformlyAcceleratedParticle::CreateContext(const double position,
                                            const double velocity) const {
  // Allocate context.
  auto context = this->AllocateContext();
  // Set continuous state.
  drake::systems::VectorBase<double>& cstate =
      context->get_mutable_continuous_state_vector();
  cstate.SetAtIndex(0, position);
  cstate.SetAtIndex(1, velocity);
  return context;
}

///
/// Main function for demo.
///
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A very simple demonstration, "
      "make sure drake-visualizer is running!");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Instantiate example system.
  auto system = std::make_unique<UniformlyAcceleratedParticle>(
      FLAGS_acceleration);
  // Get context with initial conditions.
  auto context =
      system->CreateContext(FLAGS_initial_position, FLAGS_initial_velocity);
  // Instantiate and configure simulator.
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *system, std::move(context));
  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();
  // Run simulation.
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace particles
}  // namespace drake_external_examples

int main(int argc, char** argv) {
  return drake_external_examples::particles::main(argc, argv);
}
