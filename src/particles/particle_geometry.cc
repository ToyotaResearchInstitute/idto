// SPDX-License-Identifier: MIT-0

#include "particle_geometry.h"

#include <memory>
#include <utility>

#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/math/rigid_transform.h>

namespace drake_external_examples {
namespace particles {

const ParticleGeometry* ParticleGeometry::AddToBuilder(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::systems::OutputPort<double>& particle_state_port,
    drake::geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto particle_geometry = builder->AddSystem(
      std::unique_ptr<ParticleGeometry>(
          new ParticleGeometry(scene_graph)));
  builder->Connect(
      particle_state_port,
      particle_geometry->get_input_port(0));
  builder->Connect(
      particle_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(particle_geometry->source_id_));

  return particle_geometry;
}

ParticleGeometry::ParticleGeometry(drake::geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();
  frame_id_ = scene_graph->RegisterFrame(
      source_id_, drake::geometry::GeometryFrame("particle"));

  this->DeclareInputPort("state", drake::systems::kVectorValued, 2);
  this->DeclareAbstractOutputPort(
      "geometry_pose", &ParticleGeometry::OutputGeometryPose);

  const drake::geometry::GeometryId id = scene_graph->RegisterGeometry(
      source_id_, frame_id_,
      std::make_unique<drake::geometry::GeometryInstance>(
          drake::math::RigidTransformd::Identity(),
          std::make_unique<drake::geometry::Sphere>(0.1), "sphere_visual"));
  scene_graph->AssignRole(
      source_id_, id, drake::geometry::MakePhongIllustrationProperties(
          Eigen::Vector4d(1, 0, 0, 1)));
}

void ParticleGeometry::OutputGeometryPose(
    const drake::systems::Context<double>& context,
    drake::geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const auto& input = get_input_port(0).Eval(context);
  const double q = input(0);
  const drake::math::RigidTransformd pose(Eigen::Vector3d{q, 0, 0});

  *poses = {{frame_id_, pose}};
}

}  // namespace particles
}  // namespace drake_external_examples
