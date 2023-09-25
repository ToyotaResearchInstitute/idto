#include "drake/common/find_resource.h"
#include "drake/common/profiler.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace airhockey {

using Eigen::Vector3d;
using geometry::Box;
using geometry::Cylinder;
using geometry::Sphere;
using math::RigidTransformd;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::PlanarJoint;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UnitInertia;

class AirHockeyExample : public TrajOptExample {
 public:
  AirHockeyExample() {
    // Set the camera viewpoint
    std::vector<double> p = {0.0, 2.0, 0.5};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Colors that we'll use
    const Vector4<double> red(0.9, 0.1, 0.0, 1.0);
    const Vector4<double> red2(0.8, 0.1, 0.0, 1.0);
    const Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Puck parameters
    const double mass = 0.1;
    const double radius = 0.1;
    const double height = 0.05;

    const SpatialInertia<double> I(
        mass, Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(radius, height));

    // Create the pusher
    const RigidBody<double>& pusher = plant->AddRigidBody("pusher", I);
    plant->RegisterVisualGeometry(pusher, RigidTransformd(),
                                  Cylinder(radius, height), "pusher", red);
    plant->RegisterVisualGeometry(
        pusher, RigidTransformd(Vector3d(0.0, 0.0, height)),
        Box(radius / 2, radius / 2, height), "handle", red2);
    plant->RegisterCollisionGeometry(pusher, RigidTransformd::Identity(),
                                     Sphere(radius), "pusher_collision",
                                     CoulombFriction<double>());

    // N.B. we need to add this joint manually (not PlanarJoint) if we want
    // actuation
    const SpatialInertia<double> I_dummy(0.0, Vector3d::Zero(),
                                         UnitInertia<double>(0, 0, 0));
    const RigidBody<double>& dummy1 = plant->AddRigidBody("dummy1", I_dummy);
    const RigidBody<double>& dummy2 = plant->AddRigidBody("dummy2", I_dummy);

    const PrismaticJoint<double>& pusher_x = plant->AddJoint<PrismaticJoint>(
        "pusher_x", plant->world_body(), {}, dummy1, {}, Vector3d(1, 0, 0));
    const PrismaticJoint<double>& pusher_y = plant->AddJoint<PrismaticJoint>(
        "pusher_y", dummy1, {}, dummy2, {}, Vector3d(0, 1, 0));
    const RevoluteJoint<double>& pusher_theta = plant->AddJoint<RevoluteJoint>(
        "pusher_theta", dummy2, {}, pusher, {}, Vector3d(0, 0, 1));

    plant->AddJointActuator("pusher_x", pusher_x);
    plant->AddJointActuator("pusher_y", pusher_y);
    plant->AddJointActuator("pusher_theta", pusher_theta);

    // Create the puck
    const RigidBody<double>& puck = plant->AddRigidBody("puck", I);
    plant->RegisterVisualGeometry(puck, RigidTransformd(),
                                  Cylinder(radius, height), "puck", blue);
    plant->RegisterVisualGeometry(puck, RigidTransformd(),
                                  Box(radius / 2, 1.5 * radius, 1.01 * height),
                                  "marker", black);
    plant->RegisterCollisionGeometry(puck, RigidTransformd::Identity(),
                                     Sphere(radius), "puck_collision",
                                     CoulombFriction<double>());

    const Vector3d damping(0.1, 0.1, 0.1);
    plant->AddJoint<PlanarJoint>("puck_joint", plant->world_body(),
                                 RigidTransformd(), puck, {}, damping);
  }
};

}  // namespace airhockey
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::airhockey::AirHockeyExample example;
  example.RunExample("drake/traj_opt/examples/airhockey.yaml");
  return 0;
}
