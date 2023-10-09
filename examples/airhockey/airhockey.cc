#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/planar_joint.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace airhockey {

using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::PlanarJoint;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using Eigen::Vector3d;

class AirHockeyExample : public TrajOptExample {
 public:
  AirHockeyExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(0.0, -0.5, 2.0);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Colors that we'll use
    const drake::Vector4<double> red(0.9, 0.1, 0.0, 1.0);
    const drake::Vector4<double> red2(0.8, 0.1, 0.0, 1.0);
    const drake::Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Puck parameters
    const double mass = 0.1;
    const double radius = 0.1;
    const double height = 0.05;

    const SpatialInertia<double> I(
        mass, Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(radius, height, Vector3d::UnitZ()));

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
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::airhockey::AirHockeyExample example;
  example.RunExample("idto/examples/airhockey/airhockey.yaml", FLAGS_test);

  return 0;
}
