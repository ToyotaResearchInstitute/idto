#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/common/find_resource.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace punyo {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using Eigen::Vector3d;

class PunyoExample : public TrajOptExample {
 public:
  PunyoExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(0.0, 2.0, 1.0);
    const Vector3d target_pose(0.0, 0.0, 0.5);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a humanoid model
    std::string sdf_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/punyoid.sdf");
    ModelInstanceIndex humanoid = Parser(plant).AddModels(sdf_file)[0];
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    plant->set_gravity_enabled(humanoid, false);

    // Add a free-floating ball to pick up
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");

    const double mass = 1.0;
    const double radius = 0.2;

    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);

    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     CoulombFriction<double>(0.5, 0.5));

    // Add some markers to the ball so we can see its rotation
    RigidTransformd X_m1(RollPitchYawd(0, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m2(RollPitchYawd(M_PI_2, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m3(RollPitchYawd(0, M_PI_2, 0), Vector3d(0, 0, 0));
    plant->RegisterVisualGeometry(ball, X_m1,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_one", black);
    plant->RegisterVisualGeometry(ball, X_m2,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_two", black);
    plant->RegisterVisualGeometry(ball, X_m3,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_three", black);

    // Add the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a humanoid model
    std::string sdf_file =
        FindIdtoResourceOrThrow("idto/examples/models/punyoid.sdf");
    ModelInstanceIndex humanoid = Parser(plant).AddModels(sdf_file)[0];
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    plant->set_gravity_enabled(humanoid, false);

    // Add a free-floating ball to pick up
    const double mass = 1.0;
    const double radius = 0.2;
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");
    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);

    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);

    ProximityProperties ball_proximity;
    AddContactMaterial(3.0, {}, CoulombFriction<double>(1.5, 1.5),
                       &ball_proximity);
    AddCompliantHydroelasticProperties(0.1, 1e5, &ball_proximity);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     ball_proximity);

    // Add some markers to the ball so we can see its rotation
    RigidTransformd X_m1(RollPitchYawd(0, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m2(RollPitchYawd(M_PI_2, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m3(RollPitchYawd(0, M_PI_2, 0), Vector3d(0, 0, 0));
    plant->RegisterVisualGeometry(ball, X_m1,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_one", black);
    plant->RegisterVisualGeometry(ball, X_m2,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_two", black);
    plant->RegisterVisualGeometry(ball, X_m3,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_three", black);

    // Add the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace punyo
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::punyo::PunyoExample example;
  example.RunExample("idto/examples/punyo/punyo.yaml", FLAGS_test);

  return 0;
}
