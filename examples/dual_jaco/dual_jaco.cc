#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace dual_jaco {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

class DualJacoExample : public TrajOptExample {
 public:
  DualJacoExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.5, 0.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add jaco arms
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_sphere_collision_v2.sdf");

    ModelInstanceIndex jaco_left = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_left, "jaco_left");
    RigidTransformd X_left(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.11));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_left), X_left);
    plant->set_gravity_enabled(jaco_left, false);

    ModelInstanceIndex jaco_right = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_right, "jaco_right");
    RigidTransformd X_right(RollPitchYaw<double>(0, 0, M_PI_2),
                            Vector3d(0, -0.27, 0.11));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_right), X_right);
    plant->set_gravity_enabled(jaco_right, false);

    // Add a manipuland
    std::string manipuland_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/box_15cm.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    // Add jaco arms, including gravity
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_hydro_collision.sdf");

    ModelInstanceIndex jaco_left = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_left, "jaco_left");
    RigidTransformd X_left(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.11));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_left), X_left);
    plant->set_gravity_enabled(jaco_left, false);

    ModelInstanceIndex jaco_right = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_right, "jaco_right");
    RigidTransformd X_right(RollPitchYaw<double>(0, 0, M_PI_2),
                            Vector3d(0, -0.27, 0.11));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_right), X_right);
    plant->set_gravity_enabled(jaco_right, false);

    // Add a manipuland with compliant hydroelastic contact
    std::string manipuland_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/box_15cm_hydro.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground with compliant hydroelastic contact
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);

    ProximityProperties ground_proximity;
    AddContactMaterial({}, {}, CoulombFriction<double>(0.5, 0.5),
                       &ground_proximity);
    AddCompliantHydroelasticProperties(0.1, 5e7, &ground_proximity);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     ground_proximity);
  }
};

}  // namespace dual_jaco
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::dual_jaco::DualJacoExample example;
  example.RunExample("idto/examples/dual_jaco/dual_jaco.yaml", FLAGS_test);

  return 0;
}
