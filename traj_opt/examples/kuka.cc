#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace kuka {

using Eigen::Vector3d;
using geometry::Box;
using math::RigidTransformd;
using multibody::CoulombFriction;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;

class KukaExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    const Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a kuka arm
    std::string robot_file = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_spheres_collision.urdf");
    ModelInstanceIndex kuka = Parser(plant).AddModelFromFile(robot_file);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    plant->disable_gravity(kuka);

    // Add a manipuland
    std::string manipuland_file =
        FindResourceOrThrow("drake/traj_opt/examples/models/box_intel_nuc.sdf");
    Parser(plant).AddAllModelsFromFile(manipuland_file);

    // Add the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace kuka
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::kuka::KukaExample example;
  example.RunExample("drake/traj_opt/examples/kuka.yaml");
  return 0;
}
