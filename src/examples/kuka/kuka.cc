#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace kuka {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

class KukaExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a kuka arm
    std::string robot_file = drake::FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_spheres_collision.urdf");
    ModelInstanceIndex kuka = Parser(plant).AddModels(robot_file)[0];
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    plant->set_gravity_enabled(kuka, false);

    // Add a manipuland
    std::string manipuland_file =
        FindIdtoResourceOrThrow("idto/examples/models/box_intel_nuc.sdf");
    Parser(plant).AddModels(manipuland_file);

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
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::kuka::KukaExample example;
  example.RunExample("idto/examples/kuka/kuka.yaml", FLAGS_test);

  return 0;
}
