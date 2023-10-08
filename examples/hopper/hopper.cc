#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace hopper {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

/**
 * A simple planar hopper, inspired by https://youtu.be/uWADBSmHebA?t=893.
 */
class HopperExample : public TrajOptExample {
 public:
  HopperExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(0.5, -2.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add a hopper
    std::string urdf_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/hopper.urdf");
    Parser(plant).AddModels(urdf_file);

    // Add collision with the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace hopper
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::hopper::HopperExample example;
  example.RunExample("idto/examples/hopper/hopper.yaml", FLAGS_test);

  return 0;
}
