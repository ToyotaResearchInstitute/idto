#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_int32(hills, 0, "number of simulated hills to walk over");
DEFINE_double(hill_height, 0.05, "height of each simulated hill, in meters");
DEFINE_double(hill_spacing, 1.0,
              "distance between each simulated hill, in meters");
DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace mini_cheetah {

using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

/**
 * A model of the MIT mini cheetah quadruped.
 */
class MiniCheetahExample : public TrajOptExample {
 public:
  MiniCheetahExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(3.0, 3.0, 1.0);
    const Vector3d target_pose(2.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add the robot
    std::string urdf_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/mini_cheetah_mesh.urdf");
    Parser(plant).AddModels(urdf_file);

    // Add collision with the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));

    // Add some extra terrain
    for (int i = 0; i < FLAGS_hills; ++i) {
      const double px = 2.0 + FLAGS_hill_spacing * static_cast<double>(i);
      const std::string name = "hill_" + std::to_string(i);
      const RigidTransformd X_hill(RollPitchYawd(M_PI_2, 0, 0),
                                   Vector3d(px, 0.0, -1.0 + FLAGS_hill_height));
      plant->RegisterVisualGeometry(plant->world_body(), X_hill,
                                    Cylinder(1.0, 25), name, green);
      plant->RegisterCollisionGeometry(plant->world_body(), X_hill,
                                       Cylinder(1.0, 25), name,
                                       CoulombFriction<double>(0.5, 0.5));
    }
  }
};

}  // namespace mini_cheetah
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::mini_cheetah::MiniCheetahExample example;
  example.RunExample("idto/examples/mini_cheetah/mini_cheetah.yaml",
                     FLAGS_test);

  return 0;
}
