#include <gflags/gflags.h>
#include "idto/traj_opt/examples/example_base.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "idto/common/find_resource.h"

DEFINE_int32(hills, 0, "number of simulated hills to walk over");
DEFINE_double(hill_height, 0.05, "height of each simulated hill, in meters");
DEFINE_double(hill_spacing, 1.0,
              "distance between each simulated hill, in meters");

namespace idto {
namespace traj_opt {
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
    std::vector<double> p = {5.0, 1.0, -5.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add the robot
    std::string urdf_file = idto::FindIDTOResourceOrThrow(
        "idto/traj_opt/examples/models/mini_cheetah_mesh.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);

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
}  // namespace traj_opt
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  idto::traj_opt::examples::mini_cheetah::MiniCheetahExample example;
  example.RunExample("idto/traj_opt/examples/mini_cheetah.yaml");
  return 0;
}
