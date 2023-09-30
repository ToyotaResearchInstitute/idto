#include <drake/multibody/tree/prismatic_joint.h>
#include "idto/traj_opt/examples/example_base.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace idto {
namespace traj_opt {
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
    std::vector<double> p = {1.0, 0.5, 1.5};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add a hopper
    std::string urdf_file =
        idto::FindIDTOResourceOrThrow("idto/traj_opt/examples/models/hopper.urdf");
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
}  // namespace traj_opt
}  // namespace idto

int main() {
  idto::traj_opt::examples::hopper::HopperExample example;
  example.RunExample("idto/traj_opt/examples/hopper.yaml");
  return 0;
}