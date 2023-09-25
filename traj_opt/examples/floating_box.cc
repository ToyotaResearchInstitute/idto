#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace floating_box {

using math::RigidTransformd;
using multibody::MultibodyPlant;
using multibody::RigidBody;

/**
 * The simplest possible system with a floating base: a box that floats through
 * the air.
 */
class FloatingBoxExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const RigidBody<double>& body = plant->AddRigidBody(
        "body", multibody::SpatialInertia<double>::MakeUnitary());

    plant->RegisterVisualGeometry(body, RigidTransformd::Identity(),
                                  geometry::Box(0.1, 0.1, 0.1), "box_visual");
  }
};

}  // namespace floating_box
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::floating_box::FloatingBoxExample example;
  example.RunExample("drake/traj_opt/examples/floating_box.yaml");
  return 0;
}
