#include "idto/traj_opt/examples/example_base.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace idto {
namespace traj_opt {
namespace examples {
namespace floating_box {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;

/**
 * The simplest possible system with a floating base: a box that floats through
 * the air.
 */
class FloatingBoxExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const RigidBody<double>& body = plant->AddRigidBody(
        "body", drake::multibody::SpatialInertia<double>::MakeUnitary());

    plant->RegisterVisualGeometry(body, RigidTransformd::Identity(),
                                  drake::geometry::Box(0.1, 0.1, 0.1),
                                  "box_visual");
  }
};

}  // namespace floating_box
}  // namespace examples
}  // namespace traj_opt
}  // namespace idto

int main() {
  idto::traj_opt::examples::floating_box::FloatingBoxExample example;
  example.RunExample("idto/traj_opt/examples/floating_box.yaml");
  return 0;
}
