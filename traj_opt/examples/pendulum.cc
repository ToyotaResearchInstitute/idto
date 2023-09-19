#include "traj_opt/examples/example_base.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace idto {
namespace traj_opt {
namespace examples {
namespace pendulum {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class PendulumExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace traj_opt
}  // namespace idto

int main() {
  idto::traj_opt::examples::pendulum::PendulumExample pendulum_example;
  pendulum_example.RunExample("anzu/traj_opt/examples/pendulum.yaml");
  return 0;
}
