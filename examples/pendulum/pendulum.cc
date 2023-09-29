#include "examples/example_base.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace idto {
namespace examples {
namespace pendulum {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class PendulumExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        drake::FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace idto

int main() {
  idto::examples::pendulum::PendulumExample pendulum_example;
  pendulum_example.RunExample("idto/examples/pendulum/pendulum.yaml");
  return 0;
}
