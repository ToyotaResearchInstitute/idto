#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

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

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::pendulum::PendulumExample pendulum_example;
  pendulum_example.RunExample("idto/examples/pendulum/pendulum.yaml",
                              FLAGS_test);

  return 0;
}
