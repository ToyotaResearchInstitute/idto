#include "examples/example_base.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "utils/find_resource.h"

namespace idto {
namespace examples {
namespace two_dof_spinner {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class TwoDofSpinnerExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file = idto::FindIDTOResourceOrThrow(
        "idto/traj_opt/examples/models/2dof_spinner.urdf");
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace two_dof_spinner
}  // namespace examples
}  // namespace idto

int main() {
  idto::examples::two_dof_spinner::TwoDofSpinnerExample
      spinner_example;
  spinner_example.RunExample("idto/traj_opt/examples/2dof_spinner.yaml");
  return 0;
}
