#include "examples/example_base.h"

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "utils/find_resource.h"

namespace idto {
namespace examples {
namespace spinner {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Matrix4d;

class SpinnerExample : public TrajOptExample {
 public:
  SpinnerExample() {
    // Set the camera viewpoint
    std::vector<double> p = {3.0, 1.0, 1.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // N.B. geometry of the spinner is chosen via gflags rather than yaml so
    // that we can use the same yaml format for all of the examples, without
    // cluttering it with spinner-specific options.
    std::string urdf_file = idto::FindIDTOResourceOrThrow(
        "examples/models/spinner_friction.urdf");
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace spinner
}  // namespace examples
}  // namespace idto

int main() {
  idto::examples::spinner::SpinnerExample spinner_example;
  spinner_example.RunExample("examples/spinner/spinner.yaml");
  return 0;
}
