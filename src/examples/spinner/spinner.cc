#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace spinner {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Matrix4d;
using Eigen::Vector3d;

class SpinnerExample : public TrajOptExample {
 public:
  SpinnerExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.0, -0.5, 2.0);
    const Vector3d target_pose(0.8, -0.5, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // N.B. geometry of the spinner is chosen via gflags rather than yaml so
    // that we can use the same yaml format for all of the examples, without
    // cluttering it with spinner-specific options.
    std::string urdf_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/spinner_friction.urdf");
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace spinner
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  idto::examples::spinner::SpinnerExample spinner_example;

  spinner_example.RunExample("idto/examples/spinner/spinner.yaml", FLAGS_test);

  return 0;
}
