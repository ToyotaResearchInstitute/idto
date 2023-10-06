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
namespace acrobot {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

class AcrobotExample : public TrajOptExample {
 public:
  AcrobotExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(0.0, 5.0, 1.0);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        "external/drake/examples/acrobot/Acrobot_no_collision.urdf";
    Parser(plant).AddModels(urdf_file);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"));
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::acrobot::AcrobotExample example;
  example.RunExample("idto/examples/acrobot/acrobot.yaml", FLAGS_test);

  return 0;
}
