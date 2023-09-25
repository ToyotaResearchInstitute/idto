#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace acrobot {

using multibody::MultibodyPlant;
using multibody::Parser;

class AcrobotExample : public TrajOptExample {
 public:
  AcrobotExample() {
    // Set the camera viewpoint
    std::vector<double> p = {0.0, 1.0, -5.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        FindResourceOrThrow("drake/examples/acrobot/Acrobot_no_collision.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"));
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::acrobot::AcrobotExample example;
  const std::string yaml_file = "drake/traj_opt/examples/acrobot.yaml";
  example.RunExample(yaml_file);

  return 0;
}
