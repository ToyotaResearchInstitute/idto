#include "examples/example_base.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "utils/find_resource.h"

namespace idto {
namespace examples {
namespace wall_ball {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

/**
 * A simple 1-DoF system with contact
 *
 */
class WallBallExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file = idto::FindIDTOResourceOrThrow(
        "idto/traj_opt/examples/models/wall_ball.urdf");
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace wall_ball
}  // namespace examples
}  // namespace idto

int main() {
  idto::examples::wall_ball::WallBallExample wall_ball_example;
  wall_ball_example.RunExample("idto/traj_opt/examples/wall_ball.yaml");
  return 0;
}
