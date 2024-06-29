#include "my_fun_library.h"

namespace idto {
namespace toy_example {

MyFunLibrary::MyFunLibrary() {}

int MyFunLibrary::SquarePlantGeneralizedPositions(
    const MultibodyPlant<double>& plant) {
  return plant.num_positions() * plant.num_positions();
}

}  // namespace toy_example
}  // namespace idto
