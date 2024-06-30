#include <iostream>

#include "my_fun_library.h"
#include <drake/multibody/plant/multibody_plant.h>

using drake::multibody::MultibodyPlant;
using idto::toy_example::MyFunLibrary;

int main(int argc, char** argv) {
  std::cout << "hello world" << std::endl;

  MultibodyPlant<double> plant = MultibodyPlant<double>(0.0);
  plant.Finalize();
  const int nq = plant.num_positions();
  std::cout << "nq: " << nq << std::endl;

  MyFunLibrary my_fun_library;
  const int nq_squared = my_fun_library.SquarePlantGeneralizedPositions(plant);
  std::cout << "nq^2: " << nq_squared << std::endl;

  return 0;
}
