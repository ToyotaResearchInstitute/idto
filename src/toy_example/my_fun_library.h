#pragma once

#include <drake/multibody/plant/multibody_plant.h>

namespace idto {
namespace toy_example {

using drake::multibody::MultibodyPlant;

class MyFunLibrary {
 public:
  MyFunLibrary();

  /**
   * Return nq^2.
   */
  int SquarePlantGeneralizedPositions(const MultibodyPlant<double>& plant);
};

}  // namespace toy_example
}  // namespace idto
