#pragma once

#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace idto {
namespace optimizer {

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;

/**
 * A container for scratch variables that we use in various intermediate
 * computations. Allows us to avoid extra allocations when speed is important.
 */
template <typename T>
struct TrajectoryOptimizerWorkspace {
  // Construct a workspace with size matching the given plant.
  TrajectoryOptimizerWorkspace(const int num_steps,
                               const MultibodyPlant<T>& plant)
      : f_ext(plant) {
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();
    const int num_vars = nq * (num_steps + 1);

    // Get number of unactuated DoFs
    int num_unactuated = 0;
    const MatrixX<T> B = plant.MakeActuationMatrix();
    if (B.size() > 0) {
      // if B is of size zero, assume the system is fully actuated
      for (int i = 0; i < nv; ++i) {
        if (B.row(i).sum() == 0) {
          ++num_unactuated;
        }
      }
    }
    const int num_eq_cons = num_unactuated * num_steps;

    // Set vector sizes
    q_size_tmp1.resize(nq);
    q_size_tmp2.resize(nq);
    q_size_tmp3.resize(nq);
    q_size_tmp4.resize(nq);

    v_size_tmp1.resize(nv);
    v_size_tmp2.resize(nv);
    v_size_tmp3.resize(nv);
    v_size_tmp4.resize(nv);
    v_size_tmp5.resize(nv);
    v_size_tmp6.resize(nv);
    v_size_tmp7.resize(nv);
    v_size_tmp8.resize(nv);

    tau_size_tmp1.resize(nv);
    tau_size_tmp2.resize(nv);
    tau_size_tmp3.resize(nv);
    tau_size_tmp4.resize(nv);
    tau_size_tmp5.resize(nv);
    tau_size_tmp6.resize(nv);
    tau_size_tmp7.resize(nv);
    tau_size_tmp8.resize(nv);
    tau_size_tmp9.resize(nv);
    tau_size_tmp10.resize(nv);
    tau_size_tmp11.resize(nv);
    tau_size_tmp12.resize(nv);

    a_size_tmp1.resize(nv);
    a_size_tmp2.resize(nv);
    a_size_tmp3.resize(nv);
    a_size_tmp4.resize(nv);
    a_size_tmp5.resize(nv);
    a_size_tmp6.resize(nv);
    a_size_tmp7.resize(nv);
    a_size_tmp8.resize(nv);
    a_size_tmp9.resize(nv);
    a_size_tmp10.resize(nv);
    a_size_tmp11.resize(nv);
    a_size_tmp12.resize(nv);

    num_vars_size_tmp1.resize(num_vars);
    num_vars_size_tmp2.resize(num_vars);
    num_vars_size_tmp3.resize(num_vars);

    num_vars_by_num_eq_cons_tmp.resize(num_vars, num_eq_cons);
    mass_matrix_size_tmp.resize(nv, nv);

    // Allocate sequences
    q_sequence_tmp1.assign(num_steps, VectorX<T>(nq));
    q_sequence_tmp2.assign(num_steps, VectorX<T>(nq));
  }

  // Storage for multibody forces
  MultibodyForces<T> f_ext;

  // Storage of size nq
  VectorX<T> q_size_tmp1;
  VectorX<T> q_size_tmp2;
  VectorX<T> q_size_tmp3;
  VectorX<T> q_size_tmp4;

  // Storage of size nv
  // These are named v, tau, and a, but this distinction is just for
  // convienience.
  VectorX<T> v_size_tmp1;
  VectorX<T> v_size_tmp2;
  VectorX<T> v_size_tmp3;
  VectorX<T> v_size_tmp4;
  VectorX<T> v_size_tmp5;
  VectorX<T> v_size_tmp6;
  VectorX<T> v_size_tmp7;
  VectorX<T> v_size_tmp8;

  VectorX<T> tau_size_tmp1;
  VectorX<T> tau_size_tmp2;
  VectorX<T> tau_size_tmp3;
  VectorX<T> tau_size_tmp4;
  VectorX<T> tau_size_tmp5;
  VectorX<T> tau_size_tmp6;
  VectorX<T> tau_size_tmp7;
  VectorX<T> tau_size_tmp8;
  VectorX<T> tau_size_tmp9;
  VectorX<T> tau_size_tmp10;
  VectorX<T> tau_size_tmp11;
  VectorX<T> tau_size_tmp12;

  VectorX<T> a_size_tmp1;
  VectorX<T> a_size_tmp2;
  VectorX<T> a_size_tmp3;
  VectorX<T> a_size_tmp4;
  VectorX<T> a_size_tmp5;
  VectorX<T> a_size_tmp6;
  VectorX<T> a_size_tmp7;
  VectorX<T> a_size_tmp8;
  VectorX<T> a_size_tmp9;
  VectorX<T> a_size_tmp10;
  VectorX<T> a_size_tmp11;
  VectorX<T> a_size_tmp12;

  // Storage of sequence of q
  std::vector<VectorX<T>> q_sequence_tmp1;
  std::vector<VectorX<T>> q_sequence_tmp2;

  // Vector of all decision variables
  VectorX<T> num_vars_size_tmp1;
  VectorX<T> num_vars_size_tmp2;
  VectorX<T> num_vars_size_tmp3;

  // Matrix of size (number of variables) * (number of equality constraints)
  MatrixX<T> num_vars_by_num_eq_cons_tmp;

  // Matrix of size nv x nv, used to store the mass matrix
  MatrixX<T> mass_matrix_size_tmp;
};

}  // namespace optimizer
}  // namespace idto
