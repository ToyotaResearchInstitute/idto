#pragma once

#include <memory>

#include "drake/multibody/fem/petsc_symmetric_block_sparse_matrix.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"

namespace drake {
namespace traj_opt {
namespace internal {

/* Function to build a PetscSymmetricBlockSparseMatrix from a
PentaDiagonalMatrix. PetscSymmetricBlockSparseMatrix::AssembleIfNecessary() was
already called on the returned matrix. */
std::unique_ptr<multibody::fem::internal::PetscSymmetricBlockSparseMatrix>
PentaDiagonalToPetscMatrix(const PentaDiagonalMatrix<double>& A);

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
