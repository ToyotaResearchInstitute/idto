#include "drake/traj_opt/penta_diagonal_to_petsc_matrix.h"

#include <vector>

using drake::multibody::fem::internal::PetscSymmetricBlockSparseMatrix;
using Eigen::MatrixXd;

namespace drake {
namespace traj_opt {
namespace internal {

std::unique_ptr<PetscSymmetricBlockSparseMatrix> PentaDiagonalToPetscMatrix(
    const PentaDiagonalMatrix<double>& M) {
  const int num_rows = M.rows();
  const int num_block_rows = M.block_rows();
  const int block_size = M.block_size();
  std::vector<int> num_upper_triangular_blocks_per_row(num_block_rows, 3);
  num_upper_triangular_blocks_per_row[num_block_rows - 2] = 2;
  num_upper_triangular_blocks_per_row[num_block_rows - 1] = 1;

  auto Mpetsc = std::make_unique<PetscSymmetricBlockSparseMatrix>(
      num_rows, block_size, num_upper_triangular_blocks_per_row);

  for (int ib = 0; ib < num_block_rows; ++ib) {
    const MatrixXd& A = M.A()[ib];
    const MatrixXd& B = M.B()[ib];
    const MatrixXd& C = M.C()[ib];
    const MatrixXd& D = M.D()[ib];
    const MatrixXd& E = M.E()[ib];

    if (ib >= 2) Mpetsc->AddToBlock(ib, ib - 2, A);
    if (ib >= 1) Mpetsc->AddToBlock(ib, ib - 1, B);
    Mpetsc->AddToBlock(ib, ib, C);
    if (ib < num_block_rows - 1) Mpetsc->AddToBlock(ib, ib + 1, D);
    if (ib < num_block_rows - 2) Mpetsc->AddToBlock(ib, ib + 2, E);
  }
  Mpetsc->AssembleIfNecessary();

  return Mpetsc;
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
