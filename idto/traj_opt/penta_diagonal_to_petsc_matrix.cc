#include "idto/traj_opt/penta_diagonal_to_petsc_matrix.h"

#include <vector>

using drake::multibody::fem::internal::PetscSymmetricBlockSparseMatrix;
using Eigen::MatrixXd;

namespace idto {
namespace traj_opt {
namespace internal {

namespace {
/* Operation equivalent to
  A.block(ib * b, jb * b, b, b) = block;
*/
template <typename T>
void TransposedAddToBlock(T* obj, int ib, int jb,
                          const MatrixX<double>& block) {
  /* Notice that `MatSetValuesBlocked()` takes row major data whereas
   Eigen's default format (in `block.data()`) is column major, as in Fortran.
   Therefore we must make a copy with the proper row major order. */
  const MatrixX<double> row_major = block.transpose();
  if (ib == jb) {
    VectorX<int> indices(1);
    indices[0] = ib;
    obj->AddToBlock(indices, row_major);
  } else {
    throw std::logic_error(
        "TODO: potentially nonsymmetric AddToBlock unimplemented");
  }
}
}  // namespace

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

    if (ib >= 2) TransposedAddToBlock(&*Mpetsc, ib, ib - 2, A);
    if (ib >= 1) TransposedAddToBlock(&*Mpetsc, ib, ib - 1, B);
    TransposedAddToBlock(&*Mpetsc, ib, ib, C);
    if (ib < num_block_rows - 1) TransposedAddToBlock(&*Mpetsc, ib, ib + 1, D);
    if (ib < num_block_rows - 2) TransposedAddToBlock(&*Mpetsc, ib, ib + 2, E);
  }
  Mpetsc->AssembleIfNecessary();

  return Mpetsc;
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace idto
