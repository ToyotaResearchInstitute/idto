#include "drake/traj_opt/penta_diagonal_to_petsc_matrix.h"

#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace traj_opt {
namespace internal {

GTEST_TEST(PentaDiagonalToPetscMatrixTest, ReconstructDense) {
  // Generate a random penta-diagonal matrix
  const int block_size = 3;
  const int num_blocks = 7;
  const int size = num_blocks * block_size;
  const MatrixXd Asrc = MatrixXd::Random(size, size);
  const PentaDiagonalMatrix<double> A =
      PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(Asrc, num_blocks,
                                                               block_size);

  auto Apetsc = PentaDiagonalToPetscMatrix(A);

  EXPECT_EQ(Apetsc->rows(), size);
  EXPECT_EQ(Apetsc->cols(), size);

  const MatrixXd dense_from_pentadiagonal = A.MakeDense();
  const MatrixXd dense_from_petsc = Apetsc->MakeDenseMatrix();

  std::cout << fmt::format("{}", fmt_eigen(dense_from_pentadiagonal))
            << std::endl;
  std::cout << fmt::format("{}", fmt_eigen(dense_from_petsc)) << std::endl;

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(CompareMatrices(dense_from_petsc, dense_from_pentadiagonal,
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
