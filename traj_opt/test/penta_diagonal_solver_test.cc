#include "drake/traj_opt/penta_diagonal_solver.h"

#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/petsc_symmetric_block_sparse_matrix.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"
#include "drake/traj_opt/penta_diagonal_to_petsc_matrix.h"

using drake::multibody::fem::internal::PetscSolverStatus;
using drake::multibody::fem::internal::PetscSymmetricBlockSparseMatrix;
using std::chrono::steady_clock;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace traj_opt {
namespace internal {

GTEST_TEST(PentaDiagonalMatrixTest, MultiplyBy) {
  // Generate a random penta-diagonal matrix
  const int block_size = 2;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const MatrixXd A = MatrixXd::Random(size, size);
  const PentaDiagonalMatrix<double> H =
      PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(A, num_blocks,
                                                               block_size);

  // Multiply by an arbitrary vector
  const VectorXd v = VectorXd::LinSpaced(size, 0.1, 1.1);
  VectorXd prod(size);

  H.MultiplyBy(v, &prod);
  const VectorXd prod_expected = H.MakeDense() * v;

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(CompareMatrices(prod, prod_expected, kTolerance,
                              MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, SymmetricMatrixEmpty) {
  const std::vector<MatrixXd> empty_diagonal;
  PentaDiagonalMatrix<double> M(empty_diagonal, empty_diagonal, empty_diagonal);
  EXPECT_EQ(M.rows(), 0);
}

GTEST_TEST(PentaDiagonalMatrixTest, MutateMatrix) {
  const int k = 3;
  PentaDiagonalMatrix<double> M(5, k);
  EXPECT_TRUE(M.is_symmetric());
  EXPECT_EQ(M.block_rows(), 5);
  EXPECT_EQ(M.block_cols(), 5);
  EXPECT_EQ(M.block_size(), 3);
  EXPECT_EQ(M.rows(), 15);
  EXPECT_EQ(M.cols(), 15);

  const MatrixXd B1 = 1.5 * MatrixXd::Ones(k, k);
  const MatrixXd B2 = 2.1 * MatrixXd::Ones(k, k);
  const MatrixXd B3 = -12.8 * MatrixXd::Ones(k, k);
  const MatrixXd B4 = 1.8 * MatrixXd::Ones(k, k);
  const MatrixXd B5 = 15.3 * MatrixXd::Ones(k, k);
  const std::vector<MatrixXd> some_diagonal = {B1, B2, B3, B3, B5};

  // These throw since M is diagonal and it only allows mutating the lower
  // diagonals.
  EXPECT_THROW(M.mutable_D(), std::exception);
  EXPECT_THROW(M.mutable_E(), std::exception);

  // Mutate diagonals.
  EXPECT_NE(M.A(), some_diagonal);
  M.mutable_A() = some_diagonal;
  EXPECT_EQ(M.A(), some_diagonal);

  EXPECT_NE(M.B(), some_diagonal);
  M.mutable_B() = some_diagonal;
  EXPECT_EQ(M.B(), some_diagonal);

  EXPECT_NE(M.C(), some_diagonal);
  M.mutable_C() = some_diagonal;
  EXPECT_EQ(M.C(), some_diagonal);

  // We've changed some terms in the matrix, so we can no longer assume it's
  // symmetric
  EXPECT_FALSE(M.is_symmetric());
}

GTEST_TEST(PentaDiagonalMatrixTest, SymmetricMatrix) {
  const int k = 5;
  const MatrixXd Z = 1.5 * MatrixXd::Zero(k, k);
  const MatrixXd B1 = 1.5 * MatrixXd::Ones(k, k);
  const MatrixXd B2 = 2.1 * MatrixXd::Ones(k, k);
  const MatrixXd B3 = -12.8 * MatrixXd::Ones(k, k);
  const MatrixXd B4 = 1.8 * MatrixXd::Ones(k, k);
  const MatrixXd B5 = 15.3 * MatrixXd::Ones(k, k);
  const MatrixXd B6 = 7.1 * MatrixXd::Ones(k, k);
  PentaDiagonalMatrix<double> M({Z, Z, B1}, {Z, B2, B3}, {B4, B5, B6});
  EXPECT_EQ(M.rows(), k * 3);
  EXPECT_EQ(M.block_rows(), 3);
  // Verify M is symmetric and is properly zero padded.
  EXPECT_EQ(M.D()[0], M.B()[1]);
  EXPECT_EQ(M.D()[1], M.B()[2]);
  EXPECT_EQ(M.D()[2], Z);
  EXPECT_EQ(M.E()[0], M.A()[2]);
  EXPECT_EQ(M.E()[1], Z);
  EXPECT_EQ(M.E()[2], Z);
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveIdentity) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const PentaDiagonalMatrix<double> H =
      PentaDiagonalMatrix<double>::MakeIdentity(num_blocks, block_size);
  PentaDiagonalFactorization Hlu(H);
  EXPECT_EQ(Hlu.status(), PentaDiagonalFactorizationStatus::kSuccess);

  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hlu.SolveInPlace(&x);

  EXPECT_EQ(x, b);
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveBlockDiagonal) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const MatrixXd I = MatrixXd::Identity(block_size, block_size);
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  const MatrixXd random_block = MatrixXd::Random(block_size, block_size);
  const MatrixXd B1 = 2.1 * I + random_block * random_block.transpose();
  const MatrixXd B2 = 3.5 * I + random_block * random_block.transpose();
  const MatrixXd B3 = 0.2 * I + random_block * random_block.transpose();

  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B(num_blocks, Z);
  std::vector<MatrixXd> C{B1, B2, B3, B1, B3};
  const PentaDiagonalMatrix<double> H(std::move(A), std::move(B), std::move(C));
  const MatrixXd Hdense = H.MakeDense();

  PentaDiagonalFactorization Hlu(H);
  EXPECT_EQ(Hlu.status(), PentaDiagonalFactorizationStatus::kSuccess);
  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hlu.SolveInPlace(&x);

  // Reference solution computed with Eigen, dense.
  const VectorXd x_expected = Hdense.ldlt().solve(b);

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveTriDiagonal) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const MatrixXd I = MatrixXd::Identity(block_size, block_size);
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  const MatrixXd random_block = MatrixXd::Random(block_size, block_size);
  const MatrixXd B1 = 2.1 * I + random_block * random_block.transpose();
  const MatrixXd B2 = 3.5 * I + random_block * random_block.transpose();
  const MatrixXd B3 = 0.2 * I + random_block * random_block.transpose();
  const MatrixXd B4 = 1.3 * I + random_block * random_block.transpose();

  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B{Z, B1, B2, B3, B4};
  std::vector<MatrixXd> C{B1, B2, B3, B1, B3};
  const PentaDiagonalMatrix<double> H(std::move(A), std::move(B), std::move(C));
  const MatrixXd Hdense = H.MakeDense();

  PentaDiagonalFactorization Hlu(H);
  EXPECT_EQ(Hlu.status(), PentaDiagonalFactorizationStatus::kSuccess);
  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hlu.SolveInPlace(&x);

  // Reference solution computed with Eigen, dense.
  const VectorXd x_expected = Hdense.ldlt().solve(b);

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, SolvePentaDiagonal) {
  const int block_size = 2;
  const int num_blocks = 21;
  const int size = num_blocks * block_size;

  // Generate an SPD matrix.
  const MatrixXd A = MatrixXd::Random(size, size);
  MatrixXd P = MatrixXd::Identity(size, size) + A * A.transpose();

  // Generate a penta-diagonal SPD matrix. Ignore off-diagonal elements of
  // P outside the 5-diagonal band.
  const PentaDiagonalMatrix<double> H =
      PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(P, num_blocks,
                                                               block_size);
  const MatrixXd Hdense = H.MakeDense();
  std::cout << "P error: " << (P - Hdense).norm() / P.norm() << std::endl;

  std::cout << "condition number: " << 1 / Hdense.llt().rcond() << std::endl;

  // Compute a ground truth value
  const VectorXd x_gt = VectorXd::LinSpaced(size, -3, 12.4);
  const VectorXd b = Hdense * x_gt;

  // Sanity check multiplication on the penta diagonal Hessian.
  VectorXd b2(b.size());
  H.MultiplyBy(x_gt, &b2);
  std::cout << "b error: " << (b - b2).norm() / b.norm() << std::endl;

  // Solution with dense algebra
  steady_clock::time_point start = steady_clock::now();
  const auto ldlt = Hdense.ldlt();
  const VectorXd x_ldlt = ldlt.solve(b);
  steady_clock::time_point end = steady_clock::now();
  std::cout << fmt::format("D(ldlt): {}", fmt_eigen(ldlt.vectorD().transpose()))
            << std::endl;
  std::cout << fmt::format(
                   "P(ldlt):\n{}",
                   fmt_eigen(ldlt.transpositionsP().indices().transpose()))
            << std::endl;
  double wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("LDLT.\n  Wall clock: {:.4g} seconds. error: {}\n",
             wall_clock_time, (x_ldlt - x_gt).norm() / x_gt.norm());

  start = steady_clock::now();
  const VectorXd x_llt = Hdense.llt().solve(b);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("LLT.\n  Wall clock: {:.4g} seconds. error: {}\n", wall_clock_time,
             (x_llt - x_gt).norm() / x_gt.norm());

  start = steady_clock::now();
  const VectorXd x_lu = Hdense.partialPivLu().solve(b);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("LU (partial piv.)\n  Wall clock: {:.4g} seconds. error: {}\n",
             wall_clock_time, (x_llt - x_gt).norm() / x_gt.norm());

  VectorXd x_sparse = b;
  // Solution with our pentadiagonal solver.
  start = steady_clock::now();
  PentaDiagonalFactorization Hlu(H);
  EXPECT_EQ(Hlu.status(), PentaDiagonalFactorizationStatus::kSuccess);
  Hlu.SolveInPlace(&x_sparse);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print(
      "PentaDiagonalFactorization.\n  Wall clock: {:.4g} seconds. error: {}\n",
      wall_clock_time, (x_sparse - x_gt).norm() / x_gt.norm());

  // Solution with PetsC solver.
  start = steady_clock::now();
  auto Hpetsc = PentaDiagonalToPetscMatrix(H);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("PentaDiagonalToPetscMatrix().\n  Wall clock: {:.4g} seconds.\n",
             wall_clock_time);
  Hpetsc->set_relative_tolerance(1.0e-16);

  // Sanity check matrices are equivalent.
  const MatrixXd dense_from_pentadiagonal = H.MakeDense();
  const MatrixXd dense_from_petsc = Hpetsc->MakeDenseMatrix();
  const double kTolerance = std::numeric_limits<double>::epsilon() * H.rows();
  EXPECT_TRUE(CompareMatrices(dense_from_petsc, dense_from_pentadiagonal,
                              kTolerance, MatrixCompareType::relative));

  // Solve with Petsc's direct solver.
  VectorXd x_petsc_chol = b;
  start = steady_clock::now();
  PetscSolverStatus status = Hpetsc->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kDirect,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kCholesky, b,
      &x_petsc_chol);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("Petsc Chol.\n  Wall clock: {:.4g} seconds. error: {}\n",
             wall_clock_time, (x_petsc_chol - x_gt).norm() / x_gt.norm());
  EXPECT_EQ(status, PetscSolverStatus::kSuccess);

  VectorXd x_petsc_minres = b;
  start = steady_clock::now();
  status = Hpetsc->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kIncompleteCholesky,
      b, &x_petsc_minres);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("Petsc MinRes.\n  Wall clock: {:.4g} seconds. error: {}\n",
             wall_clock_time, (x_petsc_minres - x_gt).norm() / x_gt.norm());
  EXPECT_EQ(status, PetscSolverStatus::kSuccess);

  VectorXd x_petsc_cg = b;
  start = steady_clock::now();
  status = Hpetsc->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kConjugateGradient,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kIncompleteCholesky,
      b, &x_petsc_cg);
  end = steady_clock::now();
  wall_clock_time = std::chrono::duration<double>(end - start).count();
  fmt::print("Petsc CG.\n  Wall clock: {:.4g} seconds. error: {}\n",
             wall_clock_time, (x_petsc_cg - x_gt).norm() / x_gt.norm());
  EXPECT_EQ(status, PetscSolverStatus::kSuccess);
}

// Solve H*x = b, where H has a high condition number
GTEST_TEST(PentaDiagonalMatrixTest, ConditionNumber) {
  const int block_size = 5;
  const int num_blocks = 30;
  const int size = num_blocks * block_size;

  std::cout << "Condition number, dense error, sparse error" << std::endl;
  for (double scale_factor = 1e1; scale_factor < 1e20; scale_factor *= 10) {
    // Generate a matrix H
    const MatrixXd A = 1e4 * MatrixXd::Random(size, size);
    const MatrixXd P = MatrixXd::Identity(size, size) + A.transpose() * A;
    PentaDiagonalMatrix<double> H =
        PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(P, num_blocks,
                                                                 block_size);
    MatrixXd Hdense = H.MakeDense();

    // Modify H so it has the desired condition number
    Eigen::JacobiSVD<MatrixXd> svd(Hdense,
                                   Eigen::ComputeThinU | Eigen::ComputeThinV);
    const MatrixXd U = svd.matrixU();
    const MatrixXd V = svd.matrixV();
    VectorXd S = svd.singularValues();
    const double S_0 = S(0);
    const double S_end = S(size - 1);
    S = S_0 * (VectorXd::Ones(size) - ((scale_factor - 1) / scale_factor) *
                                          (S_0 * VectorXd::Ones(size) - S) /
                                          (S_0 - S_end));

    const MatrixXd H_reconstructed = U * S.asDiagonal() * V.transpose();
    H = PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(
        H_reconstructed, num_blocks, block_size);
    Hdense = H.MakeDense();

    // Define a ground truth solution x
    const VectorXd x_gt = VectorXd::Random(size);

    // Define the vector b
    const VectorXd b = Hdense * x_gt;

    // Compute x using the Thomas algorithm (sparse)
    PentaDiagonalFactorization Hlu(H);
    EXPECT_EQ(Hlu.status(), PentaDiagonalFactorizationStatus::kSuccess);
    VectorXd x_sparse = b;
    Hlu.SolveInPlace(&x_sparse);

    // Compute x using LDLT (dense)
    const VectorXd x_dense = Hdense.ldlt().solve(b);

    // Compare with ground truth
    const double cond = 1 / Hdense.ldlt().rcond();
    const double dense_error = (x_gt - x_dense).norm();
    const double sparse_error = (x_gt - x_sparse).norm();
    std::cout << fmt::format("{}, {}, {}\n", cond, dense_error, sparse_error);

    const auto ldlt = Hdense.ldlt();
    std::cout << "Dmin(ldlt): " << ldlt.vectorD().transpose().minCoeff()
              << std::endl;
    std::cout << "Dmax(ldlt): " << ldlt.vectorD().transpose().maxCoeff()
              << std::endl;
  }
}

GTEST_TEST(PentaDiagonalMatrixTest, ExtractDiagonal) {
  const int block_size = 5;
  const int num_blocks = 30;
  const int size = num_blocks * block_size;

  // Generate a random matrix H
  const MatrixXd A = MatrixXd::Random(size, size);
  const MatrixXd P = MatrixXd::Identity(size, size) + A.transpose() * A;
  PentaDiagonalMatrix<double> H =
      PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(P, num_blocks,
                                                               block_size);
  MatrixXd Hdense = H.MakeDense();

  // Compute diagonal with dense and sparse operations
  const VectorXd dense_diagonal = Hdense.diagonal();
  VectorXd sparse_diagonal(size);
  H.ExtractDiagonal(&sparse_diagonal);

  const double kTolerance = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(dense_diagonal, sparse_diagonal, kTolerance,
                              MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, ScaleByDiagonal) {
  const int block_size = 5;
  const int num_blocks = 3;
  const int size = num_blocks * block_size;

  // Generate a random matrix H
  const MatrixXd A = MatrixXd::Random(size, size);
  const MatrixXd P = MatrixXd::Identity(size, size) + A.transpose() * A;
  PentaDiagonalMatrix<double> H =
      PentaDiagonalMatrix<double>::MakeSymmetricFromLowerDense(P, num_blocks,
                                                               block_size);
  const MatrixXd Hdense = H.MakeDense();

  // Generate a random scaling factor
  const VectorXd scale_factor = VectorXd::Random(size);

  // Compare dense and sparse versions
  const MatrixXd H_scaled_dense =
      scale_factor.asDiagonal() * Hdense * scale_factor.asDiagonal();
  H.ScaleByDiagonal(scale_factor);
  const MatrixXd H_scaled_sparse = H.MakeDense();

  const double kTolerance = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(H_scaled_dense, H_scaled_sparse, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
