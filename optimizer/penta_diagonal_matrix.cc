#include "optimizer/penta_diagonal_matrix.h"

#include <utility>

#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>

namespace idto {
namespace optimizer {
namespace internal {

template <typename T>
PentaDiagonalMatrix<T>::PentaDiagonalMatrix(int num_blocks, int block_size,
                                            bool is_symmetric)
    : is_symmetric_(is_symmetric) {
  const MatrixX<T> Z = MatrixX<T>::Zero(block_size, block_size);
  A_.resize(num_blocks, Z);
  B_.resize(num_blocks, Z);
  C_.resize(num_blocks, Z);
  D_.resize(num_blocks, Z);
  E_.resize(num_blocks, Z);
}

template <typename T>
PentaDiagonalMatrix<T>::PentaDiagonalMatrix(std::vector<MatrixX<T>> A,
                                            std::vector<MatrixX<T>> B,
                                            std::vector<MatrixX<T>> C,
                                            std::vector<MatrixX<T>> D,
                                            std::vector<MatrixX<T>> E)
    : A_(std::move(A)),
      B_(std::move(B)),
      C_(std::move(C)),
      D_(std::move(D)),
      E_(std::move(E)) {
  // Minimum sanity check.
  DRAKE_DEMAND(A_.size() == B_.size());
  DRAKE_DEMAND(A_.size() == C_.size());
  DRAKE_DEMAND(A_.size() == D_.size());
  DRAKE_DEMAND(A_.size() == E_.size());
  // Thorough sanity check in debug builds only.
  DRAKE_ASSERT(VerifySizes());
}

template <typename T>
PentaDiagonalMatrix<T>::PentaDiagonalMatrix(std::vector<MatrixX<T>> A,
                                            std::vector<MatrixX<T>> B,
                                            std::vector<MatrixX<T>> C)
    : A_(std::move(A)), B_(std::move(B)), C_(std::move(C)) {
  // Minimum sanity check.
  DRAKE_DEMAND(A_.size() == B_.size());
  DRAKE_DEMAND(A_.size() == C_.size());

  // Alocate space for D and E.
  // TODO(amcastro-tri): Consider not allocating space for D/E in the symmetric
  // case.
  D_ = B_;
  E_ = A_;

  // Copy the lower triangular part to the upper triangular part.
  MakeSymmetric();
}

template <typename T>
void PentaDiagonalMatrix<T>::MakeSymmetric() {
  // Minimum sanity check.
  DRAKE_DEMAND(B_.size() == A_.size());
  DRAKE_DEMAND(C_.size() == A_.size());
  DRAKE_DEMAND(D_.size() == A_.size());
  DRAKE_DEMAND(E_.size() == A_.size());

  // We overwrite the (strictly) upper triangular part of C with its lower
  // triangular part.
  const int size = A_.size();
  for (int i = 0; i < size; ++i) {
    C_[i].template triangularView<Eigen::StrictlyUpper>() = C_[i].transpose();
  }

  // D = B.
  // N.B. The first entry in B is zero and we skip its copy.
  // The last entry in D_ is defined to be zero.
  if (size >= 2) {
    for (int i = 0; i < size - 1; ++i) {
      D_[i] = B_[i + 1].transpose();
    }
    D_[size - 1].setZero();
  }

  // E = A.
  // N.B. The first two entries in A are zero and we skip their copy.
  // The last two entries in E are defined to be zero.
  if (size >= 3) {
    for (int i = 0; i < size - 2; ++i) {
      E_[i] = A_[i + 2].transpose();
    }
    E_[size - 1].setZero();
    E_[size - 2].setZero();
  }

  is_symmetric_ = true;

  // Thorough sanity check in debug builds only.
  // N.B. We place it here at the bottom since we need all five bands to be
  // properly initialized.
  DRAKE_ASSERT(VerifySizes());
}

template <typename T>
PentaDiagonalMatrix<T> PentaDiagonalMatrix<T>::MakeIdentity(int num_blocks,
                                                            int block_size) {
  const MatrixX<T> Z = MatrixX<T>::Zero(block_size, block_size);
  const MatrixX<T> Id = MatrixX<T>::Identity(block_size, block_size);
  std::vector<MatrixX<T>> A(num_blocks, Z);
  std::vector<MatrixX<T>> B(num_blocks, Z);
  std::vector<MatrixX<T>> C(num_blocks, Id);
  return PentaDiagonalMatrix(std::move(A), std::move(B), std::move(C));
}

template <typename T>
bool PentaDiagonalMatrix<T>::VerifySizes() const {
  const int k = block_size();
  if (!VerifyAllBlocksOfSameSize(A_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(B_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(C_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(D_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(E_, k)) return false;
  return true;
}

template <typename T>
PentaDiagonalMatrix<T> PentaDiagonalMatrix<T>::MakeSymmetricFromLowerDense(
    const MatrixX<T>& M, int num_blocks, int block_size) {
  const MatrixX<T> Z = MatrixX<T>::Zero(block_size, block_size);
  std::vector<MatrixX<T>> A(num_blocks, Z);
  std::vector<MatrixX<T>> B(num_blocks, Z);
  std::vector<MatrixX<T>> C(num_blocks, Z);
  for (int i = 0; i < num_blocks; ++i) {
    if (i >= 2)
      A[i] =
          M.block(i * block_size, (i - 2) * block_size, block_size, block_size);
    if (i >= 1)
      B[i] =
          M.block(i * block_size, (i - 1) * block_size, block_size, block_size);
    C[i] = M.block(i * block_size, i * block_size, block_size, block_size);
  }
  return PentaDiagonalMatrix(std::move(A), std::move(B), std::move(C));
}

template <typename T>
MatrixX<T> PentaDiagonalMatrix<T>::MakeDense() const {
  const int num_blocks = block_cols();
  MatrixX<T> M = MatrixX<T>::Zero(rows(), cols());
  for (int i = 0; i < num_blocks; ++i) {
    if (i >= 2)
      M.block(i * block_size(), (i - 2) * block_size(), block_size(),
              block_size()) = A_[i];
    if (i >= 1)
      M.block(i * block_size(), (i - 1) * block_size(), block_size(),
              block_size()) = B_[i];
    M.block(i * block_size(), i * block_size(), block_size(), block_size()) =
        C_[i];
    if (i < num_blocks - 1)
      M.block(i * block_size(), (i + 1) * block_size(), block_size(),
              block_size()) = D_[i];
    if (i < num_blocks - 2)
      M.block(i * block_size(), (i + 2) * block_size(), block_size(),
              block_size()) = E_[i];
  }
  return M;
}

template <typename T>
bool PentaDiagonalMatrix<T>::VerifyAllBlocksOfSameSize(
    const std::vector<MatrixX<T>>& X, int size) {
  for (const MatrixX<T>& Xblock : X) {
    if (Xblock.rows() != size || Xblock.cols() != size) return false;
  }
  return true;
}

template <typename T>
void PentaDiagonalMatrix<T>::MultiplyBy(const VectorX<T>& v,
                                        VectorX<T>* result) const {
  DRAKE_DEMAND(v.size() == rows());
  DRAKE_DEMAND(result->size() == cols());
  const int bs = block_size();

  for (int i = 0; i < block_rows(); ++i) {
    auto result_block = result->segment(i * bs, bs);

    // Diagonal blocks are always present
    result_block = C_[i] * v.segment(i * bs, bs);

    // Off-diagonal blocks existence depends on which row we're looking at
    if (i >= 1) {
      result_block += B_[i] * v.segment((i - 1) * bs, bs);
    }
    if (i >= 2) {
      result_block += A_[i] * v.segment((i - 2) * bs, bs);
    }
    if (i < block_rows() - 1) {
      result_block += D_[i] * v.segment((i + 1) * bs, bs);
    }
    if (i < block_rows() - 2) {
      result_block += E_[i] * v.segment((i + 2) * bs, bs);
    }
  }
}

template <typename T>
void PentaDiagonalMatrix<T>::ExtractDiagonal(VectorX<T>* diagonal) const {
  DRAKE_DEMAND(is_symmetric());
  DRAKE_DEMAND(diagonal->size() == rows());

  const int bs = block_size();
  for (int i = 0; i < block_rows(); ++i) {
    diagonal->segment(i * bs, bs) = C_[i].diagonal();
  }
}

template <typename T>
void PentaDiagonalMatrix<T>::ScaleByDiagonal(const VectorX<T>& scale_factor) {
  DRAKE_DEMAND(is_symmetric());
  DRAKE_DEMAND(scale_factor.size() == rows());

  const int bs = block_size();
  const int size = A_.size();

  for (int i = 0; i < block_rows(); ++i) {
    // Diagonal blocks are always present
    C_[i] = scale_factor.segment(i * bs, bs).asDiagonal() * C_[i] *
            scale_factor.segment(i * bs, bs).asDiagonal();

    // Off-diagonal blocks may not exist for first and last rows
    if (i >= 1) {
      B_[i] = scale_factor.segment(i * bs, bs).asDiagonal() * B_[i] *
              scale_factor.segment((i - 1) * bs, bs).asDiagonal();
    }
    if (i >= 2) {
      A_[i] = scale_factor.segment(i * bs, bs).asDiagonal() * A_[i] *
              scale_factor.segment((i - 2) * bs, bs).asDiagonal();
    }
  }

  // D = B'
  if (size >= 2) {
    for (int i = 0; i < size - 1; ++i) {
      D_[i] = B_[i + 1].transpose();
    }
  }

  // E = A'
  if (size >= 3) {
    for (int i = 0; i < size - 2; ++i) {
      E_[i] = A_[i + 2].transpose();
    }
  }
}

}  // namespace internal
}  // namespace optimizer
}  // namespace idto

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::idto::optimizer::internal::PentaDiagonalMatrix);
