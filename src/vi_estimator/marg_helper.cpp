/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/utils/assert.h>
#include <basalt/vi_estimator/marg_helper.h>

namespace basalt {

template <class Scalar_>
void MargHelper<Scalar_>::marginalizeHelperSqToSq(
    MatX& abs_H, VecX& abs_b, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg, MatX& marg_H, VecX& marg_b) {
  int keep_size = idx_to_keep.size();
  int marg_size = idx_to_marg.size();

  BASALT_ASSERT(keep_size + marg_size == abs_H.cols());

  // Fill permutation matrix
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(idx_to_keep.size() +
                                                idx_to_marg.size());

  {
    auto it = idx_to_keep.begin();
    for (size_t i = 0; i < idx_to_keep.size(); i++) {
      indices[i] = *it;
      it++;
    }
  }

  {
    auto it = idx_to_marg.begin();
    for (size_t i = 0; i < idx_to_marg.size(); i++) {
      indices[idx_to_keep.size() + i] = *it;
      it++;
    }
  }

  const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> p(
      indices);

  const Eigen::PermutationMatrix<Eigen::Dynamic> pt = p.transpose();

  abs_b.applyOnTheLeft(pt);
  abs_H.applyOnTheLeft(pt);
  abs_H.applyOnTheRight(p);

  // Use of fullPivLu compared to alternatives was determined experimentally. It
  // is more stable than ldlt when the matrix is numerically close ot
  // indefinite, but it is faster than the even more stable
  // fullPivHouseholderQr.

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).ldlt();

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).fullPivLu();
  //  MatX H_mm_inv = H_mm_decomposition.inverse();

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).colPivHouseholderQr();

  // DO NOT USE!!!
  // Pseudoinverse with SVD. Uses iterative method and results in severe low
  // accuracy. Use the version below (COD) for pseudoinverse
  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size)
  //          .jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Eigen version of pseudoinverse
  auto H_mm_decomposition = abs_H.bottomRightCorner(marg_size, marg_size)
                                .completeOrthogonalDecomposition();
  MatX H_mm_inv = H_mm_decomposition.pseudoInverse();

  // Should be more numerially stable version of:
  abs_H.topRightCorner(keep_size, marg_size) *= H_mm_inv;
  //  abs_H.topRightCorner(keep_size, marg_size) =
  //      H_mm_decomposition
  //          .solve(abs_H.topRightCorner(keep_size, marg_size).transpose())
  //          .transpose();

  marg_H = abs_H.topLeftCorner(keep_size, keep_size);
  marg_b = abs_b.head(keep_size);

  marg_H -= abs_H.topRightCorner(keep_size, marg_size) *
            abs_H.bottomLeftCorner(marg_size, keep_size);
  marg_b -= abs_H.topRightCorner(keep_size, marg_size) * abs_b.tail(marg_size);

  abs_H.resize(0, 0);
  abs_b.resize(0);
}

template <class Scalar_>
void MargHelper<Scalar_>::marginalizeHelperSqToSqrt(
    MatX& abs_H, VecX& abs_b, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg, MatX& marg_sqrt_H, VecX& marg_sqrt_b) {
  // TODO: We might lose the strong initial pose prior if there are no obs
  // during marginalization (e.g. during first marginalization). Unclear when
  // this can happen. Keeping keyframes w/o any points in the optimization
  // window might not make sense. --> Double check if and when this currently
  // occurs. If no, add asserts. If yes, somehow deal with it differently.

  int keep_size = idx_to_keep.size();
  int marg_size = idx_to_marg.size();

  BASALT_ASSERT(keep_size + marg_size == abs_H.cols());

  // Fill permutation matrix
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(idx_to_keep.size() +
                                                idx_to_marg.size());

  {
    // 将H矩阵的变量重新排序，将保留的变量放在H矩阵的前面
    auto it = idx_to_keep.begin();
    for (size_t i = 0; i < idx_to_keep.size(); i++) {
      indices[i] = *it;
      it++;
    }
  }

  {
    // 遍历边缘化的变量，将要边缘化的变量放在H矩阵的后面
    auto it = idx_to_marg.begin();
    for (size_t i = 0; i < idx_to_marg.size(); i++) {
      indices[idx_to_keep.size() + i] = *it;
      it++;
    }
  }

  // notice: PermutationWrapper是列主导，不是行主导
  const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> p(
      indices);

  const Eigen::PermutationMatrix<Eigen::Dynamic> pt = p.transpose();

  abs_b.applyOnTheLeft(pt);
  abs_H.applyOnTheLeft(pt);
  abs_H.applyOnTheRight(p);

  // Use of fullPivLu compared to alternatives was determined experimentally. It
  // is more stable than ldlt when the matrix is numerically close ot
  // indefinite, but it is faster than the even more stable
  // fullPivHouseholderQr.

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).ldlt();

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).fullPivLu();
  //  MatX H_mm_inv = H_mm_decomposition.inverse();

  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size).colPivHouseholderQr();

  // DO NOT USE!!!
  // Pseudoinverse with SVD. Uses iterative method and results in severe low
  // accuracy. Use the version below (COD) for pseudoinverse
  //  auto H_mm_decomposition =
  //      abs_H.bottomRightCorner(marg_size, marg_size)
  //          .jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Eigen version of pseudoinverse
  auto H_mm_decomposition = abs_H.bottomRightCorner(marg_size, marg_size)
                                .completeOrthogonalDecomposition();
  MatX H_mm_inv = H_mm_decomposition.pseudoInverse();

  // Should be more numerially stable version of:
  abs_H.topRightCorner(keep_size, marg_size) *= H_mm_inv;
  //  abs_H.topRightCorner(keep_size, marg_size).noalias() =
  //      H_mm_decomposition.solve(abs_H.bottomLeftCorner(marg_size, keep_size))
  //          .transpose();

  MatX marg_H;
  VecX marg_b;

  marg_H = abs_H.topLeftCorner(keep_size, keep_size);
  marg_b = abs_b.head(keep_size);

  marg_H -= abs_H.topRightCorner(keep_size, marg_size) *
            abs_H.bottomLeftCorner(marg_size, keep_size);
  marg_b -= abs_H.topRightCorner(keep_size, marg_size) * abs_b.tail(marg_size);

  Eigen::LDLT<Eigen::Ref<MatX>> ldlt(marg_H);

  VecX D_sqrt = ldlt.vectorD().array().max(0).sqrt().matrix();

  // After LDLT, we have
  //     marg_H == P^T*L*D*L^T*P,
  // so we compute the square root as
  //     marg_sqrt_H = sqrt(D)*L^T*P,
  // such that
  //     marg_sqrt_H^T marg_sqrt_H == marg_H.
  marg_sqrt_H.setIdentity(keep_size, keep_size);
  marg_sqrt_H = ldlt.transpositionsP() * marg_sqrt_H;
  marg_sqrt_H = ldlt.matrixU() * marg_sqrt_H;  // U == L^T
  marg_sqrt_H = D_sqrt.asDiagonal() * marg_sqrt_H;

  // For the right hand side, we want
  //     marg_b == marg_sqrt_H^T * marg_sqrt_b
  // so we compute
  //     marg_sqrt_b = (marg_sqrt_H^T)^-1 * marg_b
  //                 = (P^T*L*sqrt(D))^-1 * marg_b
  //                 = sqrt(D)^-1 * L^-1 * P * marg_b
  marg_sqrt_b = ldlt.transpositionsP() * marg_b;
  ldlt.matrixL().solveInPlace(marg_sqrt_b);

  // We already clamped negative values in D_sqrt to 0 above, but for values
  // close to 0 we set b to 0.
  for (int i = 0; i < marg_sqrt_b.size(); ++i) {
    if (D_sqrt(i) > std::sqrt(std::numeric_limits<Scalar>::min()))
      marg_sqrt_b(i) /= D_sqrt(i);
    else
      marg_sqrt_b(i) = 0;
  }

  //  std::cout << "marg_sqrt_H diff: "
  //            << (marg_sqrt_H.transpose() * marg_sqrt_H - marg_H).norm() << "
  //            "
  //            << marg_H.norm() << std::endl;

  //  std::cout << "marg_sqrt_b diff: "
  //            << (marg_sqrt_H.transpose() * marg_sqrt_b - marg_b).norm()
  //            << std::endl;

  abs_H.resize(0, 0);
  abs_b.resize(0);
}

template <class Scalar_>
void MargHelper<Scalar_>::marginalizeHelperSqrtToSqrt(
    MatX& Q2Jp, VecX& Q2r, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg, MatX& marg_sqrt_H, VecX& marg_sqrt_b) {
  Eigen::Index keep_size = idx_to_keep.size();
  Eigen::Index marg_size = idx_to_marg.size();

  BASALT_ASSERT(keep_size + marg_size == Q2Jp.cols()); // 断言pose总维数等于Q_2^T * J_p的列数
  BASALT_ASSERT(Q2Jp.rows() == Q2r.rows());

  // 填充置换矩阵（它在每行和每列中只有一个1，而在其他地方则为0。每一行恰有一个 1，每一列恰有一个 1。）
  // 这里从填充内容看，并不是真正数学意义上的置换矩阵
  // Fill permutation matrix
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(idx_to_keep.size() +
                                                idx_to_marg.size()); // 定义了一个动态行数，而只有1列的矩阵indices.

  {
    auto it = idx_to_marg.begin(); // 前面x行填充的是Hessian中被marg的序号
    for (size_t i = 0; i < idx_to_marg.size(); i++) {
      indices[i] = *it;
      it++;
    }
  }

  {
    auto it = idx_to_keep.begin(); // 后面y行填充的是Hessian中保留的序号
    for (size_t i = 0; i < idx_to_keep.size(); i++) {
      indices[idx_to_marg.size() + i] = *it;
      it++;
    }
  }

  // 注意notice: Eigen::PermutationWrapper是列主导，不是行主导，该操作使 p 成为真正的置换矩阵
  // TODO: check if using TranspositionMatrix is faster
  const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> p(
      indices);

  // 先看公式：A.applyOnTheRight(B);  // equivalent to A *= B
  // A.applyOnTheLeft(B); // equivalent to A = B * A
  // 那么这里表示Q2Jp = Q2Jp * p
  // 要被边缘化的帧状态被排序到最左边的列中 (摘自：frame states to be marginalized are sorted into the leftmost columns)
  Q2Jp.applyOnTheRight(p); // 这里右乘置换矩阵，表示将Q2Jp相应的列进行交换，使得被marg部分位于前面的列，保留部分位于后面的列
                           // 左乘置换矩阵，交换矩阵的行，右乘置换矩阵，交换矩阵的列
                           // 当一个矩阵乘上一个置换矩阵时，所得到的是原来矩阵的横行（置换矩阵在左）或纵列（置换矩阵在右）经过置换后得到的矩阵。

  Eigen::Index marg_rank = 0;
  Eigen::Index total_rank = 0;

  // 然后连续的Householder变换得到上三角矩阵（Successive in-place Householder transformations result in upper-triangular matrix）
  {
    const Scalar rank_threshold =
        std::sqrt(std::numeric_limits<Scalar>::epsilon()); // epsilon=2.22045e-16一个很小的小量

    const Eigen::Index rows = Q2Jp.rows();
    const Eigen::Index cols = Q2Jp.cols();

    VecX tempVector;
    tempVector.resize(cols + 1);
    Scalar* tempData = tempVector.data();

    for (Eigen::Index k = 0; k < cols && total_rank < rows; ++k) {
      Eigen::Index remainingRows = rows - total_rank;
      Eigen::Index remainingCols = cols - k - 1;

      Scalar beta;
      Scalar hCoeff;
      Q2Jp.col(k).tail(remainingRows).makeHouseholderInPlace(hCoeff, beta);

      if (std::abs(beta) > rank_threshold) {
        Q2Jp.coeffRef(total_rank, k) = beta;

        Q2Jp.bottomRightCorner(remainingRows, remainingCols)
            .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
                                       hCoeff, tempData + k + 1);
        Q2r.tail(remainingRows)
            .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
                                       hCoeff, tempData + cols);
        total_rank++;
      } else {
        Q2Jp.coeffRef(total_rank, k) = 0;
      }

      // Overwrite householder vectors with 0
      Q2Jp.col(k).tail(remainingRows - 1).setZero();

      // Save the rank of marginalize-out part
      if (k == marg_size - 1) {
        marg_rank = total_rank;
      }
    }
  }

  Eigen::Index keep_valid_rows =
      std::max(total_rank - marg_rank, Eigen::Index(1));

  // 边缘化状态的列和对应的顶部行，以及底部的零行将被删除 （Columns for marginalized states and corresponding top-rows, and zero rows at the bottom are dropped）
  marg_sqrt_H = Q2Jp.block(marg_rank, marg_size, keep_valid_rows, keep_size);
  marg_sqrt_b = Q2r.segment(marg_rank, keep_valid_rows);

  Q2Jp.resize(0, 0);
  Q2r.resize(0);
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

// Note: double specialization is unconditional, b/c NfrMapper depends on it.
//#ifdef BASALT_INSTANTIATIONS_DOUBLE
template class MargHelper<double>;
//#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class MargHelper<float>;
#endif

}  // namespace basalt
