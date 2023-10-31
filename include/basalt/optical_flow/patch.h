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
#pragma once

#include <Eigen/Dense>
#include <sophus/se2.hpp>

#include <basalt/image/image.h>
#include <basalt/optical_flow/patterns.h>

namespace basalt {

template <typename Scalar, typename Pattern>
struct OpticalFlowPatch {
  static constexpr int PATTERN_SIZE = Pattern::PATTERN_SIZE; // 对于Pattern51来说，PATTERN_SIZE应该是52

  typedef Eigen::Matrix<int, 2, 1> Vector2i;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 1, 2> Vector2T;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 1> VectorP;

  typedef Eigen::Matrix<Scalar, 2, PATTERN_SIZE> Matrix2P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 2> MatrixP2;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 3> MatrixP3;
  typedef Eigen::Matrix<Scalar, 3, PATTERN_SIZE> Matrix3P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 4> MatrixP4;
  typedef Eigen::Matrix<int, 2, PATTERN_SIZE> Matrix2Pi;

  static const Matrix2P pattern2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OpticalFlowPatch() = default;

  // @img: 金字塔指定层的图像
  // @pos: 对应层的坐标
  OpticalFlowPatch(const Image<const uint16_t> &img, const Vector2 &pos) {
    setFromImage(img, pos);
  }

  template <typename ImgT>
  static void setData(const ImgT &img, const Vector2 &pos, Scalar &mean,
                      VectorP &data, const Sophus::SE2<Scalar> *se2 = nullptr) {
    int num_valid_points = 0;
    Scalar sum = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      Vector2 p;
      if (se2) {
        p = pos + (*se2) * pattern2.col(i);
      } else {
        p = pos + pattern2.col(i);
      };

      if (img.InBounds(p, 2)) {
        Scalar val = img.template interp<Scalar>(p);
        data[i] = val;
        sum += val;
        num_valid_points++;
      } else {
        data[i] = -1;
      }
    }

    mean = sum / num_valid_points;
    data /= mean;
  }

  // 雅可比计算：(在对应pattern对应原图位置中的雅克比)
  // 这部分在slam14光流小节中也实现了，只不过slam14是算一下正向的再算一下反向的光流，而basalt只用反向光流。
  // 目的：减小计算量反向光流是在上一帧上计算，因此只需要计算一遍
  // 理论部分: 对应单个像素的光流 ∂r/∂se2 = ∂I/∂pix * ∂pix/∂se2
  // ∂I/∂pix 表示图像梯度，因为图像是个离散的表达，因此这部分其实是采用f'(x) = f(x+Δx)−f(x) / Δx   f'(x) = \frac{f(x+\Delta x) - f(x)}{\Delta x}
  // 进行计算的，简单说，就是相邻像素差就是图像梯度了，但是为了保证精度，basalt做了线性插值。

  template <typename ImgT>
  static void setDataJacSe2(const ImgT &img, const Vector2 &pos, Scalar &mean,
                            VectorP &data, MatrixP3 &J_se2) {
    //- 雅可比是残差对状态量的偏导，这里的残差构建和几何雅可比，似乎都用到了扰动模型
    //- 正向光流法，求雅可比的时候，用的是第二个图像I2处的梯度
    // 本算法用的是反向光流法：即用第一个图像I1的梯度来代替
    // r = I2 - I1
    // J = ∂r/∂se2 = - ∂I/∂xi * ∂xi/∂se2

    int num_valid_points = 0;
    Scalar sum = 0;
    Vector3 grad_sum_se2(0, 0, 0);

    Eigen::Matrix<Scalar, 2, 3> Jw_se2; // 2 * 3的矩阵, 这个属于几何雅可比
    Jw_se2.template topLeftCorner<2, 2>().setIdentity(); // 左上角2 * 2设为单位阵，即前面两列由单位阵占据

    // 对于每个pattern内部的点进行计算
    for (int i = 0; i < PATTERN_SIZE; i++) { // PATTERN_SIZE=52的时候，表示patch里面有52个点，pattern2里面是坐标的偏移量
      Vector2 p = pos + pattern2.col(i); // 位于图像的位置，点的位置加上pattern里面的偏移量，得到在patch里面的新的位姿

      // Fill jacobians with respect to SE2 warp 对Jw_se2的第2列（列下标从0开始的,也即最后一列）赋值 //- 下面两行完全是为了构建几何雅可比。
      Jw_se2(0, 2) = -pattern2(1, i); // 取pattern2的第1行，第i列。 对于Pattern51来说，pattern2表示的是2*52的矩阵
      Jw_se2(1, 2) = pattern2(0, i); // 取pattern2的第0行，第i列

      if (img.InBounds(p, 2)) { // 判断加了偏移量的点p是否在图像内，border=2
        // valGrad[0]表示图像强度，valGrad[1]表示x方向梯度，valGrad[0]表示y方向梯度
        Vector3 valGrad = img.template interpGrad<Scalar>(p); // interp是interpolation的缩写，表示利用双线性插值计算图像灰度和图像梯度 ( x方向梯度, y方向梯度 )
        data[i] = valGrad[0]; // 赋值图像灰度值
        sum += valGrad[0]; // 统计总图像强度
        // J_se2在Pattern51的情况下是52*3，每一行是1*3. //?具体含义有待补充：其实这一部分是，梯度*几何雅可比
        J_se2.row(i) = valGrad.template tail<2>().transpose() * Jw_se2; // 链式法则: 取valGrad的后2位元素，即图像梯度，列向量转置后，变成1*2，再乘以2*3 
        grad_sum_se2 += J_se2.row(i); // 所有行的梯度相加
        num_valid_points++;
      } else {
        data[i] = -1;
      }
    }

    mean = sum / num_valid_points; // 总灰度除以有效点数，得到平均亮度值，可以消除曝光时间引起的图像光度尺度变化，但无法消除光度的偏移变化

    const Scalar mean_inv = num_valid_points / sum; // 平均亮度的逆

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (data[i] >= 0) { // 如果patch里面序号i对应的点的图像强度大于等于0，
        J_se2.row(i) -= grad_sum_se2.transpose() * data[i] / sum; //? //TODO: -= 和 /  谁的优先级更高
        data[i] *= mean_inv;
      } else { // 否则无效的图像强度，该行直接置为0
        J_se2.row(i).setZero();
      }
    }
    J_se2 *= mean_inv; // 至此，完成了梯度雅可比和几何雅可比的链式法则求偏导的整个过程。
  }

  void setFromImage(const Image<const uint16_t> &img, const Vector2 &pos) {
    this->pos = pos;

    MatrixP3 J_se2;

    setDataJacSe2(img, pos, mean, data, J_se2); // 求雅可比J_se2.

    Matrix3 H_se2 = J_se2.transpose() * J_se2; // H = J^T * J
    Matrix3 H_se2_inv;
    H_se2_inv.setIdentity();
    H_se2.ldlt().solveInPlace(H_se2_inv); // 求H^(-1)

    H_se2_inv_J_se2_T = H_se2_inv * J_se2.transpose(); // 求H^(-1) * J^T

    // NOTE: while it's very unlikely we get a source patch with all black
    // pixels, since points are usually selected at corners, it doesn't cost
    // much to be safe here.
    // 注意：这时得到一个所有黑色像素的patch是非常不可能的，因为通常是在角上选择点，在这里安全的开销并不多
    // 全黑的patch 不能被归一化；会导致0平均值，同时，H_se2_inv_J_se2_T会包含非数（NaN）并且数据会包含无穷（inf）

    // all-black patch cannot be normalized; will result in mean of "zero" and
    // H_se2_inv_J_se2_T will contain "NaN" and data will contain "inf"
    valid = mean > std::numeric_limits<Scalar>::epsilon() &&
            H_se2_inv_J_se2_T.array().isFinite().all() &&
            data.array().isFinite().all();
  }

  inline bool residual(const Image<const uint16_t> &img,
                       const Matrix2P &transformed_pattern,
                       VectorP &residual) const {
    Scalar sum = 0;
    int num_valid_points = 0;

    // 对pattern的每一个数据进行计算 这里还没有做差，只是求取了每个pattern在像素处的值
    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (img.InBounds(transformed_pattern.col(i), 2)) {
        residual[i] = img.interp<Scalar>(transformed_pattern.col(i));
        sum += residual[i]; // 求总和
        num_valid_points++;
      } else {
        residual[i] = -1;
      }
    }

    // all-black patch cannot be normalized
    if (sum < std::numeric_limits<Scalar>::epsilon()) { // 小于优化的值了 返回
      residual.setZero();
      return false;
    }

    int num_residuals = 0;

    // 对于pattern的每个点进行计算
    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (residual[i] >= 0 && data[i] >= 0) {
        const Scalar val = residual[i]; // 这地方相当于做类型转换
        residual[i] = num_valid_points * val / sum - data[i]; // 归一化后再相减
        num_residuals++;

      } else {
        residual[i] = 0;
      }
    }

    return num_residuals > PATTERN_SIZE / 2; // 超过一半的值才是符合的
  }

  Vector2 pos = Vector2::Zero();
  VectorP data = VectorP::Zero();  // negative if the point is not valid

  // MatrixP3 J_se2;  // total jacobian with respect to se2 warp
  // Matrix3 H_se2_inv;
  Matrix3P H_se2_inv_J_se2_T = Matrix3P::Zero();

  Scalar mean = 0;

  bool valid = false;
};

template <typename Scalar, typename Pattern>
const typename OpticalFlowPatch<Scalar, Pattern>::Matrix2P
    OpticalFlowPatch<Scalar, Pattern>::pattern2 = Pattern::pattern2;

}  // namespace basalt
