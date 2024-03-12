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

#include <basalt/vi_estimator/landmark_database.h>

namespace basalt {

template <class Scalar>
Sophus::SE3<Scalar> computeRelPose(
    const Sophus::SE3<Scalar>& T_w_i_h, const Sophus::SE3<Scalar>& T_i_c_h,
    const Sophus::SE3<Scalar>& T_w_i_t, const Sophus::SE3<Scalar>& T_i_c_t,
    Sophus::Matrix6<Scalar>* d_rel_d_h = nullptr,
    Sophus::Matrix6<Scalar>* d_rel_d_t = nullptr) {
  Sophus::SE3<Scalar> tmp2 = (T_i_c_t).inverse(); // 求逆之后，变成target frame的imu到camera

  // T_w_i_h表示的是host frame的imu到world的位姿
  // T_w_i_t表示的是target frame的imu到world的位姿
  // T_i_c_h表示的是host frame的camera到imu的外参
  // T_i_c_t表示的是target frame的camear到imu的外参
  // 计算T_t_i_h_i, 即计算host frame的imu到target frame的imu的相对位姿
  Sophus::SE3<Scalar> T_t_i_h_i;
  T_t_i_h_i.so3() = T_w_i_t.so3().inverse() * T_w_i_h.so3();
  T_t_i_h_i.translation() =
      T_w_i_t.so3().inverse() * (T_w_i_h.translation() - T_w_i_t.translation());

  Sophus::SE3<Scalar> tmp = tmp2 * T_t_i_h_i; // 计算tmp = T_t_c_h_i, 即得到host frame的imu到target frame的camera
  Sophus::SE3<Scalar> res = tmp * T_i_c_h;    // 计算res = T_t_c_h_i * T_i_c_h = T_t_c_h_c, 即得到host frame的camera到target frame的camera

  // 右扰动，求相对位姿对 host frame 的导数
  if (d_rel_d_h) {
    Sophus::Matrix3<Scalar> R = T_w_i_h.so3().inverse().matrix();

    Sophus::Matrix6<Scalar> RR;
    RR.setZero();
    RR.template topLeftCorner<3, 3>() = R;
    RR.template bottomRightCorner<3, 3>() = R;

    *d_rel_d_h = tmp.Adj() * RR;
  }

  // 左扰动，求相对位姿对 target frame 的导数
  if (d_rel_d_t) {
    Sophus::Matrix3<Scalar> R = T_w_i_t.so3().inverse().matrix();

    Sophus::Matrix6<Scalar> RR;
    RR.setZero();
    RR.template topLeftCorner<3, 3>() = R;
    RR.template bottomRightCorner<3, 3>() = R;

    *d_rel_d_t = -tmp2.Adj() * RR;
  }

  return res;
}

// 线性化路标点
/*!
@brief 线性化路标点：主帧的2d点经过逆投影得到主帧的3d点，然后经过相对位姿变换得到目标帧的3d点，然后投影得到目标帧的2d点
@return if point is linearized. i.e. if projection is valid
@param[in] kpt_obs是2x1的位置?
@param[in] kpt_pos是路标点
@param[in] T_t_h是host camera frame 到 target camera frame的相对位姿
@param[in] cam是相机模型
@param[out] res是残差
@param[out] d_res_d_xi
@param[out] d_res_d_p 残差对3d点的求导
*/
template <class Scalar, class CamT>
inline bool linearizePoint(
    const Eigen::Matrix<Scalar, 2, 1>& kpt_obs, const Keypoint<Scalar>& kpt_pos,
    const Eigen::Matrix<Scalar, 4, 4>& T_t_h, const CamT& cam,
    Eigen::Matrix<Scalar, 2, 1>& res,
    Eigen::Matrix<Scalar, 2, POSE_SIZE>* d_res_d_xi = nullptr,
    Eigen::Matrix<Scalar, 2, 3>* d_res_d_p = nullptr,
    Eigen::Matrix<Scalar, 4, 1>* proj = nullptr) {
  static_assert(std::is_same_v<typename CamT::Scalar, Scalar>);

  // Todo implement without jacobians
  Eigen::Matrix<Scalar, 4, 2> Jup;
  Eigen::Matrix<Scalar, 4, 1> p_h_3d;
  // 反投影（逆投影）公式unproject，得到3d坐标和雅可比
  p_h_3d = StereographicParam<Scalar>::unproject(kpt_pos.direction, &Jup);
  p_h_3d[3] = kpt_pos.inv_dist; // 逆深度

  const Eigen::Matrix<Scalar, 4, 1> p_t_3d = T_t_h * p_h_3d; // 得到目标帧的3d坐标

  Eigen::Matrix<Scalar, 2, 4> Jp;
  bool valid = cam.project(p_t_3d, res, &Jp); // 投影函数：3d --> 2d
  valid &= res.array().isFinite().all();

  if (!valid) {
    //      std::cerr << " Invalid projection! kpt_pos.dir "
    //                << kpt_pos.dir.transpose() << " kpt_pos.id " <<
    //                kpt_pos.id
    //                << " idx " << kpt_obs.kpt_id << std::endl;

    //      std::cerr << "T_t_h\n" << T_t_h << std::endl;
    //      std::cerr << "p_h_3d\n" << p_h_3d.transpose() << std::endl;
    //      std::cerr << "p_t_3d\n" << p_t_3d.transpose() << std::endl;

    return false; // 如果从host帧投影无效，则直接返回false
  }

  if (proj) {
    proj->template head<2>() = res; // 前两维保存从主导帧投影过来的2d坐标
    (*proj)[2] = p_t_3d[3] / p_t_3d.template head<3>().norm(); // lwx: 1 / d
  }
  res -= kpt_obs; // 从主导帧投影过来的2d点减去目标帧的2d点，得到重投影残差

  if (d_res_d_xi) {
    Eigen::Matrix<Scalar, 4, POSE_SIZE> d_point_d_xi;
    d_point_d_xi.template topLeftCorner<3, 3>() =
        Eigen::Matrix<Scalar, 3, 3>::Identity() * kpt_pos.inv_dist;
    d_point_d_xi.template topRightCorner<3, 3>() =
        -Sophus::SO3<Scalar>::hat(p_t_3d.template head<3>());
    d_point_d_xi.row(3).setZero();

    *d_res_d_xi = Jp * d_point_d_xi;
  }

  if (d_res_d_p) {
    Eigen::Matrix<Scalar, 4, 3> Jpp;
    Jpp.setZero();
    Jpp.template block<3, 2>(0, 0) = T_t_h.template topLeftCorner<3, 4>() * Jup;
    Jpp.col(2) = T_t_h.col(3);

    *d_res_d_p = Jp * Jpp;
  }

  return true;
}

}  // namespace basalt
