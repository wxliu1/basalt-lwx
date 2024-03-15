/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2021, Vladyslav Usenko and Nikolaus Demmel.
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

#include <basalt/linearization/linearization_abs_qr.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <basalt/utils/ba_utils.h>
#include <basalt/linearization/imu_block.hpp>
#include <basalt/utils/cast_utils.hpp>

namespace basalt {

template <typename Scalar, int POSE_SIZE>
LinearizationAbsQR<Scalar, POSE_SIZE>::LinearizationAbsQR(
    BundleAdjustmentBase<Scalar>* estimator, const AbsOrderMap& aom,
    const Options& options, const MargLinData<Scalar>* marg_lin_data,
    const ImuLinData<Scalar>* imu_lin_data,
    const std::set<FrameId>* used_frames,
    const std::unordered_set<KeypointId>* lost_landmarks,
    int64_t last_state_to_marg)
    : options_(options),
      estimator(estimator),
      lmdb_(estimator->lmdb),
      frame_poses(estimator->frame_poses),
      calib(estimator->calib),
      aom(aom),
      used_frames(used_frames),
      marg_lin_data(marg_lin_data),
      imu_lin_data(imu_lin_data),
      pose_damping_diagonal(0),
      pose_damping_diagonal_sqrt(0) {
  UNUSED(last_state_to_marg);

  BASALT_ASSERT_STREAM(
      options.lb_options.huber_parameter == estimator->huber_thresh,
      "Huber threshold should be set to the same value");

  BASALT_ASSERT_STREAM(options.lb_options.obs_std_dev == estimator->obs_std_dev,
                       "obs_std_dev should be set to the same value");

  // Allocate memory for relative pose linearization 为相对位姿线性化分配内存
  for (const auto& [tcid_h, target_map] : lmdb_.getObservations()) {
    // if (used_frames && used_frames->count(tcid_h.frame_id) == 0) continue;
    const size_t host_idx = host_to_idx_.size();
    host_to_idx_.try_emplace(tcid_h, host_idx);
    host_to_landmark_block.try_emplace(tcid_h); // 这里只添加以tcid_h为key的元素部分

    // assumption: every host frame has at least target frame with
    // observations
    // NOTE: in case a host frame loses all of its landmarks due
    // to outlier removal or marginalization of other frames, it becomes quite
    // useless and is expected to be removed before optimization.
    // 假定: 每个主机帧都至少有带有观测结果的目标帧
    // 注意: 万一由于删除outlier或者其它帧们的边缘化，一个主导帧失去了它所有的路标点，那么它变得毫无用处，并期望在优化前被删除
    BASALT_ASSERT(!target_map.empty());

    for (const auto& [tcid_t, obs] : target_map) {
      // assumption: every target frame has at least one observation 假设：每个 target frame 至少有一个观测
      BASALT_ASSERT(!obs.empty());

      std::pair<TimeCamId, TimeCamId> key(tcid_h, tcid_t);
      relative_pose_lin.emplace(key, RelPoseLin<Scalar>()); // 以host frame和target frame的id组成的pair作为key
    }
  }

  // Populate lookup for relative poses grouped by host-frame 填充按主机帧分组的相对姿态的查找 
  for (const auto& [tcid_h, target_map] : lmdb_.getObservations()) {
    // if (used_frames && used_frames->count(tcid_h.frame_id) == 0) continue;
    relative_pose_per_host.emplace_back(); // emplace_back()未带参数，在vector末尾添加一个按默认值构造的元素。

    for (const auto& [tcid_t, _] : target_map) {
      std::pair<TimeCamId, TimeCamId> key(tcid_h, tcid_t);
      auto it = relative_pose_lin.find(key);

      BASALT_ASSERT(it != relative_pose_lin.end());

      relative_pose_per_host.back().emplace_back(it);
    }
  }

  num_cameras = frame_poses.size(); // 相机帧数

  landmark_ids.clear();
  for (const auto& [k, v] : lmdb_.getLandmarks()) { // k是特征点id, v是特征点
    if (used_frames || lost_landmarks) {
      //- marginalize()时，被marg帧作为host frame的landmarks, 以及lost_landmarks都将作为被marg的点添加进来
      if (used_frames && used_frames->count(v.host_kf_id.frame_id)) {
        landmark_ids.emplace_back(k); // 如果路标点的host frame是被marg的帧，则也添加进来
      } else if (lost_landmarks && lost_landmarks->count(k)) {
        landmark_ids.emplace_back(k); // marginalize()时，执行该行，添加需要被marg的landmark id
      }
    } else {
      landmark_ids.emplace_back(k); // opitimize()时，执行该行，添加需要被optimize的landmark id
    }
  }
  size_t num_landmakrs = landmark_ids.size();

  // std::cout << "num_landmakrs " << num_landmakrs << std::endl;

  landmark_blocks.resize(num_landmakrs); // 路标块的size是按照优化或者边缘化的路标点个数来确定的

  {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        KeypointId lm_id = landmark_ids[r];
        auto& lb = landmark_blocks[r];
        auto& landmark = lmdb_.getLandmark(lm_id);

        lb = LandmarkBlock<Scalar>::template createLandmarkBlock<POSE_SIZE>();

        lb->allocateLandmark(landmark, relative_pose_lin, calib, aom,
                             options.lb_options);
      }
    };

    tbb::blocked_range<size_t> range(0, num_landmakrs);
    tbb::parallel_for(range, body); // 并行分配路标块
  }
#if 0
  // print logs. 2024-3-4
  std::string str;
  if(lost_landmarks)
  {
    str = "marg";
  }
  else
  {
    str = "opt";
  }
  std::cout << "o_or_m=" << str << " num_landmakrs=" << num_landmakrs << " aom.total_size=" << aom.total_size << std::endl;
  for(int i = 0; i < num_landmakrs; i++)
  {
    KeypointId lm_id = landmark_ids[i];
    auto& landmark = lmdb_.getLandmark(lm_id);

    std::cout << "host kf id=" << landmark.host_kf_id.frame_id << " lm_id=" << lm_id << " obs.size:" << landmark.obs.size() << std::endl;

    int j = 0;
    int k = 0;
    for (const auto& [tcid_t, pos] : landmark.obs) 
    {
      std::cout << "target kf id=" << tcid_t.frame_id << std::endl;

      if (aom.abs_order_map.count(tcid_t.frame_id) > 0) {
        j++;
      } else {
        // Observation droped for marginalization
        k++;
      }
    }

    std::cout << "j = " << j << " k = " << k << std::endl;

    break ;
  }
  // the end.
#endif

  landmark_block_idx.reserve(num_landmakrs);

  num_rows_Q2r = 0;
  for (size_t i = 0; i < num_landmakrs; i++) {
    landmark_block_idx.emplace_back(num_rows_Q2r);
    const auto& lb = landmark_blocks[i];
    // lb->numQ2rows()返回的是：单个路标点的观测点的个数(pose_lin_vec.size()) * 残差维数(2) // 统计每个路标用于存储路标雅可比的矩阵的行数
    //- 20240311正解：lb->numQ2rows()返回的是如root ba论文中的Figure 3(a)所示的Q_2^T x J_p加最后3行空白部分，
    //- 即图3(a)中除去Q_1^T x J_p所占的3行，剩余的部分，由于landmark block的storage矩阵的最后3行是空白的，所以返回的
    //- 实际行数包含了Q_2^T x J_p的行数加上最后空白的3行
    //- 简而言之：num_rows_Q2r保存的是所有landmarks的Q_2^T x r行数之和，而landmark_block_idx保存的是每一个landmark在大的Q2r和大的Q2Jp中的序号
    num_rows_Q2r += lb->numQ2rows();

    host_to_landmark_block.at(lb->getHostKf()).emplace_back(lb.get()); // 存储以host_kf_id为key的landmark_block普通指针
  }

  if (imu_lin_data) {
    for (const auto& kv : imu_lin_data->imu_meas) {
      imu_blocks.emplace_back(
          new ImuBlock<Scalar>(kv.second, imu_lin_data, aom));
    }
  }

  //    std::cout << "num_rows_Q2r " << num_rows_Q2r << " num_poses " <<
  //    num_cameras
  //              << std::endl;
}

template <typename Scalar, int POSE_SIZE>
LinearizationAbsQR<Scalar, POSE_SIZE>::~LinearizationAbsQR() = default;

template <typename Scalar_, int POSE_SIZE_>
void LinearizationAbsQR<Scalar_, POSE_SIZE_>::log_problem_stats(
    ExecutionStats& stats) const {
  UNUSED(stats);
}

template <typename Scalar, int POSE_SIZE>
Scalar LinearizationAbsQR<Scalar, POSE_SIZE>::linearizeProblem(
    bool* numerically_valid) {
  // reset damping and scaling (might be set from previous iteration)
  // 重置阻尼和缩放（可以从上一次迭代中设置） 
  pose_damping_diagonal = 0;
  pose_damping_diagonal_sqrt = 0;
  marg_scaling = VecX();

  // 不管是optimize还是marginalize，都会先对所有的观测关联的host frame 和 target frame进行相对位姿线性化
  //? TODO:疑问: marg时，还需要把所有的观测对应的相对位姿全部线性化一遍吗?只线性化marg frame和lost landmarks关联的host frame和target frame不就可以么?
  // Linearize relative poses 线性化相对位姿
  for (const auto& [tcid_h, target_map] : lmdb_.getObservations()) {
    // if (used_frames && used_frames->count(tcid_h.frame_id) == 0) continue;

    for (const auto& [tcid_t, _] : target_map) {
      std::pair<TimeCamId, TimeCamId> key(tcid_h, tcid_t);
      RelPoseLin<Scalar>& rpl = relative_pose_lin.at(key); // relative_pose_lin是以host frame和target frame的id组成的pair作为key

      if (tcid_h != tcid_t) {
        // 获取host frame和target frame的位姿
        const PoseStateWithLin<Scalar>& state_h =
            estimator->getPoseStateWithLin(tcid_h.frame_id);
        const PoseStateWithLin<Scalar>& state_t =
            estimator->getPoseStateWithLin(tcid_t.frame_id);


        /*
         * 毅神的理解：
         * 通常我们要固定一些Jacobian，需要固定的是整个Frame的Jacobian。
         * 但Basalt的这段代码却只固定了相对姿态跟自身的导数，即 DT_h^t / DT_h ，和 DT_h^t / DT_t 。
         * 而对于 Dr_{it} / DT_h^t  来说，却在随着最优化迭代的过程中保持更新，对于点云的Jacobian来说，也在保持更新。
         * 
         * 这一波操作，我一开始并不是特别明白。但仔细思考的时候，大致猜到了缘由：
         * 1、作者是按照图2的方式划分子图的，并且每个子图在host frame为参考系下独立做Schur消元。
         * 2、划分子图后，每个子图可以看成以一个host frame作为参考系下的子图。
         * 在子图中，host frame的坐标值为这个子图的坐标原点，所有target frame的姿态数值，都是相对于该host frame的相对姿态。
         * 因此在该相对参考系的坐标下，所有变量都不存在零空间。
         * 也可以理解为，在子图的处理环节，host frame的坐标已经为Identity，所以不存在VIO中的不可观性。
         * 就是因为零空间在相对坐标系下不存在，按照常规优化就好，无需FEJ操作。
         */

        // compute relative pose & Jacobians at linearization point 在线性化点处计算相对位姿和Jacobians
        // 使用FEJ线性化点计算 d_rel_d_h， d_rel_d_t
        Sophus::SE3<Scalar> T_t_h_sophus =
            computeRelPose(state_h.getPoseLin(), calib.T_i_c[tcid_h.cam_id],
                           state_t.getPoseLin(), calib.T_i_c[tcid_t.cam_id],
                           &rpl.d_rel_d_h, &rpl.d_rel_d_t);

        // if either state is already linearized, then the current state
        // estimate is different from the linearization point, so recompute
        // the value (not Jacobian) again based on the current state.
        // 当Pose存在线性化点的时候，使用优化迭代时候的姿态计算相对姿态 T_t_h_sophus
        if (state_h.isLinearized() || state_t.isLinearized()) {
          T_t_h_sophus =
              computeRelPose(state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                             state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);
        }

        rpl.T_t_h = T_t_h_sophus.matrix();
      } else {
        rpl.T_t_h.setIdentity();
        rpl.d_rel_d_h.setZero();
        rpl.d_rel_d_t.setZero();
      }
    }
  }

  // 对于optimize，线性化lmdb里面的所有路标点，而对于marg，这里只线性化lost landmarks（未连接的路标点，即不被当前帧观测到的路标点）
  // Linearize landmarks 线性化路标点
  size_t num_landmarks = landmark_blocks.size();

  auto body = [&](const tbb::blocked_range<size_t>& range,
                  std::pair<Scalar, bool> error_valid) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      // linearizeLandmark()求重投影残差，然后线性化观测点，线性化host frame和target frame的绝对位姿
      // 进而填充该路标点的landmark_block: storage[J_p | pad | J_l | res]
      error_valid.first += landmark_blocks[r]->linearizeLandmark();
      error_valid.second =
          error_valid.second && !landmark_blocks[r]->isNumericalFailure();
    }
    return error_valid;
  };

  // initial_value作为初值，第一个分量表示加权残差和，第二个分量表示线性化是否数值成功
  std::pair<Scalar, bool> initial_value = {0.0, true};
  // join用于汇总各个线程块的结果，这里表示把所有路标的加权误差相加，线性化的数值是否有效进行逻辑与
  auto join = [](auto p1, auto p2) { // p2为每个线程的返回值
    p1.first += p2.first;
    p1.second = p1.second && p2.second;
    return p1; // 返回总误差，以及所有点的线性化是否数值上成功
  };

  tbb::blocked_range<size_t> range(0, num_landmarks);
  auto reduction_res = tbb::parallel_reduce(range, initial_value, body, join);

  if (numerically_valid) *numerically_valid = reduction_res.second;

  if (imu_lin_data) {
    for (auto& imu_block : imu_blocks) {
      reduction_res.first += imu_block->linearizeImu(estimator->frame_states);
    }
  }

  if (marg_lin_data) {
    Scalar marg_prior_error;
    estimator->computeMargPriorError(*marg_lin_data, marg_prior_error);
    reduction_res.first += marg_prior_error;
  }

  return reduction_res.first; // 返回总误差
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::performQR() {
  auto body = [&](const tbb::blocked_range<size_t>& range) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      landmark_blocks[r]->performQR(); // 每个landmark block 单独执行performQR
    }
  };
  // 摘自rootba论文: As Givens rotations operate on individual rows, marginalization can be performed for each landmark block separately, possibly in parallel.
  // 意即：由于Givens rotations（实现QR分解的一种方法）在单独行上操作，可以并行地，在每个landmark block上分别地执行marg
  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_for(range, body);
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::setPoseDamping(
    const Scalar lambda) {
  BASALT_ASSERT(lambda >= 0);

  pose_damping_diagonal = lambda;
  pose_damping_diagonal_sqrt = std::sqrt(lambda);
}

template <typename Scalar, int POSE_SIZE>
Scalar LinearizationAbsQR<Scalar, POSE_SIZE>::backSubstitute(
    const VecX& pose_inc) {
  BASALT_ASSERT(pose_inc.size() == signed_cast(aom.total_size));

  bool inc_invalid = false;

  auto body = [&](const tbb::blocked_range<size_t>& range, Scalar l_diff) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      // landmark_blocks[r]->backSubstitute(pose_inc, l_diff);
      bool ret = landmark_blocks[r]->backSubstitute(pose_inc, l_diff);
      if(!ret)
      {
        inc_invalid = true;
        break;
      }
    }
    return l_diff;
  };

  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  // 返回汇总的cost difference
  Scalar l_diff =
      tbb::parallel_reduce(range, Scalar(0), body, std::plus<Scalar>());
  if (imu_lin_data) {
    for (auto& imu_block : imu_blocks) {
      imu_block->backSubstitute(pose_inc, l_diff);
    }
  }

  if (marg_lin_data) {
    size_t marg_size = marg_lin_data->H.cols();
    VecX pose_inc_marg = pose_inc.head(marg_size);

    // 加上marg prior的cost change
    l_diff += estimator->computeMargPriorModelCostChange(
        *marg_lin_data, marg_scaling, pose_inc_marg);
  }

  if(inc_invalid) return -0.000001;

  return l_diff;
}

template <typename Scalar, int POSE_SIZE>
typename LinearizationAbsQR<Scalar, POSE_SIZE>::VecX
LinearizationAbsQR<Scalar, POSE_SIZE>::getJp_diag2() const {
  // TODO: group relative by host frame

  struct Reductor {
    Reductor(size_t num_rows,
             const std::vector<LandmarkBlockPtr>& landmark_blocks)
        : num_rows_(num_rows), landmark_blocks_(landmark_blocks) {
      res_.setZero(num_rows);
    }

    void operator()(const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        lb->addJp_diag2(res_);
      }
    }

    Reductor(Reductor& a, tbb::split)
        : num_rows_(a.num_rows_), landmark_blocks_(a.landmark_blocks_) {
      res_.setZero(num_rows_);
    };

    inline void join(const Reductor& b) { res_ += b.res_; }

    size_t num_rows_;
    const std::vector<LandmarkBlockPtr>& landmark_blocks_;
    VecX res_;
  };

  Reductor r(aom.total_size, landmark_blocks);

  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_reduce(range, r);
  // r(range);

  if (imu_lin_data) {
    for (auto& imu_block : imu_blocks) {
      imu_block->addJp_diag2(r.res_);
    }
  }

  // TODO: We don't include pose damping here, b/c we use this to compute
  // jacobian scale. Make sure this is clear in the the usage, possibly rename
  // to reflect this, or add assert such that it fails when pose damping is
  // set.

  // Note: ignore damping here

  // Add marginalization prior
  // Asumes marginalization part is in the head. Check for this is located
  // outside
  if (marg_lin_data) {
    size_t marg_size = marg_lin_data->H.cols();
    if (marg_scaling.rows() > 0) {
      r.res_.head(marg_size) += (marg_lin_data->H * marg_scaling.asDiagonal())
                                    .colwise()
                                    .squaredNorm();
    } else {
      r.res_.head(marg_size) += marg_lin_data->H.colwise().squaredNorm();
    }
  }

  return r.res_;
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::scaleJl_cols() {
  auto body = [&](const tbb::blocked_range<size_t>& range) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      landmark_blocks[r]->scaleJl_cols();
    }
  };

  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_for(range, body);
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::scaleJp_cols(
    const VecX& jacobian_scaling) {
  //    auto body = [&](const tbb::blocked_range<size_t>& range) {
  //      for (size_t r = range.begin(); r != range.end(); ++r) {
  //        landmark_blocks[r]->scaleJp_cols(jacobian_scaling);
  //      }
  //    };

  //    tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  //    tbb::parallel_for(range, body);

  if (true) {
    // In case of absolute poses, we scale Jp in the LMB.

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks[r]->scaleJp_cols(jacobian_scaling);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
    tbb::parallel_for(range, body);
  } else {
    // In case LMB use relative poses we cannot directly scale the relative pose
    // Jacobians. We have
    //
    //     Jp * diag(S) = Jp_rel * J_rel_to_abs * diag(S)
    //
    // so instead we scale the rel-to-abs jacobians.
    //
    // Note that since we do perform operations like J^T * J on the relative
    // pose Jacobians, we should maybe consider additional scaling like
    //
    //     (Jp_rel * diag(S_rel)) * (diag(S_rel)^-1 * J_rel_to_abs * diag(S)),
    //
    // but this might be only relevant if we do something more sensitive like
    // also include camera intrinsics in the optimization.

    for (auto& [k, v] : relative_pose_lin) {
      size_t h_idx = aom.abs_order_map.at(k.first.frame_id).first;
      size_t t_idx = aom.abs_order_map.at(k.second.frame_id).first;

      v.d_rel_d_h *=
          jacobian_scaling.template segment<POSE_SIZE>(h_idx).asDiagonal();

      v.d_rel_d_t *=
          jacobian_scaling.template segment<POSE_SIZE>(t_idx).asDiagonal();
    }
  }

  if (imu_lin_data) {
    for (auto& imu_block : imu_blocks) {
      imu_block->scaleJp_cols(jacobian_scaling);
    }
  }

  // Add marginalization scaling
  if (marg_lin_data) {
    // We are only supposed to apply the scaling once.
    BASALT_ASSERT(marg_scaling.size() == 0);

    size_t marg_size = marg_lin_data->H.cols();
    marg_scaling = jacobian_scaling.head(marg_size);
  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::setLandmarkDamping(Scalar lambda) {
  auto body = [&](const tbb::blocked_range<size_t>& range) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      landmark_blocks[r]->setLandmarkDamping(lambda);
    }
  };

  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_for(range, body);
}

// @brief get_dense_Q2Jp_Q2r这个函数与get_dense_H_b最大的不同是：
// 后者把每个landmark block的\widetilde{H}_{pp}和\widetilde{b_p}叠加起来，
// 而前者把每个landmark block的Q_2^T x J_p和Q_2^T x r, 赋值到大矩阵Q2Jp和Q2r相应的位置上
// Q2Jp中按行排列所有的Q_2^T x J_p和H_m^{\prime}
// Q2r中按行排列所有的Q_2^T x r和b_m^{\prime}
template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::get_dense_Q2Jp_Q2r(
    MatX& Q2Jp, VecX& Q2r) const {
  // num_rows_Q2r保存的是所有landmarks的Q_2^T x r行数之和，而landmark_block_idx[i]保存的是每一个landmark在大的Q2r和大的Q2Jp中的序号
  size_t total_size = num_rows_Q2r; // 所有landmark block的Q_2^T x r 的行数之和
  size_t poses_size = aom.total_size; // 参与marg的pose总维数

  size_t lm_start_idx = 0;

  // Space for IMU data if present
  size_t imu_start_idx = total_size;
  if (imu_lin_data) {
    total_size += imu_lin_data->imu_meas.size() * POSE_VEL_BIAS_SIZE;
  }

  // Space for damping if present
  size_t damping_start_idx = total_size;
  if (hasPoseDamping()) { // 至少vo模式下，并没有用的damping
    total_size += poses_size;
  }

  // Space for marg-prior if present
  size_t marg_start_idx = total_size;
  if (marg_lin_data) total_size += marg_lin_data->H.rows();

  Q2Jp.setZero(total_size, poses_size); // 设置Q2Jp的维数，row_num=total_size, col_num=poses_size
  Q2r.setZero(total_size);

  auto body = [&](const tbb::blocked_range<size_t>& range) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      const auto& lb = landmark_blocks[r];
      // landmark_block_idx[r] 返回的是每个landmark block 在大的Q2r和大的Q2Jp中的序号
      // 从landmark block的storage矩阵中取出Q_2^T x J_p和Q_2^T x r, 赋值到大矩阵Q2Jp和Q2r相应的位置上
      lb->get_dense_Q2Jp_Q2r(Q2Jp, Q2r, lm_start_idx + landmark_block_idx[r]);
    }
  };

  // go over all host frames
  // range的范围是0到参与optimize或者marg的landmarks总个数，即: range ∈ [0, num_landmakrs)
  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_for(range, body);
  // body(range);

  if (imu_lin_data) {
    size_t start_idx = imu_start_idx;
    for (const auto& imu_block : imu_blocks) {
      imu_block->add_dense_Q2Jp_Q2r(Q2Jp, Q2r, start_idx);
      start_idx += POSE_VEL_BIAS_SIZE;
    }
  }

  // Add damping
  get_dense_Q2Jp_Q2r_pose_damping(Q2Jp, damping_start_idx); // 至少vo模式下，阻尼并没有用到

  // Add marginalization 添加边缘化先验 ，初始先验是6x6的对角阵（对角元素1e4）
  get_dense_Q2Jp_Q2r_marg_prior(Q2Jp, Q2r, marg_start_idx); // marg_start_idx是先验在Q2Jp中的行的起始索引
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::get_dense_H_b(MatX& H,
                                                          VecX& b) const {
  struct Reductor { // 汇总所有的landmark block
    Reductor(size_t opt_size,
             const std::vector<LandmarkBlockPtr>& landmark_blocks)
        : opt_size_(opt_size), landmark_blocks_(landmark_blocks) {
      H_.setZero(opt_size_, opt_size_); // 设置H和b为0矩阵
      b_.setZero(opt_size_);
    }

    void operator()(const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        auto& lb = landmark_blocks_[r];
        lb->add_dense_H_b(H_, b_); // 初值H和b都是0矩阵
      }
    }

    Reductor(Reductor& a, tbb::split)
        : opt_size_(a.opt_size_), landmark_blocks_(a.landmark_blocks_) {
      H_.setZero(opt_size_, opt_size_);
      b_.setZero(opt_size_);
    };

    //- 汇总：令landmark block块的J=Q_2^T x J_p, r=Q_2^T x r,
    // 则可以得到每个landmark block的H = J^T x J和b=J^T x r
    // 最后把所有landmarks的H和b累加起来
    inline void join(Reductor& b) {
      H_ += b.H_;
      b_ += b.b_;
    }

    size_t opt_size_;
    const std::vector<LandmarkBlockPtr>& landmark_blocks_;

    MatX H_;
    VecX b_;
  };

  size_t opt_size = aom.total_size; //位姿总维数

  Reductor r(opt_size, landmark_blocks);

  // go over all host frames
  // range的范围是0到参与优化的landmarks总个数，即: range ∈ [0, num_landmakrs)
  tbb::blocked_range<size_t> range(0, landmark_block_idx.size());
  tbb::parallel_reduce(range, r);

  // Add imu
  add_dense_H_b_imu(r.H_, r.b_);

  // Add damping
  add_dense_H_b_pose_damping(r.H_); // 看sqrt_keypoint_vo模式下，并没有用到damping

  // Add marginalization 添加marg先验
  // 分别在r.H_的左上角加上H_m^{\prime}, 在r.b_的头部加上b_m^{\prime}, 即：
  // r.H_.topLeftCorner(marg_size, marg_size) += H_m^{\prime}
  // r.b_.head(marg_size) += b_m^{\prime};
  add_dense_H_b_marg_prior(r.H_, r.b_);

  // 把所有路标点通过Nullspace Marginalization后的H_p和b_p叠加，再叠加左上角的marg. prior之后，进行赋值返回。
  H = std::move(r.H_);
  b = std::move(r.b_);
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::get_dense_Q2Jp_Q2r_pose_damping(
    MatX& Q2Jp, size_t start_idx) const {
  size_t poses_size = num_cameras * POSE_SIZE;
  if (hasPoseDamping()) {
    Q2Jp.template block(start_idx, 0, poses_size, poses_size)
        .diagonal()
        .array() = pose_damping_diagonal_sqrt;
  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::get_dense_Q2Jp_Q2r_marg_prior(
    MatX& Q2Jp, VecX& Q2r, size_t start_idx) const {
  if (!marg_lin_data) return;

  BASALT_ASSERT(marg_lin_data->is_sqrt);

  size_t marg_rows = marg_lin_data->H.rows();
  size_t marg_cols = marg_lin_data->H.cols();

  VecX delta;
  estimator->computeDelta(marg_lin_data->order, delta); // 获取线性化之后的状态量的累计增量
  // 用线性化之后累计的增量delta 来更新先验信息
  // H_m_new = H_m 雅可比被固定住了，
  // b_m_new = b_m + H_m * delta 但是残差r随着状态量优化，在不断变化，因而b也变化了。

  if (marg_scaling.rows() > 0) {
    Q2Jp.template block(start_idx, 0, marg_rows, marg_cols) =
        marg_lin_data->H * marg_scaling.asDiagonal();
  } else {
    Q2Jp.template block(start_idx, 0, marg_rows, marg_cols) = marg_lin_data->H;
  }

  Q2r.template segment(start_idx, marg_rows) =
      marg_lin_data->H * delta + marg_lin_data->b;
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::add_dense_H_b_pose_damping(
    MatX& H) const {
  if (hasPoseDamping()) {
    H.diagonal().array() += pose_damping_diagonal;
  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::add_dense_H_b_marg_prior(
    MatX& H, VecX& b) const {
  if (!marg_lin_data) return;

  // Scaling not supported ATM
  BASALT_ASSERT(marg_scaling.rows() == 0);

  Scalar marg_prior_error;
  estimator->linearizeMargPrior(*marg_lin_data, aom, H, b, marg_prior_error);

  //  size_t marg_size = marg_lin_data->H.cols();

  //  VecX delta;
  //  estimator->computeDelta(marg_lin_data->order, delta);

  //  if (marg_scaling.rows() > 0) {
  //    H.topLeftCorner(marg_size, marg_size) +=
  //        marg_scaling.asDiagonal() * marg_lin_data->H.transpose() *
  //        marg_lin_data->H * marg_scaling.asDiagonal();

  //    b.head(marg_size) += marg_scaling.asDiagonal() *
  //                         marg_lin_data->H.transpose() *
  //                         (marg_lin_data->H * delta + marg_lin_data->b);

  //  } else {
  //    H.topLeftCorner(marg_size, marg_size) +=
  //        marg_lin_data->H.transpose() * marg_lin_data->H;

  //    b.head(marg_size) += marg_lin_data->H.transpose() *
  //                         (marg_lin_data->H * delta + marg_lin_data->b);
  //  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::add_dense_H_b_imu(
    DenseAccumulator<Scalar>& accum) const {
  if (!imu_lin_data) return;

  for (const auto& imu_block : imu_blocks) {
    imu_block->add_dense_H_b(accum);
  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::add_dense_H_b_imu(MatX& H,
                                                              VecX& b) const {
  if (!imu_lin_data) return;

  // workaround: create an accumulator here, to avoid implementing the
  // add_dense_H_b(H, b) overload in ImuBlock
  DenseAccumulator<Scalar> accum;
  accum.reset(b.size());

  for (const auto& imu_block : imu_blocks) {
    imu_block->add_dense_H_b(accum);
  }

  H += accum.getH();
  b += accum.getB();
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

#ifdef BASALT_INSTANTIATIONS_DOUBLE
// Scalar=double, POSE_SIZE=6
template class LinearizationAbsQR<double, 6>;
#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
// Scalar=float, POSE_SIZE=6
template class LinearizationAbsQR<float, 6>;
#endif

}  // namespace basalt
