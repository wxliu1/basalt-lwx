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

#include <basalt/utils/imu_types.h>
#include <basalt/utils/eigen_utils.hpp>

namespace basalt {

template <class Scalar_>
struct KeypointObservation {
  using Scalar = Scalar_;

  int kpt_id;
  Eigen::Matrix<Scalar, 2, 1> pos;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// keypoint position defined relative to some frame
template <class Scalar_>
struct Keypoint {
  using Scalar = Scalar_;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  using ObsMap = Eigen::aligned_map<TimeCamId, Vec2>;
  using MapIter = typename ObsMap::iterator;

  // 3D position parameters
  Vec2 direction; //  在赤平面的投影
  Scalar inv_dist; // 逆深度

  // Observations
  TimeCamId host_kf_id; // 主帧id
  ObsMap obs; // 观测map(key是观测的timestamp & camera id, value是 赤平面向量)

  //? backup和restore函数用于将数据进行备份和恢复(这里可能是为fej准备的)。
  inline void backup() {
    backup_direction = direction;
    backup_inv_dist = inv_dist;
  }

  inline void restore() {
    direction = backup_direction;
    inv_dist = backup_inv_dist;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Vec2 backup_direction;
  Scalar backup_inv_dist;
};

/*
// 同时还存在一个特征点数据库:LandmarkDatabase，在滑动窗口中一直维护的是这个数据库的内容，这个内容主要是kpts和observations这两个成员变量

// key是特征点id value是特征点 
Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>> kpts;
// key是帧率id+相机id value是 map 这个map是 key是帧率+相机id value是特征点列表
std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>> observations;
*/
template <class Scalar_>
class LandmarkDatabase { // 特征(路标)点数据库
 public:
  using Scalar = Scalar_;

  // Non-const
  void addLandmark(KeypointId lm_id, const Keypoint<Scalar>& pos);

  void removeFrame(const FrameId& frame);

  void removeKeyframes(const std::set<FrameId>& kfs_to_marg,
                       const std::set<FrameId>& poses_to_marg,
                       const std::set<FrameId>& states_to_marg_all);

  void addObservation(const TimeCamId& tcid_target,
                      const KeypointObservation<Scalar>& o);

  Keypoint<Scalar>& getLandmark(KeypointId lm_id);

  // Const
  const Keypoint<Scalar>& getLandmark(KeypointId lm_id) const;

  std::vector<TimeCamId> getHostKfs() const;

  std::vector<const Keypoint<Scalar>*> getLandmarksForHost(
      const TimeCamId& tcid) const;

  const std::unordered_map<TimeCamId,
                           std::map<TimeCamId, std::set<KeypointId>>>&
  getObservations() const;

  const Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>>&
  getLandmarks() const;

  bool landmarkExists(int lm_id) const;

  size_t numLandmarks() const;

  int numObservations() const;

  int numObservations(KeypointId lm_id) const;

  void removeLandmark(KeypointId lm_id);

  void removeObservations(KeypointId lm_id, const std::set<TimeCamId>& obs);

  inline void backup() {
    for (auto& kv : kpts) kv.second.backup();
  }

  inline void restore() {
    for (auto& kv : kpts) kv.second.restore();
  }

  void Reset(); // 2023-11-19.

 private:
  using MapIter =
      typename Eigen::aligned_unordered_map<KeypointId,
                                            Keypoint<Scalar>>::iterator;
  MapIter removeLandmarkHelper(MapIter it);
  typename Keypoint<Scalar>::MapIter removeLandmarkObservationHelper(
      MapIter it, typename Keypoint<Scalar>::MapIter it2);

  // kpts是保存同一个特征id的所有观测
  Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>> kpts; // key是特征点id, value是特征点 

  std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>
      observations; // key是host_kf_id，由时间戳+相机id组成； value是 map， 这个map是： key是tcid_target，由时间戳+相机id组成， value是特征点id的集合
      // 简而言之，observations是由host frame 对应的目标帧的所有特征点id

  static constexpr int min_num_obs = 2;
};

}  // namespace basalt
