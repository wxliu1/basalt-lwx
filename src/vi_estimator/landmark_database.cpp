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

#include <algorithm>

#include <basalt/vi_estimator/landmark_database.h>

namespace basalt {

// 2023-11-19
template <class Scalar_>
void LandmarkDatabase<Scalar_>::Reset()
{
  kpts.clear();
  observations.clear();
}
// the end.  

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addLandmark(KeypointId lm_id,
                                            const Keypoint<Scalar> &pos) {
  auto &kpt = kpts[lm_id];
  kpt.direction = pos.direction;
  kpt.inv_dist = pos.inv_dist;
  kpt.host_kf_id = pos.host_kf_id;
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeFrame(const FrameId &frame) {
  for (auto it = kpts.begin(); it != kpts.end();) {
    for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
      if (it2->first.frame_id == frame)
        it2 = removeLandmarkObservationHelper(it, it2);
      else
        it2++;
    }

    if (it->second.obs.size() < min_num_obs) {
      it = removeLandmarkHelper(it);
    } else {
      ++it;
    }
  }
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeKeyframes(
    const std::set<FrameId> &kfs_to_marg,
    const std::set<FrameId> &poses_to_marg,
    const std::set<FrameId> &states_to_marg_all) {
  for (auto it = kpts.begin(); it != kpts.end();) {
    if (kfs_to_marg.count(it->second.host_kf_id.frame_id) > 0) {
      it = removeLandmarkHelper(it);
    } else {
      for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
        FrameId fid = it2->first.frame_id;
        if (poses_to_marg.count(fid) > 0 || states_to_marg_all.count(fid) > 0 ||
            kfs_to_marg.count(fid) > 0)
          it2 = removeLandmarkObservationHelper(it, it2);
        else
          it2++;
      }

      if (it->second.obs.size() < min_num_obs) {
        it = removeLandmarkHelper(it);
      } else {
        ++it;
      }
    }
  }
}

template <class Scalar_>
std::vector<TimeCamId> LandmarkDatabase<Scalar_>::getHostKfs() const {
  std::vector<TimeCamId> res;

  for (const auto &kv : observations) res.emplace_back(kv.first);

  return res;
}

// @param tcid 主帧的时间戳(frame id) + 相机id(0 or 1)组成的对象
template <class Scalar_>
std::vector<const Keypoint<Scalar_> *>
LandmarkDatabase<Scalar_>::getLandmarksForHost(const TimeCamId &tcid) const {
  std::vector<const Keypoint<Scalar> *> res;

  // observations.at(tcid)返回的是一个map: std::map<TimeCamId, std::set<KeypointId>>
  // 因此，外层循环遍历map,返回键值对：k, obs
  // k是目标帧标识，由时间戳+相机id组成，obs是目标帧对应的路标点id集合(set)
  for (const auto &[k, obs] : observations.at(tcid))
    // 内层循环: v是路标点id
    ///TODO: 此处会添加重复的路标点id???? 由于多个目标帧会观测到同一个路标点，因此这里有重复添加的可能性??
    for (const auto &v : obs) res.emplace_back(&kpts.at(v));

  return res;
}

/*!
@brief 添加观测路标：step1,给kpts的指定的kpt_id，增加新的观测; step2,给host frame标识的容器observations增加新的路标点
@param tcid_target 由时间戳和相机id构成的变量
@param o           由特征点id和2d坐标构成的变量
@return void
*/ 
template <class Scalar_>
void LandmarkDatabase<Scalar_>::addObservation(
    const TimeCamId &tcid_target, const KeypointObservation<Scalar> &o) {
  auto it = kpts.find(o.kpt_id);
  BASALT_ASSERT(it != kpts.end());

  // step1 更新特征点map容器kpts的以特征点id为key的元素的value，value类型为Keypoint，给其增加新的观测
  // 通俗讲，就是给一个指定路标点标识kpt_id，把当前帧的2d观测点添加进去
  it->second.obs[tcid_target] = o.pos;

  // step2 给主帧id对应的value，添加以目标帧id标识的特征点id
  // 通俗讲，就是添加主帧对应的目标帧的路标点id
  observations[it->second.host_kf_id][tcid_target].insert(it->first);
}

template <class Scalar_>
Keypoint<Scalar_> &LandmarkDatabase<Scalar_>::getLandmark(KeypointId lm_id) {
  return kpts.at(lm_id);
}

template <class Scalar_>
const Keypoint<Scalar_> &LandmarkDatabase<Scalar_>::getLandmark(
    KeypointId lm_id) const {
  return kpts.at(lm_id);
}

template <class Scalar_>
const std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>
    &LandmarkDatabase<Scalar_>::getObservations() const {
  return observations;
}

template <class Scalar_>
const Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar_>>
    &LandmarkDatabase<Scalar_>::getLandmarks() const {
  return kpts;
}

template <class Scalar_>
bool LandmarkDatabase<Scalar_>::landmarkExists(int lm_id) const {
  return kpts.count(lm_id) > 0;
}

template <class Scalar_>
size_t LandmarkDatabase<Scalar_>::numLandmarks() const {
  return kpts.size();
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations() const {
  int num_observations = 0;

  for (const auto &[_, val_map] : observations) {
    for (const auto &[_, val] : val_map) {
      num_observations += val.size();
    }
  }

  return num_observations;
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations(KeypointId lm_id) const {
  return kpts.at(lm_id).obs.size();
}

template <class Scalar_>
typename LandmarkDatabase<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkHelper(
    LandmarkDatabase<Scalar>::MapIter it) {
  auto host_it = observations.find(it->second.host_kf_id);

  for (const auto &[k, v] : it->second.obs) {
    auto target_it = host_it->second.find(k);
    target_it->second.erase(it->first);

    if (target_it->second.empty()) host_it->second.erase(target_it);
  }

  if (host_it->second.empty()) observations.erase(host_it);

  return kpts.erase(it);
}

template <class Scalar_>
typename Keypoint<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkObservationHelper(
    LandmarkDatabase<Scalar>::MapIter it,
    typename Keypoint<Scalar>::MapIter it2) {
  auto host_it = observations.find(it->second.host_kf_id);
  auto target_it = host_it->second.find(it2->first);
  target_it->second.erase(it->first);

  if (target_it->second.empty()) host_it->second.erase(target_it);
  if (host_it->second.empty()) observations.erase(host_it);

  return it->second.obs.erase(it2);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeLandmark(KeypointId lm_id) {
  auto it = kpts.find(lm_id);
  if (it != kpts.end()) removeLandmarkHelper(it);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeObservations(
    KeypointId lm_id, const std::set<TimeCamId> &obs) {
  auto it = kpts.find(lm_id);
  BASALT_ASSERT(it != kpts.end());

  for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
    if (obs.count(it2->first) > 0) {
      it2 = removeLandmarkObservationHelper(it, it2);
    } else
      it2++;
  }

  if (it->second.obs.size() < min_num_obs) {
    removeLandmarkHelper(it);
  }
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

// Note: double specialization is unconditional, b/c NfrMapper depends on it.
//#ifdef BASALT_INSTANTIATIONS_DOUBLE
template class LandmarkDatabase<double>;
//#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class LandmarkDatabase<float>;
#endif

}  // namespace basalt
