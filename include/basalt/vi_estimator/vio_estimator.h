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

#include <atomic>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/imu_types.h>

#include <functional> // 2023-11-19.

namespace basalt {


struct VioVisualizationData {
  typedef std::shared_ptr<VioVisualizationData> Ptr;

  int64_t t_ns;

  Eigen::aligned_vector<Sophus::SE3d> states;
  Eigen::aligned_vector<Sophus::SE3d> frames;

  Eigen::aligned_vector<Eigen::Vector3d> points;
  std::vector<int> point_ids;

  OpticalFlowResult::Ptr opt_flow_res;

  std::vector<Eigen::aligned_vector<Eigen::Vector4d>> projections;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class VioEstimatorBase {
 public:
  typedef std::shared_ptr<VioEstimatorBase> Ptr;

  VioEstimatorBase()
      : out_state_queue(nullptr),
        out_marg_queue(nullptr),
        out_vis_queue(nullptr) {
    vision_data_queue.set_capacity(10);
    imu_data_queue.set_capacity(300);
    last_processed_t_ns = 0;
    finished = false;
  }

  /*
   * 原子操作std::atomic, 用于多线程并发需要同步时，保护单个变量
   * 为何已经有互斥量了，还要引入std::atomic呢，这是因为互斥量保护的数据范围比较大，我们期望更小范围的保护。
   * 并且当共享数据为一个变量时，原子操作std::atomic效率更高。
   *
   */

  std::atomic<int64_t> last_processed_t_ns;
  std::atomic<bool> finished;

  /*
   * tbb::concurrent_bounded_queue 并发队列，bounded指示该队列是有界限的，不能无限扩张
   * 一般普通的容器，比如vector或者queue，无法保证线程安全，普通的数据结构实现多线程是不安全的，利用tbb的数据结构实现线程安全
   * 因此在使用多线程并发运行时，对这些容器的操作（比如添加、删除等）需要用tbb自带的
   *
   */

  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> vision_data_queue; // 视觉数据并发队列
  tbb::concurrent_bounded_queue<ImuData<double>::Ptr> imu_data_queue;

  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr>*
      out_state_queue = nullptr;
  tbb::concurrent_bounded_queue<MargData::Ptr>* out_marg_queue = nullptr;
  tbb::concurrent_bounded_queue<VioVisualizationData::Ptr>* out_vis_queue =
      nullptr;

  virtual void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                          const Eigen::Vector3d& vel_w_i,
                          const Eigen::Vector3d& bg,
                          const Eigen::Vector3d& ba) = 0;

  virtual void initialize(const Eigen::Vector3d& bg,
                          const Eigen::Vector3d& ba) = 0;

  virtual void maybe_join() = 0;

  virtual inline void drain_input_queues() {
    // Input threads should abort when vio is finished, but might be stuck in
    // full push to full queue. So this can help to drain queues after joining
    // the processing thread.
    while (!imu_data_queue.empty()) {
      ImuData<double>::Ptr d;
      imu_data_queue.pop(d);
    }
    while (!vision_data_queue.empty()) {
      OpticalFlowResult::Ptr d;
      vision_data_queue.pop(d);
    }
  }

  virtual inline void debug_finalize() {}

  virtual Sophus::SE3d getT_w_i_init() = 0;
  virtual void setT_w_i_init(Sophus::SE3d Twi) = 0; // 2023-11-13.
  virtual void Reset() = 0; // 2023-11-19.

  // Legacy functions. Should not be used in the new code. 遗留函数。不应该在新代码中使用
  virtual void setMaxStates(size_t val) = 0;
  virtual void setMaxKfs(size_t val) = 0;

  virtual void addIMUToQueue(const ImuData<double>::Ptr& data) = 0;
  virtual void addVisionToQueue(const OpticalFlowResult::Ptr& data) = 0;

  std::function<void(void)> reset_;
};

class VioEstimatorFactory {
 public:
  static VioEstimatorBase::Ptr getVioEstimator(const VioConfig& config,
                                               const Calibration<double>& cam,
                                               const Eigen::Vector3d& g,
                                               bool use_imu, bool use_double);
};

double alignSVD(const std::vector<int64_t>& filter_t_ns,
                const Eigen::aligned_vector<Eigen::Vector3d>& filter_t_w_i,
                const std::vector<int64_t>& gt_t_ns,
                Eigen::aligned_vector<Eigen::Vector3d>& gt_t_w_i);
}  // namespace basalt
