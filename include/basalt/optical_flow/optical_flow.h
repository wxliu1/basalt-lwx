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

#include <memory>

#include <Eigen/Geometry>

#include <basalt/utils/vio_config.h>

#include <basalt/io/dataset_io.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/stereographic_param.hpp>
#include <basalt/utils/sophus_utils.hpp>

#include <tbb/concurrent_queue.h>

namespace basalt {

using KeypointId = size_t;

struct OpticalFlowInput {
  using Ptr = std::shared_ptr<OpticalFlowInput>;

  int64_t t_ns; // 用于存放时间戳
  std::vector<ImageData> img_data; // 存放双目图像， 0-左目，1-右目
};

struct OpticalFlowResult {
  using Ptr = std::shared_ptr<OpticalFlowResult>;

  int64_t t_ns; //用于存放时间戳
  // 用vector存放双目的观测，0-左目，1-右目
  // 同时每一个相机的观测用map存储
  // KeypointId特征点ID号,AffineCompact2f特征点在图像上的像素位置，Eigen存放用于描述二维上的平移和旋转
  std::vector<Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>>
      observations;

  std::vector<std::map<KeypointId, size_t>> pyramid_levels;

  OpticalFlowInput::Ptr input_images;
};

class OpticalFlowBase {
 public:
  using Ptr = std::shared_ptr<OpticalFlowBase>;

  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr> input_queue;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>* output_queue = nullptr;

  Eigen::MatrixXf patch_coord;
};

// 光流的构造：1.点到点光流 2.帧到帧光流
class OpticalFlowFactory {
 public:
  static OpticalFlowBase::Ptr getOpticalFlow(const VioConfig& config,
                                             const Calibration<double>& cam);
};
}  // namespace basalt
