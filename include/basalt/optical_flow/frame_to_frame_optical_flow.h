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

#include <thread>

#include <sophus/se2.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/optical_flow/patch.h>

#include <basalt/image/image_pyr.h>
#include <basalt/utils/keypoints.h>

// 2023-11-21
#include <condition_variable>
extern std::condition_variable vio_cv;
// the end.
namespace basalt {

/// Unlike PatchOpticalFlow, FrameToFrameOpticalFlow always tracks patches
/// against the previous frame, not the initial frame where a track was created.
/// While it might cause more drift of the patch location, it leads to longer
/// tracks in practice.
template <typename Scalar, template <typename> typename Pattern>
class FrameToFrameOpticalFlow : public OpticalFlowBase {
 public:
  typedef OpticalFlowPatch<Scalar, Pattern<Scalar>> PatchT;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;

  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

  typedef Sophus::SE2<Scalar> SE2;

  FrameToFrameOpticalFlow(const VioConfig& config,
                          const basalt::Calibration<double>& calib)
      : t_ns(-1), frame_counter(0), last_keypoint_id(0), config(config) {
    
    grid_size_ = config.optical_flow_detection_grid_size;
    max_iterations_ = config.optical_flow_max_iterations;

    if(isSlowVelocity_)
    {
      // grid_size_ = config.optical_flow_detection_grid_size * 2;
      // grid_size_ = config.optical_flow_detection_grid_size + 20;
      grid_size_ = config.optical_flow_detection_grid_size + config.delta_grid_size;
      std::cout << "grid size: " << grid_size_ << std::endl;
    }

    // 设置输入队列大小: 输入队列设置容量  
    input_queue.set_capacity(10);

    // 相机内参
    this->calib = calib.cast<Scalar>();

    // 转换pattern数据类型
    patch_coord = PatchT::pattern2.template cast<float>();

    // 如果视觉前端为双目，构建基础矩阵 // 如果是双目相机 计算两者的E矩阵
    if (calib.intrinsics.size() > 1) {
      Eigen::Matrix4d Ed;
      Sophus::SE3d T_i_j = calib.T_i_c[0].inverse() * calib.T_i_c[1];
      computeEssential(T_i_j, Ed); // 计算基础矩阵
      E = Ed.cast<Scalar>();
    }

    // 开启处理线程
    processing_thread.reset(
        new std::thread(&FrameToFrameOpticalFlow::processingLoop, this));
  }

  ~FrameToFrameOpticalFlow() { processing_thread->join(); }

  // 2023-11-19
  void Reset()
  {
    isReset_ = true;

    // reset_mutex.lock();
    // t_ns = -1;
    // frame_counter = 0;
    // last_keypoint_id = 0;
    // OpticalFlowInput::Ptr curr_frame;
    // while (!input_queue.empty()) input_queue.pop(curr_frame); // drain input_queue
    
    // reset_mutex.unlock();
  }
  // the end.

  virtual void SetZeroVelocity(bool bl) {
    
    if(isZeroVelocity_ != bl)
    {
      isZeroVelocity_ = bl;
      if(isZeroVelocity_)
      {
        // grid_size_ = config.optical_flow_detection_grid_size * 2;
        max_iterations_ = config.optical_flow_max_iterations / 2;
      }
      else
      {
        // grid_size_ = config.optical_flow_detection_grid_size;
        max_iterations_ = config.optical_flow_max_iterations;
      }

      // std::cout << "grid size: " << grid_size_ << std::endl;
      std::cout << "max iterations: " << max_iterations_ << std::endl;
    }

  }

   virtual void SetSlowVelocity(bool bl) {
    if(isSlowVelocity_ != bl)
    {
      isSlowVelocity_ = bl;
      if(isSlowVelocity_)
      {
        // grid_size_ = config.optical_flow_detection_grid_size * 2;
        // grid_size_ = config.optical_flow_detection_grid_size + 20;
        grid_size_ = config.optical_flow_detection_grid_size + config.delta_grid_size;
      }
      else
      {
        grid_size_ = config.optical_flow_detection_grid_size;
      }

      std::cout << "grid size: " << grid_size_ << std::endl;
    }
   }

  void processingLoop() {
    OpticalFlowInput::Ptr input_ptr;

    // processingLoop循环处理部分：拿到一帧的数据指针、处理一帧processFrame
    while (true) {
      if(GetReset()) // 2023-11-20 11:12
      {
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // only imitate program to quit and run
        t_ns = -1;
        frame_counter = 0;
        last_keypoint_id = 0;
        
        OpticalFlowInput::Ptr curr_frame;
        while (!input_queue.empty()) input_queue.pop(curr_frame); // drain input_queue
        OpticalFlowResult::Ptr output_frame;
        while (!output_queue->empty()) output_queue->pop(output_frame); // drain output_queue

        SetReset(false);
        
        std::cout << "reset front end.\n";
        // vio_cv.notify_one();
        reset_();
        
      }
      // reset_mutex.lock(); // 2023-11-19
      input_queue.pop(input_ptr); // 从输入流获得图片

      // 如果获得的图片为空，在输出队列添加一个空的元素
      if (!input_ptr.get()) {
        if (output_queue) output_queue->push(nullptr);
        break;
      }

      // 追踪特征点，添加特征点，剔除外点，将追踪结果push到输出队列
      processFrame(input_ptr->t_ns, input_ptr);
      // reset_mutex.unlock(); // 2023-11-19
    }
  }

  // processFrame这里进行图像金字塔创建+跟踪+剔除特征点
  void processFrame(int64_t curr_t_ns, OpticalFlowInput::Ptr& new_img_vec) {
    
    // 如果图像的数据为空（指针）直接返回
    for (const auto& v : new_img_vec->img_data) {
      if (!v.img.get()) return;
    }
    
    if (t_ns < 0) { // 第一次进入: 第一次处理帧时，t_ns == -1.
      // 开始初始化
      std::cout <<"front end init.\n";
      t_ns = curr_t_ns;

      transforms.reset(new OpticalFlowResult);
      // step l : feature 像素位姿的观测容器transforms初始化
      transforms->observations.resize(calib.intrinsics.size()); // 设置观测容器大小，对于双目来说size就是2
      transforms->t_ns = t_ns; // 时间戳复制

      // step 2 : 设置图像金字塔,注意为金字塔开辟的是一个数组
      pyramid.reset(new std::vector<basalt::ManagedImagePyr<uint16_t>>); // 初始化容器
      // step 2.1 金字塔的个数对应相机的个数
      pyramid->resize(calib.intrinsics.size());

      // step2.2 并行构建金字塔：多线程执行图像金子塔的构建
      // 参数1.指定参数范围 参数2匿名的函数体
      tbb::parallel_for(tbb::blocked_range<size_t>(0, calib.intrinsics.size()), // 迭代范围用数学区间表示是[0, 2)
                        [&](const tbb::blocked_range<size_t>& r) { // [&]表示以引用方式捕获外部作用域的所有变量, [&]表示外部参数传引用，如果没有const修饰时可修改值。
                          for (size_t i = r.begin(); i != r.end(); ++i) {
                            //遍历每一个相机，构建图像金字塔
                            //参数1 : 原始图片, 参数2 : 建金字塔层数
                            // basalt的构建图像金字塔是自己实现的
                            pyramid->at(i).setFromImage(
                                *new_img_vec->img_data[i].img,
                                config.optical_flow_levels);
                          }
                        });

      // step3: 将图像的指针放入到transforms中，用于可视化
      transforms->input_images = new_img_vec;

      // step4: 添加特征点
      addPoints();
      // step5: 使用对极几何剔除外点
      filterPoints();
      // 初始化结束
      // 2023-11-20
      std::cout << "optical flow: cam0 observation count: " << transforms->observations.at(0).size() << std::endl;
      if(transforms->observations.at(0).size() <= 0)
      {
        t_ns = -1;
        if (output_queue && frame_counter % config.optical_flow_skip_frames == 0)
        output_queue->push(transforms);

        return ;
      }
      // the end.
    } else { // 非第一次进入

      // 开始追踪
      // 追踪简要流程：
      // 对每个特征点在金字塔高层级到低层级进行追踪(由粗到精)
      // 追踪当前帧的所有特征点
      // 反向追踪上的才算真正成功的

      // step 1: 更新时间
      t_ns = curr_t_ns; // 拷贝时间戳

      // step 2.1: 更新last image的金子塔
      old_pyramid = pyramid; // 保存上一图像的金字塔

      // step2.2: 构造current image 的金宇塔
      pyramid.reset(new std::vector<basalt::ManagedImagePyr<uint16_t>>); // 重新设置新指针
      pyramid->resize(calib.intrinsics.size());
      tbb::parallel_for(tbb::blocked_range<size_t>(0, calib.intrinsics.size()),
                        [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = r.begin(); i != r.end(); ++i) {
                            pyramid->at(i).setFromImage(
                                *new_img_vec->img_data[i].img,
                                config.optical_flow_levels);
                          }
                        });

      // step3: 追踪特征点
      OpticalFlowResult::Ptr new_transforms; // 新的返回参数
      new_transforms.reset(new OpticalFlowResult);
      new_transforms->observations.resize(calib.intrinsics.size());
      new_transforms->t_ns = t_ns;

      // lwx last left to current left , last right to current right // 对当前帧的和上一帧进行跟踪（左目与左目 右目与右目）
      for (size_t i = 0; i < calib.intrinsics.size(); i++) {
        trackPoints(old_pyramid->at(i), pyramid->at(i),
                    transforms->observations[i],
                    new_transforms->observations[i]);
      }

      // step 4: save track result
      transforms = new_transforms; // 这里transforms重新赋值追踪之后的特征点
      transforms->input_images = new_img_vec;

      // step 5: add feature 增加点
      addPoints(); // 追踪之后，继续提取新的点
      // step 6: 如果是双目相机，使用对极几何剔除外点
      filterPoints(); // 使用双目E剔除点
      // 追踪结束
    }

    // 判断是否定义了输出队列,如果输出队列不为空，将结果push到输出队列
    // 类似vins指定频率发布图像，防止imu相比视觉频率低导致相邻帧没有imu数据，使图像跳帧播放
    if (output_queue && frame_counter % config.optical_flow_skip_frames == 0) {
      output_queue->push(transforms); // 光流结果推送到输出队列里 //- 其实是将光流法追踪的结果推送到了后端状态估计器
      // 这里补充说一点，basalt的前端相当于生产者，生产者线程向队列中添加特征；basalt的后端相当于消费者，消费者线程从队列中获取特征进行处理。
    }

    // 跟踪数量增加
    frame_counter++; // 图像的数目累加
  }

  // trackPoints函数是用来追踪两幅图像的特征点的，输入是 金字塔1, 金字塔2, 1中的点, 输出的是2追踪1的点（即1中的点，被经过追踪之后，得到在图像2中的像素坐标）
  // 这里的1指的是上一帧，那么2就是当前帧；或者1指的是当前帧的左目， 那么2指的是当前帧的右目
  void trackPoints(const basalt::ManagedImagePyr<uint16_t>& pyr_1,
                   const basalt::ManagedImagePyr<uint16_t>& pyr_2,
                   const Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>&
                       transform_map_1, // 1中的点
                   Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>&
                       transform_map_2) const { // 用于返回追踪到的点
    // num_points为1中点的个数
    size_t num_points = transform_map_1.size();

    std::vector<KeypointId> ids;
    Eigen::aligned_vector<Eigen::AffineCompact2f> init_vec;

    ids.reserve(num_points);
    init_vec.reserve(num_points);

    // 1.特征点类型转换map->vector
    for (const auto& kv : transform_map_1) {
      ids.push_back(kv.first); // 1中点的id
      init_vec.push_back(kv.second); // 1中点的信息（在2d图像上的旋转和平移信息）
    }

    // 定义输出结果的容器
    tbb::concurrent_unordered_map<KeypointId, Eigen::AffineCompact2f,
                                  std::hash<KeypointId>>
        result;

    auto compute_func = [&](const tbb::blocked_range<size_t>& range) {
      
      // 遍历每一个特征点
      for (size_t r = range.begin(); r != range.end(); ++r) { // r表示点在vector容器中的序号
        const KeypointId id = ids[r]; // 得到点的id

        // 取出特征点在参考帧或者左目中的像素位置transform_1
        const Eigen::AffineCompact2f& transform_1 = init_vec[r];
        //用transform_1 初始化特征点在当前帧或者右目的位置
        Eigen::AffineCompact2f transform_2 = transform_1;

        // 使用特征点 进行光流正向追踪
        bool valid = trackPoint(pyr_1, pyr_2, transform_1, transform_2);

        if (valid) {
          // 如果正向追踪合法，使用反向追踪，由当前帧追踪参考帧
          Eigen::AffineCompact2f transform_1_recovered = transform_2;

          valid = trackPoint(pyr_2, pyr_1, transform_2, transform_1_recovered);

          if (valid) {
            Scalar dist2 = (transform_1.translation() -
                            transform_1_recovered.translation())
                               .squaredNorm();

            // 判断正向光流和反向光流的误差是否合法，合法则保存结果
            if (dist2 < config.optical_flow_max_recovered_dist2) {
              result[id] = transform_2;
            }
          }
        }
      }
    };

    tbb::blocked_range<size_t> range(0, num_points); // 定义遍历范围，用数学半开半闭区间表示为[0, num_points).

    // 并行（计算）追踪特征点，SPMD（Single Program/Multiple Data 单一的过程中，多个数据或单个程序，多数据）
    tbb::parallel_for(range, compute_func);
    // compute_func(range);

    transform_map_2.clear();
    transform_map_2.insert(result.begin(), result.end());
  }

  // trackPoint函数是追踪一个点
  inline bool trackPoint(const basalt::ManagedImagePyr<uint16_t>& old_pyr,
                         const basalt::ManagedImagePyr<uint16_t>& pyr,
                         const Eigen::AffineCompact2f& old_transform,
                         Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    // AffineCompact2f有旋转和平移成员
    // 投影或仿射变换矩阵，参见Transform类。泛型仿射变换用Transform类表示，其实质是（Dim+1）^2的矩阵
    transform.linear().setIdentity(); //? 这里是将transform(本质是一个矩阵，包含旋转和平移)设为单位矩阵吗，应该仅仅是旋转部分设为单位阵

    /*
     * 类`Transform`表示使用齐次运算的仿射变换或投影变换。 
     *
     * Transform::linear()直接返回变换矩阵中的线性部分it returns the linear part of the transformation.
     * Transform::rotation()返回线性部分中的旋转分量。
     * 但由于线性部分包含 not only rotation but also reflection, shear and scaling
     *
     * Eigen::Transform::linear()用法的补充说明：
     * 例如，一个仿射变换'A'是由一个线性部分'L'和一个平移't'组成的，这样由'A'变换一个点'p'等价于：p' = L * p + t
     * 
     *                 | L   t |
     * 所以变换矩阵 T = |       |    的Linear部分即为左上角的Eigen::Matrix3d旋转矩阵。
     *                 | 0   1 |
     * 
     * 代码中另外一个表示矩阵的方式：
     * [L, t]
     * [0, 1]
     * 
     * 对于本程序来说，Eigen::AffineCompact2f其实就是 Eigen::Transform 的别名，
     * 其本质是3*3的矩阵，左上角2*2 用来表示旋转， 右边2*1表示平移，
     * 平移最根本的目的，在这里是为了表示点在二维像素平面上的坐标。 
     * 
     * 用齐次向量表示：
     * [p'] = [L t] * [p] = A * [p]
     * [1 ]   [0 1]   [1]       [1]
     * 
     * with:
     * 
     * A = [L t]
     *     [0 1]
     * 
     * 所以线性部分对应于齐次矩阵表示的左上角。它对应于旋转、缩放和剪切的混合物。
     * 
     */

    // 从金字塔最顶层到最底层迭代优化
    for (int level = config.optical_flow_levels; level >= 0 && patch_valid;
         level--) {
      
      // 计算尺度
      const Scalar scale = 1 << level; // 相当于 scale = 2 ^ level

      transform.translation() /= scale; // 像素坐标，缩放到对应层

      // 获得patch在对应层初始位置
      PatchT p(old_pyr.lvl(level), old_transform.translation() / scale); // 参考帧的金字塔对应层，以及对应的坐标

      patch_valid &= p.valid;
      if (patch_valid) {
        // Perform tracking on current level
        patch_valid &= trackPointAtLevel(pyr.lvl(level), p, transform);
      }

      // 得到的结果变换到第0层对应的尺度
      transform.translation() *= scale;
    }

    transform.linear() = old_transform.linear() * transform.linear();

    return patch_valid;
  }

  inline bool trackPointAtLevel(const Image<const uint16_t>& img_2,
                                const PatchT& dp,
                                Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    // 指定循环次数，且patch合法
    // int iteration = 0; // added this line for test 2023-12-15.
    for (int iteration = 0;
        //  patch_valid && iteration < config.optical_flow_max_iterations;
         patch_valid && iteration < max_iterations_;
         iteration++) {
      typename PatchT::VectorP res;

      // patch旋转平移到当前帧
      typename PatchT::Matrix2P transformed_pat =
          transform.linear().matrix() * PatchT::pattern2;
      transformed_pat.colwise() += transform.translation();

      // 计算参考恢和当前帧patern对应的像素值差
      patch_valid &= dp.residual(img_2, transformed_pat, res);

      if (patch_valid) {
        // 计算增量，扰动更新
        const Vector3 inc = -dp.H_se2_inv_J_se2_T * res; // 求增量Δx = - H^-1 * J^T * r

        // avoid NaN in increment (leads to SE2::exp crashing)
        patch_valid &= inc.array().isFinite().all();

        // avoid very large increment
        patch_valid &= inc.template lpNorm<Eigen::Infinity>() < 1e6;

        if (patch_valid) {
          transform *= SE2::exp(inc).matrix(); // 更新状态量

          const int filter_margin = 2;

          // 判断更新后的像素坐标是否在图像img_2范围内
          patch_valid &= img_2.InBounds(transform.translation(), filter_margin);
        }
      }
    }

    // std::cout << "num_it = " << iteration << std::endl;

    return patch_valid;
  }

  void addPoints() {

    // step 1 在当前帧第0层检测特征(划分网格，在网格中只保存响应比较好的和以前追踪的)
    Eigen::aligned_vector<Eigen::Vector2d> pts0;

    // 将以前追踪到的点放入到pts0,进行临时保存
    // kv为Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>类型的容器中的一个元素（键值对）
    for (const auto& kv : transforms->observations.at(0)) { // at(0)表示取左目的观测数据
      pts0.emplace_back(kv.second.translation().cast<double>()); // 取2d图像的平移部分，即二维像素坐标
    }

    KeypointsData kd; // 用来存储新检测到的特征点

    // 每个cell的大小默认是50 ， 每个cell提取1个特征点
    // 检测特征点
    // 参数1.图像 参数2.输出特征点容器 参数3,制定cell大小 参数4,每个cell多少特征点 参数5.成功追踪的特征点传递进去
/*
 * comment 2023-12-15.
    detectKeypoints(pyramid->at(0).lvl(0), kd,
                    config.optical_flow_detection_grid_size, 1, pts0);                
*/
    detectKeypoints(pyramid->at(0).lvl(0), kd, grid_size_, 1, pts0);

    Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f> new_poses0,
        new_poses1;

    // step 2 遍历特征点, 每个特征点利用特征点 初始化光流金字塔的初值
    // 添加新的特征点的观测值
    for (size_t i = 0; i < kd.corners.size(); i++) {
      Eigen::AffineCompact2f transform;
      transform.setIdentity(); //旋转 设置为单位阵
      transform.translation() = kd.corners[i].cast<Scalar>(); // 角点坐标，保存到transform的平移部分

      // 特征点转换成输出结果的数据类型map
      transforms->observations.at(0)[last_keypoint_id] = transform; // 键值对来存储特征点，类型为Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>
      new_poses0[last_keypoint_id] = transform;

      last_keypoint_id++; // last keypoint id 是一个类成员变量
    }

    // step 3 如果是有双目的话，使用左目新提的点，进行左右目追踪。右目保留和左目追踪上的特征点
    //如果是双目相机,我们使用光流追踪算法，即计算Left image 提取的特征点在right image图像中的位置
    if (calib.intrinsics.size() > 1) {//相机内参是否大于1
      // 使用左目提取的特征点使用光流得到右目上特征点的位置
      trackPoints(pyramid->at(0), pyramid->at(1), new_poses0, new_poses1);
      // 保存结果，因为是右目的点，因此保存到下标1中
      for (const auto& kv : new_poses1) {
        transforms->observations.at(1).emplace(kv);
      }
    }
  }

  void filterPoints() {
    // 如果相机内参小于2，无法使用双目基础矩阵剔除外点
    if (calib.intrinsics.size() < 2) return;

    // set记录哪些特征点需要被剔除
    std::set<KeypointId> lm_to_remove;

    std::vector<KeypointId> kpid;
    Eigen::aligned_vector<Eigen::Vector2f> proj0, proj1;

    // step l: 获得left image 和 right image 都可以看到的feature
    // 遍历右目特征点，查询是否在左目中被观测
    for (const auto& kv : transforms->observations.at(1)) {
      auto it = transforms->observations.at(0).find(kv.first);

      // 如果在左目中查询到，则把特征点对应ID保存
      if (it != transforms->observations.at(0).end()) {
        proj0.emplace_back(it->second.translation());
        proj1.emplace_back(kv.second.translation());
        kpid.emplace_back(kv.first);
      }
    }

    // step 2: 将feature 反投影为归一化坐标的3d点
    Eigen::aligned_vector<Eigen::Vector4f> p3d0, p3d1;
    std::vector<bool> p3d0_success, p3d1_success;

    calib.intrinsics[0].unproject(proj0, p3d0, p3d0_success);
    calib.intrinsics[1].unproject(proj1, p3d1, p3d1_success);

    // step 3: 使用对极几何剔除外点
    for (size_t i = 0; i < p3d0_success.size(); i++) {
      if (p3d0_success[i] && p3d1_success[i]) {
        const double epipolar_error =
            std::abs(p3d0[i].transpose() * E * p3d1[i]);

        //如果距离大于判定阈值 则不合法
        if (epipolar_error > config.optical_flow_epipolar_error) {
          lm_to_remove.emplace(kpid[i]);
        }
      } else {
        lm_to_remove.emplace(kpid[i]);
      }
    }

    // step 4: 只剔除外点在right image中的观测
    for (int id : lm_to_remove) {
      transforms->observations.at(1).erase(id);
      // 只剔除右目，保留左目，就不会造成左目光流中断
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  int64_t t_ns;

  size_t frame_counter;

  KeypointId last_keypoint_id;

  VioConfig config;
  basalt::Calibration<Scalar> calib;

  OpticalFlowResult::Ptr transforms; // 用于存放第一次提取的全新特征点，或者光流追踪之后的特征点加上提取的新点
  std::shared_ptr<std::vector<basalt::ManagedImagePyr<uint16_t>>> old_pyramid,
      pyramid; // 智能指针指向vector，下标0，表示左目的金字塔，下标1表示右目的金字塔
               // old_pyramid表示上一图像的金字塔，pyramid表示当前图像的金字塔

  Matrix4 E;

  std::shared_ptr<std::thread> processing_thread;

  // 2023-11-19.
  std::mutex reset_mutex; 
  bool isReset_ { false };
  void SetReset(bool bl) { isReset_ =bl; }
  inline bool GetReset() { return isReset_;}
  // the end.

  bool isZeroVelocity_ { false };
  bool isSlowVelocity_ { true };
  std::atomic<int> grid_size_ { 0 };
  std::atomic<int> max_iterations_ { 0 };
};

}  // namespace basalt
