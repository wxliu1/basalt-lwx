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

#include <basalt/vi_estimator/marg_helper.h>
#include <basalt/vi_estimator/sqrt_keypoint_vo.h>

#include <basalt/optimization/accumulator.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/system_utils.h>
#include <basalt/utils/cast_utils.hpp>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

#include <basalt/linearization/linearization_base.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <fmt/format.h>

#include <chrono>

#include "../wx_system.h"

// 2023-11-13
#if 1
#include "../imu/imu_process.h"
using namespace wx;
extern ImuProcess* g_imu;
#endif
// the end.

// 2023-11-21
#include <condition_variable>
extern std::mutex vio_m;
extern std::condition_variable vio_cv;
// the end.

namespace basalt {

template <class Scalar_>
SqrtKeypointVoEstimator<Scalar_>::SqrtKeypointVoEstimator(
    const basalt::Calibration<double>& calib_, const VioConfig& config_)
    : take_kf(true), // 表明第一帧为关键帧
      frames_after_kf(0),
      initialized(false),
      config(config_),
      lambda(config_.vio_lm_lambda_initial), // LM法中的拉格朗日乘子λ
      min_lambda(config_.vio_lm_lambda_min),
      max_lambda(config_.vio_lm_lambda_max),
      lambda_vee(2) {
  obs_std_dev = Scalar(config.vio_obs_std_dev);
  huber_thresh = Scalar(config.vio_obs_huber_thresh);
  calib = calib_.cast<Scalar>(); // 包含内参、外参、分辨率等

  // Setup marginalization 边缘化设定 （在SqrtBA 中，边缘化是用 QR 分解来完成）
  marg_data.is_sqrt = config.vio_sqrt_marg; // 默认配置项为true.
  marg_data.H.setZero(POSE_SIZE, POSE_SIZE); // POSE_SIZE = 6
  marg_data.b.setZero(POSE_SIZE);

  // Version without prior 仅用于调式和日志输出的目的
  nullspace_marg_data.is_sqrt = marg_data.is_sqrt;
  nullspace_marg_data.H.setZero(POSE_SIZE, POSE_SIZE); // 6 * 6的H
  nullspace_marg_data.b.setZero(POSE_SIZE); // 6 * 1的b

  // prior on pose 位姿先验
  if (marg_data.is_sqrt) {
    marg_data.H.diagonal().setConstant(
        std::sqrt(Scalar(config.vio_init_pose_weight))); // marg矩阵H的对角线设置为常量'初始位姿权重'，初始值为1e8, 开方后为1e4.
  } else {
    marg_data.H.diagonal().setConstant(Scalar(config.vio_init_pose_weight));
  }

  std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n"
            << marg_data.H << std::endl;

  max_states = config.vio_max_states;
  max_kfs = config.vio_max_kfs;

  vision_data_queue.set_capacity(10); // 视觉并发队列设置容量为10
  imu_data_queue.set_capacity(300);
}

template <class Scalar_>
inline bool SqrtKeypointVoEstimator<Scalar_>::GetResetAlgorithm()
{
  return isResetAlgorithm_;
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::SetResetAlgorithm(bool bl)
{
  if(bl && initialized == false) 
  {
    std::cout << "system is not initialized, can't set reset algorithm flag.\n";
    return ;
  }

  isResetAlgorithm_ = bl;
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::SetResetInitPose(bool bl)
{
  isResetInitPose_ = bl;
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::Reset()
{
  if(initialized == true)
  {
    // std::cout << "1 start to reset vio backend" << std::endl;
    // std::unique_lock<std::mutex> lk(vio_m); // 2023-12-21.
    // vio_cv.wait(lk);

    isReset_ = true;
  }
  else
  {
    std::cout << "system is not initialized, can't reset.\n";
  }

  // move to ExcuteReset()
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::ExcuteReset()
{
  // move here for waiting. 2023-11-21.
  // std::unique_lock<std::mutex> lk(vio_m);
  // vio_cv.wait(lk);
  // the end.

  std::cout << "reset backend.\n";
  initialized = false;

  marg_data.is_sqrt = config.vio_sqrt_marg;
  marg_data.H.setZero(POSE_SIZE, POSE_SIZE); // POSE_SIZE = 6
  marg_data.b.setZero(POSE_SIZE);
  if (marg_data.is_sqrt) {
    marg_data.H.diagonal().setConstant(
        std::sqrt(Scalar(config.vio_init_pose_weight))); // marg矩阵H的对角线设置为常量'初始位姿权重'，初始值为1e8
  } else {
    marg_data.H.diagonal().setConstant(Scalar(config.vio_init_pose_weight));
  }

  marg_data.order.abs_order_map.clear();
  marg_data.order.total_size = 0;
  marg_data.order.items = 0;

  // Version without prior 仅用于调式和日志输出的目的
  nullspace_marg_data.is_sqrt = marg_data.is_sqrt;
  nullspace_marg_data.H.setZero(POSE_SIZE, POSE_SIZE); // 6 * 6的H
  nullspace_marg_data.b.setZero(POSE_SIZE); // 6 * 1的b

  nullspace_marg_data.order.abs_order_map.clear();
  nullspace_marg_data.order.total_size = 0;
  nullspace_marg_data.order.items = 0;

  // prior on pose 位姿先验
  if (marg_data.is_sqrt) {
    marg_data.H.diagonal().setConstant(
        std::sqrt(Scalar(config.vio_init_pose_weight))); // marg矩阵H的对角线设置为常量'初始位姿权重'，初始值为1e8
  } else {
    marg_data.H.diagonal().setConstant(Scalar(config.vio_init_pose_weight));
  }

  const PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);
  T_w_i_init = p.getPose();//.template cast<double>()
  // reset first pose with identity matrix for tks.
  
  T_w_i_prev = T_w_i_init;

  frame_poses.clear();
  frame_states.clear();
  prev_opt_flow_res.clear();
  num_points_kf.clear();
  lmdb.Reset();
  kf_ids.clear();
  take_kf = true;
  frames_after_kf = 0;
  last_state_t_ns = -1;
  marg_frame_index = -1;

  lambda = config.vio_lm_lambda_initial;
  min_lambda = config.vio_lm_lambda_min;
  max_lambda = config.vio_lm_lambda_max;
  lambda_vee = 2;

  drain_input_queues();

  if(isResetInitPose_)
  {
    T_w_i_init = SE3();
    // T_w_i_prev = SE3();
    isResetInitPose_ = false;
    if(g_imu)
    g_imu->reset();

    // for another use: call CRos1IO::Reset
    if(resetPublishedOdom_)
    {
      resetPublishedOdom_();
    }
  }

  // TODO:
  // stats_all_.Reset();
  // stats_sums_.Reset();
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::initialize(
    int64_t t_ns, const Sophus::SE3d& T_w_i, const Eigen::Vector3d& vel_w_i,
    const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  UNUSED(vel_w_i);

  initialized = true;
  T_w_i_init = T_w_i.cast<Scalar>();

  last_state_t_ns = t_ns;
  frame_poses[t_ns] = PoseStateWithLin<Scalar>(t_ns, T_w_i_init, true);

  marg_data.order.abs_order_map[t_ns] = std::make_pair(0, POSE_SIZE);
  marg_data.order.total_size = POSE_SIZE;
  marg_data.order.items = 1;

  initialize(bg, ba);
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::SetFirstVisualPose() // 2023-11-18.
{
  // std::cout << std::boolalpha << "initFirstPoseFlag=" << imu->InitFirstPoseFlag() << std::endl;

  if (!g_imu->InitFirstPoseFlag()) {
    g_imu->SetFirstPoseFlag(true);

    Eigen::Matrix3d R_WC0;
/*    
    R_WC0 << 1, 0, 0, 0, 0, 1, 0, -1,
        0;  // 如果是相机模式，第一个位姿绕x轴旋转-90度，使相机z轴对应着世界系y轴，即镜头前方对应世界系y轴方向。
            // 或者直接把这个位姿，加到FrameState的初始位姿T_w_cl(R_WC0,
            // t_WC0)中
*/
    R_WC0 << 0, 0, 1, 0, 1, 0, -1, 0, 0; // 绕y轴逆时针旋转90度，使镜头对着 world frame 的 x 轴 

    Eigen::Matrix3d R_WC0_tmp;
    R_WC0_tmp << 1, 0, 0, 0, 0, 1, 0, -1,0; // 再绕x轴旋转-90度 

    R_WC0 = R_WC0_tmp * R_WC0;

    Vector3d t_WC0 = Vector3d(0, 0, 0);

    const Sophus::SE3d TWC0(R_WC0, t_WC0);
    std::cout << "[lwx] first camera pose: TWC0= \n"
              << TWC0.matrix() << std::endl
              << std::endl;

    // vio->setT_w_i_init(TWC0);
    this->setT_w_i_init(TWC0);

  }

}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::initialize(const Eigen::Vector3d& bg,
                                                  const Eigen::Vector3d& ba) {
  UNUSED(bg);
  UNUSED(ba);

  auto proc_func = [&] { // 用lambda表达式定义线程函数
    OpticalFlowResult::Ptr prev_frame, curr_frame;
    bool add_pose = false;

    while (true) {

      if(GetReset())
      {
        ExcuteReset();
        SetReset(false);
        add_pose = false;
        prev_frame = nullptr;
        SetResetAlgorithm(false);
        std::cout << "reset backend thread.\n";
        wx::TFileSystemHelper::WriteLog("reset algorithm completed.");
      }
      
      // get next optical flow result (blocking if queue empty) 获取光流结果，如果队列为空会阻塞
      vision_data_queue.pop(curr_frame);

      if (config.vio_enforce_realtime) { // 如果强制实时，那么当队列中有新的帧，则丢掉当前的帧，获取最新帧
        // drop current frame if another frame is already in the queue.
        while (!vision_data_queue.empty()) vision_data_queue.pop(curr_frame);
      }

      if (!curr_frame.get()) { // 如果当前帧为空指针，则退出循环
        break;
      }

      // Correct camera time offset (relevant for VIO) 校正相机时间偏移量（与Vio相关） 
      // curr_frame->t_ns += calib.cam_time_offset_ns;

      // this is VO not VIO --> just drain IMU queue and ignore 这里是VO,只要排干imu的队列 并忽略 
      while (!imu_data_queue.empty()) {
        ImuData<double>::Ptr d;
        imu_data_queue.pop(d);
      }

      // 2023-11-18
      if (sys_cfg_.use_imu) {
        // nanosecond to second
        double curTime =  curr_frame->t_ns / 1e9  + sys_cfg_.td;
        bool bl = g_imu->ProcessData(curTime);
        if(!bl) continue ;
        
      }
      else {
      
        SetFirstVisualPose();
      }
      // the end.

      if (!initialized) { // initialized初始为false. 第一帧初始化
        // std::cout << " back end init. " << std::boolalpha << "add_pose=" << add_pose << " prev_frame=" << prev_frame << std::endl;
        std::cout << " back end init. " << " first cam0 observation count: " << curr_frame->observations[0].size() << std::endl;
        #if 1
        char szLog[512] = { 0 };
        sprintf(szLog, "back end init.  first cam0 observation count: %d", curr_frame->observations[0].size());
         wx::TFileSystemHelper::WriteLog(szLog);
        #endif

        last_state_t_ns = curr_frame->t_ns; // 图像时间戳

        frame_poses[last_state_t_ns] =
            PoseStateWithLin(last_state_t_ns, T_w_i_init, true); // 保存第一帧的位姿状态，第一帧的初始姿态T_w_i_init设为单位阵

        marg_data.order.abs_order_map[last_state_t_ns] =
            std::make_pair(0, POSE_SIZE); // key-value: key is timestamp & value is a pair of '(0, POSE_SIZE)'
        marg_data.order.total_size = POSE_SIZE;
        marg_data.order.items = 1;

        nullspace_marg_data.order = marg_data.order;

        std::cout << "Setting up filter: t_ns " << last_state_t_ns << std::endl;
        std::cout << "T_w_i\n" << T_w_i_init.matrix() << std::endl;
        
        #if 0
        // for test 2023-11-20.
        int cam0_num_observations = curr_frame->observations[0].size();
        // if(cam0_num_observations <= 0) continue ;
        // the end.
        #endif

        if (config.vio_debug || config.vio_extended_logging) {
          logMargNullspace();
        }

        initialized = true;
      }

      if (prev_frame) {
        add_pose = true;
      }

      //measure(curr_frame, add_pose); // 测量: 后端优化的入口
      bool bl = measure(curr_frame, add_pose); // 测量: 后端优化的入口 // modified 2023-11-20
      prev_frame = curr_frame;
#if 0 // 2023-12-21.
      // 2023-11-20
      // if(isResetAlgorithm_)
      if(!bl)
      {
        isResetAlgorithm_ = false;
        add_pose = false;
        prev_frame = nullptr;
        #if 0
        std::chrono::milliseconds dura(100);
        // std::chrono::milliseconds dura(40);
        std::this_thread::sleep_for(dura);
        while (!vision_data_queue.empty()) vision_data_queue.pop(curr_frame);
        std::cout << "reset backend thread.\n";
        #endif

        // std::unique_lock<std::mutex> lk(vio_m);
        // vio_cv.wait(lk);
        std::cout << "reset backend thread.\n";
      }
      
      // the end.
#endif      
    }

    if (out_vis_queue) out_vis_queue->push(nullptr);
    if (out_marg_queue) out_marg_queue->push(nullptr);
    if (out_state_queue) out_state_queue->push(nullptr);

    finished = true;

    std::cout << "Finished VIOFilter " << std::endl;
  };

  processing_thread.reset(new std::thread(proc_func));
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::addVisionToQueue(
    const OpticalFlowResult::Ptr& data) {
  vision_data_queue.push(data);
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::addIMUToQueue(
    const ImuData<double>::Ptr& data) {
  UNUSED(data);
}

// @param[in] opt_flow_meas 当前帧的光流结果
// @param[in] add_pose 第一个帧时为false, 后面均为true.
// @return true
template <class Scalar_>
bool SqrtKeypointVoEstimator<Scalar_>::measure(
    const OpticalFlowResult::Ptr& opt_flow_meas, const bool add_pose) {
  stats_sums_.add("frame_id", opt_flow_meas->t_ns).format("none"); // 执行状态
  Timer t_total;

  //  std::cout << "=== measure frame " << opt_flow_meas->t_ns << "\n";
  //  std::cout.flush();

  // TODO: For VO there is probably no point to non kfs as state in the sliding
  // window... Just do pose-only optimization and never enter them into the
  // sliding window.
  // TODO: 对于VO，可能没有指向非kfs作为滑动窗口的状态…只是只做姿态优化，永远不要将它们输入滑动窗口。

  // TODO: If we do pose-only optimization first for all frames (also KFs), this
  // may also speed up the joint optimization.
  // TODO: 如果我们首先对所有帧（也包括KFs）进行仅姿态优化，这也会加速联合优化。 

  // TODO: Moreover, initial pose-only localization may allow us to project
  // untracked landmarks and "rediscover them" by optical flow or feature /
  // patch matching.
  // TODO: 此外，初始的仅姿态定位可能允许我们投影未被跟踪的路标，并通过光流或特征补丁匹配重新发现它们。

  // 翻译过来，大概意思指：所有帧都可以做姿态优化，而只有关键帧才进滑窗。

  if (add_pose) {
    // The state for the first frame is added by the initialization code, but
    // otherwise we insert a new pose state here. So add_pose is only false
    // right after initialization.
    // 第一帧的状态是由初始化代码添加的，否则我们将在这里插入一个新的姿态状态。所以add_pose只是在初始化时才是假的。（即后面add_pose都为true.）

    const PoseStateWithLin<Scalar>& curr_state =
        frame_poses.at(last_state_t_ns);

    last_state_t_ns = opt_flow_meas->t_ns;

    PoseStateWithLin next_state(opt_flow_meas->t_ns, curr_state.getPose());
    frame_poses[last_state_t_ns] = next_state; // 保存的是当前帧的时间戳，但是对应的是上一帧的状态
  } //if (add_pose)

  // invariants: opt_flow_meas->t_ns is last pose state and equal to
  // last_state_t_ns
  // 不变量：opt_flow_meas->t_ns是最后一个姿态状态，并且等于last_state_t_ns 
  BASALT_ASSERT(opt_flow_meas->t_ns == last_state_t_ns);
  BASALT_ASSERT(!frame_poses.empty());
  // 由于frame_poses是保存帧的时间戳、位姿的map, 当程序未重启而连续第二次播放数据集时，
  // 以时间戳为key存放对应位姿的元素，就不一定是map容器类型frame_poses的最后一个元素了。
  // 原因在于，帧的时间戳已经存在于map了，因此frame_poses[last_state_t_ns] = next_state;只是修改value,并没有增加新的键值对。
  BASALT_ASSERT(last_state_t_ns == frame_poses.rbegin()->first); // tmp comment on 2023-12-18.

  // save results
  prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas; // 保存当前帧的光流结果

  constexpr int MIN_OBSERVATIONS = 6;

  // 2023-11-15.
  int cam0_num_observations = opt_flow_meas->observations[0].size();
  if(cam0_num_observations < MIN_OBSERVATIONS)
  {
    std::cout << "cam0 observation count: " << opt_flow_meas->observations[0].size() << std::endl;
    #if 1
    char szLog[512] = { 0 };
    sprintf(szLog, "cam0 observation count: %d", opt_flow_meas->observations[0].size());
    wx::TFileSystemHelper::WriteLog(szLog);
    #endif
  }
  
  // the end.
  

  // For feature tracks that exist as landmarks, add the new frames as
  // additional observations. For every host frame, compute how many of it's
  // landmarks are tracked by the current frame: this will help later during
  // marginalization to remove the host frames with a low number of landmarks
  // connected with the latest frame. For new tracks, remember their ids for
  // possible later landmark creation.
/*
  对于作为路标而存在的特征跟踪，添加新的帧作为额外的观察。
  对于每个主导帧，计算当前帧跟踪了多少它的路标数量：这将有助于稍后在边缘化期间删除与最新帧连接的路标数量较少的主导帧。
  对于新的跟踪，记住他们的id，为以后可能的路标创造。
  
  * 补充说明（解读）一下 on 2024-2-23： 
  * 1、这里把滑窗里面的每个帧作为主导帧（host frame)，把当前帧或者说新帧作为目标帧（target frame);
  * 2、计算每个主导帧的路标点，被当前帧跟踪的数量，作为依据，稍后，与最新帧连接的路标数量较少的主导帧将会被边缘化;
  * 3、对于新的跟踪，作为稍后可能的路标创键，记住他们的id.(原因在于，只有能经过三角化初始化了逆深度的点，才会作为路标点保留下来存入lmdb)
*/
  int connected0 = 0;                           // num tracked landmarks cam0 用于统计相机0跟踪的路标个数
  std::map<int64_t, int> num_points_connected;  // num tracked landmarks by host 用于统计每个主帧追踪的特征点的个数
  std::unordered_set<int> unconnected_obs0;     // new tracks cam0 左目新增加的点集
  std::vector<std::vector<int>> connected_obs0(
      opt_flow_meas->observations.size()); // 对于双目这个vector的size是2.

  // step1 遍历当前帧左右目的观测，看是否有特征点存在于数据库
  for (size_t i = 0; i < opt_flow_meas->observations.size(); i++) { // 遍历当前帧的相机：i=0对应cam0, i=1对应cam1.
    TimeCamId tcid_target(opt_flow_meas->t_ns, i);

    for (const auto& kv_obs : opt_flow_meas->observations[i]) { // 遍历当前帧的相机cam_i的特征点
      int kpt_id = kv_obs.first; // 特征点id
      
      // 特征点在路标数据库中是否存在， 即判断路标点是否跟踪成功
      if (lmdb.landmarkExists(kpt_id)) { 
        const TimeCamId& tcid_host = lmdb.getLandmark(kpt_id).host_kf_id;

        KeypointObservation<Scalar> kobs;
        kobs.kpt_id = kpt_id;
        kobs.pos = kv_obs.second.translation().cast<Scalar>(); // kv_obs.second的类型是Eigen::AffineCompact2f
                                                               // 包含平移、旋转和缩放
        //如果跟踪成功， 则添加新帧的观测到lmdb
        lmdb.addObservation(tcid_target, kobs);
        // obs[tcid_host][tcid_target].push_back(kobs);

        num_points_connected[tcid_host.frame_id]++; // 主帧对应的特征点在新帧中被追踪，个数加1
        connected_obs0[i].emplace_back(kpt_id); // 将路标点id保存到连接vector中

        if (i == 0) connected0++; // 统计相机0跟踪的路标个数
      } else {
        if (i == 0) {
          unconnected_obs0.emplace(kpt_id); // cam0中新提取的FAST角点加入到集合中（在路标数据库中查找不到的路标点）
                                            // 那么算法的第一帧上的所有cam0的点，都会被添加到unconnected_obs0中。
        }
      }
    } // for (const auto& kv_obs
  } // for (size_t i = 0

  // 添加关键帧策略判断：如果当前帧cam0(左目) 观测到的3d点与未能观测到3d点的特征点数比值小于一定的阈值。
  // vio_new_kf_keypoints_thresh默认为0.7，vio_min_frames_after_kf默认为5，
  // 当前帧cam0跟踪的点的个数与当前帧总的点的个数的比值如果小于0.7，
  // 并且从上一个关键帧之后的帧的数量大于5，则当前帧应该成为关键帧。
  if (Scalar(connected0) / (connected0 + unconnected_obs0.size()) <
          Scalar(config.vio_new_kf_keypoints_thresh) &&
      frames_after_kf > config.vio_min_frames_after_kf)
    take_kf = true; // 如果当前帧应该成为关键帧，那么take_kf为true.

  //if (config.vio_debug) {
  if (config.vio_debug || cam0_num_observations < MIN_OBSERVATIONS) {
    std::cout << "connected0: " << connected0 << " unconnected0: "
              << unconnected_obs0.size() << std::boolalpha << " take_kf: " << take_kf << std::endl;
  }

  // step2 通过连接的特征点（其实就是追踪到的点）优化单帧（当前帧）位姿 //?这里面具体如何优化值得细看。
  BundleAdjustmentBase<Scalar>::optimize_single_frame_pose(
      frame_poses[last_state_t_ns], connected_obs0); // [out]参数1：当前帧的位姿状态；[in]参数2：连接的特征点的ids.

  // step3 如果添加关键帧，则把当前帧左目新增加的点，先通过三角测量计算逆深度，然后加入到landmark数据库，否则只统计非连续关键帧的个数
  if (take_kf) {
    // For keyframes, we don't only add pose state and observations to existing
    // landmarks (done above for all frames), but also triangulate new
    // landmarks.
    // 对于关键帧，我们不仅向现有路标添加位姿状态和观察（上面为所有帧做），而且还三角化新的路标。 
    // 对于关键帧，我们不仅为上面所有帧的现有路标添加姿态状态和观察结果，而且还三角化新的路标。

    // Triangulate new points from one of the observations (with sufficient
    // baseline) and make keyframe for camera 0
    // 从其中一个观测值中三角化新的点（有足够的基线），并为相机0创建关键帧 
    // 从一个有足够基线的观测值中三角化新的点，并为相机0设置关键帧

    take_kf = false;
    frames_after_kf = 0; // 重置非关键帧的统计
    kf_ids.emplace(last_state_t_ns); // 添加当前帧的时间戳，以时间戳为kf的id

    TimeCamId tcidl(opt_flow_meas->t_ns, 0);

    int num_points_added = 0;
    for (int lm_id : unconnected_obs0) { // 遍历未连接的特征点id，即新提取的FAST角点 //![2024-3-1] 后半句未必准确。
      // Find all observations
      std::map<TimeCamId, KeypointObservation<Scalar>> kp_obs;

      // ??? review时发现：这里有必要遍历prev_opt_flow_res吗？？因为当前帧的新提取的点，肯定只能在当前左帧和右帧中才能查找的到
      //![2024-3-1] 再次review发现:lm_id不一定是当前帧的新提取的点，还有可能是之前某个帧新提取的点，只不过未能三角化，因而没有存储到lmdb
      // 对于当前帧的一个未连接的特征点：
      // 先按光流结果遍历，再按相机遍历，查找特征点（相当于滑窗每个帧，查找2次特征点）
      for (const auto& kv : prev_opt_flow_res) { // 遍历所有帧的光流结果：kv是时间戳和光流结果的键值对。
        for (size_t k = 0; k < kv.second->observations.size(); k++) { // 遍历相机（0-左目，1-右目）
          auto it = kv.second->observations[k].find(lm_id); // 根据key(特征点id)查找特征点
          if (it != kv.second->observations[k].end()) {
            TimeCamId tcido(kv.first, k); // 时间戳和相机序号构建TimeCamId对象

            KeypointObservation<Scalar> kobs;
            kobs.kpt_id = lm_id;
            kobs.pos = it->second.translation().template cast<Scalar>(); // it 是map类型的迭代器，是特征点id和紧凑仿射变换的键值对

            // obs[tcidl][tcido].push_back(kobs);
            kp_obs[tcido] = kobs; // kp_obs保存的是滑窗中所有可以观测到id为lm_id的特征点的时间戳和平移信息。
          }
        }
      }

      // 通过三角化恢复当前帧cam0的特征点的逆深度，并将成功恢复逆深度的3d点加入到landmark数据库，最后将视觉测量关系加入到数据库。
      // triangulate 三角测量
      bool valid_kp = false;
      // vio_min_triangulation_dist默认0.05或者0.07，最小三角测量距离
      const Scalar min_triang_distance2 =
          Scalar(config.vio_min_triangulation_dist *
                 config.vio_min_triangulation_dist);
      for (const auto& kv_obs : kp_obs) {
        if (valid_kp) break; // 如果该点变成有效，则终止遍历
        TimeCamId tcido = kv_obs.first;

        // 当前帧（目标帧）左目的特征id为lm_id的点的像素坐标
        const Vec2 p0 = opt_flow_meas->observations.at(0)
                            .at(lm_id)
                            .translation()
                            .cast<Scalar>();
        // 滑窗主导帧的对应相机的特征id为lm_id的点的像素坐标                     
        const Vec2 p1 = prev_opt_flow_res[tcido.frame_id]
                            ->observations[tcido.cam_id]
                            .at(lm_id)
                            .translation()
                            .template cast<Scalar>();

        Vec4 p0_3d, p1_3d; // 反（逆）投影恢复3d坐标。将像素坐标转换至相机坐标（前3维）
        bool valid1 = calib.intrinsics[0].unproject(p0, p0_3d);
        bool valid2 = calib.intrinsics[tcido.cam_id].unproject(p1, p1_3d);
        if (!valid1 || !valid2) continue;

        // 计算两帧之间的相对运动：当前帧(target frame)的位姿的逆 * 滑窗的关键帧(host frame)的位姿, 得到T_th
        SE3 T_i0_i1 = getPoseStateWithLin(tcidl.frame_id).getPose().inverse() *
                      getPoseStateWithLin(tcido.frame_id).getPose();
        // 对于纯VO的外参来说 calib.T_i_c[0]是左目到左目的外参，其实是单位阵；
        // calib.T_i_c[1]是右目到左目的外参
        SE3 T_0_1 =
            calib.T_i_c[0].inverse() * T_i0_i1 * calib.T_i_c[tcido.cam_id];

        // 实验还发现，如果vio_min_triangulation_dist设置的值超过基线则选不到点。
        // 对于第一帧来说，做三角测量，应该是通过右目成功跟踪匹配的点来完成的吧
        if (T_0_1.translation().squaredNorm() < min_triang_distance2) continue; // 如果基线不够，则跳过
        

        // 通过三角化恢复当前帧cam0的特征点的逆深度:
        // 根据相机的运动（R和t, 对于单目是对极几何求出，而双目并不是）和归一化坐标，通过三角测量，估计地图点的深度
        // 对点进行三角测量并返回齐次表示，p0_triangulated的前3维是单位长度方向向量，最后一维是逆深度
        Vec4 p0_triangulated = triangulate(p0_3d.template head<3>(),
                                           p1_3d.template head<3>(), T_0_1);

        // 将成功恢复逆深度的3d点加入到landmark数据库
        if (p0_triangulated.array().isFinite().all() &&
            p0_triangulated[3] > 0 && p0_triangulated[3] < Scalar(3.0)) {
          // 将当前帧的点存入特征点数据库
          Keypoint<Scalar> kpt_pos;
          kpt_pos.host_kf_id = tcidl; // 主帧id为：当前帧的时间戳（作为帧id）+ 相机序号0（作为相机id）构成。
          kpt_pos.direction =
              StereographicParam<Scalar>::project(p0_triangulated);
          kpt_pos.inv_dist = p0_triangulated[3];
          lmdb.addLandmark(lm_id, kpt_pos); /// 以特征点id为key，存入路标点数据库；value 是主帧id + 方位向量 + 逆深度
                                            ///- 从这里看出，VINS是首次提取FAST角点的帧是host frame.而basalt是首次三角化成功的路标对应的帧是 host frame.

          num_points_added++; // 恢复逆深度的点数量加1
          valid_kp = true;
        }
      }

      if (valid_kp) {
        // 将视觉测量关系加入到数据库。
        for (const auto& kv_obs : kp_obs) {
          // kv_obs.first为TimeCamId类型，kv_obs.second是KeypointObservation类型，包含特征点id,以及像素坐标
          lmdb.addObservation(kv_obs.first, kv_obs.second);
        }

        // TODO: non-linear refinement of landmark position from all
        // observations; may speed up later joint optimization
      }
    }

    /*
     * 关于上述未连接的路标点的处理过程，补充说明一下：
     * 1、对于未连接的每一个点（在lmdb中查找不到），遍历prev_opt_flow_res，查找lm_id相同的路标点，
     * 2、如查找到了，则把帧的timestamp +CamId作为key,以及lm_id和pos作为value, 尽数存储到kp_obs.
     * 3、遍历kp_obs，与当前帧的光流结果opt_flow_meas中的lm_id的路标点进行三角化，如果三角化成功，
     * 4、以当前帧作为host frame，将成功恢复逆深度的3d点加入到landmark数据库，即添加到kpts.
     * 5、将观测到lm_id的所有帧的路标点加入到lmdb的observations, 同时也会往kpts[lm_Id].obs中添加观测。
     * 6、回到步骤1循环处理，直到所有未连接的点全部处理完毕，只有初始化了逆深度的点才会添加到lmdb.
     */

    num_points_kf[opt_flow_meas->t_ns] = num_points_added; // 统计当前kf新添加的特征点的个数
  } else {
    // take_kf == false
    frames_after_kf++; // 统计自最后一个关键帧之后连续的非关键帧的个数。
  }

  // step4 遍历特征点数据库，判断每一个特征点如果不在当前帧的观测里面，则添加到lost_landmaks
  std::unordered_set<KeypointId> lost_landmaks;
  if (config.vio_marg_lost_landmarks) { // vio_marg_lost_landmarks默认为true.
    for (const auto& kv : lmdb.getLandmarks()) { // kv是Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>>类型的迭代器
      bool connected = false;
      for (size_t i = 0; i < opt_flow_meas->observations.size(); i++) {
        if (opt_flow_meas->observations[i].count(kv.first) > 0) // kv.first 是特征点id.
          connected = true;
      }
      if (!connected) {
        lost_landmaks.emplace(kv.first); // 在当前帧的观测数据中找不到，则加到集合lost_landmaks
      }
    }
  }

  // step5 优化和边缘化
  // lost_landmaks是一个集合，lmdb中的所有landmark,不在当前帧的观测里面，则会加入该集合
  // 简言之，lmdb中的所有landmarks, 凡是在当前帧中观察不到，就添加进lost_landmaks
  //optimize_and_marg(num_points_connected, lost_landmaks);
  int converged = optimize_and_marg(num_points_connected, lost_landmaks);
/*
  if(!converged)
  {
    std::cout << std::boolalpha << "converged=" << converged << std::endl;
  }
*/

  // test 2023-11-17
  //if(cam0_num_observations < 6)
  //{
    // std::cout << std::boolalpha << "converged=" << converged 
    //   << " lost_landmaks: " << lost_landmaks.size() << std::endl;

    // reset_(); // 2023-11-19.
  //}
  // the end.

#if 1

  // 2023-11-21
  bool isBigTranslation = false;
  const PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);
  const auto delta_tf = T_w_i_prev.inverse() * p.getPose();
  Vector3d P = delta_tf.template cast<double>().translation();
  double xdiff = P.x();
  double ydiff = P.y();
  // double zdiff = P.z();
  constexpr double dt_threshold = 6.0;//5.0; // 5.0; // 1.0
  double norm = sqrt(xdiff * xdiff + ydiff * ydiff);
  //if(fabs(xdiff) > dt_threshold || fabs(ydiff) > dt_threshold) // neglect z axis
  if(norm > dt_threshold) // neglect z axis
  {
    std::cout << "[lwx] BIG translation.\n";
    isBigTranslation = true;

    #if 1
    wx::TFileSystemHelper::WriteLog("BIG translation.");
    #endif
  }
  // the end.

  // std::cout << std::boolalpha << "converged=" << converged << std::endl;

#ifdef _USE_IMU_POSE_ // just for tks.  
  // 2023-11-13
  if (sys_cfg_.use_imu) 
  {
    //  if(converged)
    //if(converged || cam0_num_observations >= 6)
    if(!isBigTranslation && ((converged == 1) || cam0_num_observations >= MIN_OBSERVATIONS) && (converged != 2))
    {
      g_imu->SetUseImuPose(false);
    }
    else
    // if(converged == false || cam0_num_observations < 6)
    // if(cam0_num_observations < 6)
    {
      std::cout << std::boolalpha << "converged=" << converged << std::endl;

      if (g_imu->GetSolverFlag() == INITIAL) 
      {
        g_imu->SetUseImuPose(false);
        g_imu->clearState();

        // if(is_not_horizontal_reboot == true)
        // {
        //   g_imu->SetFirstPoseFlag(true);
        // }
        // else
        // {  
        //   pub_odom_.Reset();
        // }
      }
      else
      {
        g_imu->SetUseImuPose(true);

        #if 1
        char szLog[256] = { 0 };
        sprintf(szLog, "converged=%d, set use imu pose = true.", converged);
        wx::TFileSystemHelper::WriteLog(szLog);
        #endif 
      }

      if (!g_imu->UseImuPose()) 
      {
        last_processed_t_ns = last_state_t_ns;
        stats_sums_.add("measure", t_total.elapsed()).format("ms");

        // return false;
        return true; // 2023-11-20.
      }
    }

    const PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);
    // g_imu->UpdateImuPose(p.getPose().template cast<double>());
    if(isBigTranslation) // 2023-11-21.
    {
      g_imu->UpdateImuPose(T_w_i_prev.template cast<double>());
    }
    else
    {
      g_imu->UpdateImuPose(p.getPose().template cast<double>());
    }
    g_imu->NonlinearOptimization(last_state_t_ns / 1e9);
    g_imu->slideWindow(marg_frame_index); // 'marg_frame_index' should be a index of removing frame 2023-11-14.

  }


  if(g_imu->UseImuPose())
  {
    std::cout << "CalcImuPose \n";
    wx::TFileSystemHelper::WriteLog("CalcImuPose");
    Sophus::SE3d tf_new;
    constexpr bool camToWorld = true;
    g_imu->CalcImuPose(tf_new, camToWorld);
    SE3 Twi = tf_new.template cast<Scalar>();
    PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);
    p.setPose(Twi);
    
  }
#endif
  
  // the end.
#endif


/*
 * 2023-12-8
 * comment this section, because i want to take some variables with out_state_queue like confidence coefficient 
 * 
  if (out_state_queue) {
    // 取出当前帧的状态量（时间戳，位姿，速度，bg, ba）存入输出状态队列，用于在pangolin上的显示
    const PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);

    T_w_i_prev = p.getPose();// 2023-11-21 10:07

    typename PoseVelBiasState<double>::Ptr data(new PoseVelBiasState<double>(
        p.getT_ns(), p.getPose().template cast<double>(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()));

    out_state_queue->push(data);
  }
*/
  // self-define these two variables by wxliu
  int nTrackedPoints = 0;
  int nOptFlowPatches = 0;

  if (out_vis_queue) {
    // 用于在pangonlin上显示追踪的特征点和光流patch
    VioVisualizationData::Ptr data(new VioVisualizationData);

    data->t_ns = last_state_t_ns;

    BASALT_ASSERT(frame_states.empty());

    for (const auto& kv : frame_poses) {
      data->frames.emplace_back(kv.second.getPose().template cast<double>());
    }

    get_current_points(data->points, data->point_ids);

    data->projections.resize(opt_flow_meas->observations.size());
    computeProjections(data->projections, last_state_t_ns); //? 计算投影, 这里面也值得看如何计算的

    data->opt_flow_res = prev_opt_flow_res[last_state_t_ns];

    out_vis_queue->push(data);

    // add two lines below to save results.
    nTrackedPoints = data->projections[0].size();
    nOptFlowPatches = data->opt_flow_res->observations[0].size();
  }

  if(isBigTranslation || ((converged != 1) && cam0_num_observations < MIN_OBSERVATIONS) || (converged == 2) || (nTrackedPoints < 6)  || g_imu->UseImuPose())
  {
    PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);
    p.setPose(T_w_i_prev);
  }

  // out_state_queue move here 2023-12-8
  if (out_state_queue) {
    // 取出当前帧的状态量（时间戳，位姿，速度，bg, ba）存入输出状态队列，用于在pangolin上的显示
    const PoseStateWithLin<Scalar>& p = frame_poses.at(last_state_t_ns);

    T_w_i_prev = p.getPose();// 2023-11-21 10:07

    // put our confidence in it. 2023-12-8
    Eigen::Vector3d bias_accel;
    bias_accel.x() = nTrackedPoints;
    bias_accel.y() = nOptFlowPatches;
    bias_accel.z() = g_imu->UseImuPose()? 1: 0;
    
    // the end.

    typename PoseVelBiasState<double>::Ptr data(new PoseVelBiasState<double>(
        p.getT_ns(), p.getPose().template cast<double>(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        // Eigen::Vector3d::Zero()));
        bias_accel));

    out_state_queue->push(data);
  }
  // the end.

  last_processed_t_ns = last_state_t_ns;

  stats_sums_.add("measure", t_total.elapsed()).format("ms"); // 统计measure函数消耗的时间

  // if(g_imu->UseImuPose()) // 2023-11-20 10:22
  // if(nTrackedPoints < 6 || g_imu->UseImuPose()) // test on 2023-12-20.
  if(isBigTranslation || ((converged != 1) && cam0_num_observations < MIN_OBSERVATIONS) || (converged == 2) || (nTrackedPoints < 6)  || g_imu->UseImuPose())
  {
    // std::cout << std::boolalpha << "begin reset algorithm." << " tracked points: " << nTrackedPoints 
    //   << " Is use imu pose? " << g_imu->UseImuPose() << std::endl;

    // 
    char szLog[512] = { 0 };
    sprintf(szLog, "begin reset algorithm. converged=%d, cam0_num_observations=%d, tracked points=%d, isBigTranslation=%d, UseImuPose=%d", 
      converged, cam0_num_observations, nTrackedPoints, isBigTranslation, g_imu->UseImuPose());
    wx::TFileSystemHelper::WriteLog(szLog);

    std::cout << std::boolalpha << szLog << std::endl;

    reset_(); 
    return false;
  }

  return true;
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::logMargNullspace() {
  nullspace_marg_data.order = marg_data.order;
  if (config.vio_debug) {
    std::cout << "======== Marg nullspace ==========" << std::endl;
    stats_sums_.add("marg_ns", checkMargNullspace());
    std::cout << "=================================" << std::endl;
  } else {
    stats_sums_.add("marg_ns", checkMargNullspace());
  }
  stats_sums_.add("marg_ev", checkMargEigenvalues());
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVoEstimator<Scalar_>::checkMargNullspace() const {
  return checkNullspace(nullspace_marg_data, frame_states, frame_poses,
                        config.vio_debug);
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVoEstimator<Scalar_>::checkMargEigenvalues() const {
  return checkEigenvalues(nullspace_marg_data, false);
}

/*!
@param num_points_connected 每个主帧在当前帧中被追踪的特征点的个数
@param lost_landmaks lmdb中不被当前帧观察到的landmarks.
*/

template <class Scalar_>
bool SqrtKeypointVoEstimator<Scalar_>::marginalize(
    const std::map<int64_t, int>& num_points_connected,
    const std::unordered_set<KeypointId>& lost_landmaks) {
  // 对于vo,frame_states为空，vo模式下位姿存放在frame_poses容器中   
  BASALT_ASSERT(frame_states.empty());

  Timer t_total;

  if (true) {
    // Marginalize

    // remove all frame_poses that are not kfs and not the current frame
    std::set<int64_t> non_kf_poses;
    for (const auto& kv : frame_poses) {
      // 如果kv的key时间戳不在kf_ids中，并且不等于当前帧的时间戳，则添加进non_kf_poses中
      if (kf_ids.count(kv.first) == 0 && kv.first != last_state_t_ns) {
        non_kf_poses.emplace(kv.first);
      }
    }

    // step1 删除除了当前帧之外的非关键帧
    // Notice:如果被marg的帧为空，那么marginalize()就只是仅仅删除非关键帧，就返回了。
    // 意味着，只有kf_ids.size() > max_kfs, 才会真正的marg。
    for (int64_t id : non_kf_poses) {
      frame_poses.erase(id);
      lmdb.removeFrame(id);
      prev_opt_flow_res.erase(id);
    }

    auto kf_ids_all = kf_ids; // 备份当前所有关键帧id
    // step2 挑选需要被marg的关键帧
    // 如果kf_ids的数量大于sliding window size,则选择用于边缘化的帧序号，存储到kfs_to_marg
    std::set<FrameId> kfs_to_marg;
    // max_kfs为最大关键帧数，根据配置项来设定（比如为5）
    while (kf_ids.size() > max_kfs) {
      int64_t id_to_marg = -1; // 用于保存即将被边缘化的frame id,其实就是以纳秒为单位的时间戳

      // starting from the oldest kf (and skipping the newest 2 kfs), try to
      // find a kf that has less than a small percentage of it's landmarks
      // tracked by the current frame
      if (kf_ids.size() > 2) {
        // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is valid
        auto end_minus_2 = std::prev(kf_ids.end(), 2);

        // 从最老的关键帧开始（并且跳过最新的2个关键帧），遍历到次新关键帧之前的一帧
        // 即：从最老的关键帧开始，一直遍历到倒数第3个关键帧结束
        for (auto it = kf_ids.begin(); it != end_minus_2; ++it) {
          // 如果该关键帧的路标，没有被当前帧跟踪，或者被当前帧跟踪的路标点的个数与该关键帧新添加的特征点的个数比值小于一个小的百分比
          // 则保存该关键帧的时间戳(frame id)
          if (num_points_connected.count(*it) == 0 ||
              (num_points_connected.at(*it) / Scalar(num_points_kf.at(*it)) <
               Scalar(config.vio_kf_marg_feature_ratio))) {
            id_to_marg = *it;
            break;
          }
        }
      }

      // Note: This score function is taken from DSO, but it seems to mostly
      // marginalize the oldest keyframe. This may be due to the fact that
      // we don't have as long-lived landmarks, which may change if we ever
      // implement "rediscovering" of lost feature tracks by projecting
      // untracked landmarks into the localized frame.
      // 注意：下面这个评分函数取自dso，但它似乎大多边缘化了最古老的关键帧。
      // 这可能是由于我们没有那么长命的路标，如果我们曾经通过将未跟踪的路标投影到局部帧中来重新发现丢失的特征跟踪，就会改变这一点。

      // 如果上述遍历，未找到需要被边缘化的frame id. 则用以下评分来找到marg. frame id
      if (kf_ids.size() > 2 && id_to_marg < 0) {
        // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is valid
        auto end_minus_2 = std::prev(kf_ids.end(), 2);

        int64_t last_kf = *kf_ids.crbegin();
        Scalar min_score = std::numeric_limits<Scalar>::max();
        int64_t min_score_id = -1;

        for (auto it1 = kf_ids.begin(); it1 != end_minus_2; ++it1) {
          // small distance to other keyframes --> higher score
          Scalar denom = 0;
          for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
            denom += 1 / ((frame_poses.at(*it1).getPose().translation() -
                           frame_poses.at(*it2).getPose().translation())
                              .norm() +
                          Scalar(1e-5));
          }

          // 关键帧离最新的关键帧距离更近的，得分更低
          // small distance to latest kf --> lower score
          Scalar score =
              std::sqrt((frame_poses.at(*it1).getPose().translation() -
                         frame_poses.at(last_kf).getPose().translation())
                            .norm()) *
              denom;

          if (score < min_score) {
            min_score_id = *it1;
            min_score = score;
          }
        }

        id_to_marg = min_score_id;
      }

      // 通过上述两种方式，一定可以选到被边缘化的帧序号
      // if no frame was selected, the logic above is faulty
      BASALT_ASSERT(id_to_marg >= 0);

      kfs_to_marg.emplace(id_to_marg);

      // Note: this looks like a leftover from VIO that is not doing anything in
      // VO -> we could check / compare / remove
      non_kf_poses.emplace(id_to_marg);

      kf_ids.erase(id_to_marg); // 从kf_ids中删除被marg的帧
    }

    // Create AbsOrderMap entries that are in the marg prior or connected to the
    // keyframes that we marginalize
    // Create AbsOrderMap entries that are in the marg prior or connected to the
    // keyframes that we marginalize

    // 创建绝对排序map对象，用于边缘化先验或者连接到marg的关键帧
    // 主要是为了构建：aom.abs_order_map:
    // 1、把marg帧作为host frame, 其对应的target frame加入到aom.abs_order_map
    // 2、把每个 lost landmark 对应的 host frame 和 target frame 加入到aom.abs_order_map
    // step3 把需要被marg的帧和被marg的路标点相关联的所有帧加入到AbsOrderMap
    AbsOrderMap aom;
    {
      // 返回的obs是host frame id和 [target frame id, lm_ids]（目标帧id和路标点id集也是map容器键值对） 组成的键值对
      const auto& obs = lmdb.getObservations();

      aom.abs_order_map = marg_data.order.abs_order_map; //- 20240314: 初始化时marg_data把第一个帧加进来了，并且marg_data.H是6x6的对角阵，对角元素1e4
      aom.total_size = marg_data.order.total_size;
      aom.items = marg_data.order.items;

      //[1] 把marg帧作为host frame, 其对应的target frame加入到aom.abs_order_map
      // 意即：如果一个观测的host frame是需要被marg的帧，观测的target frame在frame_poses中，则把该target frame加入到aom.abs_order_map
      // 如此一来，那么被marg帧，以及存在共视的target frame都会被加入到aom.abs_order_map
      for (const auto& kv : frame_poses) {
        if (aom.abs_order_map.count(kv.first) == 0) {
          // step1 如果frame_poses里面的帧kv在aom.abs_order_map中找不到，
          // step2 则先从lmdb的观测中查找host frame id 如果其属于marg frame id，
          // step3 则进一步查找该host frame id 对应的目标帧容器，如果target frame id属于frame_poses中的一帧，
          // step4 则add_pose置为true，先后跳出step3, step2两层循环，
          // step5 更新aom,增加items以及pose的total size,以及abs_order_map的键值对，接着回到step1。
          bool add_pose = false;

          for (const auto& [host, target_map] : obs) {
            // if one of the host frames that we marg out
            if (kfs_to_marg.count(host.frame_id) > 0) {
              for (const auto& [target, obs_map] : target_map) {
                // has observations in the frame also add it to marg prior
                if (target.frame_id == kv.first) {
                  add_pose = true;
                  break;
                }
              }
            }
            // Break if we already found one observation.
            if (add_pose) break;
          }

          if (add_pose) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_SIZE);

            aom.total_size += POSE_SIZE;
            aom.items++;
          }
        }
      }

      //[2] 把每个 lost landmark 对应的 host frame 和 target frame 加入到aom.abs_order_map
      // If marg lost landmakrs add corresponding frames to linearization
      if (config.vio_marg_lost_landmarks) {
        // lost_landmaks是lmdb中不被当前帧观测到的landmarks
        for (const auto& lm_id : lost_landmaks) {
          // lm是Keypoint类型
          const auto& lm = lmdb.getLandmark(lm_id);
          // 如果不被观测的landmark所对应的主导帧id在aom.abs_order_map查找不到，则添加进来
          if (aom.abs_order_map.count(lm.host_kf_id.frame_id) == 0) {
            aom.abs_order_map[lm.host_kf_id.frame_id] =
                std::make_pair(aom.total_size, POSE_SIZE);

            aom.total_size += POSE_SIZE;
            aom.items++;
          }

          // 如果不被观测的landmark所对应的目标帧id在aom.abs_order_map查找不到，则添加进来
          for (const auto& [target, o] : lm.obs) {
            if (aom.abs_order_map.count(target.frame_id) == 0) {
              aom.abs_order_map[target.frame_id] =
                  std::make_pair(aom.total_size, POSE_SIZE);

              aom.total_size += POSE_SIZE;
              aom.items++;
            }
          }
        }
      }
    }

    //    std::cout << "marg order" << std::endl;
    //    aom.print_order();

    //    std::cout << "marg prior order" << std::endl;
    //    marg_order.print_order();

    if (config.vio_debug) {
      std::cout << "non_kf_poses.size() " << non_kf_poses.size() << std::endl;
      for (const auto& v : non_kf_poses) std::cout << v << ' ';
      std::cout << std::endl;

      std::cout << "kfs_to_marg.size() " << kfs_to_marg.size() << std::endl;
      for (const auto& v : kfs_to_marg) std::cout << v << ' ';
      std::cout << std::endl;

      std::cout << "last_state_t_ns " << last_state_t_ns << std::endl;

      std::cout << "frame_poses.size() " << frame_poses.size() << std::endl;
      for (const auto& v : frame_poses) std::cout << v.first << ' ';
      std::cout << std::endl;
    }

    // 如果被marg的帧，在aom.abs_order_map中查找不到，
    // 则从frame_poses, prev_opt_flow_res, lmdb中删除之
    // Remove unconnected frames
    if (!kfs_to_marg.empty()) {
      for (auto it = kfs_to_marg.cbegin(); it != kfs_to_marg.cend();) {
        if (aom.abs_order_map.count(*it) == 0) {
          frame_poses.erase(*it);
          prev_opt_flow_res.erase(*it);
          lmdb.removeKeyframes({*it}, {}, {});
          it = kfs_to_marg.erase(it);
        } else {
          it++;
        }
      }
    }

    // step4: 开始marg
    if (!kfs_to_marg.empty()) {
      // 被marg的kf容器不为空，才会真正执行边缘化
      Timer t_actual_marg;

      // 如果当前帧（或者说最新帧）是关键帧则进行边缘化
      // Marginalize only if last state is a keyframe
      BASALT_ASSERT(kf_ids_all.count(last_state_t_ns) > 0);

      size_t asize = aom.total_size;
      //      double marg_prior_error;

      //      DenseAccumulator accum;
      //      accum.reset(asize);

      // 默认配置项为ABS_QR,  因此is_lin_sqrt为true
      bool is_lin_sqrt = isLinearizationSqrt(config.vio_linearization_type);

      // 以下这两个变量的命名很有意思，如果线性化类型vio_linearization_type为ABS_QR，
      // 并且vio_sqrt_marg为true，那么Q2Jp_or_H = Q_2^T * J_p,  Q2r_or_b = Q_2 * r
      // 否则Q2Jp_or_H = H, Q2r_or_b = b.
      MatX Q2Jp_or_H;
      VecX Q2r_or_b;

      {
        // Linearize points
        Timer t_linearize;

        typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
        lqr_options.lb_options.huber_parameter = huber_thresh;
        lqr_options.lb_options.obs_std_dev = obs_std_dev;
        lqr_options.linearization_type = config.vio_linearization_type;

        // 跟optimize()函数中不一样的地方，在于：
        // optimize中的aom保存的是frame_poses里面的每一帧，
        // 而marginalize中的aom保存的是需要被marg的帧和与被marg帧存在共视的帧，以及观测到lost landmarks的帧，
        // 另外，marginalize时，还多了kfs_to_marg和lost_landmaks两个参数。
        // 此处构建lqr，实际调用了linearization_abs_qr.cpp文件里的LinearizationAbsQR构造函数
        //TODO: LinearizationAbsQR构造函数值得细看
        // 再补充说一下：这里是把跟marg帧相关的target frame，以及跟lost landmark相关的host frame 和target frame,
        // 以及 lost landmarks一起来构建marg Hessian矩阵（或者说是构建Jacobian矩阵），然后得到Q_2^T * J_p和Q_2 * r
        auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
            this, aom, lqr_options, &marg_data, nullptr, &kfs_to_marg,
            &lost_landmaks);
        // step4.1 线性化相对位姿和路标点
        lqr->linearizeProblem();
        // step4.2 marg路标点(in-place marginalization on landmark block)
        lqr->performQR(); // 这里相当于进行nullspace marginalization.得到Q_2^T * J_p和Q_2^T * r 
                          // 这一步其实和optimize()里面的步骤是一样的。使用ns，用于marginalize landmarks 

        // vio_sqrt_marg默认配置项为true, 因此 marg_data.is_sqrt也为true。
        if (is_lin_sqrt && marg_data.is_sqrt) {
          // step4.3 获取所有 landmark block 的 Q_2^T x J_p和Q_2^T x r, 以及marg prior H_m^{\prime}和b_m^{\prime}, 按行排列组成的Q2Jp, Q2r
          lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b); // marg的时候，执行的是get_dense_Q2Jp_Q2r
        } else {
          lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);
        }

        stats_sums_.add("marg_linearize", t_linearize.elapsed()).format("ms");
      }

      // 如果kfs_to_marg不为空并且保存marg数据的队列指针out_marg_queue不为空，则进行marg数据保存（详见marg_data_io.cpp构造函数里面的线程函数save_func）
      // Save marginalization prior
      if (out_marg_queue && !kfs_to_marg.empty()) {
        // int64_t kf_id = *kfs_to_marg.begin();

        {
          MargData::Ptr m(new MargData);
          m->aom = aom;

          // vio_sqrt_marg默认配置项为true.
          if (is_lin_sqrt && marg_data.is_sqrt) {
            m->abs_H =
                (Q2Jp_or_H.transpose() * Q2Jp_or_H).template cast<double>();
            m->abs_b =
                (Q2Jp_or_H.transpose() * Q2r_or_b).template cast<double>();
          } else {
            m->abs_H = Q2Jp_or_H.template cast<double>();

            m->abs_b = Q2r_or_b.template cast<double>();
          }

          assign_cast_map_values(m->frame_poses, frame_poses);
          assign_cast_map_values(m->frame_states, frame_states);
          m->kfs_all = kf_ids_all;
          m->kfs_to_marg = kfs_to_marg;
          m->use_imu = false;

          for (int64_t t : m->kfs_all) {
            m->opt_flow_res.emplace_back(prev_opt_flow_res.at(t));
          }

          out_marg_queue->push(m);
        }
      }

      // step4.4 将aom.abs_order_map中的所有位姿序号分为要保留的和要边缘化的，分别存储起来
      //- 判断aom.abs_order_map中的帧时间戳，如果是需要被marg的帧，则6维的pose序号添加到idx_to_marg, 否则添加进idx_to_keep.
      // aom.abs_order_map是一个map of key-value: key is timestamp & value is a pair of something like '(0, POSE_SIZE)'
      std::set<int> idx_to_keep, idx_to_marg;
      for (const auto& kv : aom.abs_order_map) {
        if (kv.second.second == POSE_SIZE) { // 这个if应该总是成立。
          int start_idx = kv.second.first;
          if (kfs_to_marg.count(kv.first) == 0) { // 如果kfs_to_marg中查找不到，则把需要保留的序号添加进idx_to_keep
            for (size_t i = 0; i < POSE_SIZE; i++)
              idx_to_keep.emplace(start_idx + i);
          } else {
            for (size_t i = 0; i < POSE_SIZE; i++)
              idx_to_marg.emplace(start_idx + i); // 否则，把marg的序号添加进idx_to_marg
          }
        } else {
          BASALT_ASSERT(false);
        }
      }

      if (config.vio_debug) {
        std::cout << "keeping " << idx_to_keep.size() << " marg "
                  << idx_to_marg.size() << " total " << asize << std::endl;
        std::cout << " frame_poses " << frame_poses.size() << " frame_states "
                  << frame_states.size() << std::endl;
      }

      if (config.vio_debug || config.vio_extended_logging) { // 仅用于调试和日志输出
        MatX Q2Jp_or_H_nullspace;
        VecX Q2r_or_b_nullspace;

        typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
        lqr_options.lb_options.huber_parameter = huber_thresh;
        lqr_options.lb_options.obs_std_dev = obs_std_dev;
        lqr_options.linearization_type = config.vio_linearization_type;

        nullspace_marg_data.order = marg_data.order;

        auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
            this, aom, lqr_options, &nullspace_marg_data, nullptr, &kfs_to_marg,
            &lost_landmaks);

        lqr->linearizeProblem();
        lqr->performQR();

        if (is_lin_sqrt && marg_data.is_sqrt) {
          lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace);
        } else {
          lqr->get_dense_H_b(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace);
        }

        MatX nullspace_sqrt_H_new;
        VecX nullspace_sqrt_b_new;

        if (is_lin_sqrt && marg_data.is_sqrt) {
          MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
              Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep, idx_to_marg,
              nullspace_sqrt_H_new, nullspace_sqrt_b_new);
        } else if (marg_data.is_sqrt) {
          MargHelper<Scalar>::marginalizeHelperSqToSqrt(
              Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep, idx_to_marg,
              nullspace_sqrt_H_new, nullspace_sqrt_b_new);
        } else {
          MargHelper<Scalar>::marginalizeHelperSqToSq(
              Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep, idx_to_marg,
              nullspace_sqrt_H_new, nullspace_sqrt_b_new);
        }

        nullspace_marg_data.H = nullspace_sqrt_H_new;
        nullspace_marg_data.b = nullspace_sqrt_b_new;
      }

      MatX marg_sqrt_H_new;
      VecX marg_sqrt_b_new;

      // step4.5 下面这一步应该是为了marginalize the frame variables
      {
        Timer t;
        if (is_lin_sqrt && marg_data.is_sqrt) { // 是否线性化和marg均使用sqrt
          ///TODO: marginalizeHelperSqrtToSqrt的作用值得仔细研究
          MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
              Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_sqrt_H_new,
              marg_sqrt_b_new); // 对于root vo来说，调用该函数进行marg，得到J_m和b_m
        } else if (marg_data.is_sqrt) {
          MargHelper<Scalar>::marginalizeHelperSqToSqrt(
              Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_sqrt_H_new,
              marg_sqrt_b_new);
        } else {
          MargHelper<Scalar>::marginalizeHelperSqToSq(
              Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_sqrt_H_new,
              marg_sqrt_b_new);
        }
        stats_sums_.add("marg_helper", t.elapsed()).format("ms");
      }

      // step4.6 参与了marg的视觉帧，设置线性化标志为true.
      for (auto& kv : frame_poses) {
        if (aom.abs_order_map.count(kv.first) > 0) {
          if (!kv.second.isLinearized()) kv.second.setLinTrue(); //? 加入到aom.abs_order_map的帧，才会设置线性化为true.
        }
      }

      // step4.7 从frame_poses, prev_opt_flow_res中删除被marg的帧
      for (const int64_t id : kfs_to_marg) {
        frame_poses.erase(id);
        prev_opt_flow_res.erase(id);
      }

      // step4.8 从lmdb中删除被marg的帧
      //TODO: 从lmdb中删除关键帧，值得细看一下
      lmdb.removeKeyframes(kfs_to_marg, kfs_to_marg, kfs_to_marg);

      // step4.9 从lmdb中删除不被当前帧观测到的路标点
      if (config.vio_marg_lost_landmarks) {
        for (const auto& lm_id : lost_landmaks) lmdb.removeLandmark(lm_id);
      }

      // step4.10 得到新的marg序号排列
      AbsOrderMap marg_order_new;

      for (const auto& kv : frame_poses) {
        if (aom.abs_order_map.count(kv.first) > 0) {
          marg_order_new.abs_order_map[kv.first] =
              std::make_pair(marg_order_new.total_size, POSE_SIZE);

          marg_order_new.total_size += POSE_SIZE;
          marg_order_new.items++;
        }
      }

      // step4.11 更新marg_data
      marg_data.H = marg_sqrt_H_new; // 对于root vo来说，marg_sqrt_H_new = J_m
      marg_data.b = marg_sqrt_b_new; // 对于root vo来说，marg_sqrt_b_new = b_m
      marg_data.order = marg_order_new;

      BASALT_ASSERT(size_t(marg_data.H.cols()) == marg_data.order.total_size);

      // Quadratic prior and "delta" of the current state to the original
      // linearization point give cost function
      //
      //    P(x) = 0.5 || J*(delta+x) + r ||^2.
      //
      // For marginalization this has been linearized at x=0 to give
      // linearization
      //
      //    P(x) = 0.5 || J*x + (J*delta + r) ||^2,
      //
      // with Jacobian J and residual J*delta + r.
      //
      // After marginalization, we recover the original form of the
      // prior. We are left with linearization (in sqrt form)
      //
      //    Pnew(x) = 0.5 || Jnew*x + res ||^2.
      //
      // To recover the original form with delta-independent r, we set
      //
      //    Pnew(x) = 0.5 || Jnew*(delta+x) + (res - Jnew*delta) ||^2,
      //
      // and thus rnew = (res - Jnew*delta).

      //TODO: 更改新构造的先验中的b，需要去掉由于当前状态到线性点偏移导致的分量
      VecX delta;
      bool bl = computeDelta(marg_data.order, delta);
      if(!bl)
      {
        return false;
      }

      // step4.12 减去b_m里面的偏移量
      marg_data.b -= marg_data.H * delta; // 计算线性点处的能量的偏置

      if (config.vio_debug || config.vio_extended_logging) {
        VecX delta;
        bool bl = computeDelta(marg_data.order, delta);
        if(!bl)
        {
          return false;
        }
        nullspace_marg_data.b -= nullspace_marg_data.H * delta;
      }

      stats_sums_.add("marg_total", t_actual_marg.elapsed()).format("ms");

      if (config.vio_debug) {
        std::cout << "marginalizaon done!!" << std::endl;
      }

      if (config.vio_debug || config.vio_extended_logging) {
        Timer t;
        logMargNullspace();
        stats_sums_.add("marg_log", t.elapsed()).format("ms");
      }
    }

    //    std::cout << "new marg prior order" << std::endl;
    //    marg_order.print_order();
  }

  stats_sums_.add("marginalize", t_total.elapsed()).format("ms");

  return true;
}

template <class Scalar_>
int SqrtKeypointVoEstimator<Scalar_>::optimize() { // change return type from 'void' to 'int' on 2023-11-13.
  
  int retval = 1;

  if (config.vio_debug) {
    std::cout << "=================================" << std::endl;
  }

  // harcoded configs
  //  bool scale_Jp = config.vio_scale_jacobian && is_qr_solver();
  //  bool scale_Jl = config.vio_scale_jacobian && is_qr_solver();

  // timing
  ExecutionStats stats;
  Timer timer_total; // 定义计时对象in milliseconds
  Timer timer_iteration;

  // construct order of states in linear system --> sort by ascending
  // timestamp
  AbsOrderMap aom;
  // abs_order_map是一个map of key-value: 在initialize函数中key is timestamp & value is a pair of '(0, POSE_SIZE)'
  aom.abs_order_map = marg_data.order.abs_order_map;
  aom.total_size = marg_data.order.total_size; // 在initialize函数中total_size = POSE_SIZE = 6.
  aom.items = marg_data.order.items; // 在initialize函数中items = 1.

  // marg_data.H和marg_data.b初始化时分别是6*6和6*1的0矩阵

  // frame_poses的每一个元素: key is a timestamp & value is a struct type of PoseStateWithLin including T_w_i_current
  // 该for loop，统计pose的总个数和维数
  for (const auto& kv : frame_poses) {
    if (aom.abs_order_map.count(kv.first) == 0) {
      aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
      aom.total_size += POSE_SIZE;
      aom.items++;
    }
  }

  // This is VO not VIO, so expect no IMU states
  BASALT_ASSERT(frame_states.empty());

  // TODO: Check why we get better accuracy with old SC loop. Possible culprits:
  // - different initial lambda (based on previous iteration)
  // - no landmark damping
  // - outlier removal after 4 iterations?
  
  // 检查为什么我们用旧的sc循环得到更好的准确性。可能的引起问题的原因['kʌlprɪts] ：
  // 1、不同的初始的λ（基于上一次迭代）
  // 2、没有路标阻尼
  // 3、在4次迭代后去除离群值吗？ 
  // ? 上面提到的SC是schur complement

  lambda = Scalar(config.vio_lm_lambda_initial); // 1e-4

  // record stats 记录状态
  stats.add("num_cams", frame_poses.size()).format("count");
  stats.add("num_lms", lmdb.numLandmarks()).format("count");
  stats.add("num_obs", lmdb.numObservations()).format("count");

  // setup landmark blocks 设置路标块
  typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
  lqr_options.lb_options.huber_parameter = huber_thresh; // 1.0
  lqr_options.lb_options.obs_std_dev = obs_std_dev; // 0.5 // standard deviation 观测标准差，方差的正的平方根
  lqr_options.linearization_type = config.vio_linearization_type; // ABS_QR
  std::unique_ptr<LinearizationBase<Scalar, POSE_SIZE>> lqr;

  {
    Timer t;
    // 创建线性化对象, 如果线性化类型为ABS_QR, 则调用LinearizationAbsQR的构造函数来构造lqr.
    lqr = LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options,
                                                       &marg_data);
    stats.add("allocateLMB", t.reset()).format("ms");
    lqr->log_problem_stats(stats);
  }

  // 开始循环迭代直至收敛，或者迭代次数耗尽
  bool terminated = false;
  bool converged = false;
  std::string message;

  int it = 0;
  int it_rejected = 0;
  //TODO: 外面一个循环，内部还有一个lm循环，是否有必要????
  for (; it <= config.vio_max_iterations && !terminated;) {
    if (it > 0) {
      timer_iteration.reset();
    }

    Scalar error_total = 0;
    VecX Jp_column_norm2;

    // TODO: execution could be done staged
    // 执行可以被分阶段完成[todo]

    Timer t;

    // ? 线性化残差，这一部分是求jacobian矩阵，具体细节值得细看
    // linearize residuals
    bool numerically_valid; // 线性化路标点是否数字上有效
    // 线性化为了得到所有路标点的landmark_block, 每个landmark block 的storage矩阵: [J_p | pad | J_l | res]
    error_total = lqr->linearizeProblem(&numerically_valid);
    BASALT_ASSERT_STREAM(
        numerically_valid,
        "did not expect numerical failure during linearization");
    stats.add("linearizeProblem", t.reset()).format("ms");

    //      // compute pose jacobian norm squared for Jacobian scaling
    //      if (scale_Jp) {
    //        Jp_column_norm2 = lqr->getJp_diag2();
    //        stats.add("getJp_diag2", t.reset()).format("ms");
    //      }

    //      // scale landmark jacobians
    //      if (scale_Jl) {
    //        lqr->scaleJl_cols();
    //        stats.add("scaleJl_cols", t.reset()).format("ms");
    //      }

    // ? 在适当位置边缘化点
    // 答疑解惑[2024-3-8]: 就地边缘化(in-place marginalization), 意即不进行拷贝和移动，在storage矩阵上进行原地操作
    // marginalize points in place
    lqr->performQR(); // 所有路标点的每个landmark_block的storage矩阵从[J_p | pad | J_l | res] 转换为 [Q_1^T J_p | R_1 | Q_1^T res]
                      //                                                                          [Q_2^T J_p | 0   | Q_2^T res]

    stats.add("performQR", t.reset()).format("ms");

    if (config.vio_debug) {
      // TODO: num_points debug output missing
      std::cout << "[LINEARIZE] Error: " << error_total << " num points "
                << std::endl;
      std::cout << "Iteration " << it << " " << error_total << std::endl;
    }

    // compute pose jacobian scaling
    //    VecX jacobian_scaling;
    //    if (scale_Jp) {
    //      // TODO: what about jacobian scaling for SC solver?

    //      // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
    //      // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
    //      jacobian_scaling = (lqr_options.lb_options.jacobi_scaling_eps +
    //                          Jp_column_norm2.array().sqrt())
    //                             .inverse();
    //    }
    // if (config.vio_debug) {
    //   std::cout << "\t[INFO] Stage 1" << std::endl;
    //}

    // inner loop for backtracking in LM (still count as main iteration though)
    // 在lm中回溯的内部循环（但仍然可以作为主迭代）
    for (int j = 0; it <= config.vio_max_iterations && !terminated; j++) {
      if (j > 0) {
        timer_iteration.reset();
        if (config.vio_debug) {
          std::cout << "Iteration " << it << ", backtracking" << std::endl;
        }
      }

      {
        //        Timer t;

        // TODO: execution could be done staged

        // set (updated) damping for poses
        //        if (config.vio_lm_pose_damping_variant == 0) {
        //          lqr->setPoseDamping(lambda);
        //          stats.add("setPoseDamping", t.reset()).format("ms");
        //        }

        //        // scale landmark Jacobians only on the first inner iteration.
        //        if (scale_Jp && j == 0) {
        //          lqr->scaleJp_cols(jacobian_scaling);
        //          stats.add("scaleJp_cols", t.reset()).format("ms");
        //        }

        //        // set (updated) damping for landmarks
        //        if (config.vio_lm_landmark_damping_variant == 0) {
      //          lqr->setLandmarkDamping(lambda);
        //          stats.add("setLandmarkDamping", t.reset()).format("ms");
        //        }
      }

      // if (config.vio_debug) {
      //   std::cout << "\t[INFO] Stage 2 " << std::endl;
      // }

      VecX inc;
      {
        Timer t;

        // get dense reduced camera system 获取稠密的简化相机系统
        MatX H;
        VecX b;

        // 看上面的注释部分// set (updated) damping for poses，因此landmark block中并未用到位姿阻尼
        lqr->get_dense_H_b(H, b); // 经过NS之后marg掉landmarks，得到稠密的位姿hessian矩阵H,维度已经大大降低

        stats.add("get_dense_H_b", t.reset()).format("ms");

        int iter = 0;
        bool inc_valid = false;
        constexpr int max_num_iter = 3;
        // 在solve RCS时，才加入阻尼： lambda的初始值赋值为config.vio_lm_lambda_initial，1e-4
        while (iter < max_num_iter && !inc_valid) {
          VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
          MatX H_copy = H;
          H_copy.diagonal() += Hdiag_lambda;

          Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
          ///TODO: Hδx = -b. 为啥这里没有体现负号??? 难道是$\widetilde{H}_{pp}(-∆x_p) = \widetilde{b_p}$???
          // 除非这里求的位姿增量inc是-{\Delta}x_p^*
          inc = ldlt.solve(b); // ldlt分解是 下三角矩阵 * 对角矩阵 * 下三角矩阵的转置
          stats.add("solve", t.reset()).format("ms");

          if (!inc.array().isFinite().all()) {
            lambda = lambda_vee * lambda; // lambda_vee初值为2.
            lambda_vee *= vee_factor;
          } else {
            inc_valid = true;
          }
          iter++;
        }

        if (!inc_valid) {
          std::cerr << "Still invalid inc after " << max_num_iter
                    << " iterations." << std::endl;
        }
      }

      // backup state (then apply increment and check cost decrease)
      backup(); // 对delta \ pose_linearized \ T_w_i_current3个分量进行备份

      // TODO: directly invert pose increment when solving; change SC
      // `updatePoints` to recieve unnegated increment

      // backsubstitute (with scaled pose increment) 用位姿增量回代
      Scalar l_diff = 0;
      {
        // negate pose increment before point update 点更新前使求得到的位姿增量符号反向
        inc = -inc; // -Δxp 转换为Δxp

        Timer t;
        // 将求得的位姿增量的解转化为Δxp，进行回代来求解路标点的增量Δxl
        l_diff = lqr->backSubstitute(inc); // 用增量来更新点
        stats.add("backSubstitute", t.reset()).format("ms");

        if(l_diff < -1e-7 || !inc.array().isFinite().all()) 
        {
          std::cout << "infinite, iteration abnormal.\n";
          wx::TFileSystemHelper::WriteLog("Numerical failure in backsubstitution. infinite, iteration abnormal.");
          retval = 2;
          return retval;
        }
        // std::cout << "l_diff=" << l_diff << std::endl;
      }

      // undo jacobian scaling before applying increment to poses
      //      if (scale_Jp) {
      //        inc.array() *= jacobian_scaling.array();
      //      }

      // apply increment to poses 应用增量到位姿
      for (auto& [frame_id, state] : frame_poses) {
        // frame_id是时间戳也是帧序号
        // aom.abs_order_map存储的是序号和POSE_SIZE维数的pair对
        // POSE_SIZE = 6维，idx是帧在位姿增量里面的维数序号从0, 6, 12, 18, ...开始
        int idx = aom.abs_order_map.at(frame_id).first;
        // applyInc位于imu_types.h文件中
        state.applyInc(inc.template segment<POSE_SIZE>(idx));
      }

      // compute stepsize
      Scalar step_norminf = inc.array().abs().maxCoeff();

      // this is VO not VIO
      BASALT_ASSERT(frame_states.empty());

      // compute error update applying increment
      Scalar after_update_marg_prior_error = 0;
      Scalar after_update_vision_error = 0;

      {
        Timer t;
        computeError(after_update_vision_error);
        computeMargPriorError(marg_data, after_update_marg_prior_error);
        stats.add("computerError2", t.reset()).format("ms");
      }

      Scalar after_error_total =
          after_update_vision_error + after_update_marg_prior_error;

      // check cost decrease compared to quadratic model cost
      Scalar f_diff;
      bool step_is_valid = false;
      bool step_is_successful = false;
      Scalar relative_decrease = 0;
      {
        // compute actual cost decrease
        // 线性化所有landmark得到的残差之和减去增量更新之后的残差
        f_diff = error_total - after_error_total;

        relative_decrease = f_diff / l_diff;

        if (config.vio_debug) {
          std::cout << "\t[EVAL] error: {:.4e}, f_diff {:.4e} l_diff {:.4e} "
                       "step_quality {:.2e} step_size {:.2e}\n"_format(
                           after_error_total, f_diff, l_diff, relative_decrease,
                           step_norminf);
        }

        // TODO: consider to remove assert. For now we want to test if we even
        // run into the l_diff <= 0 case ever in practice
        // BASALT_ASSERT_STREAM(l_diff > 0, "l_diff " << l_diff);

        // l_diff <= 0 is a theoretical possibility if the model cost change is
        // tiny and becomes numerically negative (non-positive). It might not
        // occur since our linear systems are not that big (compared to large
        // scale BA for example) and we also abort optimization quite early and
        // usually don't have large damping (== tiny step size).
        // 如果l_diff > 0 && relative_decrease > 0迭代就是成功的。
        step_is_valid = l_diff > 0;
        step_is_successful = step_is_valid && relative_decrease > 0;
      }

      double iteration_time = timer_iteration.elapsed();
      double cumulative_time = timer_total.elapsed();

      stats.add("iteration", iteration_time).format("ms");
      {
        basalt::MemoryInfo mi;
        if (get_memory_info(mi)) {
          stats.add("resident_memory", mi.resident_memory);
          stats.add("resident_memory_peak", mi.resident_memory_peak);
        }
      }

      if (step_is_successful) {
        BASALT_ASSERT(step_is_valid);

        if (config.vio_debug) {
          //          std::cout << "\t[ACCEPTED] lambda:" << lambda
          //                    << " Error: " << after_error_total << std::endl;

          std::cout << "\t[ACCEPTED] error: {:.4e}, lambda: {:.1e}, it_time: "
                       "{:.3f}s, total_time: {:.3f}s\n"
                       ""_format(after_error_total, lambda, iteration_time,
                                 cumulative_time);
        }

        lambda *= std::max<Scalar>(
            Scalar(1.0) / 3,
            1 - std::pow<Scalar>(2 * relative_decrease - 1, 3));
        lambda = std::max(min_lambda, lambda);

        lambda_vee = initial_vee;

        it++;

        // check function and parameter tolerance
        if ((f_diff > 0 && f_diff < Scalar(1e-6)) ||
            step_norminf < Scalar(1e-4)) {
          converged = true;
          terminated = true;
        }

        // stop inner lm loop 退出lm内循环
        break;
      } else {
        std::string reason = step_is_valid ? "REJECTED" : "INVALID";

        if (config.vio_debug) {
          //          std::cout << "\t[REJECTED] lambda:" << lambda
          //                    << " Error: " << after_error_total << std::endl;

          std::cout << "\t[{}] error: {}, lambda: {:.1e}, it_time:"
                       "{:.3f}s, total_time: {:.3f}s\n"
                       ""_format(reason, after_error_total, lambda,
                                 iteration_time, cumulative_time);
        }

        lambda = lambda_vee * lambda;
        lambda_vee *= vee_factor;

        //        lambda = std::max(min_lambda, lambda);
        //        lambda = std::min(max_lambda, lambda);

        restore(); // 恢复（回滚）状态。
        it++; // 继续迭代
        it_rejected++;

        if (lambda > max_lambda) {
          terminated = true;
          message =
              "Solver did not converge and reached maximum damping lambda";
        }
      } // if (step_is_successful)
    } // for (int j = 0; it <= config.vio_max_iterations && !terminated; j++)
  } // for (; it <= config.vio_max_iterations && !terminated;) 

  // std::cout << "num_it = " << it << std::endl;

  stats.add("optimize", timer_total.elapsed()).format("ms");
  stats.add("num_it", it).format("count");
  stats.add("num_it_rejected", it_rejected).format("count");

  // TODO: call filterOutliers at least once (also for CG version)

  stats_all_.merge_all(stats);
  stats_sums_.merge_sums(stats);

  if (config.vio_debug) {
    if (!converged) {
      if (terminated) {
        std::cout << "Solver terminated early after {} iterations: {}"_format(
            it, message);
      } else {
        std::cout
            << "Solver did not converge after maximum number of {} iterations"_format(
                   it);
      }
    }

    stats.print();

    std::cout << "=================================" << std::endl;
  }

  // return converged; // 2023-11-13.

  retval = (retval == 2)? retval : (converged? 1: 0);
  return retval;
}

template <class Scalar_>
int SqrtKeypointVoEstimator<Scalar_>::optimize_and_marg( // change return type from 'void' to 'int' on 2023-11-13.
    const std::map<int64_t, int>& num_points_connected,
    const std::unordered_set<KeypointId>& lost_landmaks) {
  // optimize();
  // bool converged = optimize();
  int converged = optimize();
  bool bl = marginalize(num_points_connected, lost_landmaks);
  if(!bl)
  {
    converged = 2;
  }

  return converged; // 2023-11-13
}

template <class Scalar_>
void SqrtKeypointVoEstimator<Scalar_>::debug_finalize() {
  std::cout << "=== stats all ===\n";
  stats_all_.print();
  std::cout << "=== stats sums ===\n";
  stats_sums_.print();

  // save files
  stats_all_.save_json("stats_all.json");
  stats_sums_.save_json("stats_sums.json");
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

#ifdef BASALT_INSTANTIATIONS_DOUBLE
template class SqrtKeypointVoEstimator<double>;
#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class SqrtKeypointVoEstimator<float>;
#endif
}  // namespace basalt
