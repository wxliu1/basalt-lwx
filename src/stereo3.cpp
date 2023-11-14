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
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

#include <fmt/format.h>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/global_control.h>


// #include <CLI/CLI.hpp>

// #include <basalt/io/dataset_io.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/vi_estimator/vio_estimator.h>
// #include <basalt/calibration/calibration.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/system_utils.h>
// #include <basalt/utils/vis_utils.h>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

#include "wx_ros2_io.h" // 2023-11-10.
#include "wx_yaml_io.h"
#include "imu/imu_process.h"

using namespace wx;

#define _SHOW_POINTS

// enable the "..."_format(...) string literal
using namespace basalt::literals;

// GUI functions
void load_data(const std::string& calib_path);

// Visualization variables
// std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;

// tbb::concurrent_bounded_queue<int64_t> vis_ts_queue; // added by wxliu on 2023-11-11.

tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>
    out_state_queue;

std::vector<int64_t> vio_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
Eigen::aligned_vector<Sophus::SE3d> vio_T_w_i;

std::vector<int64_t> gt_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> gt_t_w_i;

std::string marg_data_path;
size_t last_frame_processed = 0;

tbb::concurrent_unordered_map<int64_t, int, std::hash<int64_t>> timestamp_to_id;

std::mutex m;
std::condition_variable cv;
bool step_by_step = false;
size_t max_frames = 0;

std::atomic<bool> terminate = false;

// VIO variables
basalt::Calibration<double> calib;

basalt::VioDatasetPtr vio_dataset;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;

// 2023-11-13

ImuProcess* g_imu = nullptr;

void SetFirstVisualPose(ImuProcess* imu)
{
  // std::cout << std::boolalpha << "initFirstPoseFlag=" << imu->InitFirstPoseFlag() << std::endl;

  if (!imu->InitFirstPoseFlag()) {
    imu->SetFirstPoseFlag(true);

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

    vio->setT_w_i_init(TWC0);

  }

}

// the end.

// 2023-11-10.
void feedImage(basalt::OpticalFlowInput::Ptr data, ImuProcess* imu) 
{
  if (sys_cfg_.use_imu) {
    // nanosecond to second
    double curTime = data->t_ns / 1e9  + sys_cfg_.td;
    bool bl = imu->ProcessData(curTime);
    if(!bl) return ;
    
  }
  else {
   
    SetFirstVisualPose(imu);
  }


  opt_flow_ptr->input_queue.push(data);

}

void feedImu(basalt::ImuData<double>::Ptr data) 
{
  vio->imu_data_queue.push(data);
}
// the end.

// Feed functions
// 读取图像并喂给光流追踪: 将图像push到前端opt_flow的输入队列中
void feed_images() {
  std::cout << "Started input_data thread " << std::endl;
  // 遍历数据集每一张图片
  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    if (vio->finished || terminate || (max_frames > 0 && i >= max_frames)) {
      // stop loop early if we set a limit on number of frames to process
      break;
    }

    if (step_by_step) {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk);
    }

    // step 1 构建光流跟踪结构体并把时间戳图像数据放进去
    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

    // 转换成视觉前端的输入类型
    data->t_ns = vio_dataset->get_image_timestamps()[i];
    data->img_data = vio_dataset->get_image_data(data->t_ns);

    timestamp_to_id[data->t_ns] = i;

    // step 2 塞入到输入队列中: 将数据压入到视觉前端的队列中
    opt_flow_ptr->input_queue.push(data);
  }

  // step 3 如果运行完塞入一个空的数据，告诉后面的数据结束
  // Indicate the end of the sequence
  opt_flow_ptr->input_queue.push(nullptr); /// 队列中push空指针，指示图像序列的结束

  std::cout << "Finished input_data thread " << std::endl;
}

// 将IMU push 到后端
void feed_imu() {
  for (size_t i = 0; i < vio_dataset->get_gyro_data().size(); i++) {
    if (vio->finished || terminate) {
      break;
    }

    basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
    data->t_ns = vio_dataset->get_gyro_data()[i].timestamp_ns;

    data->accel = vio_dataset->get_accel_data()[i].data;
    data->gyro = vio_dataset->get_gyro_data()[i].data;

    vio->imu_data_queue.push(data);
  }
  vio->imu_data_queue.push(nullptr);
}

// 2023-11-11
void sigintHandler(int sig) {
    
    printf("sig=%d\n",sig);
    terminate = true;
    signal(sig,SIG_DFL);//设置收到信号SIGINT采取默认方式响应（ctrl+c~结束进程）
    
    // 执行特定的中断处理操作
    // 例如，关闭打开的文件、释放资源等
    // 然后退出程序

    exit(0);
}

void stop() 
{
  std::cout << "stop\n";
  vio->imu_data_queue.push(nullptr);
  opt_flow_ptr->input_queue.push(nullptr);
  terminate = true;

    // if (terminate.load() == false) 
    // {
    //   vio->imu_data_queue.push(nullptr);
    //   opt_flow_ptr->input_queue.push(nullptr);
    //   terminate.store(true);
    // }
}
// the end.

int main(int argc, char** argv) {

  // 注册中断信号处理函数
  //signal(SIGINT, sigintHandler); // 2023-11-11

/*
 * comment on 2023-11-10.
  // step 1: 命令行参数解析  
  CLI::App app{"App description"};
*/

  // 2023-11-10
  struct TYamlIO yaml;
  yaml.ReadConfiguration();
  // the end.

  // 2023-11-13
  sys_cfg_.use_imu = yaml.use_imu;
  ImuProcess imu;
  imu.setExtrinsic(yaml.RIC[0], yaml.TIC[0]);
  g_imu = &imu;
  // the end.


  // global thread limit is in effect until global_control object is destroyed
  // 全局线程限制在全局控制对象被销毁之前一直有效
  std::unique_ptr<tbb::global_control> tbb_global_control;
  if (yaml.num_threads > 0) {
    tbb_global_control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, yaml.num_threads);
  }

  if (!yaml.config_path.empty()) {
    vio_config.load(yaml.config_path);

    if (vio_config.vio_enforce_realtime) {
      vio_config.vio_enforce_realtime = false;
      std::cout
          << "The option vio_config.vio_enforce_realtime was enabled, "
             "but it should only be used with the live executables (supply "
             "images at a constant framerate). This executable runs on the "
             "datasets and processes images as fast as it can, so the option "
             "will be disabled. "
          << std::endl;
    }
  }

  // step 2: load camera calibration
  load_data(yaml.cam_calib_path);

 /*
  * comment on 2023-11-10.
  // step 3: 加载不同的数据集，视觉前端初始化
  */
  // move here. 2023-11-10
  opt_flow_ptr =
    basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
  // the end.  

  // step 4: 后端初始化
  //const int64_t start_t_ns = vio_dataset->get_image_timestamps().front(); // comment.
  {
    // 后端初始化，选择是否使用IMU
    vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, yaml.use_imu, yaml.use_double); // 创建后端估计器对象
    
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 后端和前端的对接 （前端光流指针指向的输出队列指针保存的是后端估计器指向的视觉数据队列的地址）
    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    // 后端和可视化线程的对接
    /*if (yaml.show_gui)*/ vio->out_vis_queue = &out_vis_queue;
    vio->out_state_queue = &out_state_queue;
  }

  // step 5: 创建marg数据的保存器
  basalt::MargDataSaver::Ptr marg_data_saver;

  // 如果提供了marg数据保存的路径，则reset marg data saver
  if (!yaml.marg_data_path.empty()) {
    marg_data_saver.reset(new basalt::MargDataSaver(yaml.marg_data_path));
    vio->out_marg_queue = &marg_data_saver->in_marg_queue;

    // Save gt.
    {
      std::string p = yaml.marg_data_path + "/gt.cereal";
      std::ofstream os(p, std::ios::binary);

      {
        cereal::BinaryOutputArchive archive(os);
        archive(gt_t_ns);
        archive(gt_t_w_i);
      }
      os.close();
    }
  }

  // 2023-11-14
  if (sys_cfg_.use_imu) {
    imu.SetTwc0_ = std::bind(&basalt::VioEstimatorBase::setT_w_i_init, vio, std::placeholders::_1);
  }
  // the end.

  // 2023-11-10.
  rclcpp::init(argc, argv);
  //auto node = std::make_shared<CRos2IO>();
  auto node = std::make_shared<CRos2IO>(sys_cfg_.use_imu);

  node->inputIMU_ = std::bind(&ImuProcess::inputIMU, &imu, std::placeholders::_1, 
    std::placeholders::_2, std::placeholders::_3);

  node->feedImage_ = std::bind(&feedImage, std::placeholders::_1, &imu);
  // node->feedImage_ = std::bind(&feedImage, std::placeholders::_1);
#if USE_TIGHT_COUPLING  
  node->feedImu_ = std::bind(&feedImu, std::placeholders::_1);
#endif 
  node->stop_ = std::bind(&stop);

  // the end.

  // step 6: 创建图像和imu线程输入数据
/*
 * comment 2023-11-10.  
  std::thread t1(&feed_images);
  std::thread t2(&feed_imu);
*/

#ifdef _SHOW_POINTS
  // 线程t3从可视化队列中取出数据(3d points, frame pose etc.)，存入可视化map.
  std::shared_ptr<std::thread> t3;
  // 2023-11-11
  {
    t3.reset(new std::thread([&]() {
      basalt::VioVisualizationData::Ptr data;
      //while (true) {
      while (!terminate) {
        out_vis_queue.pop(data);

        if (data.get()) {
          node->PublishPoints(data);
        } else {
          break;
        }
      }

      std::cout << "Finished t3" << std::endl;
    }));
  }
  // the end.
#endif

  // 线程t4用于取出状态（速度，平移，ba, bg），在pangolin的显示
  std::thread t4([&]() {
    basalt::PoseVelBiasState<double>::Ptr data;

    //while (true) {
    while (!terminate) {
      out_state_queue.pop(data);

      if (!data.get()) break;

      node->PublishOdometry(data); // 2023-11-11
      
    }

    std::cout << "Finished t4" << std::endl;
  });

  std::shared_ptr<std::thread> t5;

  auto print_queue_fn = [&]() {
    std::cout << "opt_flow_ptr->input_queue "
              << opt_flow_ptr->input_queue.size()
              << " opt_flow_ptr->output_queue "
              << opt_flow_ptr->output_queue->size() << " out_state_queue "
              << out_state_queue.size() << " imu_data_queue "
              << vio->imu_data_queue.size() << std::endl;
  };

  if (yaml.print_queue) { // cli参数，默认false.用于打印队列的size.
    t5.reset(new std::thread([&]() {
      while (!terminate) {
        print_queue_fn();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }));
  }

  // 2023-11-10
  rclcpp::spin(node);
  //rclcpp::shutdown();
  node->stop_();
  // the end.

  // wait first for vio to complete processing
  vio->maybe_join();

  // input threads will abort when vio is finished, but might be stuck in full
  // push to full queue, so drain queue now
  vio->drain_input_queues();

/*
 * comment 2023-11-10.
  // join input threads
  t1.join();
  t2.join();
*/

  // std::cout << "Data input finished, terminate auxiliary threads.";
  terminate = true;

  // join other threads
  // if (t3) t3->join(); // comment 2023-11-14.
  t4.join();
  if (t5) t5->join();

  // after joining all threads, print final queue sizes.
  if (yaml.print_queue) {
    std::cout << "Final queue sizes:" << std::endl;
    print_queue_fn();
  }


  return 0;
}

void load_data(const std::string& calib_path) {
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib);
    std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras"
              << std::endl;

  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
}
