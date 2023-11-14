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

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <basalt/io/dataset_io.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/system_utils.h>
#include <basalt/utils/vis_utils.h>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

#include "wx_ros2_io.h" // 2023-11-10.
#include "wx_yaml_io.h"
#include "imu/imu_process.h"

using namespace wx;

// enable the "..."_format(...) string literal
using namespace basalt::literals;

// GUI functions
void draw_image_overlay(pangolin::View& v, size_t cam_id);
void draw_scene(pangolin::View& view);
void load_data(const std::string& calib_path);
bool next_step();
bool prev_step();
void draw_plots();
void alignButton();
void alignDeviceButton();
void saveTrajectoryButton();

// Pangolin variables
constexpr int UI_WIDTH = 200;

using Button = pangolin::Var<std::function<void(void)>>;

pangolin::DataLog imu_data_log, vio_data_log, error_data_log;
pangolin::Plotter* plotter;

pangolin::Var<int> show_frame("ui.show_frame", 0, 0, 1500);

pangolin::Var<bool> show_flow("ui.show_flow", false, false, true);
pangolin::Var<bool> show_obs("ui.show_obs", true, false, true);
pangolin::Var<bool> show_ids("ui.show_ids", false, false, true);

pangolin::Var<bool> show_est_pos("ui.show_est_pos", true, false, true);
pangolin::Var<bool> show_est_vel("ui.show_est_vel", false, false, true);
pangolin::Var<bool> show_est_bg("ui.show_est_bg", false, false, true);
pangolin::Var<bool> show_est_ba("ui.show_est_ba", false, false, true);

pangolin::Var<bool> show_gt("ui.show_gt", true, false, true);

Button next_step_btn("ui.next_step", &next_step);
Button prev_step_btn("ui.prev_step", &prev_step);

pangolin::Var<bool> continue_btn("ui.continue", false, false, true);
pangolin::Var<bool> continue_fast("ui.continue_fast", true, false, true);

Button align_se3_btn("ui.align_se3", &alignButton);

pangolin::Var<bool> euroc_fmt("ui.euroc_fmt", true, false, true);
pangolin::Var<bool> tum_rgbd_fmt("ui.tum_rgbd_fmt", false, false, true);
pangolin::Var<bool> kitti_fmt("ui.kitti_fmt", false, false, true);
pangolin::Var<bool> save_groundtruth("ui.save_groundtruth", false, false, true);
Button save_traj_btn("ui.save_traj", &saveTrajectoryButton);

pangolin::Var<bool> follow("ui.follow", true, false, true);

// pangolin::Var<bool> record("ui.record", false, false, true);

pangolin::OpenGlRenderState camera;

// Visualization variables
std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;

tbb::concurrent_bounded_queue<int64_t> vis_ts_queue; // added by wxliu on 2023-11-11.

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

  bool show_gui = true;
  bool print_queue = false;
  std::string cam_calib_path;
  std::string dataset_path;
  std::string dataset_type;
  std::string config_path;
  std::string result_path;
  std::string trajectory_fmt;
  bool trajectory_groundtruth;
  int num_threads = 0;
  bool use_imu = true;
  bool use_double = false;

  // step 1: 命令行参数解析  
  CLI::App app{"App description"};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--cam-calib", cam_calib_path,
                 "Ground-truth camera calibration used for simulation.")
      ->required();

  app.add_option("--dataset-path", dataset_path, "Path to dataset.")
      ->required();

  app.add_option("--dataset-type", dataset_type, "Dataset type <euroc, bag>.")
      ->required();

  app.add_option("--marg-data", marg_data_path,
                 "Path to folder where marginalization data will be stored.");

  app.add_option("--print-queue", print_queue, "Print queue.");
  app.add_option("--config-path", config_path, "Path to config file.");
  app.add_option("--result-path", result_path,
                 "Path to result file where the system will write RMSE ATE.");
  app.add_option("--num-threads", num_threads, "Number of threads.");
  app.add_option("--step-by-step", step_by_step, "Path to config file.");
  app.add_option("--save-trajectory", trajectory_fmt,
                 "Save trajectory. Supported formats <tum, euroc, kitti>");
  app.add_option("--save-groundtruth", trajectory_groundtruth,
                 "In addition to trajectory, save also ground turth");
  app.add_option("--use-imu", use_imu, "Use IMU.");
  app.add_option("--use-double", use_double, "Use double not float.");
  app.add_option(
      "--max-frames", max_frames,
      "Limit number of frames to process from dataset (0 means unlimited)");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }
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
  {
    basalt::DatasetIoInterfacePtr dataset_io =
        basalt::DatasetIoFactory::getDatasetIo(dataset_type);

    dataset_io->read(dataset_path);

    // 获取数据集中图像的文件名和和IMU数据
    vio_dataset = dataset_io->get_data();

    // 设定可视化变量图像的播放的范围
    show_frame.Meta().range[1] = vio_dataset->get_image_timestamps().size() - 1;
    show_frame.Meta().gui_changed = true;
    
    // 视觉前端初始化: 根据pattern的格式不同重新新建了光流追踪的线程
    // calib 包含：相机到imu的外参，相机内参，图像分辨率，渐晕，imu速率，noise, bias等。
    // vio_config 前端和后端的相关配置，比如光流的各个参数，vio的各个参数比如最大关键帧数等。
    opt_flow_ptr =
        basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);



    // 保存groudtruth时间戳和对应的平移
    for (size_t i = 0; i < vio_dataset->get_gt_pose_data().size(); i++) {
      gt_t_ns.push_back(vio_dataset->get_gt_timestamps()[i]);
      gt_t_w_i.push_back(vio_dataset->get_gt_pose_data()[i].translation());
    }    
  }

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
    if (yaml.show_gui) vio->out_vis_queue = &out_vis_queue;
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

  vio_data_log.Clear();

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

  if (yaml.show_gui)
  {
    t3.reset(new std::thread([&]() {
      basalt::VioVisualizationData::Ptr data;

      while (true) {
        out_vis_queue.pop(data);

        if (data.get()) {
          vis_map[data->t_ns] = data;
          vis_ts_queue.push(data->t_ns); // added by wxliu on 2023-11-11.
        } else {
          break;
        }
      }

      std::cout << "Finished t3" << std::endl;
    }));
  }
  // 2023-11-11
  else 
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

    // 2023-11-10.
    bool bFirst = true;
    int64_t start_t_ns = 0;
    // the end.

    //while (true) {
    while (!terminate) {
      out_state_queue.pop(data);

      if (!data.get()) break;

      if(1) // 2023-11-11
      {
        node->PublishOdometry(data);
      }
      else{

      int64_t t_ns = data->t_ns;
      if(bFirst)
      {
        bFirst = false;
        start_t_ns = t_ns;
      }

      // std::cerr << "t_ns " << t_ns << std::endl;
      Sophus::SE3d T_w_i = data->T_w_i;
      Eigen::Vector3d vel_w_i = data->vel_w_i;
      Eigen::Vector3d bg = data->bias_gyro;
      Eigen::Vector3d ba = data->bias_accel;

      vio_t_ns.emplace_back(data->t_ns);
      vio_t_w_i.emplace_back(T_w_i.translation());
      vio_T_w_i.emplace_back(T_w_i);

      if (yaml.show_gui) {
        std::vector<float> vals;
        vals.push_back((t_ns - start_t_ns) * 1e-9);

        for (int i = 0; i < 3; i++) vals.push_back(vel_w_i[i]);
        for (int i = 0; i < 3; i++) vals.push_back(T_w_i.translation()[i]);
        for (int i = 0; i < 3; i++) vals.push_back(bg[i]);
        for (int i = 0; i < 3; i++) vals.push_back(ba[i]);

        vio_data_log.Log(vals);
      }
    } // added this line on 2023-11-11.
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

#ifdef _SHOW_UI_

  auto time_start = std::chrono::high_resolution_clock::now();

  // record if we close the GUI before VIO is finished.
  bool aborted = false;

  if (yaml.show_gui) {
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    pangolin::View& main_display = pangolin::CreateDisplay().SetBounds(
        0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    pangolin::View& img_view_display = pangolin::CreateDisplay()
                                           .SetBounds(0.4, 1.0, 0.0, 0.4)
                                           .SetLayout(pangolin::LayoutEqual);

    pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
        0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100, -10.0, 10.0, 0.01f,
                                    0.01f);
    plot_display.AddDisplay(*plotter);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    while (img_view.size() < calib.intrinsics.size()) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display.AddDisplay(*iv);
      iv->extern_draw_function =
          std::bind(&draw_image_overlay, std::placeholders::_1, idx);
    }

    Eigen::Vector3d cam_p(-0.5, -3, -5);
    cam_p = vio->getT_w_i_init().so3() * calib.T_i_c[0].so3() * cam_p;

    camera = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0,
                                  pangolin::AxisZ));

    pangolin::View& display3D =
        pangolin::CreateDisplay()
            .SetAspect(-640 / 480.0)
            .SetBounds(0.4, 1.0, 0.4, 1.0)
            .SetHandler(new pangolin::Handler3D(camera));

    display3D.extern_draw_function = draw_scene;

    main_display.AddDisplay(img_view_display);
    main_display.AddDisplay(display3D);
std::cout << "224---\n";
    while (!pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
std::cout << "226---\n";
      if (follow) {
        size_t frame_id = show_frame;
        //int64_t t_ns = vio_dataset->get_image_timestamps()[frame_id]; // comment 2023-11-11
        int64_t t_ns = 0;
        vis_ts_queue.pop(t_ns);
        auto it = vis_map.find(t_ns);

        if (it != vis_map.end()) {
          Sophus::SE3d T_w_i;
          if (!it->second->states.empty()) {
            T_w_i = it->second->states.back();
          } else if (!it->second->frames.empty()) {
            T_w_i = it->second->frames.back();
          }
          T_w_i.so3() = Sophus::SO3d();

          camera.Follow(T_w_i.matrix());
        }
      }

      display3D.Activate(camera);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      img_view_display.Activate();

      if (show_frame.GuiChanged()) { // 在pangolin上可以通过点击"show_frame"按钮来回放帧和轨迹的状态
        for (size_t cam_id = 0; cam_id < calib.intrinsics.size(); cam_id++) {
          size_t frame_id = static_cast<size_t>(show_frame);
          int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];

          std::vector<basalt::ImageData> img_vec =
              vio_dataset->get_image_data(timestamp);

          pangolin::GlPixFormat fmt;
          fmt.glformat = GL_LUMINANCE;
          fmt.gltype = GL_UNSIGNED_SHORT;
          fmt.scalable_internal_format = GL_LUMINANCE16;

          if (img_vec[cam_id].img.get())
            img_view[cam_id]->SetImage(
                img_vec[cam_id].img->ptr, img_vec[cam_id].img->w,
                img_vec[cam_id].img->h, img_vec[cam_id].img->pitch, fmt);
        }

        draw_plots();
      }

      if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() ||
          show_est_ba.GuiChanged() || show_est_bg.GuiChanged()) {
        draw_plots();
      }

      if (euroc_fmt.GuiChanged()) {
        euroc_fmt = true;
        tum_rgbd_fmt = false;
        kitti_fmt = false;
      }

      if (tum_rgbd_fmt.GuiChanged()) {
        tum_rgbd_fmt = true;
        euroc_fmt = false;
        kitti_fmt = false;
      }

      if (kitti_fmt.GuiChanged()) {
        kitti_fmt = true;
        euroc_fmt = false;
        tum_rgbd_fmt = false;
      }

      //      if (record) {
      //        main_display.RecordOnRender(
      //            "ffmpeg:[fps=50,bps=80000000,unique_filename]///tmp/"
      //            "vio_screencap.avi");
      //        record = false;
      //      }

      pangolin::FinishFrame();

      if (continue_btn) {
        if (!next_step())
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      if (continue_fast) {
        int64_t t_ns = vio->last_processed_t_ns;
        if (timestamp_to_id.count(t_ns)) {
          show_frame = timestamp_to_id[t_ns];
          show_frame.Meta().gui_changed = true;
        }

        if (vio->finished) {
          continue_fast = false;
        }
      }
    }

    // If GUI closed but VIO not yet finished --> abort input queues, which in
    // turn aborts processing
    if (!vio->finished) {
      std::cout << "GUI closed but odometry still running --> aborting.\n";
      print_queue_fn();  // print queue size at time of aborting
      terminate = true;
      aborted = true;
    }
  }
#endif

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

#if 0
  auto time_end = std::chrono::high_resolution_clock::now();
  const double duration_total =
      std::chrono::duration<double>(time_end - time_start).count();

  // TODO: remove this unconditional call (here for debugging);
  const double ate_rmse =
      basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i);
  vio->debug_finalize();
  std::cout << "Total runtime: {:.3f}s\n"_format(duration_total);

  {
    basalt::ExecutionStats stats;
    stats.add("exec_time_s", duration_total);
    stats.add("ate_rmse", ate_rmse);
    stats.add("ate_num_kfs", vio_t_w_i.size());
    stats.add("num_frames", vio_dataset->get_image_timestamps().size());

    {
      basalt::MemoryInfo mi;
      if (get_memory_info(mi)) {
        stats.add("resident_memory_peak", mi.resident_memory_peak);
      }
    }

    stats.save_json("stats_vio.json");
  }

  if (!aborted && !yaml.trajectory_fmt.empty()) {
    std::cout << "Saving trajectory..." << std::endl;

    if (yaml.trajectory_fmt == "kitti") {
      kitti_fmt = true;
      euroc_fmt = false;
      tum_rgbd_fmt = false;
    }
    if (yaml.trajectory_fmt == "euroc") {
      euroc_fmt = true;
      kitti_fmt = false;
      tum_rgbd_fmt = false;
    }
    if (yaml.trajectory_fmt == "tum") {
      tum_rgbd_fmt = true;
      euroc_fmt = false;
      kitti_fmt = false;
    }

    save_groundtruth = yaml.trajectory_groundtruth;

    saveTrajectoryButton(); // 响应pangonlin上用于保存轨迹的按钮
  }

  if (!aborted && !yaml.result_path.empty()) {
    double error = basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i);

    auto exec_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_end - time_start);

    std::ofstream os(yaml.result_path);
    {
      cereal::JSONOutputArchive ar(os);
      ar(cereal::make_nvp("rms_ate", error));
      ar(cereal::make_nvp("num_frames",
                          vio_dataset->get_image_timestamps().size()));
      ar(cereal::make_nvp("exec_time_ns", exec_time_ns.count()));
    }
    os.close();
  }
#endif

  return 0;
}

// 画出图片覆盖物
void draw_image_overlay(pangolin::View& v, size_t cam_id) { // 画出图片上的特征点，以及特征点跟踪的个数
  UNUSED(v);

  //  size_t frame_id = show_frame;
  //  basalt::TimeCamId tcid =
  //      std::make_pair(vio_dataset->get_image_timestamps()[frame_id],
  //      cam_id);

  size_t frame_id = show_frame;
  auto it = vis_map.find(vio_dataset->get_image_timestamps()[frame_id]);

  if (show_obs) { // 显示追踪的特征点
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (it != vis_map.end() && cam_id < it->second->projections.size()) {
      const auto& points = it->second->projections[cam_id];

      if (points.size() > 0) {
        double min_id = points[0][2], max_id = points[0][2];

        for (const auto& points2 : it->second->projections)
          for (const auto& p : points2) {
            min_id = std::min(min_id, p[2]);
            max_id = std::max(max_id, p[2]);
          }

        for (const auto& c : points) {
          const float radius = 6.5;

          float r, g, b;
          getcolor(c[2] - min_id, max_id - min_id, b, g, r);
          glColor3f(r, g, b);

          pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

          if (show_ids)
            pangolin::GlFont::I().Text("%d", int(c[3])).Draw(c[0], c[1]);
        }
      }

      glColor3f(1.0, 0.0, 0.0);
      pangolin::GlFont::I()
          .Text("Tracked %d points", points.size())
          .Draw(5, 20);
    }
  }

  if (show_flow) { // 显示光流patch
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (it != vis_map.end()) {
      const Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>&
          kp_map = it->second->opt_flow_res->observations[cam_id];

      for (const auto& kv : kp_map) {
        Eigen::MatrixXf transformed_patch =
            kv.second.linear() * opt_flow_ptr->patch_coord;
        transformed_patch.colwise() += kv.second.translation();

        for (int i = 0; i < transformed_patch.cols(); i++) {
          const Eigen::Vector2f c = transformed_patch.col(i);
          pangolin::glDrawCirclePerimeter(c[0], c[1], 0.5f);
        }

        const Eigen::Vector2f c = kv.second.translation();

        if (show_ids)
          pangolin::GlFont::I().Text("%d", kv.first).Draw(5 + c[0], 5 + c[1]);
      }

      pangolin::GlFont::I()
          .Text("%d opt_flow patches", kp_map.size())
          .Draw(5, 20);
    }
  }
}

void draw_scene(pangolin::View& view) {
  UNUSED(view);
  view.Activate(camera);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  glPointSize(3);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor3ubv(cam_color);
  if (!vio_t_w_i.empty()) {
    size_t end = std::min(vio_t_w_i.size(), size_t(show_frame + 1));
    Eigen::aligned_vector<Eigen::Vector3d> sub_gt(vio_t_w_i.begin(),
                                                  vio_t_w_i.begin() + end);
    pangolin::glDrawLineStrip(sub_gt);
  }

  glColor3ubv(gt_color);
  if (show_gt) pangolin::glDrawLineStrip(gt_t_w_i);

  size_t frame_id = show_frame;
  int64_t t_ns = vio_dataset->get_image_timestamps()[frame_id];
  auto it = vis_map.find(t_ns);

  if (it != vis_map.end()) {
    for (size_t i = 0; i < calib.T_i_c.size(); i++)
      if (!it->second->states.empty()) {
        render_camera((it->second->states.back() * calib.T_i_c[i]).matrix(),
                      2.0f, cam_color, 0.1f);
      } else if (!it->second->frames.empty()) {
        render_camera((it->second->frames.back() * calib.T_i_c[i]).matrix(),
                      2.0f, cam_color, 0.1f);
      }

    for (const auto& p : it->second->states)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);

    for (const auto& p : it->second->frames)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, pose_color, 0.1f);

    glColor3ubv(pose_color);
    pangolin::glDrawPoints(it->second->points);
  }

  pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
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

bool next_step() {
  if (show_frame < int(vio_dataset->get_image_timestamps().size()) - 1) {
    show_frame = show_frame + 1;
    show_frame.Meta().gui_changed = true;
    cv.notify_one();
    return true;
  } else {
    return false;
  }
}

bool prev_step() {
  if (show_frame > 1) {
    show_frame = show_frame - 1;
    show_frame.Meta().gui_changed = true;
    return true;
  } else {
    return false;
  }
}

void draw_plots() {
  plotter->ClearSeries();
  plotter->ClearMarkers();

  if (show_est_pos) {
    plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "position x", &vio_data_log);
    plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "position y", &vio_data_log);
    plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "position z", &vio_data_log);
  }

  if (show_est_vel) {
    plotter->AddSeries("$0", "$1", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "velocity x", &vio_data_log);
    plotter->AddSeries("$0", "$2", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "velocity y", &vio_data_log);
    plotter->AddSeries("$0", "$3", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "velocity z", &vio_data_log);
  }

  if (show_est_bg) {
    plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "gyro bias x", &vio_data_log);
    plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "gyro bias y", &vio_data_log);
    plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
  }

  if (show_est_ba) {
    plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "accel bias x", &vio_data_log);
    plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "accel bias y",
                       &vio_data_log);
    plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "accel bias z", &vio_data_log);
  }

  double t = vio_dataset->get_image_timestamps()[show_frame] * 1e-9;
  plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
                     pangolin::Colour::White());
}

void alignButton() { basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i); }

void saveTrajectoryButton() {
  if (tum_rgbd_fmt) {
    {
      std::ofstream os("trajectory.txt");

      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for (size_t i = 0; i < vio_t_ns.size(); i++) {
        const Sophus::SE3d& pose = vio_T_w_i[i];
        os << std::scientific << std::setprecision(18) << vio_t_ns[i] * 1e-9
           << " " << pose.translation().x() << " " << pose.translation().y()
           << " " << pose.translation().z() << " " << pose.unit_quaternion().x()
           << " " << pose.unit_quaternion().y() << " "
           << pose.unit_quaternion().z() << " " << pose.unit_quaternion().w()
           << std::endl;
      }

      os.close();
    }

    if (save_groundtruth) {
      std::ofstream os("groundtruth.txt");

      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for (size_t i = 0; i < gt_t_ns.size(); i++) {
        const Eigen::Vector3d& pos = gt_t_w_i[i];
        os << std::scientific << std::setprecision(18) << gt_t_ns[i] * 1e-9
           << " " << pos.x() << " " << pos.y() << " " << pos.z() << " "
           << "0 0 0 1" << std::endl;
      }

      os.close();
    }

    std::cout
        << "Saved trajectory in TUM RGB-D Dataset format in trajectory.txt"
        << std::endl;
  } else if (euroc_fmt) {
    std::ofstream os("trajectory.csv");

    os << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w "
          "[],q_RS_x [],q_RS_y [],q_RS_z []"
       << std::endl;

    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      const Sophus::SE3d& pose = vio_T_w_i[i];
      os << std::scientific << std::setprecision(18) << vio_t_ns[i] << ","
         << pose.translation().x() << "," << pose.translation().y() << ","
         << pose.translation().z() << "," << pose.unit_quaternion().w() << ","
         << pose.unit_quaternion().x() << "," << pose.unit_quaternion().y()
         << "," << pose.unit_quaternion().z() << std::endl;
    }

    std::cout << "Saved trajectory in Euroc Dataset format in trajectory.csv"
              << std::endl;
  } else {
    std::ofstream os("trajectory_kitti.txt");

    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      Eigen::Matrix<double, 3, 4> mat = vio_T_w_i[i].matrix3x4();
      os << std::scientific << std::setprecision(12) << mat.row(0) << " "
         << mat.row(1) << " " << mat.row(2) << " " << std::endl;
    }

    os.close();

    std::cout
        << "Saved trajectory in KITTI Dataset format in trajectory_kitti.txt"
        << std::endl;
  }
}
