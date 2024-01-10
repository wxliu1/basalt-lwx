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

#include "wx_ros1_io.h" // 2023-11-10.
#include "wx_yaml_io.h"
#include "imu/imu_process.h"
#include <signal.h>

#include "tks_pro/tks_pro.hpp"

#include <unistd.h>
#include <fcntl.h>


// #define _RECORD_BAG_

#if 0//def _RECORD_BAG_
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ctime>
#include <iomanip>
#include <topic_tools/shape_shifter.h>
#endif

#include "util/compile_date_time.h"

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

// 2023-11-20.
// std::mutex reset_mutex;
// std::mutex opt_mutex;
// std::mutex vio_mutex;
// the end.

// 2023-11-21.
std::mutex vio_m;
std::condition_variable vio_cv;
// the end.

bool step_by_step = false;
size_t max_frames = 0;

std::atomic<bool> terminate = false;

// VIO variables
basalt::Calibration<double> calib;

basalt::VioDatasetPtr vio_dataset;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;

std::shared_ptr<Tks_pro> tks_pro;
bool is_forward = false;

// 2023-11-13
ImuProcess* g_imu = nullptr;
// the end.

std::function<void(void)> resetPublishedOdom_;
std::function<void(bool)> setForward_;

#ifdef _RECORD_BAG_
std::function<void(void)> openRosbag_;
std::function<void(void)> closeRosBag_;
#endif

void zeroVelocity(bool bl)
{
  opt_flow_ptr->SetZeroVelocity(bl);

  // if(bl != g_isZeroVelocity)
  // {
  //   g_isZeroVelocity = bl;
  //   if(g_isZeroVelocity)
  //   {
  //     opt_flow_ptr->Reset();
  //     vio->Reset();
  //   }
  // }

}

void slowVelocity(bool bl)
{
  opt_flow_ptr->SetSlowVelocity(bl);
}

// 2023-11-10.
void feedImage(basalt::OpticalFlowInput::Ptr data, ImuProcess* imu) 
{
  opt_flow_ptr->input_queue.push(data);
}

void feedImu(basalt::ImuData<double>::Ptr data) 
{
  vio->imu_data_queue.push(data);
}
// the end.

// Feed functions

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

void Reset()
{

/* 
 * 2023-11-20.
 * another reset method is to quit threads and reconstruct the objects:
  opt_flow_ptr->input_queue.push(nullptr);
  vio->imu_data_queue.push(nullptr);

  opt_mutex.lock();
  // reconstruct optical flow object
  opt_flow_ptr =
    basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
  opt_mutex.unlock();
  vio_mutex.lock();
  // reconstruct vio object
  vio_mutex.unlock();
*/

/*
 * comment 2023-12-21.
  opt_flow_ptr->Reset();
  vio->Reset();

 */

/*
 * comment 2023-12-22.

  // new method: ensure vio can be reset.
  std::shared_ptr<std::thread> t1;
  t1.reset(new std::thread([&]() {
    // while(true) {
    // }
    vio->Reset();

    std::cout << "Finished t1: vio reset." << std::endl;
  }));

  opt_flow_ptr->Reset();

  if(t1) {
    t1->join();
    // t1->detach();
  } 
  // the end.
*/

  wx::TFileSystemHelper::WriteLog("try to reset alogorithm.");

  // Make sure it's safe
  if(!vio->GetResetAlgorithm())
  {
    vio->SetResetAlgorithm(true);
    if(vio->GetResetAlgorithm())
    opt_flow_ptr->Reset();
     wx::TFileSystemHelper::WriteLog("reset alogorithm completed.");
  }
  else
  {
    std::cout << "reset alogorithm is not complete yet." << std::endl;
    wx::TFileSystemHelper::WriteLog("reset alogorithm is not complete yet.");
  }
  

}

void ClearPose()
{
  if(!vio->GetResetAlgorithm())
  {
    vio->SetResetAlgorithm(true);
    if(vio->GetResetAlgorithm())
    {
      vio->SetResetInitPose(true);
      opt_flow_ptr->Reset();
    }

  }
  else
  {
    std::cout << "reset alogorithm is not complete yet." << std::endl;
  }
}

void command(TYamlIO *yaml)
{
  std::cout << "1 command()" << std::endl;
  while (1)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    if (c == 'r')
    // if (strInput == "reboot stereo3")
    {
      std::cout << "press 'r' to reset algorithm" << std::endl;
      Reset();
      std::cout << "reset command over." << std::endl;
    }
    else if('c' == c)
    {
      std::cout << "press 'c' to clear pose & reset algorithm" << std::endl;
      ClearPose();
      std::cout << "clear command over." << std::endl;
    }

    #ifdef _RECORD_BAG_
    
    {
      if(c == 'w')
      {
        if (!yaml->record_bag)
        {
          std::cout << "press 'w' to write bag" << std::endl;
          if(openRosbag_)
          openRosbag_();
        }
        else{ 
          std::cout << "now is automatic record bag mode. can't open bag manually." << std::endl;
        }
        
      }
      else if(c == 'x')
      {
        if(!yaml->record_bag)
        {
          std::cout << "press 'x' to close bag" << std::endl;
          if(closeRosBag_)
          closeRosBag_();
        }
        else
        {
          std::cout << "now is automatic record bag mode. can't close bag manually." << std::endl;
        }
      }
    }
    
    #endif

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

void reset_algorithm_thread(TYamlIO *yaml)
{
  std::cout<< std::boolalpha << " reset_algorithm_thread is_forward=" << is_forward
    << "  tks_pro->IsForward() = " << tks_pro->IsForward() << std::endl;
  while (1)
  {
  #if 1
    if(tks_pro.get() && tks_pro->IsForward())
    {
      if(is_forward == false)
      {
        wx::TFileSystemHelper::WriteLog("backward to forward, reset algorithm");
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * yaml->change_end_wait_time)));
        is_forward = true;
        std::cout << "because of 'is_forward' changed from false to true, we start to reset algorithm." << std::endl;
        Reset();
        setForward_(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(resetPublishedOdom_)
        {
          resetPublishedOdom_();
        }
      }
    }
    else
    {
      if(is_forward)
      {
        wx::TFileSystemHelper::WriteLog("forward to backward. algorithm is disabled.");
        std::cout << "'is_forward' changed from true to false." << std::endl;
        is_forward = false;
        setForward_(false);
      }
    }
  #endif

    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}

#if 0//def _RECORD_BAG_
void record_rosbag_file()
{
  // format bag name
  std::time_t current_time = std::time(nullptr);
  std::tm* time_info = std::localtime(&current_time);
  char buffer[50];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", time_info);
  std::string bag_name = "./" + std::string(buffer) + ".bag";

  // add topic list
  std::vector<std::string> topic_list;
  topic_list.emplace_back("/atp_info");
  topic_list.emplace_back("/camera_left_info");
  topic_list.emplace_back("/camera_right_info");
  topic_list.emplace_back("/image_left");
  topic_list.emplace_back("/image_left_info");
  topic_list.emplace_back("/image_right");
  topic_list.emplace_back("/image_right_info");
  topic_list.emplace_back("/imu");

  rosbag::Bag bag;

  try {
      bag.open(bag_name.c_str(), rosbag::bagmode::Write);
  } catch (rosbag::BagException &e) {
      ROS_ERROR("open rosbag failed: %s", e.what());
      return ;
  }

  std::vector<ros::Subscriber> sub_list;
  for (auto & item : topic_list) {
      ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(item, 1000, std::bind(genericCallback, std::placeholders::_1, &bag, item));        
      sub_list.push_back(sub);
      ROS_INFO("subscribed to %s", item.c_str());
  }

  ros::spin();

  bag.close();
}
#endif

static int check_pid_lock()
{
	int fd;
	pid_t id;
	struct flock lk;
	char fname[128];

	// sprintf(fname, "./lock/stereo3.pid");
	sprintf(fname, "./stereo3.pid");
	if ((fd = open(fname, O_CREAT | O_WRONLY, 0666)) < 0) {
		// log(LOG_ERR, "Fail open %s\n", fname);
		return -1;
	}

	lk.l_type = F_WRLCK;
	lk.l_start = 0;
	lk.l_len = 0;
	lk.l_whence = SEEK_SET;
	if (fcntl(fd, F_SETLK, &lk) == -1) {
		// log(LOG_ERR, "Fail lock %s\n", fname);
		return -1;
	}
	id = getpid();
	if (write(fd, &id, sizeof(pid_t)) != sizeof(pid_t)) {
		// log(LOG_ERR, "Fail write PID to %s\n", fname);
		return -1;
	}
	return 0;
}

int GetModuleFileName(char *pathbuf, int buflen)
{
	int ret;
	void *address;
    address = (void*)(&GetModuleFileName);

	if((pathbuf == NULL) || (buflen <=0))
		return -1;

    Dl_info dl_info;
    dl_info.dli_fname = 0;
    ret = dladdr(address, &dl_info);
    if (0 != ret && dl_info.dli_fname != NULL) {
        if(strlen(dl_info.dli_fname) >= buflen) {
			return -2;
		}

		strcpy(pathbuf, dl_info.dli_fname);
		return 0;
    }

    return -3;
}

int main(int argc, char** argv) {

  if (check_pid_lock()) return -1;

  // 注册中断信号处理函数
  //signal(SIGINT, sigintHandler); // 2023-11-11

/*
 * comment on 2023-11-10.
  // step 1: 命令行参数解析  
  CLI::App app{"App description"};
*/


#if 0 //def _RECORD_BAG_
  std::thread record_process;
  record_process = std::thread(record_rosbag_file);
  // record_process.join();
#endif

  // int *p = NULL;
  // std::cout<<*p<<std::endl;

  sys_cfg_.ReadSystemCfg(); // 2023-11-17.

  // 2023-11-10
  struct TYamlIO yaml;
  yaml.ReadConfiguration();

#ifdef _VERIFY_CONFIG_FILE 
  std::cout << "calib_path=" << yaml.cam_calib_path << std::endl
    << "config_path=" << yaml.config_path << std::endl
    << "dt_ns = " << yaml.dt_ns << std::endl
    << "output_data_file = " << yaml.output_data_file << std::endl;
 
  int cnt = yaml.vec_tracked_points.size();
  std::string strConfidenceInterval = "tracked_points:[";
  for(int i = 0; i < cnt; i++)
  {
    strConfidenceInterval += std::to_string(yaml.vec_tracked_points[i]) + ",";
    if(i == cnt - 1)
    {
      strConfidenceInterval[strConfidenceInterval.size() - 1] = ']';
    }
  }

  std::cout << strConfidenceInterval << std::endl;

  cnt = yaml.vec_confidence_levels.size();
  strConfidenceInterval = "confidence_levels:[";
  for(int i = 0; i < cnt; i++)
  {
    strConfidenceInterval += std::to_string(yaml.vec_confidence_levels[i]);
    if(i == cnt - 1)
    {
      strConfidenceInterval[strConfidenceInterval.size() - 1] = ']';
    }
    else
    {
      strConfidenceInterval += ",";
    }
  }

  std::cout << strConfidenceInterval << std::endl;
#endif

  // the end.

  std::cout << std::boolalpha << "output_log=" << yaml.output_log << std::endl;
  sys_cfg_.output_log = yaml.output_log;
  if(yaml.output_log)
  {
    char szModulePath[260] = { 0 };
    if(0 == GetModuleFileName(szModulePath, 260)) {
        //szModulePath 就是.so文件的绝对路径。
        // module path:./zc_server
        std::cout << "module path:" << szModulePath << std::endl;
    }

    // char absolutePath[1024] = { 0 };
    // wx::liu::FileSystemHelper::getAbsolutePath(absolutePath, argv[0]);
    // printf("absolutePath = %s\n", absolutePath);

    // szModulePath: /root/dev/stereo3_ros1_ws/install/lib/stereo3/stereo3
    char *ptr = strrchr(szModulePath, '/');
    *ptr = '\0';

    strcat(szModulePath, "/logs");

    sys_cfg_.log_file_path = szModulePath;
    std::cout << "log file path:" << sys_cfg_.log_file_path << std::endl;


    // strcat(szModulePath, "/");
    wx::TFileSystemHelper::createDirectoryIfNotExists(szModulePath);
    // wx::TFileSystemHelper::CreateDir(szModulePath);

  #ifdef _PROGRAM_VERSION_  
    std::cout<< "PROJECT_VERSION_REVISION=" << _PROGRAM_VERSION_ << std::endl;
    // char szLog[256] = "_PROGRAM_VERSION_"; //{ 0 };
    // sprintf(szLog, "\nprogram_version_revision = %s", *_PROGRAM_VERSION_);
    // wx::TFileSystemHelper::WriteLog(szLog);
  #endif 

    wx::TFileSystemHelper::WriteLog("main()");
    char szLog[256] = { 0 };
    // sprintf(szLog, "program_version_revision = %s_%s", __DATE__, __TIME__);
    sprintf(szLog, "program_version_revision = %s", g_build_date_time);
    wx::TFileSystemHelper::WriteLog(szLog);

  }

  std::thread keyboard_command_process;
  keyboard_command_process = std::thread(command, &yaml);
  // keyboard_command_process.join();

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
#if USE_TIGHT_COUPLING     
    vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, yaml.use_imu, yaml.use_double); // 创建后端估计器对象
#else // LOOSE_COUPLING
    vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, false, yaml.use_double);
#endif

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

  vio->reset_ = std::bind(&Reset); // 2023-11-19.

  opt_flow_ptr->reset_ = std::bind(&basalt::VioEstimatorBase::Reset, vio);

  // 2023-11-14
  if (sys_cfg_.use_imu) {
    imu.SetTwc0_ = std::bind(&basalt::VioEstimatorBase::setT_w_i_init, vio, std::placeholders::_1);
  }
  // the end.

  // 2023-11-10.
 
  ros::init(argc, argv, "stereo3_node");  // node name
  ros::MultiThreadedSpinner spinner(6);  // use 6 threads

  // wx::CRos1IO node{ros::NodeHandle{"~"}, yaml.use_imu, yaml.fps, yaml.vec_tracked_points, yaml.vec_confidence_levels, yaml.dt_ns};
  // wx::CRos1IO node(ros::NodeHandle{"~"}, yaml); // it's ok.
/*
 * comment 2023-12-23.  
  wx::CRos1IO node {ros::NodeHandle{"~"}, yaml };
*/
  ros::NodeHandle n("~");
  wx::CRos1IO node{n, yaml};

  node.inputIMU_ = std::bind(&ImuProcess::inputIMU, &imu, std::placeholders::_1, 
    std::placeholders::_2, std::placeholders::_3);

  node.feedImage_ = std::bind(&feedImage, std::placeholders::_1, &imu);
  // node->feedImage_ = std::bind(&feedImage, std::placeholders::_1);
#if USE_TIGHT_COUPLING  
  node->feedImu_ = std::bind(&feedImu, std::placeholders::_1);
#endif 
  node.stop_ = std::bind(&stop);
  node.zeroVelocity_ = std::bind(&zeroVelocity, std::placeholders::_1);
  node.slowVelocity_ = std::bind(&slowVelocity, std::placeholders::_1);
  node.reset_ = std::bind(&Reset);
  // vio->resetPublishedOdom_ = std::bind(&wx::CRos1IO::ResetPublishedOdom, &node);
  vio->resetPublishedOdom_ = std::bind(&wx::CRos1IO::Reset, &node); // For another use
  resetPublishedOdom_ = std::bind(&wx::CRos1IO::ResetPublishedOdom, &node);
  setForward_ = std::bind(&wx::CRos1IO::SetForward, &node, std::placeholders::_1);
#ifdef _RECORD_BAG_
  openRosbag_ = std::bind(&wx::CRos1IO::OpenRosbag, &node);
  closeRosBag_ = std::bind(&wx::CRos1IO::CloseRosBag, &node);
#endif
  // the end.

  std::thread reset_process;

  if(yaml.tks_pro_integration){
    tks_pro = std::make_shared<Tks_pro>(n, yaml);
    node.add_odom_frame_ = std::bind(&Tks_pro::add_odom_frame, tks_pro, std::placeholders::_1, 
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

    node.isForward_ = std::bind(&Tks_pro::IsForward, tks_pro);

    // std::thread reset_process;
    reset_process = std::thread(reset_algorithm_thread, &yaml);
    // reset_process.join();

    tks_pro->atp_cb_ = std::bind(&wx::CRos1IO::atp_cb, &node, std::placeholders::_1);

  }

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
        out_vis_queue.pop(data); // 当队列中数据为空时，该并发队列处于阻塞状态。

        if (data.get()) {
          node.PublishPoints(data);
          node.PublishFeatureImage(data);
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

      //node->PublishOdometry(data); // 2023-11-11
      node.PublishPoseAndPath(data); // 2023-11-16
      
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
  // ros::spin(); // comment 
  spinner.spin();  // add this line 2023-12-04
  node.CloseRosBag();
#ifdef _RECORD_BAG_  
  node.stop_();
#endif  
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
