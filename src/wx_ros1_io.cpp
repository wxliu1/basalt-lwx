
// @author: wxliu
// @date: 2023-11-9

#include "wx_ros1_io.h"
#include <basalt/image/image.h>
// #include <basalt/utils/vis_utils.h>
#include <opencv2/highgui.hpp>

#include <basalt/io/dataset_io.h>

#include <queue>

extern basalt::OpticalFlowBase::Ptr opt_flow_ptr;  // 2023-11-28.

// #include <boost/circular_buffer.hpp>

// #define _TEST_ROS2_IO

// #define _FILTER_IN_SECONDS_
// #define _VELOCITY_FILTER_
#define _MULTI_VELOCITY_FILTER_

// #define _REMOVE_OUTLIER_FILTER_

// #define _Linear_Fitted5_

#define _VERIFY_CONFIG_FILE

using namespace wx;

CRos1IO::CRos1IO(const ros::NodeHandle& pnh, const TYamlIO& yaml) noexcept
    // CRos1IO::CRos1IO(const ros::NodeHandle& pnh, bool use_imu, int fps, long
    // dt_ns) noexcept
    : pnh_(pnh),
      yaml_(yaml),
      fps_(yaml_.fps),
      dt_ns_(yaml_.dt_ns),
      sub_image0_(pnh_, "/image_left", 5),
      sub_image1_(pnh_, "/image_right", 5)
// , sub_image0_info_(this, "/image_left_info")
// , sub_image1_info_(this, "/image_right_info")
// , sync_stereo_(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_,
// 10) // method 1
{
#ifdef _KF_
  // Kalman_Filter_Init(&kalman_filter_pt1, 0.001, 2, 25, 1);
  Kalman_Filter_Init(&kalman_filter_pt1, 0.001, 2, 0, 1);
#endif

#ifdef _VERIFY_CONFIG_FILE
  std::cout << std::boolalpha << "CRos1IO---\n"
            << "calib_path=" << yaml_.cam_calib_path << std::endl
            << "config_path=" << yaml_.config_path << std::endl
            << "dt_ns = " << yaml_.dt_ns << std::endl
            << "slow_velocity = " << yaml_.slow_velocity
            << "  zero_velocity = " << yaml_.zero_velocity << std::endl
            << "mean_value = " << yaml_.mean_value << std::endl
            << "number_of_255 = " << yaml_.number_of_255 << std::endl
            << "record_bag = " << yaml_.record_bag << std::endl
            << "record_duration = " << yaml_.record_duration << std::endl;

  int cnt = yaml_.vec_tracked_points.size();
  std::string strConfidenceInterval = "tracked_points:[";
  for (int i = 0; i < cnt; i++) {
    strConfidenceInterval += std::to_string(yaml_.vec_tracked_points[i]) + ",";
    if (i == cnt - 1) {
      strConfidenceInterval[strConfidenceInterval.size() - 1] = ']';
    }
  }

  std::cout << strConfidenceInterval << std::endl;

  cnt = yaml_.vec_confidence_levels.size();
  strConfidenceInterval = "confidence_levels:[";
  for (int i = 0; i < cnt; i++) {
    strConfidenceInterval += std::to_string(yaml_.vec_confidence_levels[i]);
    if (i == cnt - 1) {
      strConfidenceInterval[strConfidenceInterval.size() - 1] = ']';
    } else {
      strConfidenceInterval += ",";
    }
  }

  std::cout << strConfidenceInterval << std::endl;
  std::cout << "CRos1IO---THE END---\n";
#endif

  // create work thread
  t_publish_myodom = std::thread(&CRos1IO::PublishMyOdomThread, this);
 #ifdef _RECORD_BAG_
  if (yaml_.record_bag)
    t_record_bag = std::thread(&CRos1IO::RecordBagThread, this);
#endif

#ifdef _RECORD_BAG_
  t_write_bag = std::thread(&CRos1IO::WriteBagThread, this);
#endif

  use_imu_ = yaml_.use_imu;
  sync_stereo_.emplace(sub_image0_, sub_image1_, 5);  // method 2
  sync_stereo_->registerCallback(&CRos1IO::StereoCb, this);

  pub_point_ = pnh_.advertise<sensor_msgs::PointCloud2>("/point_cloud2", 10);
  pub_odom_ = pnh_.advertise<nav_msgs::Odometry>("/pose_odom", 10);

  path_msg_.poses.reserve(
      1024);  // reserve的作用是更改vector的容量（capacity），使vector至少可以容纳n个元素。
              // 如果n大于vector当前的容量，reserve会对vector进行扩容。其他情况下都不会重新分配vector的存储空间

  // pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path_odom", 2);
  pub_path_ = pnh_.advertise<nav_msgs::Path>("/path_odom", 1);

  if (use_imu_) {
    // sub_imu_ = pnh.subscribe("/imu", 2000, imu_callback,
    // ros::TransportHints().tcpNoDelay());
    sub_imu_ = pnh_.subscribe("/imu", 2000, &CRos1IO::imu_callback, this,
                              ros::TransportHints().tcpNoDelay());
  }

  // pub_warped_img =
  // this->create_publisher<sensor_msgs::msg::Image>("/feature_img", 1); //
  // feature_img // warped_img
  pub_warped_img = pnh_.advertise<sensor_msgs::Image>("feature_img", 5);

  pub_my_odom_ = pnh_.advertise<myodom::MyOdom>("/my_odom", 10);
}

#ifdef _RECORD_BAG_
void CRos1IO::OpenRosbag() {
  try {
    CloseRosBag();
    std::cout << "1 open rosbag" << std::endl;
    bag_.open(bag_name.c_str(), rosbag::bagmode::Write);
    SetRecordBag(true);
  } catch (rosbag::BagException& e) {
    ROS_ERROR("open rosbag failed: %s", e.what());
    // return ;
  }
}

void CRos1IO::CloseRosBag() {
  if (GetRecordBag()) {
    SetRecordBag(false);
    usleep(1000 * 100);  // 100ms
    std::cout << "1 close rosbag" << std::endl;
    bag_.close();
  }
}

void CRos1IO::WriteBagThread()
{
  TStereoImage stereo_img_msg;
  sensor_msgs::ImuConstPtr imu_msg;
  while (1) {
    
    if (!record_bag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue ;
    }

    // if (!imu_queue.empty()) {
    while (!imu_queue.empty()) {
      imu_queue.pop(imu_msg);
      // handle data now.
       bag_.write("/imu", imu_msg->header.stamp, *imu_msg);
    }

    if (!stereo_img_queue.empty()) {
      stereo_img_queue.pop(stereo_img_msg);
      // handle data now.
      bag_.write("/image_left", stereo_img_msg.image0_ptr->header.stamp, *stereo_img_msg.image0_ptr);
      bag_.write("/image_right", stereo_img_msg.image1_ptr->header.stamp, *stereo_img_msg.image1_ptr);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }  // while(1)
}

#endif


CRos1IO::~CRos1IO() {
  t_publish_myodom.detach();

#ifdef _RECORD_BAG_
#if 0  
  t_record_bag.detach();
#endif
  if (yaml_.record_bag) CloseRosBag();
  t_write_bag.detach();
#endif  
}

#ifdef _RECORD_BAG_
void CRos1IO::RecordBagThread() {
  while (1) {
    OpenRosbag();
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::seconds(yaml_.record_duration));
  }
}
#endif

void CRos1IO::PublishMyOdomThread() {
  basalt::PoseVelBiasState<double>::Ptr data;
  while (1) {
    // std::chrono::milliseconds dura(200);
    // std::this_thread::sleep_for(dura);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while (!pvb_queue.empty()) {
      pvb_queue.pop(data);
      // if (!data.get()) { // 如果当前帧为空指针，则退出循环
      //   break;
      // }

      PublishMyOdom(data, pvb_queue.empty());

      // handle data now.
    }

  }  // while(1)
}

void CRos1IO::StereoCb(const sm::ImageConstPtr& image0_ptr,
                       const sm::ImageConstPtr& image1_ptr) {
  // std::cout << "received images.\n";
#ifdef _RECORD_BAG_
  // if(yaml_.record_bag)
  if (record_bag) {
    // m_rec_.lock();
    // bag_.write("/image_left", image0_ptr->header.stamp, *image0_ptr);
    // bag_.write("/image_right", image1_ptr->header.stamp, *image1_ptr);
    // m_rec_.unlock();

    TStereoImage stereoImage;
    stereoImage.image0_ptr = image0_ptr;
    stereoImage.image1_ptr = image1_ptr;
    stereo_img_queue.push(stereoImage);
  }
#endif  

#ifdef _IS_FORWARD_
  if (isForward_ && !isForward_()) return;
#endif

  static u_int64_t prev_t_ns = 0;

  u_int64_t t_ns =
      image0_ptr->header.stamp.nsec + image0_ptr->header.stamp.sec * 1e9;

  if (prev_t_ns >= t_ns)  // 2023-12-18.
  {
    return;
  } else {
    prev_t_ns = t_ns;
  }

  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
  data->img_data.resize(CRos1IO::NUM_CAMS);
  data->t_ns = t_ns;

  const uint8_t* data_in0 = nullptr;
  const uint8_t* data_in1 = nullptr;

#ifdef _NOISE_SUPPRESSION_
  auto image0 = cb::toCvShare(image0_ptr)->image;  // cv::Mat类型
  auto image1 = cb::toCvShare(image1_ptr)->image;
  // auto image0 = cb::toCvCopy(image0_ptr)->image;  // cv::Mat类型
  // auto image1 = cb::toCvCopy(image1_ptr)->image;

  // cv::imshow("original image", image0);
  // cv::waitKey(0);
#ifdef _GAUSSIAN_BlUR_
  cv::GaussianBlur(image0, image0, cv::Size(3, 3), 0);  // 高斯滤波
  cv::GaussianBlur(image1, image1, cv::Size(3, 3), 0);  // 高斯滤波
#else
  cv::medianBlur(image0, image0, 3);  // 中值滤波
  cv::medianBlur(image1, image1, 3);  // 中值滤波
#endif

  data_in0 = image0.ptr();
  data_in1 = image1.ptr();

  // cv::imshow("noise suppression image", image0); // image_ns
  // cv::waitKey(0);
  // return ;
#else
  data_in0 = (const uint8_t*)image0_ptr->data.data();
  data_in1 = (const uint8_t*)image1_ptr->data.data();
#endif

#if 0  // for check bright points
  auto image0 = cb::toCvShare(image0_ptr)->image;
  // cv::Mat mask = image0 > 255;
  cv::Mat mask = image0 == 255;
  int count = cv::countNonZero(mask);
  // ROS_INFO("Number of pixels with value > 250: %d", count);
  ROS_INFO("Number of pixels equal to 255: %d", count);


  cv::Scalar meanValue = cv::mean(image0);
  float MyMeanValue = meanValue.val[0];//.val[0]表示第一个通道的均值
  std::cout<<"Average of all pixels in image0 with 1st channel is "<< MyMeanValue << std::endl;
#endif

  // 拷贝左目图像数据
  data->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(
      image0_ptr->width, image0_ptr->height));

  // 图像数据转化为uint16_t
  // const uint8_t* data_in = (const uint8_t*)image0_ptr->data.data();
  uint16_t* data_out = data->img_data[0].img->ptr;
  size_t full_size = image0_ptr->width * image0_ptr->height;
  for (size_t i = 0; i < full_size; i++) {
    int val = data_in0[i];
    val = val << 8;
    data_out[i] = val;
  }

  // 拷贝右目图像数据
  data->img_data[1].img.reset(new basalt::ManagedImage<uint16_t>(
      image1_ptr->width, image1_ptr->height));
  // data_in = (const uint8_t*)image1_ptr->data.data();
  data_out = data->img_data[1].img->ptr;
  full_size = image1_ptr->width * image1_ptr->height;
  for (size_t i = 0; i < full_size; i++) {
    int val = data_in1[i];
    val = val << 8;
    data_out[i] = val;
  }

  feedImage_(data);
}

void CRos1IO::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)  // const
{
#ifdef _RECORD_BAG_  
  // if(yaml_.record_bag)
  if (record_bag) {
    // if imu_callback is a const callback function, then i cant write anymore.
    // so...

    // m_rec_.lock();
    // bag_.write("/imu", imu_msg->header.stamp, *imu_msg);
    // m_rec_.unlock();

    imu_queue.push(imu_msg);
  }
#endif  

#ifdef _IS_FORWARD_
  if (isForward_ && !isForward_()) return;
#endif

  // double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * (1e-9);
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;

#if USE_TIGHT_COUPLING
  int64_t t_ns = imu_msg->header.stamp.nsec + imu_msg->header.stamp.sec * 1e9;
  basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
  data->t_ns = t_ns;
  data->accel = Vector3d(dx, dy, dz);
  data->gyro = Vector3d(rx, ry, rz);

  feedImu_(data);

#else  // LOOSE_COUPLING
  double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * (1e-9);
  Vector3d acc(dx, dy, dz);
  Vector3d gyr(rx, ry, rz);
  inputIMU_(t, acc, gyr);

  // pg_.inputIMU(t, acc, gyr);

#endif

#if 0
  double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * (1e-9);
  char szTime[60] = { 0 };
  sprintf(szTime, "%f", t);
  // std::cout << "[lwx] imu timestamp t=" << szTime << "acc = " << acc.transpose() << "  gyr = " << gyr.transpose() << std::endl;
  std::cout << "[lwx] imu timestamp t=" << szTime << "acc = " << data->accel.transpose() << "  gyr = " << data->gyro.transpose() << std::endl;
#endif
}

void CRos1IO::PublishPoints(basalt::VioVisualizationData::Ptr data) {
  if (pub_point_.getNumSubscribers() == 0) return;

  sensor_msgs::PointCloud2 cloud_msg;
  // Time (int64_t nanoseconds=0, rcl_clock_type_t clock=RCL_SYSTEM_TIME)
  cloud_msg.header.stamp = ros::Time(data->t_ns * 1.0 * 1e-9);  // this->now();
  // cloud_msg.header.stamp.sec = data->t_ns / 1e9;
  // cloud_msg.header.stamp.nsec = data->t_ns % (1e9);
  cloud_msg.header.frame_id = "odom";
  cloud_msg.point_step =
      12;  // 16 // 一个点占16个字节 Length of a point in bytes
           // cloud.fields = MakePointFields("xyzi"); //
  // 描述了二进制数据块中的通道及其布局。
  cloud_msg.fields = MakePointFields("xyz");  // tmp comment 2023-12-6
  /*
    cloud_msg.height = 1;
    cloud_msg.width = 0;
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 12;
    cloud_msg.row_step = 0;
    cloud_msg.is_dense = true;

    // 将点云数据转换为二进制数据，并存储在 PointCloud2 消息对象中
    cloud_msg.width = data->points.size();
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& point : data->points) {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  */

  // int total_size = data->points.size();
  cloud_msg.height = 1;
  cloud_msg.width = data->points.size();
  cloud_msg.data.resize(
      cloud_msg.width *
      cloud_msg.point_step);  // point_step指示每个点的字节数为16字节

  int i = 0;
  for (const auto& point : data->points) {
    auto* ptr = reinterpret_cast<float*>(cloud_msg.data.data() +
                                         i * cloud_msg.point_step);

    ptr[0] = point.x();
    ptr[1] = point.y();
    ptr[2] = point.z();
    // ptr[3] = static_cast<float>(patch.vals[0] / 255.0); // 图像强度转换为颜色

    i++;
  }

  // 发布点云
  pub_point_.publish(cloud_msg);
}

void CRos1IO::PublishOdometry(basalt::PoseVelBiasState<double>::Ptr data) {
  if (pub_odom_.getNumSubscribers() == 0) return;

  // 创建nav_msgs::msg::Odometry消息对象
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(data->t_ns * 1.0 * 1e-9);  // this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = data->T_w_i.translation().x();
  odom_msg.pose.pose.position.y = data->T_w_i.translation().y();
  odom_msg.pose.pose.position.z = data->T_w_i.translation().z();
  odom_msg.pose.pose.orientation.x = data->T_w_i.unit_quaternion().x();
  odom_msg.pose.pose.orientation.y = data->T_w_i.unit_quaternion().y();
  odom_msg.pose.pose.orientation.z = data->T_w_i.unit_quaternion().z();
  odom_msg.pose.pose.orientation.w = data->T_w_i.unit_quaternion().w();

  // 发布位姿
  pub_odom_.publish(odom_msg);
}

void CRos1IO::Reset() {
  path_msg_.poses.clear();
  path_msg_.poses.reserve(1024);
}

void CRos1IO::PublishPoseAndPath(basalt::PoseVelBiasState<double>::Ptr data) {
  geometry_msgs::PoseStamped pose_msg;
  // pose_msg.header.stamp = this->now();
  pose_msg.header.stamp =
      ros::Time(data->t_ns * 1.0 / 1e9);  ///< timestamp of the state in seconds
                                          ///< [nanoseconds];  // 帧采集时间
  // pose_msg.header.stamp.sec = data->t_ns / 1e9;
  // pose_msg.header.stamp.nsec = data->t_ns % 1e9;

  pose_msg.header.frame_id = "odom";
  // Sophus2Ros(tf, pose_msg.pose);
  pose_msg.pose.position.x = data->T_w_i.translation().x();
  pose_msg.pose.position.y = data->T_w_i.translation().y();
  pose_msg.pose.position.z = data->T_w_i.translation().z();
  pose_msg.pose.orientation.x = data->T_w_i.unit_quaternion().x();
  pose_msg.pose.orientation.y = data->T_w_i.unit_quaternion().y();
  pose_msg.pose.orientation.z = data->T_w_i.unit_quaternion().z();
  pose_msg.pose.orientation.w = data->T_w_i.unit_quaternion().w();

#ifdef _PUBLISH_VELOCITY_
#if 0
  PublishMyOdom(data, true);
#else
  pvb_queue.push(data);
#endif
#endif

  if (pub_odom_.getNumSubscribers() > 0) {
    nav_msgs::Odometry odom_msg;
    // odom_msg.header.stamp = time;
    // odom_msg.header.frame_id = "odom";
    odom_msg.header = pose_msg.header;
    odom_msg.pose.pose = pose_msg.pose;

    // 发布位姿
    pub_odom_.publish(odom_msg);
  }

  if (pub_path_.getNumSubscribers() == 0) return;

  // 发布轨迹   path_odom话题
  // path_msg_.header.stamp = this->now();
  path_msg_.header.stamp = ros::Time(
      data->t_ns * 1.0 /
      1e9);  ///< timestamp of the state in nanoseconds;  // 帧采集时间
  path_msg_.header.frame_id = "odom";
  path_msg_.poses.push_back(pose_msg);
  pub_path_.publish(path_msg_);

  // std::cout << " postion: " << pose_msg.pose.position.x << ", "
  //   << pose_msg.pose.position.y << ", " << pose_msg.pose.position.z <<
  //   std::endl;
}

void bubble_sort(double* arr, int length) {
  int i = 0;
  for (i = 0; i < length - 1; i++) {
    int flag = 0;  // flag标识
    int j = 0;
    for (j = 0; j < length - 1 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
        flag = 1;  // 发生数据交换，置flag为1
      }
    }
    if (flag == 0)  // 若flag为0，则没有发生交换，跳出循环
    {
      break;
    }
  }
}

bool sortByY(const double& p1, const double& p2) {
  return p1 < p2;  // 升序排列
}

double ClacFitted(std::vector<double>& vec, double vel) {
  double new_velocity;
  if (vec.size() < 5)
  // if(q_velocities.size() < 5)
  {
    vec.emplace_back(vel);
    // q_velocities.push(period_distance / delta_s);
    new_velocity = vel;
  } else {
    // q_velocities.pop();
    // q_velocities.push(period_distance / delta_s);

    for (int i = 0; i < 4; i++) {
      vec[i] = vec[i + 1];
    }
    vec[4] = vel;

    // for(auto vel : vec) std::cout << vel << " ";
    // std::cout << std::endl;
    std::vector<double> vec_tmp(vec);

    std::sort(vec_tmp.begin(), vec_tmp.end(), sortByY);

    new_velocity = (vec_tmp[1] + vec_tmp[2] + vec_tmp[3]) / 3;
  }

  return new_velocity;
}

void CRos1IO::PublishMyOdom(basalt::PoseVelBiasState<double>::Ptr data,
                            bool bl_publish /* = false*/) {
  // #if 1//def _PUBLISH_VELOCITY_
  //  if(1) // compute velocity.

  // use member variable 'dt_ns_'
  double fps_inv = 1.0 / fps_;
  // int DATA_CNT = 0.2 * fps_;

  // static int odometry_cnt = 0;
  static int64_t t_ns = 0;
  static int64_t last_t_ns = 0;
  static double total_distance = 0.0;
  static double period_distance = 0.0;
  static Vector2d last_pose(0, 0);
  static double prev_velocity = 0.0;
  static double total_odom = 0.0;
  static bool reset_velociy = false;
#ifdef _FILTER_IN_SECONDS_
  static constexpr int ELEM_CNT =
      4;  // 10;//5; // if elem_cnt equal to 10. it's mean that we use about 2
          // seconds's data to average.
  static double array_period_odom[ELEM_CNT] = {0};
  static double array_delta_s[ELEM_CNT] = {0};
  static int keep_count = 0;
#endif

#ifdef _MULTI_VELOCITY_FILTER_
  static constexpr int ELEM_CNT = 4;
  static double array_velocity[ELEM_CNT] = {0};
  static constexpr double array_weight[ELEM_CNT] = {0.1, 0.2, 0.3, 0.4};
  static int keep_count = 0;
  static unsigned char reset_cnt = 0;
#endif

#ifdef _REMOVE_OUTLIER_FILTER_
  static constexpr int ELEM_CNT = 5;  // 6
  static double array_velocity[ELEM_CNT] = {0};
  static constexpr double array_weight[ELEM_CNT] = {0.0, 0.0, 0.1, 0.2,
                                                    0.3};  //, 0.4 };
  static int keep_count = 0;
#endif

#ifdef _Linear_Fitted5_
  static std::vector<double> vec_velocities[20];
  // static std::queue<double> q_velocities;

  static constexpr int ELEM_CNT = 2;  // 4;
  static double array_velocity[ELEM_CNT] = {0};
  // static constexpr double array_weight[ELEM_CNT] = { 0.1, 0.2, 0.3, 0.4 };
  static constexpr double array_weight[ELEM_CNT] = {0.45, 0.55};
  static int keep_count = 0;
  static unsigned char reset_cnt = 0;
  static std::vector<double> vec_new_vel;

#endif

#ifdef _KF_
  // Kalman_Filter_Init(&kalman_filter_pt1, 0.001, 2, 25, 1);
  // float pt1_KMfilter = Kalman_Filter_Iterate(&kalman_filter_pt1,
  // SenPt100Array[0].CalibValue);
#endif

  if (isResetOdom) {
    std::cout << std::boolalpha << "isResetOdom=" << isResetOdom << std::endl;
    prev_velocity = 0.0;
    period_distance = 0.0;
    total_distance = 0.0;
    last_t_ns = 0;
    t_ns = 0;
    total_odom = 0.0;
    last_pose.x() = 0;
    last_pose.y() = 0;
#ifdef _MULTI_VELOCITY_FILTER_
    keep_count = 0;
    reset_cnt = 0;
#endif

    isResetOdom = false;
  }

  if (t_ns == 0) {
    t_ns = data->t_ns - fps_inv * 1e9;
    last_t_ns = data->t_ns - fps_inv * 1e9;
  }

  if(last_pose.x() == 0 && last_pose.y() == 0)
  {
    last_pose.x() = data->T_w_i.translation().x();
    last_pose.y() = data->T_w_i.translation().y();
  }

  Vector2d curr_pose;
  curr_pose.x() = data->T_w_i.translation().x();
  curr_pose.y() = data->T_w_i.translation().y();
  double delta_distance = (curr_pose - last_pose).norm();

  double frame_delta_s = (data->t_ns - last_t_ns) * 1.0 * (1e-9);

  period_distance += delta_distance;
  total_distance += delta_distance;
  // std::cout << " delta_distance=" << delta_distance << "  period_distance="
  // << period_distance << std::endl;

  double delta_s = (data->t_ns - t_ns) * 1.0 * (1e-9);

  // if (period_distance / delta_s > 25.0) {
  //   std::cout << "frame_delta_distance=" << delta_distance
  //             << "  frame_delta_s=" << frame_delta_s
  //             << "  period_distance=" << period_distance
  //             << "  delta_s=" << delta_s << std::endl;

  //   std::cout << "curr_pose" << curr_pose.transpose() << std::endl;          
  //   std::cout << "last_pose" << last_pose.transpose() << std::endl;          
  // }


  last_pose = curr_pose;

  last_t_ns = data->t_ns;


#ifdef _Linear_Fitted5_
  // double new_vel = delta_distance / delta_s;
  double new_vel = delta_distance / frame_delta_s;
  // new_vel = ClacFitted(vec_velocities, new_vel);
  // new_vel = ClacFitted(vec_velocities2, new_vel);
  // new_vel = ClacFitted(vec_velocities3, new_vel);
  // new_vel = ClacFitted(vec_velocities4, new_vel);
  // new_vel = ClacFitted(vec_velocities5, new_vel);

  for (int i = 0; i < 20; i++) {
    new_vel = ClacFitted(vec_velocities[i], new_vel);
  }

  vec_new_vel.emplace_back(new_vel);

#endif

  // if(odometry_cnt == 0)
  // if(delta_s > 0.18) // 20ms
  if (bl_publish) {
    double curr_velocity = 0.0;
    // std::cout << "delta_s:" << delta_s << std::endl;
#ifdef _FILTER_IN_SECONDS_
    if (keep_count < ELEM_CNT) {
      array_period_odom[keep_count++] = period_distance;
      array_delta_s[keep_count++] = delta_s;
    } else {
      int i = 0;
      for (i = 0; i < ELEM_CNT - 1; i++) {
        array_period_odom[i] = array_period_odom[i + 1];
        array_delta_s[i] = array_delta_s[i + 1];
      }

      array_period_odom[i] = period_distance;
      array_delta_s[i] = delta_s;
    }

    if (1) {
      int i = 0;
      double tmp_odom = 0.0;
      double tmp_delta_s = 0.0;
      for (i = 0; i < keep_count; i++) {
        tmp_odom += array_period_odom[i];
        tmp_delta_s += array_delta_s[i];
      }

      curr_velocity = tmp_odom / tmp_delta_s;
    }

#elif defined _MULTI_VELOCITY_FILTER_

    if (keep_count < ELEM_CNT) {
      array_velocity[keep_count++] = period_distance / delta_s;
    } else {
      int i = 0;
      for (i = 0; i < ELEM_CNT - 1; i++) {
        array_velocity[i] = array_velocity[i + 1];
      }

      array_velocity[i] = period_distance / delta_s;
    }

    if (keep_count < ELEM_CNT) {
      for (int i = 0; i < keep_count; i++) {
        curr_velocity += array_velocity[i];
      }

      curr_velocity = curr_velocity / keep_count;
    } else {
      for (int i = 0; i < keep_count; i++) {
        curr_velocity += array_velocity[i] * array_weight[i];
      }
    }

#elif defined _REMOVE_OUTLIER_FILTER_
/*
  if(keep_count < ELEM_CNT)
  {
    array_velocity[keep_count++] = period_distance / delta_s;
  }
  else
  {
    int i = 0;
    for(i = 0; i < ELEM_CNT - 1; i++)
    {
      array_velocity[i] = array_velocity[i + 1];
    }

    array_velocity[i] = period_distance  / delta_s;
  }

  //-
  if(keep_count < ELEM_CNT)
  {
    for(int i = 0; i < keep_count; i++)
    {
      curr_velocity += array_velocity[i];
    }

    // curr_velocity = curr_velocity / keep_count;
    curr_velocity = period_distance / delta_s;
  }
  else
  {
    // for(int i = 0; i < keep_count; i++)
    // {
    //   curr_velocity += array_velocity[i] * array_weight[i];
    // }

    // int N = ELEM_CNT;
    // curr_velocity = (-array_velocity[N - 5] + 4.0 * array_velocity[N - 4]
  - 6.0 * array_velocity[N - 3] +
    //   4.0 * array_velocity[N - 2] + 69.0 * array_velocity[N - 1]) / 70.0;

    bubble_sort(array_velocity, 5);
    curr_velocity = (array_velocity[1] + array_velocity[2] + array_velocity[3])
  / 3;
  }
*/
/*
  if(vec_velocities.size() < 5)
  // if(q_velocities.size() < 5)
  {
    vec_velocities.emplace_back(period_distance / delta_s);
    // q_velocities.push(period_distance / delta_s);
    curr_velocity = period_distance / delta_s;
  }
  else
  {
    // q_velocities.pop();
    // q_velocities.push(period_distance / delta_s);

    for(int i = 0; i < 4; i++)
    {
      vec_velocities[i] = vec_velocities[i + 1];
    }
    vec_velocities[4] = period_distance / delta_s;

    // for(auto vel : vec_velocities) std::cout << vel << " ";
    // std::cout << std::endl;
    std::vector<double> vec_tmp(vec_velocities);

    std::sort(vec_tmp.begin(), vec_tmp.end(), sortByY);

    curr_velocity = (vec_tmp[1] + vec_tmp[2] + vec_tmp[3]) / 3;
  }
*/
#elif defined _Linear_Fitted5_

    // curr_velocity = new_velocity2;
    // curr_velocity = new_velocity4;

    // curr_velocity = new_vel;
    for (auto vel : vec_new_vel) curr_velocity += vel;
    curr_velocity /= vec_new_vel.size();

    vec_new_vel.clear();

    if (keep_count < ELEM_CNT) {
      array_velocity[keep_count++] = curr_velocity;
    } else {
      int i = 0;
      for (i = 0; i < ELEM_CNT - 1; i++) {
        array_velocity[i] = array_velocity[i + 1];
      }

      array_velocity[i] = curr_velocity;
    }

    if (keep_count < ELEM_CNT) {
      for (int i = 0; i < keep_count; i++) {
        curr_velocity += array_velocity[i];
      }

      curr_velocity = curr_velocity / keep_count;
    } else {
      curr_velocity = 0.0;
      for (int i = 0; i < keep_count; i++) {
        curr_velocity += array_velocity[i] * array_weight[i];
      }
    }

#else
    curr_velocity = period_distance / delta_s;
#endif

    if (prev_velocity < 1e-6) {
      prev_velocity = curr_velocity;
    }

#ifdef _FILTER_IN_SECONDS_
    // double publish_velocity = curr_velocity;
    double publish_velocity = pow(curr_velocity, yaml_.coefficient);
    // std::cout << " curr_veloctiy=" << curr_velocity << " publish_velocity="
    // << publish_velocity << std::endl;

#elif defined _MULTI_VELOCITY_FILTER_
    // double publish_velocity = curr_velocity;
    double publish_velocity = pow(curr_velocity, yaml_.coefficient);
#else

#ifdef _VELOCITY_FILTER_
    double publish_velocity = (prev_velocity + curr_velocity) / 2;
#else
    double publish_velocity = curr_velocity;
#endif

#endif

#ifdef _MULTI_VELOCITY_FILTER_

#if 1
    // if(fabs(publish_velocity - prev_velocity) > 2) // 3 or 2 or other number
    // ?
    if (fabs(publish_velocity - prev_velocity) >
        0.5)  // 3 or 2 or other number ?
    {
      // std::cout << " --- reset count:" << (int)reset_cnt << " prev_v:" <<
      // prev_velocity << " curr_v:" << publish_velocity << std::endl;
      if (reset_cnt < 3) {
        // std::cout << " --- reset velocity.---\n";
        publish_velocity = prev_velocity * 0.5 + publish_velocity * 0.5;
        reset_cnt++;
        // if(keep_count >= 1)
        array_velocity[keep_count - 1] = publish_velocity;
      } else {
        reset_cnt = 0;
      }

    } else {
      reset_cnt = 0;
    }
#else
    if (fabs(publish_velocity - prev_velocity) > 0.4) {
      double acc = (array_velocity[2] - array_velocity[0]) / 0.6;
      std::cout << " prev_v:" << prev_velocity
                << " curr_v:" << publish_velocity;
      publish_velocity = array_velocity[2] + acc * 0.2;
      std::cout << "  acc:" << acc << "  new curr_v:" << publish_velocity
                << std::endl;
      array_velocity[keep_count - 1] = publish_velocity;
    }
#endif

    if (publish_velocity > 200.0 / 9)  // 80 km/h limit
    {
      publish_velocity = prev_velocity;
      // if(keep_count >= 1)
      array_velocity[keep_count - 1] = publish_velocity;
    }

#elif defined _REMOVE_OUTLIER_FILTER_
    /*
      if(fabs(publish_velocity - prev_velocity) > 0.5)
      {
        // if(keep_count < ELEM_CNT)
        // {
        //   curr_velocity = array_velocity[keep_count - 1];
        // }
        // else
        {
          double acc_vel[5] = { 0 };

          for(int i = 0; i < keep_count - 1; i++)
          {
            acc_vel[i] = array_velocity[i + 1] - array_velocity[i];
          }

          bubble_sort(acc_vel, 5);

          double acc_average = (acc_vel[1] + acc_vel[2] + acc_vel[3]) / 3;

          if(acc_average > 0.5)  acc_average = 0;
          else if (acc_average < -0.5)  acc_average = -0;

          publish_velocity = prev_velocity + acc_average;
        }
      }
    */

#else

#ifdef _VELOCITY_FILTER_
    if (reset_velociy == false) {
      // if(fabs(publish_velocity - prev_velocity) > 2) // 3 or 2 or other
      // number ?
      if (fabs(publish_velocity - prev_velocity) >
          0.5)  // 3 or 2 or other number ?
      {
        std::cout << " --- reset velocity.---\n";
        publish_velocity = prev_velocity;
        reset_velociy = true;
      }
    } else {
      reset_velociy = false;
    }
#endif

    if (publish_velocity > 200.0 / 9)  // 80 km/h limit
    {
      publish_velocity = prev_velocity;
    }

#endif

    bool is_reliable = true;
    if (fabs(publish_velocity - prev_velocity) >= 0.3) {
      is_reliable = false;
    }

    if (publish_velocity <
        yaml_.zero_velocity)  // 0.05) // put here better than other place.
    {
      zeroVelocity_(true);
      publish_velocity = 0.00;
    } else {
      zeroVelocity_(false);
    }

    if (publish_velocity < yaml_.slow_velocity)  // 3.0)
    {
      slowVelocity_(true);
    } else {
      slowVelocity_(false);
    }

    prev_velocity = publish_velocity;

    double period_odom = publish_velocity * delta_s;
    total_odom += period_odom;

    if (publish_velocity > yaml_.abnormal_velocity) {
      std::cout << "The speed " << publish_velocity
                << " is ABNORMAL. reset algorithm." << std::endl;

      std::cout << std::boolalpha << "isResetOdom=" << isResetOdom << std::endl;

      prev_velocity = 0.0;
      period_distance = 0.0;
      total_distance = 0.0;
      last_t_ns = 0;
      t_ns = 0;
      total_odom = 0.0;
      last_pose.x() = 0;
      last_pose.y() = 0;
#ifdef _MULTI_VELOCITY_FILTER_
      keep_count = 0;
      reset_cnt = 0;
#endif

      reset_();
      return;
    }

    // if (pub_my_odom_.getNumSubscribers() > 0)
    {
      myodom::MyOdom odom_msg;
      // odom_msg.header.stamp = rclcpp::Time(data->t_ns);
      odom_msg.header.stamp = ros::Time((data->t_ns - dt_ns_) * 1.0 /
                                        1e9);  // complementary timestamp
      odom_msg.header.frame_id = "odom";

      // double delta_s = (data->t_ns - t_ns) * 1.0 * (1e-9);

      odom_msg.velocity = publish_velocity;  // period_distance / delta_s;
      odom_msg.delta_time = delta_s;
      // odom_msg.period_odom = period_distance;
      odom_msg.period_odom = period_odom;
      // odom_msg.total_odom = total_distance; // TODO:
      odom_msg.total_odom = total_odom;

      int nTrackedPoints = data->bias_accel.x();
      int nOptFlowPatches = data->bias_accel.y();
      int nUseImu = data->bias_accel.z();
      /*
            if(nUseImu == 1 || nTrackedPoints <= 10 || nOptFlowPatches <= 10)
            {
              odom_msg.confidence_coefficient = 0;
            }
            else if(nTrackedPoints < 20)
            {
              odom_msg.confidence_coefficient = 0.5;
            }
            else
            {
              odom_msg.confidence_coefficient = 1.0;
            }
      */
      double confidence_coefficient = 0.0;
      int nSize = yaml_.vec_tracked_points.size();

      if (nTrackedPoints > yaml_.vec_tracked_points[nSize - 1]) {
        confidence_coefficient = 1.0;
      } else {
        for (int i = 0; i < nSize; i++) {
          // if(nTrackedPoints <= yaml_.vec_tracked_points[i])
          if (nTrackedPoints <= yaml_.vec_tracked_points[i] &&
              publish_velocity >= 0.05)  // if velociy is 0, tracked points is
                                         // fewer than moving 2023-12-15
          {
            confidence_coefficient = yaml_.vec_confidence_levels[i];
#if 0
            nTracked_confidence_count++;
#endif
            break;
          }
        }
      }

#if 0
      if(confidence_coefficient < 1e-6 && isLampOn_)
      {
        zero_confidence ++;
      }

      if(isLampOn_)
      {
        total_confidence ++;
      }
      std::cout << " total_confidence: " << total_confidence 
        << "zero_confidence :" << zero_confidence 
        << " Percentage:" << (zero_confidence * 1.0 / total_confidence * 100) << "%" << std::endl;
#endif

#if 0
      if(confidence_coefficient == 1 && is_reliable == false)
      {
        nVelocity_confidence_count++;
      }

      if(confidence_coefficient == 1 && isLampOn_ == false)
      {
        nLamp_confidence_count++;
      }

      std::cout << "---statistics---\n" << "track - confidence == 0 counter: " << nTracked_confidence_count << std::endl
          << "velocity - confidence == 0 counter: " << nVelocity_confidence_count << std::endl
          << "lamp - confidence == 0 counter: " << nLamp_confidence_count << std::endl
          << "---the end---" << std::endl;
#endif

      // if(nUseImu == 1)
      if ((nUseImu == 1) || (is_reliable == false) || isLampOn_ == false) {
        confidence_coefficient = 0.0;
      }

      odom_msg.confidence_coefficient = confidence_coefficient;

      // publish velocity, period odom and total odom.
      if (pub_my_odom_.getNumSubscribers() > 0) pub_my_odom_.publish(odom_msg);

      if (add_odom_frame_) {
        add_odom_frame_(publish_velocity, period_odom, total_odom,
                        confidence_coefficient);
      }

      // std::cout << "confidence : " << data->bias_accel.transpose() << "
      // confidence coefficient:" << confidence_coefficient << std::endl; // for
      // test.
    }

    t_ns = data->t_ns;
    period_distance = 0.0;
  }

  // #endif
}

inline void CRos1IO::getcolor(float p, float np, float& r, float& g, float& b) {
  float inc = 4.0 / np;
  float x = p * inc;
  r = 0.0f;
  g = 0.0f;
  b = 0.0f;

  if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
    r = 1.0f;
  else if (4 <= x && x <= 5)
    r = x - 4;
  else if (1 <= x && x <= 2)
    r = 1.0f - (x - 1);

  if (1 <= x && x <= 3)
    g = 1.0f;
  else if (0 <= x && x <= 1)
    g = x - 0;
  else if (3 <= x && x <= 4)
    g = 1.0f - (x - 3);

  if (3 <= x && x <= 5)
    b = 1.0f;
  else if (2 <= x && x <= 3)
    b = x - 2;
  else if (5 <= x && x <= 6)
    b = 1.0f - (x - 5);
}

void CRos1IO::PublishFeatureImage(basalt::VioVisualizationData::Ptr data) {
  // if (pub_warped_img.getNumSubscribers() == 0)
  //   return;

  static cv::Mat disp_frame;

#ifdef _RESET_ALOGORITHM_BY_LAMP_ON_
  static bool is_reset_alogorithm_by_lamp_on = true;
#endif

  // step1 convert image
  uint16_t* data_in = nullptr;
  uint8_t* data_out = nullptr;

  // for(int cam_id = 0; cam_id < NUM_CAMS; cam_id++)
  for (int cam_id = 0; cam_id < 1; cam_id++) {
    // img_data is a vector<ImageData>
    basalt::ImageData imageData =
        data->opt_flow_res->input_images->img_data[cam_id];
    // std::cout << "w=" << imageData.img->w << "  h=" << imageData.img->h <<
    // std::endl;
    data_in = imageData.img->ptr;
    disp_frame =
        cv::Mat::zeros(imageData.img->h, imageData.img->w, CV_8UC1);  // CV_8UC3
    data_out = disp_frame.ptr();

    size_t full_size =
        imageData.img->size();  // disp_frame.cols * disp_frame.rows;
    for (size_t i = 0; i < full_size; i++) {
      int val = data_in[i];
      val = val >> 8;
      data_out[i] = val;
      // disp_frame.at(<>)
    }

    // check if lamp is on or off 2023-12-20.
    // if(1)
    {
#if 1
      // static int rev_cnt = 0;
      cv::Mat mask = disp_frame == 255;
      int count = cv::countNonZero(mask);
      // if (count > 15)
        // std::cout << "Number of intensity equal to 255 is " << count
                  // << std::endl;
        // more than 30 pixels with 255 denote lamp on.

        // if(rev_cnt == 0)
        // {
        //   rev_cnt = count;
        // }
        // int nAverage = (rev_cnt + count) / 2;
        // rev_cnt = count;
#else
      cv::Scalar meanValue = cv::mean(disp_frame);
      float MyMeanValue = meanValue.val[0];  //.val[0]表示第一个通道的均值
      // std::cout<<"Average of all pixels in image0 with 1st channel is "<<
      // MyMeanValue << std::endl;
#endif
      // if(MyMeanValue >= yaml_.mean_value)
      if (count >= yaml_.number_of_255) {
        if (!isLampOn_) {
          isLampOn_ = true;
          std::cout << std::boolalpha << "lamp on :" << isLampOn_ << std::endl;
#ifdef _RESET_ALOGORITHM_BY_LAMP_ON_
          if (is_reset_alogorithm_by_lamp_on) {
            reset_();
            is_reset_alogorithm_by_lamp_on = false;
          }
#endif
        }

      } else {
        if (isLampOn_) {
          isLampOn_ = false;
          std::cout << std::boolalpha << "lamp on :" << isLampOn_ << std::endl;
        }
      }
    }

    if (pub_warped_img.getNumSubscribers() == 0) return;
    // the end.

    /*
        cv::Mat img(cv::Size(imageData.img->w, imageData.img->h), CV_16UC1,
       imageData.img->ptr); #ifdef _DEBUG_ cv::imshow("feature_img", img);
        // cv::waitKey(0);
        cv::waitKey(1);
        #endif

        cv::Mat disp_frame;
        img.convertTo(disp_frame, CV_8UC1);

    */
    // disp_frame.convertTo(disp_frame, CV_8UC3);
    // just gray to bgr can show colored text and pixel.
    cv::cvtColor(disp_frame, disp_frame, CV_GRAY2BGR);  // CV_GRAY2RGB

#ifdef _DEBUG_
    cv::imshow("feature_img", disp_frame);
    // cv::waitKey(0);
    cv::waitKey(1);
#endif

// bool show_obs = true;
// if (show_obs) { // 显示追踪的特征点
#ifdef SHOW_TRACKED_POINTS

    const auto& points = data->projections[cam_id];

    if (points.size() > 0) {
      double min_id = points[0][2], max_id = points[0][2];

      for (const auto& points2 : data->projections)
        for (const auto& p : points2) {
          min_id = std::min(min_id, p[2]);
          max_id = std::max(max_id, p[2]);
        }

      for (const auto& c : points) {
        const float radius = 6.5;

        float r, g, b;
        getcolor(c[2] - min_id, max_id - min_id, b, g, r);
        // glColor3f(r, g, b);
        // pangolin::glDrawCirclePerimeter(c[0], c[1], radius);
        // pangolin里面的1.0，对应opencv里面的255
        b *= 255;
        g *= 255;
        r *= 255;
        cv::circle(disp_frame, cv::Point(c[0], c[1]), radius,
                   cv::Scalar(b, g, r));
        // cv::circle(disp_frame, cv::Point(c[0], c[1]), radius, cv::Scalar(255,
        // 255, 255));
      }
    }

    // glColor3f(1.0, 0.0, 0.0); // to r, g, b
    // pangolin::GlFont::I()
    //     .Text("Tracked %d points", points.size())
    //     .Draw(5, 20);

    if (1) {
      /*
      void cv::putText  ( InputOutputArray  img,  // 要添加备注的图片
      const String &  text,  // 要添加的文字内容
      Point  org,  //
     要添加的文字基准点或原点坐标，左上角还是左下角取决于最后一个参数bottomLeftOrigin的取值
                   // Bottom-left corner of the text string in the image.
      int  fontFace,
      double  fontScale, //
     字体相较于最初尺寸的缩放系数。若为1.0f，则字符宽度是最初字符宽度，若为0.5f则为默认字体宽度的一半
      Scalar  color,  // 字体颜色
      int  thickness = 1,  // 字体笔画的粗细程度
      int  lineType = LINE_8,  // 字体笔画线条类型

      bool  bottomLeftOrigin = false  // 如果取值为TRUE，则Point
     org指定的点为插入文字的左上角位置，如果取值为默认值false则指定点为插入文字的左下角位置.
                                      // When true, the image data origin is at
     the bottom-left corner. Otherwise, it is at the top-left corner.
     )

      fontFace文字的字体类型（Hershey字体集），可供选择的有
      FONT_HERSHEY_SIMPLEX：正常大小无衬线字体
      FONT_HERSHEY_PLAIN：小号无衬线字体
      FONT_HERSHEY_DUPLEX：正常大小无衬线字体，比FONT_HERSHEY_SIMPLEX更复杂
      FONT_HERSHEY_COMPLEX：正常大小有衬线字体
      FONT_HERSHEY_TRIPLEX：正常大小有衬线字体，比FONT_HERSHEY_COMPLEX更复杂
      FONT_HERSHEY_COMPLEX_SMALL：FONT_HERSHEY_COMPLEX的小译本
      FONT_HERSHEY_SCRIPT_SIMPLEX：手写风格字体
      FONT_HERSHEY_SCRIPT_COMPLEX：手写风格字体，比FONT_HERSHEY_SCRIPT_SIMPLEX更复杂
      这些参数和FONT_ITALIC同时使用就会得到相应的斜体字

    */

      /*
        //创建空白图用于绘制文字
              cv::Mat image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
              //设置蓝色背景
              image.setTo(cv::Scalar(100, 0, 0));
       */
      // 设置绘制文本的相关参数

      std::string strText = "";
      char szText[100] = {0};
      sprintf(szText, "Tracked %d points", points.size());
      strText = szText;
      // show text
      int font_face = cv::FONT_HERSHEY_SIMPLEX;  // cv::FONT_HERSHEY_COMPLEX;
      double font_scale = 0.5;                   // 2;//1;
      int thickness = 1;  // 2; // 字体笔画的粗细程度，有默认值1
      int baseline;
      // 获取文本框的长宽
      // cv::Size text_size = cv::getTextSize(text, font_face, font_scale,
      // thickness, &baseline);
      cv::Size text_size =
          cv::getTextSize(strText, font_face, font_scale, thickness, &baseline);

      // 将文本框居中绘制
      cv::Point origin;  // 计算文字左下角的位置
      // origin.x = image.cols / 2 - text_size.width / 2;
      // origin.y = image.rows / 2 + text_size.height / 2;
      origin.x = 0;  // 2023-11-28
      // origin.x = disp_frame.cols - text_size.width;
      origin.y = text_size.height;
      // cv::putText(disp_frame, text, origin, font_face, font_scale,
      // cv::Scalar(0, 255, 255), thickness, 8, 0);
      //  cv::putText(disp_frame, text, origin, font_face, font_scale,
      //  cv::Scalar(0, 0, 0), thickness, 8, false);
      cv::putText(disp_frame, strText, origin, font_face, font_scale,
                  cv::Scalar(255, 0, 255), thickness, 8, false);
    }
// }
#endif

// bool show_flow = true;
// if (show_flow) { // 显示光流patch
#ifdef SHOW_FLOW_PATCHES

    const Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>&
        kp_map = data->opt_flow_res->observations[cam_id];

    for (const auto& kv : kp_map) {
      Eigen::MatrixXf transformed_patch =
          kv.second.linear() * opt_flow_ptr->patch_coord;
      transformed_patch.colwise() += kv.second.translation();

      for (int i = 0; i < transformed_patch.cols(); i++) {
        const Eigen::Vector2f c = transformed_patch.col(i);
        // pangolin::glDrawCirclePerimeter(c[0], c[1], 0.5f);
        cv::circle(disp_frame, cv::Point(c[0], c[1]), 0.5f,
                   cv::Scalar(0, 0, 255));
      }

      const Eigen::Vector2f c = kv.second.translation();
    }

    // pangolin::GlFont::I()
    //     .Text("%d opt_flow patches", kp_map.size())
    //     .Draw(5, 20);

    std::string strText = "";
    char szText[100] = {0};
    sprintf(szText, "%d opt_flow patches", kp_map.size());
    strText = szText;
    // show text
    int font_face = cv::FONT_HERSHEY_SIMPLEX;  // cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;                   // 1;//2;//1;
    int thickness = 1;  // 2; // 字体笔画的粗细程度，有默认值1
    int baseline;
    // 获取文本框的长宽
    // cv::Size text_size = cv::getTextSize(text, font_face, font_scale,
    // thickness, &baseline);
    cv::Size text_size =
        cv::getTextSize(strText, font_face, font_scale, thickness, &baseline);

    // 将文本框居中绘制
    cv::Point origin;  // 计算文字左下角的位置
    // origin.x = image.cols / 2 - text_size.width / 2;
    // origin.y = image.rows / 2 + text_size.height / 2;
    origin.x = disp_frame.cols - text_size.width;
    origin.y = text_size.height;
    // cv::putText(disp_frame, text, origin, font_face, font_scale,
    // cv::Scalar(0, 255, 255), thickness, 8, 0);
    //  cv::putText(disp_frame, text, origin, font_face, font_scale,
    //  cv::Scalar(0, 0, 0), thickness, 8, false);
    cv::putText(disp_frame, strText, origin, font_face, font_scale,
                cv::Scalar(0, 255, 255), thickness, 8, false);

// }
#endif

#ifdef _OPEN_CV_SHOW_
    cv::imshow("feature_img", disp_frame);
    // cv::waitKey(0);
    cv::waitKey(1);
#endif

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", disp_frame).toImageMsg();
    msg->header.stamp = ros::Time(data->t_ns * 1.0 / 1e9);
    msg->header.frame_id = "odom";
    // warped_img.header = image0_ptr->header;
    /*
     * 以下设置似乎显得多余?  yeah.
        msg->height = disp_frame.rows;
        msg->width = disp_frame.cols;
        // msg->is_bigendian = false;
        // msg->step = 640; // 640 * 1 * 3
        // warped_img.data = image0_ptr->data;
        // msg->encoding = "mono8";
    */
    pub_warped_img.publish(*msg);
  }
}