
// @author: wxliu
// @date: 2023-11-9

#include "wx_ros2_io.h"
#include <basalt/image/image.h>


//#include <boost/circular_buffer.hpp>

// #define _TEST_ROS2_IO

using namespace wx;

bool USE_IMU { true };


CRos2IO::CRos2IO(bool use_imu)
  : Node("CRos2IO")
  , sub_image0_(this, "/image_left")
  , sub_image1_(this, "/image_right")
  , sub_image0_info_(this, "/image_left_info")
  , sub_image1_info_(this, "/image_right_info")
  // , sync_stereo_(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 10) // method 1
{
  use_imu_ = use_imu;
  //std::cout << "create imu subscription\n";
  if(use_imu_)
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&CRos2IO::imu_callback, this, _1));


  //std::cout << "create image subscription\n";
  // sync_stereo_.registerCallback(&CRos2IO::StereoCb, this); // method 1

  sync_stereo_.emplace(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 5); // method 2
/*  
  sync_stereo_->registerCallback(
    boost::bind(&CRos2IO::StereoCb, this, _1, _2, _3, _4));
*/
  sync_stereo_->registerCallback(&CRos2IO::StereoCb, this);

  pub_point_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_cloud2", 10);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

 #if 0 
  // 用于停止ros2节点
  sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
    "/sys_stop", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
      sys_stop_callback(msg);
    }
  );
#endif



/*  
  //同一个组别，类型设置为Reentrant
  rclcpp::CallbackGroup::SharedPtr callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_sub_;

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), imu_callback, sub_opt);
*/  
}

void CRos2IO::StereoCb(const sensor_msgs::msg::Image::SharedPtr &image0_ptr,
  const sensor_msgs::msg::Image::SharedPtr &image1_ptr,
  const sensor_pub::msg::ImageInfo::SharedPtr &image0_info_ptr,
  const sensor_pub::msg::ImageInfo::SharedPtr &image1_info_ptr)
{
  //std::cout << "received images.\n";
  
  u_int64_t t_ns = image0_ptr->header.stamp.nanosec +
    image0_ptr->header.stamp.sec * 1e9;

  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
  data->img_data.resize(CRos2IO::NUM_CAMS);
  data->t_ns = t_ns;

  // 拷贝左目图像数据
  data->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(image0_ptr->width,
    image0_ptr->height));

  // 图像数据转化为uint16_t
  const uint8_t* data_in = (const uint8_t*)image0_ptr->data.data();
  uint16_t* data_out = data->img_data[0].img->ptr;
  size_t full_size = image0_ptr->width * image0_ptr->height;
  for (size_t j = 0; j < full_size; j++) {
    int val = data_in[j];
    val = val << 8;
    data_out[j] = val;
  }

  // 拷贝右目图像数据
  data->img_data[1].img.reset(new basalt::ManagedImage<uint16_t>(image1_ptr->width,
    image1_ptr->height));
  data_in = (const uint8_t*)image1_ptr->data.data();
  data_out = data->img_data[1].img->ptr;
  full_size = image1_ptr->width * image1_ptr->height;
  for (size_t j = 0; j < full_size; j++) {
    int val = data_in[j];
    val = val << 8;
    data_out[j] = val;
  }

  feedImage_(data);
}

void CRos2IO::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) const
{
  // double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;

#if USE_TIGHT_COUPLING
  int64_t t_ns = imu_msg->header.stamp.nanosec + imu_msg->header.stamp.sec * 1e9;
  basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
  data->t_ns = t_ns;
  data->accel = Vector3d(dx, dy, dz);
  data->gyro = Vector3d(rx, ry, rz);

  feedImu_(data);

#else // LOOSE_COUPLING
  double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
  Vector3d acc(dx, dy, dz);
  Vector3d gyr(rx, ry, rz);
  inputIMU_(t, acc, gyr);

  // pg_.inputIMU(t, acc, gyr);

#endif

#if 0
  double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
  char szTime[60] = { 0 };
  sprintf(szTime, "%f", t);
  // std::cout << "[lwx] imu timestamp t=" << szTime << "acc = " << acc.transpose() << "  gyr = " << gyr.transpose() << std::endl;
  std::cout << "[lwx] imu timestamp t=" << szTime << "acc = " << data->accel.transpose() << "  gyr = " << data->gyro.transpose() << std::endl;
#endif
}

void CRos2IO::PublishPoints(basalt::VioVisualizationData::Ptr data)
{
  if (pub_point_ == nullptr || pub_point_->get_subscription_count() == 0) 
    return;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = "odom";
  cloud_msg.height = 1;
  cloud_msg.width = 0;
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count = 1;
  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
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
  // 发布点云
  pub_point_->publish(cloud_msg);
}

void CRos2IO::PublishOdometry(basalt::PoseVelBiasState<double>::Ptr data)
{
  if (pub_odom_ == nullptr || pub_odom_->get_subscription_count() == 0) 
    return;

  // 创建nav_msgs::msg::Odometry消息对象
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = data->T_w_i.translation().x();
  odom_msg.pose.pose.position.y = data->T_w_i.translation().y();
  odom_msg.pose.pose.position.z = data->T_w_i.translation().z();
  odom_msg.pose.pose.orientation.x = data->T_w_i.unit_quaternion().x();
  odom_msg.pose.pose.orientation.y = data->T_w_i.unit_quaternion().y();
  odom_msg.pose.pose.orientation.z = data->T_w_i.unit_quaternion().z();
  odom_msg.pose.pose.orientation.w = data->T_w_i.unit_quaternion().w();

  // 发布位姿
  pub_odom_->publish(odom_msg);
}

#ifdef _TEST_ROS2_IO
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CRos2IO>());
  rclcpp::shutdown();
  return 0;
}
#endif

#if 0

// this section is a example of ros2 usage which is not need derived from 'rclcpp::Node'.

// for subsriber
rclcpp::Node::SharedPtr g_node = nullptr;

void topic_callback(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_subscriber");
  auto subscription =
    g_node->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}

// for publisher
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  std_msgs::msg::String message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    message.data = "Hello, world! " + std::to_string(publish_count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    try {
      publisher->publish(message);
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        node->get_logger(),
        "unexpectedly failed with %s",
        e.what());
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
#endif