
// @author: wxliu
// @date: 2023-11-9

#include "wx_ros2_io.h"
#include <basalt/image/image.h>
// #include <basalt/utils/vis_utils.h>
#include <opencv2/highgui.hpp>

#include <basalt/io/dataset_io.h>

extern basalt::OpticalFlowBase::Ptr opt_flow_ptr; // 2023-11-28.

//#include <boost/circular_buffer.hpp>

// #define _TEST_ROS2_IO

using namespace wx;

// CRos2IO::CRos2IO(bool use_imu)
CRos2IO::CRos2IO(bool use_imu, int fps, long dt_ns)
  : Node("CRos2IO")
  , fps_(fps)
  , dt_ns_(dt_ns)
  , sub_image0_(this, "/image_left")
  , sub_image1_(this, "/image_right")
  // , sub_image0_info_(this, "/image_left_info")
  // , sub_image1_info_(this, "/image_right_info")
  // , sync_stereo_(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 10) // method 1
{
  use_imu_ = use_imu;
  //std::cout << "create imu subscription\n";
/*  
  if(use_imu_)
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&CRos2IO::imu_callback, this, _1));
*/

  //std::cout << "create image subscription\n";
  // sync_stereo_.registerCallback(&CRos2IO::StereoCb, this); // method 1

  // sync_stereo_.emplace(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 5); // method 2
  sync_stereo_.emplace(sub_image0_, sub_image1_, 5); // method 2
/*  
  sync_stereo_->registerCallback(
    boost::bind(&CRos2IO::StereoCb, this, _1, _2, _3, _4));
*/
  sync_stereo_->registerCallback(&CRos2IO::StereoCb, this);

  pub_point_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_cloud2", 10);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/pose_odom", 10);

 #if 0 
  // 用于停止ros2节点
  sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
    "/sys_stop", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
      sys_stop_callback(msg);
    }
  );
#endif

  path_msg_.poses.reserve(1024); // reserve的作用是更改vector的容量（capacity），使vector至少可以容纳n个元素。
                                 // 如果n大于vector当前的容量，reserve会对vector进行扩容。其他情况下都不会重新分配vector的存储空间

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path_odom", 2);



  if(use_imu_)
  {
    //同一个组别，类型设置为Reentrant
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_sub_;

    // sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), imu_callback, sub_opt);
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), std::bind(&CRos2IO::imu_callback, this, _1), sub_opt);
  }
  
// #ifdef SHOW_WARPED_IMG
  pub_warped_img = this->create_publisher<sensor_msgs::msg::Image>("/feature_img", 1); // feature_img // warped_img
// #endif

  pub_my_odom_ = this->create_publisher<myodom::msg::MyOdom>("/my_odom", 10);
 
}

void CRos2IO::StereoCb(const sensor_msgs::msg::Image::SharedPtr &image0_ptr,
  const sensor_msgs::msg::Image::SharedPtr &image1_ptr/*,
  const sensor_pub::msg::ImageInfo::SharedPtr &image0_info_ptr,
  const sensor_pub::msg::ImageInfo::SharedPtr &image1_info_ptr*/)
{
  //std::cout << "received images.\n";
  
  u_int64_t t_ns = image0_ptr->header.stamp.nanosec +
    image0_ptr->header.stamp.sec * 1e9;

  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
  data->img_data.resize(CRos2IO::NUM_CAMS);
  data->t_ns = t_ns;

  const uint8_t* data_in0 = nullptr;
  const uint8_t* data_in1 = nullptr;

#ifdef  _NOISE_SUPPRESSION_
  auto image0 = cb::toCvShare(image0_ptr)->image; // cv::Mat类型
  auto image1 = cb::toCvShare(image1_ptr)->image;
  // auto image0 = cb::toCvCopy(image0_ptr)->image;  // cv::Mat类型
  // auto image1 = cb::toCvCopy(image1_ptr)->image;

  // cv::imshow("original image", image0);	
  // cv::waitKey(0);
#ifdef _GAUSSIAN_BlUR_
  cv::GaussianBlur(image0, image0, cv::Size(3, 3), 0); // 高斯滤波
  cv::GaussianBlur(image1, image1, cv::Size(3, 3), 0); // 高斯滤波
#else  
  cv::medianBlur(image0, image0, 3); // 中值滤波
  cv::medianBlur(image1, image1, 3); // 中值滤波
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

  // 拷贝左目图像数据
  data->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(image0_ptr->width,
    image0_ptr->height));

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
  data->img_data[1].img.reset(new basalt::ManagedImage<uint16_t>(image1_ptr->width,
    image1_ptr->height));
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
  // Time (int64_t nanoseconds=0, rcl_clock_type_t clock=RCL_SYSTEM_TIME)
  cloud_msg.header.stamp = rclcpp::Time(data->t_ns);//this->now();
  // cloud_msg.header.stamp.sec = data->t_ns / 1e9;
  // cloud_msg.header.stamp.nanosec = data->t_ns % (1e9);
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
  odom_msg.header.stamp = rclcpp::Time(data->t_ns);//this->now();
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

void CRos2IO::Reset()
{
    path_msg_.poses.clear();
    path_msg_.poses.reserve(1024);
}

void CRos2IO::PublishPoseAndPath(basalt::PoseVelBiasState<double>::Ptr data)
{  
  geometry_msgs::msg::PoseStamped pose_msg;
  // pose_msg.header.stamp = this->now();
  // pose_msg.header.stamp = ros::Time(data->t_ns * 1.0 / 1e9);  ///< timestamp of the state in nanoseconds;  // 帧采集时间
  pose_msg.header.stamp = rclcpp::Time(data->t_ns);
  // pose_msg.header.stamp.sec = data->t_ns / 1e9;
  // pose_msg.header.stamp.nanosec = data->t_ns % 1e9;

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
  // if(1) // compute velocity.
  {
    // static constexpr int DATA_CNT = 5;
    // static constexpr double fps_inv = 1 / 25; // assume fps = 25
/*
    static constexpr int DATA_CNT = 10;
    static constexpr double fps_inv = 1 / 50; // assume fps = 50
    // static constexpr int64_t dt_ns = (46472013 - 1) * 1e9 + 924371744;
    static constexpr int64_t dt_ns = (46472013 - 0) * 1e9 +  643412768; // modified 2023-12-5.
*/
    // use member variable 'dt_ns_'
    double fps_inv = 1 / fps_;
    int DATA_CNT = 0.2 * fps_;

    static int odometry_cnt = 0;
    static int64_t t_ns = 0;
    static double total_distance = 0.0;
    static double period_distance = 0.0;
    static Vector2d last_pose(0, 0);

    static int64_t last_ns = 0;
    static double last_velocity = 0.0;

    if(t_ns == 0)
    {
       t_ns = data->t_ns - fps_inv * 1e9; 
    }

    if(last_ns == 0)
    {
       last_ns = data->t_ns - fps_inv * 1e9; 
    }

    odometry_cnt++;
    if(odometry_cnt == 1)
    {
      period_distance = 0.0;
      // t_ns = data->t_ns;
    }
    else if(odometry_cnt == DATA_CNT)
    {
      odometry_cnt = 0;
    }

    Vector2d curr_pose;
    curr_pose.x() = data->T_w_i.translation().x();
    curr_pose.y() = data->T_w_i.translation().y();
    double delta_distance = (curr_pose - last_pose).norm();
    
    double frame_delta_s = (data->t_ns - last_ns) * 1.0 * (1e-9); 
    double curr_velocity = delta_distance / frame_delta_s;
    if(last_velocity < 1e-3)
    {
      curr_velocity = last_velocity;
    }
    double accelerated_velocity = (curr_velocity - last_velocity) / frame_delta_s;
    if(accelerated_velocity > 1.2 || accelerated_velocity < -1.35 || curr_velocity > (200 / 9)) // 80km/h
    {
      curr_velocity = last_velocity;
      delta_distance = last_velocity * frame_delta_s;
    }

    period_distance += delta_distance;
    total_distance += delta_distance;

    last_pose = curr_pose;
    last_ns = data->t_ns;
    last_velocity = curr_velocity;

    if(odometry_cnt == 0)
    {
      // double delta_s = (data->t_ns - t_ns) * 1.0 * (1e-9);
      // std::cout << "velocity is " << (period_distance / 0.2) << std::endl;
      // std::cout << "velocity is " << (period_distance / delta_s) << std::endl;

      if (pub_my_odom_ != nullptr && pub_my_odom_->get_subscription_count() > 0)
      {
        myodom::msg::MyOdom odom_msg;
        // odom_msg.header.stamp = rclcpp::Time(data->t_ns);
        odom_msg.header.stamp = rclcpp::Time(data->t_ns - dt_ns_); // complementary timestamp
        odom_msg.header.frame_id = "odom";

        double delta_s = (data->t_ns - t_ns) * 1.0 * (1e-9); 

        odom_msg.velocity = period_distance / delta_s;
        odom_msg.delta_time = delta_s;
        odom_msg.period_odom = period_distance;
        odom_msg.total_odom = total_distance;

        // publish velocity, period odom and total odom.
        pub_my_odom_->publish(odom_msg);
      }

      t_ns = data->t_ns;
    }

  }

#endif

  if (pub_odom_ != nullptr && pub_odom_->get_subscription_count() > 0)
  {
    nav_msgs::msg::Odometry odom_msg;
    // odom_msg.header.stamp = time;
    // odom_msg.header.frame_id = "odom";
    odom_msg.header = pose_msg.header;
    odom_msg.pose.pose = pose_msg.pose;

    // 发布位姿
    pub_odom_->publish(odom_msg);
  }
 
  if (pub_path_ == nullptr || pub_path_->get_subscription_count() == 0)
    return ;

  // 发布轨迹   path_odom话题
  // path_msg_.header.stamp = this->now();
  path_msg_.header.stamp = rclcpp::Time(data->t_ns * 1.0 / 1e9);  ///< timestamp of the state in nanoseconds;  // 帧采集时间
  path_msg_.header.frame_id = "odom";
  path_msg_.poses.push_back(pose_msg);
  pub_path_->publish(path_msg_);

  // std::cout << " postion: " << pose_msg.pose.position.x << ", " 
  //   << pose_msg.pose.position.y << ", " << pose_msg.pose.position.z << std::endl;

}

inline void CRos2IO::getcolor(float p, float np, float& r, float& g, float& b) {
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

void CRos2IO::PublishFeatureImage(basalt::VioVisualizationData::Ptr data)
{
  static cv::Mat disp_frame;

  // step1 convert image
  uint16_t *data_in = nullptr;
  uint8_t *data_out = nullptr;

  // for(int cam_id = 0; cam_id < NUM_CAMS; cam_id++)
  for(int cam_id = 0; cam_id < 1; cam_id++)
  {
    // img_data is a vector<ImageData>
    basalt::ImageData imageData = data->opt_flow_res->input_images->img_data[cam_id];
    // std::cout << "w=" << imageData.img->w << "  h=" << imageData.img->h << std::endl;
   data_in = imageData.img->ptr;
    disp_frame = cv::Mat::zeros(imageData.img->h, imageData.img->w, CV_8UC1); // CV_8UC3
    data_out = disp_frame.ptr();

    size_t full_size = imageData.img->size();//disp_frame.cols * disp_frame.rows;
    for (size_t i = 0; i < full_size; i++) {
      int val = data_in[i];
      val = val >> 8;
      data_out[i] = val;
      // disp_frame.at(<>)
    }
 /**/

/*
    cv::Mat img(cv::Size(imageData.img->w, imageData.img->h), CV_16UC1, imageData.img->ptr);
    #ifdef _DEBUG_
    cv::imshow("feature_img", img);	
    // cv::waitKey(0);
    cv::waitKey(1);
    #endif

    cv::Mat disp_frame;
    img.convertTo(disp_frame, CV_8UC1);
    
*/
    // disp_frame.convertTo(disp_frame, CV_8UC3);
    // just gray to bgr can show colored text and pixel.
    cv::cvtColor(disp_frame, disp_frame, CV_GRAY2BGR); // CV_GRAY2RGB
    
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
            cv::circle(disp_frame, cv::Point(c[0], c[1]), radius, cv::Scalar(b, g, r));
            // cv::circle(disp_frame, cv::Point(c[0], c[1]), radius, cv::Scalar(255, 255, 255));

          }
        }

        // glColor3f(1.0, 0.0, 0.0); // to r, g, b
        // pangolin::GlFont::I()
        //     .Text("Tracked %d points", points.size())
        //     .Draw(5, 20);

        if(1)
        {
  /*
  void cv::putText  ( InputOutputArray  img,  // 要添加备注的图片
  const String &  text,  // 要添加的文字内容
  Point  org,  // 要添加的文字基准点或原点坐标，左上角还是左下角取决于最后一个参数bottomLeftOrigin的取值
               // Bottom-left corner of the text string in the image.
  int  fontFace,  
  double  fontScale, // 字体相较于最初尺寸的缩放系数。若为1.0f，则字符宽度是最初字符宽度，若为0.5f则为默认字体宽度的一半 
  Scalar  color,  // 字体颜色
  int  thickness = 1,  // 字体笔画的粗细程度
  int  lineType = LINE_8,  // 字体笔画线条类型
  
  bool  bottomLeftOrigin = false  // 如果取值为TRUE，则Point org指定的点为插入文字的左上角位置，如果取值为默认值false则指定点为插入文字的左下角位置.
                                  // When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner. 
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
//设置绘制文本的相关参数


          std::string strText = "";
          char szText[100] = { 0 };
          sprintf(szText, "Tracked %d points", points.size());
          strText = szText;
          // show text
          int font_face = cv::FONT_HERSHEY_SIMPLEX;// cv::FONT_HERSHEY_COMPLEX; 
          double font_scale = 0.5;//2;//1;
          int thickness = 1;//2; // 字体笔画的粗细程度，有默认值1
          int baseline;
          //获取文本框的长宽
          //cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
          cv::Size text_size = cv::getTextSize(strText, font_face, font_scale, thickness, &baseline);
        
          //将文本框居中绘制
          cv::Point origin; //计算文字左下角的位置
          // origin.x = image.cols / 2 - text_size.width / 2;
          // origin.y = image.rows / 2 + text_size.height / 2;
          origin.x = 0; // 2023-11-28
          // origin.x = disp_frame.cols - text_size.width;
          origin.y = text_size.height;
          //cv::putText(disp_frame, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
          // cv::putText(disp_frame, text, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness, 8, false);
          cv::putText(disp_frame, strText, origin, font_face, font_scale, cv::Scalar(255, 0, 255), thickness, 8, false);

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
          cv::circle(disp_frame, cv::Point(c[0], c[1]), 0.5f, cv::Scalar(0, 0, 255));
        }

        const Eigen::Vector2f c = kv.second.translation();

      }

      // pangolin::GlFont::I()
      //     .Text("%d opt_flow patches", kp_map.size())
      //     .Draw(5, 20);

      std::string strText = "";
      char szText[100] = { 0 };
      sprintf(szText, "%d opt_flow patches", kp_map.size());
      strText = szText;
      // show text
      int font_face = cv::FONT_HERSHEY_SIMPLEX;// cv::FONT_HERSHEY_COMPLEX; 
      double font_scale = 0.5;//1;//2;//1;
      int thickness = 1;//2; // 字体笔画的粗细程度，有默认值1
      int baseline;
      //获取文本框的长宽
      //cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
      cv::Size text_size = cv::getTextSize(strText, font_face, font_scale, thickness, &baseline);
    
      //将文本框居中绘制
      cv::Point origin; //计算文字左下角的位置
      // origin.x = image.cols / 2 - text_size.width / 2;
      // origin.y = image.rows / 2 + text_size.height / 2;
      origin.x = disp_frame.cols - text_size.width;
      origin.y = text_size.height;
      //cv::putText(disp_frame, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
      // cv::putText(disp_frame, text, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness, 8, false);
      cv::putText(disp_frame, strText, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, false); 
      
    // }
    #endif


    #ifdef _OPEN_CV_SHOW_
    cv::imshow("feature_img", disp_frame);	
    // cv::waitKey(0);
    cv::waitKey(1);
    #endif

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", disp_frame).toImageMsg();
    msg->header.stamp = rclcpp::Time(data->t_ns * 1.0 / 1e9);
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
    pub_warped_img->publish(*msg);
    
  }
    
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