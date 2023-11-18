
// created by wxliu on 2023-11-9
#ifndef _WX_ROS2_IO_H_
#define _WX_ROS2_IO_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_pub/msg/image_info.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <functional>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/imu/imu_types.h>
#include <basalt/vi_estimator/vio_estimator.h>

//#include <Eigen/Dense>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;


template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
    using is_specialized = std::true_type;
    using custom_type = std::string;
    using ros_message_type = std_msgs::msg::String;

    static
    void
    convert_to_ros_message(
        const custom_type & source,
        ros_message_type & destination)
    {
        destination.data = source;
    }

    static
    void
    convert_to_custom(
        const ros_message_type & source,
        custom_type & destination)
    {
        destination = source.data;
    }
};

namespace wx {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
// using Quaterniond = Eigen::Quaterniond; 
// namespace cb = cv_bridge;
// namespace sm = sensor_msgs;
// namespace gm = geometry_msgs;
namespace mf = message_filters;

class CRos2IO : public rclcpp::Node
{
public:    
    CRos2IO(bool use_imu);
    void PublishPoints(basalt::VioVisualizationData::Ptr data);
    void PublishOdometry(basalt::PoseVelBiasState<double>::Ptr data);
    // void SetImu(bool bl);
    //bool Imu() const { return use_imu_ };
    void Reset();
    //void PublishPoint(basalt::VioVisualizationData::Ptr data); // TODO: transfer timestamp of sampling.
    void PublishPoseAndPath(basalt::PoseVelBiasState<double>::Ptr data);

private:    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) const; 
    //void imu_callback(const sensor_msgs::msg::Imu & msg); const

    void StereoCb(const sensor_msgs::msg::Image::SharedPtr &image0_ptr,
        const sensor_msgs::msg::Image::SharedPtr &image1_ptr/*,
        const sensor_pub::msg::ImageInfo::SharedPtr &image0_info_ptr,
        const sensor_pub::msg::ImageInfo::SharedPtr &image1_info_ptr*/);

#if 0
    // sys_stop_sub回调函数，用于stop
    // 指令：ros2 topic pub /sys_stop std_msgs/msg/Empty "{}" -1
    void sys_stop_callback(const std_msgs::msg::Empty::SharedPtr msg) {
        UNUSED(msg);
        //stop();
        
        // ros2节点如果未停止，则停止
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }    
#endif

public:
    std::function<void(basalt::OpticalFlowInput::Ptr)> feedImage_;
    std::function<void(basalt::ImuData<double>::Ptr)> feedImu_;
    std::function<void(void)> stop_;
    std::function<void(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)>inputIMU_;

private:
    bool use_imu_ = false;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    mf::Subscriber<sensor_msgs::msg::Image> sub_image0_;
    mf::Subscriber<sensor_msgs::msg::Image> sub_image1_;
    mf::Subscriber<sensor_pub::msg::ImageInfo> sub_image0_info_;
    mf::Subscriber<sensor_pub::msg::ImageInfo> sub_image1_info_;
    static constexpr int NUM_CAMS = 2;
/*
 * 如果直接定义sync_stereo_，需要构造CRos2IO时，提前构造之，因为其没有缺省的构造函数
 * sync_stereo_(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 10)
 * 另一个方法是借助std::optional
    mf::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_pub::msg::ImageInfo, sensor_pub::msg::ImageInfo> sync_stereo_; // method 1
*/
    using SyncStereo = mf::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image/*,
        sensor_pub::msg::ImageInfo, sensor_pub::msg::ImageInfo*/>;
    std::optional<SyncStereo> sync_stereo_; // method 2

#if 0
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
#endif    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    nav_msgs::msg::Path path_msg_;

/*
 * minimal publisher and subscriber
    using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
    CRos2IO()
    : Node("wx_Ros2_IO")
    {
        // subscribe
        subscription_ = this->create_subscription<MyAdaptedType>(
            "topic", 10, std::bind(&CRos2IO::topic_callback, this, _1));

        subscription2_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&CRos2IO::topic_callback2, this, _1));

        // publish
        publisher_ = this->create_publisher<MyAdaptedType>("topic", 10);

        publisher2_ = this->create_publisher<std_msgs::msg::String>("topic", 10);    

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CRos2IO::timer_callback, this));

        
    }

private:
    void topic_callback(const std::string & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    }
    rclcpp::Subscription<MyAdaptedType>::SharedPtr subscription_;

    void topic_callback2(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;

private:
  void timer_callback()
  {
    std::string message = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<MyAdaptedType>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  size_t count_;
*/
};

}

#endif