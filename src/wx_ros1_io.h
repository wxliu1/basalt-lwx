
// created by wxliu on 2023-11-9
#ifndef _WX_ROS2_IO_H_
#define _WX_ROS2_IO_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <tbb/concurrent_queue.h>

#include <cv_bridge/cv_bridge.h>

// #include "std_msgs/string.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// #include <sensor_pub/ImageInfo.h>
#include "sensor_pub/ImageInfo.h"
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/PointCloud2Iterator.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/imu/imu_types.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <myodom/MyOdom.h>

//#include <Eigen/Dense>

using namespace std::chrono_literals;

using std::placeholders::_1;


// #define _NOISE_SUPPRESSION_

namespace wx {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
// using Quaterniond = Eigen::Quaterniond; 
using PointFields = std::vector<sensor_msgs::PointField>;
using sensor_msgs::PointField;

namespace cb = cv_bridge;
namespace sm = sensor_msgs;
// namespace gm = geometry_msgs;
namespace mf = message_filters;

class CRos1IO
{
public:    
    CRos1IO(const ros::NodeHandle& pnh, bool use_imu, int fps, long dt_ns = 0) noexcept;
    ~CRos1IO();
    void PublishPoints(basalt::VioVisualizationData::Ptr data);
    void PublishOdometry(basalt::PoseVelBiasState<double>::Ptr data);

    void Reset();
    void PublishPoseAndPath(basalt::PoseVelBiasState<double>::Ptr data); // TODO: transfer timestamp of sampling.
    void PublishFeatureImage(basalt::VioVisualizationData::Ptr data);
    void PublishMyOdom(basalt::PoseVelBiasState<double>::Ptr data, bool bl_publish = false);

private: 
    void PublishMyOdomThread();
    void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) const; 

    void StereoCb(const sm::ImageConstPtr& image0_ptr,
        const sm::ImageConstPtr& image1_ptr);

    inline void getcolor(float p, float np, float& r, float& g, float& b);

    /// @brief Get size of datatype from PointFiled
    int GetPointFieldDataTypeBytes(uint8_t dtype) noexcept {
        // INT8 = 1u,
        // UINT8 = 2u,
        // INT16 = 3u,
        // UINT16 = 4u,
        // INT32 = 5u,
        // UINT32 = 6u,
        // FLOAT32 = 7u,
        // FLOAT64 = 8u,
        switch (dtype) {
            case 0U:
            return 0;
            case 1U:  // int8
            case 2U:  // uint8
            return 1;
            case 3U:  // int16
            case 4U:  // uint16
            return 2;
            case 5U:  // int32
            case 6U:  // uint32
            case 7U:  // float32
            return 4;
            case 8U:  // float64
            return 8;
            default:
            return 0;
        }
    }

    inline PointFields MakePointFields(const std::string& fstr) {
        std::vector<PointField> fields;
        fields.reserve(fstr.size());

        int offset{0};
        PointField field;

        for (auto s : fstr) {
            s = std::tolower(s);
            if (s == 'x' || s == 'y' || s == 'z') {
            field.name = s;
            field.offset = offset;
            field.datatype = PointField::FLOAT32;
            field.count = 1;
            } else if (s == 'i') {
            field.name = "intensity";
            field.offset = offset;
            field.datatype = PointField::FLOAT32;
            field.count = 1;
            } else {
            continue;
            }

            // update offset
            offset += GetPointFieldDataTypeBytes(field.datatype) * field.count;
            fields.push_back(field);
        }

        return fields;
    }

public:
    std::function<void(basalt::OpticalFlowInput::Ptr)> feedImage_;
    std::function<void(basalt::ImuData<double>::Ptr)> feedImu_;
    std::function<void(void)> stop_;
    std::function<void(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)>inputIMU_;

private:
    ros::NodeHandle pnh_;
    // double dt_s_ { 0.0 }; // if dt_s_ equal to '0.0', it's mean our sensor is already timestamp synchronization with atp.
    std::thread t_publish_myodom;
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> pvb_queue;
    long dt_ns_ { 0 };
    int fps_ { 50 };
    bool use_imu_ = false;
    ros::Subscriber sub_imu_;
    mf::Subscriber<sm::Image> sub_image0_;
    mf::Subscriber<sm::Image> sub_image1_;
    // mf::Subscriber<sensor_pub::msg::ImageInfo> sub_image0_info_;
    // mf::Subscriber<sensor_pub::msg::ImageInfo> sub_image1_info_;
    static constexpr int NUM_CAMS = 2;
/*
 * 如果直接定义sync_stereo_，需要构造CRos2IO时，提前构造之，因为其没有缺省的构造函数
 * sync_stereo_(sub_image0_, sub_image1_, sub_image0_info_, sub_image1_info_, 10)
 * 另一个方法是借助std::optional
    mf::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_pub::msg::ImageInfo, sensor_pub::msg::ImageInfo> sync_stereo_; // method 1
*/
    using SyncStereo = mf::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>;
    std::optional<SyncStereo> sync_stereo_; // method 2

#if 0
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
#endif    
    ros::Publisher pub_point_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_path_;
    nav_msgs::Path path_msg_;

    ros::Publisher pub_warped_img;

    ros::Publisher pub_my_odom_;

/*
 * minimal publisher and subscriber
 *
 */

};

}

#endif