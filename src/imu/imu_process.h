
// created [wxliu 2023-6-10]

#ifndef _IMU_PROCESS_H_
#define _IMU_PROCESS_H_

//#include <sensor_msgs/Imu.h> // comment - wxliu 2023-6-16

#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/integration_base.h"
#include "../utility/utility.h"
#include <sophus/se3.hpp>
#include <functional>
#include <map>
using std::map;
#include<vector>
using std::vector;
#include<utility>
using std::pair;
#include <mutex>
#include <queue>
using std::queue;

namespace wx {

static constexpr int WINDOW_SIZE = 10;
static constexpr int NUM_OF_CAM = 2;
//constexpr int USE_IMU = 1;  // comment 2023-6-19

class ImageFrame {
 public:
  ImageFrame(){};
  ImageFrame(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points,
      double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t;
  Matrix3d R;
  Vector3d T;
  IntegrationBase* pre_integration;
  bool is_key_frame;
};

enum SolverFlag { INITIAL, NON_LINEAR };

struct ImuProcess
{
  public:
    explicit ImuProcess();
    void readExtrinsic();
    void InitRosIO();
    void reset();
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void setExtrinsic(const Matrix3d& ric, const Vector3d& tic); // added by wxliu 2023-6-16
    //void setNodeHandle(const ros::NodeHandle& pnh); // comment - wxliu 2023-6-16
    void clearState();
    void slideWindow(int marg_frame_index); 
    bool ProcessData(double current_time) noexcept;
    void UpdateImuPose(const Sophus::SE3d& T_w_cl) noexcept;
    bool IsDeviceHorizontal() const noexcept { return is_device_horizontal; }

    const Sophus::SE3d& Twc0() const noexcept { return T_w_c0; }
    //void SetTwc0(const Sophus::SE3d& T_w_cl) noexcept { T_w_c0 = T_w_cl; }

    bool InitFirstPoseFlag() const noexcept { return initFirstPoseFlag; }
    void SetFirstPoseFlag(bool bl) noexcept { initFirstPoseFlag = bl; }

    bool UseImuPose() const noexcept { return use_imu_pose; }
    void SetUseImuPose(bool bl) noexcept { use_imu_pose = bl; }

    SolverFlag GetSolverFlag() const noexcept { return  solver_flag; }

    void NonlinearOptimization(double current_time);
    void CalcImuPose(Sophus::SE3d& T_w_cl, bool camToWorld);

private:
    //void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg); // comment - wxliu 2023-6-16
    
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    
    
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void solveGyroscopeBias(vector<pair<double, ImageFrame>> &all_image_frame, Vector3d* Bgs);
    
    //void PublishOdom2(const std_msgs::Header& header, const Sophus::SE3d& tf);

    void optimization();
    void vector2double();
    void double2vector();


public:
    static constexpr int WINDOW_SIZE = 10;
    static constexpr int NUM_OF_CAM = 2;
    std::function<void(const Sophus::SE3d&)> SetTwc0_; // 2023-6-12
    std::function<void(bool)>SetNotHorizontalReboot_; // is_not_horizontal_reboot
    

public:
  Matrix3d ric[2]; // move here 2023-7-18.
  Vector3d tic[2];


private:
    /*
     * comment by wxliu 2023-6-16
    ros::NodeHandle pnh_;
    ros::Subscriber sub_imu_;
    */
    bool is_device_horizontal { true }; 
    bool receive_imu{false};
    bool initFirstPoseFlag{false};
    Sophus::SE3d T_w_c0;
    int not_horizontal_cnt { 0 };

     // wxliu 2023-7-18.
    int imu_init_data_cnt { 0 };
    vector<pair<double, Eigen::Vector3d>> accVector_;
    // the end.

  std::mutex mProcess;
  std::mutex mBuf;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  double prevTime, curTime;
  
  bool first_imu;
  Vector3d acc_0, gyr_0;

  //Matrix3d ric[2];
  //Vector3d tic[2];

  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];  // imuToWorld
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];

  IntegrationBase* pre_integrations[(WINDOW_SIZE + 1)];
  vector<double> dt_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  // map<double, ImageFrame> all_image_frame;
  vector<pair<double, ImageFrame>> all_image_frame;
  IntegrationBase* tmp_pre_integration;
  int frame_count;

  bool use_imu_pose{false};
  
  SolverFlag solver_flag;

 
  bool failure_occur;
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  // double para_Ex_Pose[2][SIZE_POSE];

}; // struct ImuProcess

// extern struct ImuProcess imu_; // wxliu 2023-6-16.

} // namespace wx

#endif