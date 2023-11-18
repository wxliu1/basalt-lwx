
// created by wxliu on 2023-11-13

#ifndef _WX_SYSTEM_H_
#define _WX_SYSTEM_H_

#include <eigen3/Eigen/Dense>

#include <string>
using std::string;

namespace wx {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

enum StateOrder
{
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12
};


enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};


struct SystemCfg{
    
    string imu_topic {"/imu"};

    bool use_imu{ true };
    double td{ 0.0 };
    //Eigen::Vector3d G{0.0, 0.0, 9.8};
    Vector3d g;
    
    SystemCfg() = default;
    void ReadSystemCfg();
  };

  extern SystemCfg sys_cfg_;


}

#endif
