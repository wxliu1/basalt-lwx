
// @author: wxliu
// @date: 2023-11-13

#include "wx_system.h"


namespace wx {

SystemCfg sys_cfg_; 

double ACC_N, ACC_W;
double GYR_N, GYR_W;

Eigen::Vector3d G{0.0, 0.0, 9.8};

void SystemCfg::ReadSystemCfg()
{
    ACC_N = 0.1;
    GYR_N = 0.01;
    ACC_W = 0.001;
    GYR_W = 0.0001;

    g = G;
}


}

