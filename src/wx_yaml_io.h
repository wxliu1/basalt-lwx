

// created by wxliu on 2023-11-10

#ifndef _WX_YAML_IO_H_
#define _WX_YAML_IO_H_

#include <string>
using std::string;

#include <vector>
#include <Eigen/Dense>

namespace wx {

struct TYamlIO
{
    // TYamlIO() = default;
    // virtual ~TYamlIO() noexcept = default;
    bool show_gui = false;
    bool print_queue = false;
    std::string cam_calib_path = "/root/lwx_dataset/config/viobot_b_calib_vo.json";
    std::string dataset_path;
    std::string dataset_type;
    std::string config_path = "/root/lwx_dataset/config/viobot_b_vio_config.json";
    //std::string config_path = "../data/kitti_config.json";
    std::string result_path;
    std::string trajectory_fmt;
    bool trajectory_groundtruth { false };
    int num_threads = 0;
    bool use_imu = false;
    bool use_double = false;
    std::string marg_data_path;
    std::vector<Eigen::Matrix3d> RIC;
    std::vector<Eigen::Vector3d> TIC;
    long dt_ns { 0 }; // if dt_s_ equal to 0, it's mean our sensor is already timestamp synchronization with atp.
    int fps { 50 };
    std::vector<int> vec_tracked_points;
    std::vector<double> vec_confidence_levels;
    double coefficient { 1.0 };
    double slow_velocity { 3.0 };
    double zero_velocity { 0.05 };
    double mean_value { 0.0 };

    void ReadConfiguration();
#ifdef _NEED_WRITE_YAML    
    void WriteConfiguration();
#endif

};

}

#endif