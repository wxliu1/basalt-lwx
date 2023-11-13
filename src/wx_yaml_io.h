

// created by wxliu on 2023-11-10

#ifndef _WX_YAML_IO_H_
#define _WX_YAML_IO_H_

#include <string>
using std::string;

namespace wx {

struct TYamlIO
{
    // TYamlIO() = default;
    // virtual ~TYamlIO() noexcept = default;
    bool show_gui = true;
    bool print_queue = false;
    std::string cam_calib_path = "/home/lwx/dataset/config/viobot_b_calib_vo.json";
    std::string dataset_path;
    std::string dataset_type;
    std::string config_path = "/home/lwx/dataset/config/viobot_b_vio_config.json";
    //std::string config_path = "../data/kitti_config.json";
    std::string result_path;
    std::string trajectory_fmt;
    bool trajectory_groundtruth;
    int num_threads = 0;
    bool use_imu = false;
    bool use_double = false;
    std::string marg_data_path;

    void ReadConfiguration();
#ifdef _NEED_WRITE_YAML    
    void WriteConfiguration();
#endif

};

}

#endif