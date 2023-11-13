
// @author: wxliu
// @date: 2023-11-10

#include "wx_yaml_io.h"

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <unistd.h>

using namespace wx;

void TYamlIO::ReadConfiguration()
{

  // read parameters from file.
  std::string config_file = "/home/lwx/./sys.yaml";
  if(access(config_file.c_str(), 0) != 0)
  {
    // '!= 0' indicate that file do not exist.
    std::ofstream fout(config_file);

    YAML::Node config = YAML::LoadFile(config_file);
    
    // example of writting items to file
    //config["score"] = 99;
    //config["time"] = time(NULL);

    config["show_gui"] = show_gui;
    config["print_queue"] = print_queue;
    config["cam_calib_path"] = cam_calib_path;
    config["dataset_path"] = dataset_path;
    config["dataset_type"] = dataset_type;
    config["config_path"] = config_path;
    config["result_path"] = result_path;
    config["trajectory_fmt"] = trajectory_fmt;
    config["trajectory_groundtruth"] = trajectory_groundtruth;
    config["num_threads"] = num_threads;
    config["use_imu"] = use_imu;
    config["use_double"] = use_double;

    fout << config;

    fout.close();
   
  }
  else
  {
    // read items:
    YAML::Node config = YAML::LoadFile(config_file);
    /*
        * here is a demo for reading:
    int score = config["score"].as<int>();
    time_t time = config["time"].as<time_t>();
    if(config["image0_topic"].Type() == YAML::NodeType::Scalar)
    std::string image0_topic = config["image0_topic"].as<std::string>();
    */

    show_gui = config["show_gui"].as<bool>();
    print_queue = config["print_queue"].as<bool>();
    cam_calib_path = config["cam_calib_path"].as<std::string>();
    dataset_path = config["dataset_path"].as<std::string>();
    dataset_type = config["dataset_type"].as<std::string>();
    config_path = config["config_path"].as<std::string>();
    result_path = config["result_path"].as<std::string>();
    trajectory_fmt = config["trajectory_fmt"].as<std::string>();
    trajectory_groundtruth = config["trajectory_groundtruth"].as<bool>();
    num_threads = config["num_threads"].as<int>();
    use_imu = config["use_imu"].as<bool>();
    use_double = config["use_double"].as<bool>();

    
  }

}

#ifdef _NEED_WRITE_YAML
void TYamlIO::WriteConfiguration()
{
    std::string config_file = "./sys.yaml";
    if(access(config_file.c_str(), 0) != 0)
    {
        // '!= 0' indicate that file do not exist.
        std::ofstream fout(config_file);

        YAML::Node config = YAML::LoadFile(config_file);

        // example of writting item to file
        config["score"] = 99;
        config["time"] = time(NULL);

        fout << config;

        fout.close();
    }
    else
    {}

    YAML::Node config = YAML::LoadFile(config_file);
    int score = config["score"].as<int>();
    time_t time = config["time"].as<time_t>();

    std::ofstream fout(config_file);
    config["score"] = 100;
    config["time"] = time(NULL);

    fout << config;

    fout.close();
    
}
#endif