
// @author: wxliu
// @date: 2023-11-10

#include "wx_yaml_io.h"

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <unistd.h>
#include <basalt/utils/assert.h>

using namespace wx;

void TYamlIO::ReadConfiguration()
{

  // read parameters from file.
  // std::string config_file = "/home/lwx/./sys.yaml";
  // std::string config_file = "/root/./sys.yaml";

  // std::string pkg_path = ament_index_cpp::get_package_share_directory("stereo3");
  // std::string config_file = pkg_path + "/config/stereo3.yaml";

  // std::string config_file = "/root/stereo3_ws/install/share/stereo3/config/sys.yaml";
  std::string config_file = "/root/dev/stereo3_ros1_ws/install/share/stereo3/config/sys.yaml";
  std::cout << "sys.yaml: " << config_file << std::endl;
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
    config["dt_ns"] = dt_ns;
    config["fps"] = fps;
    config["coefficient"] = coefficient;
    config["slow_velocity"] = slow_velocity;
    config["zero_velocity"] = zero_velocity;
    config["mean_value"] = mean_value;

    config["tks_pro_integration"] = tks_pro_integration;
    config["src_ip_address"] = src_ip_address;
    config["dest_ip_adderss"] = dest_ip_adderss;
    config["debug_mode"] = debug_mode;
    config["output_data_file"] = output_data_file;
    config["data_output"] = data_output;
    config["data_display"] = data_display;
    config["bag_flag"] = bag_flag;
    config["atp_id"] = atp_id;

    config["number_of_255"] = number_of_255;
    config["log_freq"] = log_freq;
    config["abnormal_velocity"] = abnormal_velocity;
    config["record_bag"] = record_bag;
    config["record_duration"] = record_duration;

    config["acc_zero_velocity"] = acc_zero_velocity;
    config["ang_zero_velocity"] = ang_zero_velocity;
    config["change_end_wait_time"] = change_end_wait_time;
    config["output_log"] = output_log;
    config["photometric_calibration"] = photometric_calibration;

    config["image_width"] = image_width;
    config["image_height"] = image_height;

    config["camera1"]["gamma"] = gamma1;
    config["camera1"]["vignette"] = vignette1;

    config["camera2"]["gamma"] = gamma2;
    config["camera2"]["vignette"] = vignette2;

    config["computing_mode"] = computing_mode;

/*
    std::vector<double> vector_T{1.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};
    
    config["body_T_cam0"]["data"] = vector_T;
*/
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
    dt_ns = config["dt_ns"].as<long>();
    fps = config["fps"].as<int>();

    // if(config["coefficient"].Type() == YAML::NodeType::Scalar)
    coefficient = config["coefficient"].as<double>();

    if(config["slow_velocity"].Type() == YAML::NodeType::Scalar)
    slow_velocity = config["slow_velocity"].as<double>();

    if(config["zero_velocity"].Type() == YAML::NodeType::Scalar)
    zero_velocity = config["zero_velocity"].as<double>();

    if(config["mean_value"].Type() == YAML::NodeType::Scalar)
    mean_value = config["mean_value"].as<double>();

    // for tks.
    if(config["tks_pro_integration"].Type() == YAML::NodeType::Scalar)
    tks_pro_integration = config["tks_pro_integration"].as<bool>();

    if(config["src_ip_address"].Type() == YAML::NodeType::Scalar)
    src_ip_address = config["src_ip_address"].as<std::string>();

    if(config["dest_ip_adderss"].Type() == YAML::NodeType::Scalar)
    dest_ip_adderss = config["dest_ip_adderss"].as<std::string>();

    if(config["debug_mode"].Type() == YAML::NodeType::Scalar)
    debug_mode = config["debug_mode"].as<bool>();

    if(config["output_data_file"].Type() == YAML::NodeType::Scalar)
    output_data_file = config["output_data_file"].as<std::string>();

    if(config["data_output"].Type() == YAML::NodeType::Scalar)
    data_output = config["data_output"].as<bool>();

    if(config["data_display"].Type() == YAML::NodeType::Scalar)
    data_display = config["data_display"].as<bool>();

    if(config["bag_flag"].Type() == YAML::NodeType::Scalar)
    bag_flag = config["bag_flag"].as<bool>();

    if(config["atp_id"].Type() == YAML::NodeType::Scalar)
    atp_id = config["atp_id"].as<int>();

    if(config["number_of_255"].Type() == YAML::NodeType::Scalar)
    number_of_255 = config["number_of_255"].as<int>();

    if(config["log_freq"].Type() == YAML::NodeType::Scalar)
      log_freq = config["log_freq"].as<int>();

    if(config["abnormal_velocity"].Type() == YAML::NodeType::Scalar)  
      abnormal_velocity = config["abnormal_velocity"].as<double>();

    if(config["record_bag"].Type() == YAML::NodeType::Scalar)
    record_bag = config["record_bag"].as<bool>(); 

    if(config["record_duration"].Type() == YAML::NodeType::Scalar)
      record_duration = config["record_duration"].as<int>(); 

    if(config["acc_zero_velocity"].Type() == YAML::NodeType::Scalar)
    acc_zero_velocity = config["acc_zero_velocity"].as<double>();

    if(config["ang_zero_velocity"].Type() == YAML::NodeType::Scalar)
    ang_zero_velocity = config["ang_zero_velocity"].as<double>();

    if(config["change_end_wait_time"].Type() == YAML::NodeType::Scalar)
    change_end_wait_time = config["change_end_wait_time"].as<double>();

    if(config["output_log"].Type() == YAML::NodeType::Scalar)
    output_log = config["output_log"].as<bool>();

    if(config["photometric_calibration"].Type() == YAML::NodeType::Scalar)
    photometric_calibration = config["photometric_calibration"].as<bool>();

    if(config["image_width"].Type() == YAML::NodeType::Scalar)
    image_width = config["image_width"].as<int>();

    if(config["image_height"].Type() == YAML::NodeType::Scalar)
    image_height = config["image_height"].as<int>();

    // read photometric calibration file path.
    if(config["camera1"]["gamma"].Type() == YAML::NodeType::Scalar)
    gamma1 = config["camera1"]["gamma"].as<std::string>();

    if(config["camera1"]["vignette"].Type() == YAML::NodeType::Scalar)
    vignette1 = config["camera1"]["vignette"].as<std::string>();

    if(config["camera2"]["gamma"].Type() == YAML::NodeType::Scalar)
    gamma2 = config["camera2"]["gamma"].as<std::string>();

    if(config["camera2"]["vignette"].Type() == YAML::NodeType::Scalar)
    vignette2 = config["camera2"]["vignette"].as<std::string>();

    if(config["computing_mode"].Type() == YAML::NodeType::Scalar)
    computing_mode = config["computing_mode"].as<int>();

    // read imu_cam extrinsic
    std::vector<double> vector_T{1.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};
    vector_T = config["body_T_cam0"]["data"].as<std::vector<double>>();

    Eigen::Map<Eigen::Matrix4d> T(vector_T.data());
    Eigen::Matrix4d T2 = T.transpose();

    
    RIC.push_back(T2.block<3, 3>(0, 0));
    TIC.push_back(T2.block<3, 1>(0, 3));

    //imu_.setExtrinsic(RIC[0], TIC[0]);

    vec_tracked_points = config["confidence_interval"]["tracked_points"].as<std::vector<int>>();
    vec_confidence_levels = config["confidence_interval"]["confidence_levels"].as<std::vector<double>>();
/*
 * // no use in relase mode.    
    assert(vec_tracked_points.size() > 0);
    assert(vec_tracked_points.size() == vec_confidence_levels.size());
 */

    BASALT_ASSERT(vec_tracked_points.size() > 0);
    BASALT_ASSERT(vec_tracked_points.size() == vec_confidence_levels.size());

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