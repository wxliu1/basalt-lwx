#pragma once 
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
//20231215增加一列is_vaild，表示视觉当前输出是否有效
class Data_output{
public:
    Data_output(std::string file_name = "/home/tl/data/data_output.csv"){
        fname = file_name;
        std::ofstream outFile(fname, std::ios::out);
         if (outFile.is_open()){  // 检查文件是否打开成功
            // 写入标题行
            outFile << "num" << ','
                << "atp_speed" << ','
                << "atp_period_odom" << ','
                << "atp_calc_odom" << ','
                << "vio_speed" << ','
                << "vio_period_odom" << ','
                << "vio_calc_odom" << ','
                << "speed_error" << ','
                << "peroid_odom_error" << ','
                << "calc_odom_error" << ','
                << "beacon_id" << ','
                << "beacon_odom" << ','
                << "is_vaild" << std::endl;
            //数字需转为字符串进行写入,csv文件结束一行写入需要"\n"或者endl进行换行
            outFile.close();
        }
        else{
            std::cout << "文件无法打开！" << std::endl;
        }
    }
    ~Data_output(){}

    void write_data(uint16_t line,uint16_t atp_speed,uint16_t atp_period_odom,uint32_t atp_calc_odom,
                    uint16_t vio_speed,uint16_t vio_period_odom,uint32_t vio_calc_odom,
                    int speed_error,int peroid_odom_error,int calc_odom_error,
                    uint32_t beacon_id_,uint32_t beacon_odom_,uint16_t is_vaild_){
        std::lock_guard<std::mutex> lock(mtx); // 加锁
        std::ifstream file(fname);
        if (!file.is_open()) {
            std::cout << "File does not exist!" << std::endl;
            return ;
        }
        else {
            std::ofstream outFile(fname, std::ios::app);
            // ********写入两行数据*********
            outFile << std::to_string(line) << ','
                    << std::to_string(atp_speed) << ','
                    << std::to_string(atp_period_odom) << ','
                    << std::to_string(atp_calc_odom) << ','
                    << std::to_string(vio_speed) << ','
                    << std::to_string(vio_period_odom) << ','
                    << std::to_string(vio_calc_odom) << ','
                    << std::to_string(speed_error) << ','
                    << std::to_string(peroid_odom_error) << ','
                    << std::to_string(calc_odom_error) << ','
                    << std::to_string(beacon_id_) << ','
                    << std::to_string(beacon_odom_) << ','
                    << std::to_string(is_vaild_) << std::endl;
            //数字需转为字符串进行写入,csv文件结束一行写入需要"\n"或者endl进行换行
            outFile.close();
        }
    }

    void change_ends(){
        std::lock_guard<std::mutex> lock(mtx); // 加锁
        std::ifstream file(fname);
        if (!file.is_open()) {
            std::cout << "File does not exist!" << std::endl;
            return ;
        }
        else {
            std::ofstream outFile(fname, std::ios::app);
            // ********""写入两行数据*********
            outFile << "change_ends" <<  std::endl;
            //数字需转为字符串进行写入,csv文件结束一行写入需要"\n"或者endl进行换行
            outFile.close();
        }
    }

private:
    std::string fname;
    std::mutex mtx;
};



