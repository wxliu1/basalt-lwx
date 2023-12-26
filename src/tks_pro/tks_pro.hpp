#pragma once 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "vio_udp.hpp"
// #include "atp_info/atp.h"
#include <atp_info/atp.h>
#include "data_output.hpp"

#include <deque>
#include "myodom/MyOdom.h"

#include <chrono>
#include <ctime>
#include <string>

#include "../wx_yaml_io.h"

class Tks_pro{
public:
    Tks_pro(ros::NodeHandle& nh, const TYamlIO &yaml) : yaml_(yaml) {
         ROS_INFO("=====================version 2023-12-26=====================");
        // 调用函数获取当前系统时间并输出
        std::string currentTimeString = getCurrentSystemTime();
        line_vio = 1,line_atp = 1;

        vio_udp_ = std::make_shared<vio_udp>(yaml_.src_ip_address.c_str(),yaml_.dest_ip_adderss.c_str());  

        vio_udp_->debug_mode = yaml_.debug_mode;

        std::string output_data_file;
        output_data_file = yaml_.output_data_file + currentTimeString + "_data_output.csv";
        std::cout << "output data file: " << output_data_file << std::endl;
        data_output_flag = yaml_.data_output;
        data_display = yaml_.data_display;
        if(data_output_flag)
            data_output = std::make_shared<Data_output>(output_data_file);
        // nh.param<bool>("odom_calc_sec_flag",odom_calc_sec_flag,false);
        odom_calc_sec_flag = false;
        bag_flag = yaml_.bag_flag;
        if(bag_flag)//是否是数据包模式，数据包模式可以直接读取atp话题的数据
            sub_atp_info = nh.subscribe("/atp_info", 2, &Tks_pro::atp_callback,this);
        else//不是数据包模式则通过UDP协议接收ATP信息，打包ATP数据，保存并且发出话题消息
            atp_info_pub = nh.advertise<atp_info::atp>("/atp_info",5);
        atp_info_init();
        // nh.param<int>("odom_times",odom_times,3);
        odom_times = 3;
        // nh.param<int>("atp_times",atp_times,6);
        atp_times = 6;
        // nh.param<int>("atp_id",atp_id_num,1);
        atp_id_num = yaml_.atp_id;
        odom_callback_flag = 0,udp_recv_flag = 0;
        confidence_coefficient = 0.0;
        change_ends_flag = false;
        is_forward = false;

        timer_UDP_send = nh.createTimer(ros::Duration(0.2), std::bind(&Tks_pro::UDP_send, this, std::placeholders::_1));
    }

    Tks_pro(ros::NodeHandle& nh){
        ROS_INFO("=====================version 2023-12-22=====================");
        // 调用函数获取当前系统时间并输出
        std::string currentTimeString = getCurrentSystemTime();
        line_vio = 1,line_atp = 1;
        std::string src_ip_address,dest_ip_adderss;
        nh.param<std::string>("src_ip_address",src_ip_address,"192.168.55.28");
        nh.param<std::string>("dest_ip_adderss",dest_ip_adderss,"192.168.55.21");
        vio_udp_ = std::make_shared<vio_udp>(src_ip_address.c_str(),dest_ip_adderss.c_str());  
        bool debug_mode;
        nh.param<bool>("debug_mode",debug_mode,false);
        vio_udp_->debug_mode = debug_mode;
        std::string output_data_file;
        nh.param<std::string>("output_data_file",output_data_file,"/home/tl/data/");
        output_data_file = output_data_file + currentTimeString + "_data_output.csv";
        nh.param<bool>("data_output",data_output_flag,false);
        nh.param<bool>("data_display",data_display,false);
        if(data_output_flag)
            data_output = std::make_shared<Data_output>(output_data_file);
        nh.param<bool>("odom_calc_sec_flag",odom_calc_sec_flag,false);
        nh.param<bool>("bag_flag",bag_flag,false);
        if(bag_flag)//是否是数据包模式，数据包模式可以直接读取atp话题的数据
            sub_atp_info = nh.subscribe("/atp_info", 2, &Tks_pro::atp_callback,this);
        else//不是数据包模式则通过UDP协议接收ATP信息，打包ATP数据，保存并且发出话题消息
            atp_info_pub = nh.advertise<atp_info::atp>("/atp_info",5);
        atp_info_init();
        nh.param<int>("odom_times",odom_times,3);
        nh.param<int>("atp_times",atp_times,6);
        nh.param<int>("atp_id",atp_id_num,1);
        odom_callback_flag = 0,udp_recv_flag = 0;
        confidence_coefficient = 0.0;
        change_ends_flag = false;
        is_forward = false;

        timer_UDP_send = nh.createTimer(ros::Duration(0.2), std::bind(&Tks_pro::UDP_send, this, std::placeholders::_1));
    }

    ~Tks_pro(){}

    void add_odom_frame(double speed_,double calc_odom_result_,float confidence_coefficient_){
        vio_udp_odom_read.lock();
        vio_speed_calc = speed_ * 100;
        // period_odom = msg->period_odom * 100;
        period_odom = vio_speed_calc * 0.2;
        calc_odom_result = calc_odom_result_ * 100;
        confidence_coefficient = confidence_coefficient_;
        odom_callback_flag = 0;
        vio_udp_odom_read.unlock();
    }

    inline bool IsForward() {return is_forward; }

private:
    void atp_info_init(){
        #ifdef _ADD_ATP_ID_
        atp_info_p.atp_id = 0;
        std::cout << "atp init.  atp_id=" << atp_info_p.atp_id << std::endl;
        #endif
        atp_info_p.atp_speed = 0;
        atp_info_p.atp_period_odom = 0;
        atp_info_p.atp_calc_odom = 0;
        atp_info_p.beacon_id = 0;
        atp_info_p.beacon_odom = 0;
    }


    std::string getCurrentSystemTime() {
        // 获取当前系统时间点
        auto now = std::chrono::system_clock::now();
        // 将时间点转换为time_t类型
        std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
        // 使用本地时间进行格式化
        struct tm *localTime = std::localtime(&currentTime);
        // 创建一个缓冲区来保存格式化后的时间字符串
        char buffer[80];
        // 使用strftime函数将时间结构体格式化为字符串
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localTime);
        // 将格式化后的时间字符串转换为std::string并返回
        return std::string(buffer);
    }

    void atp_callback(const atp_info::atpPtr &msg){
        #ifdef _ADD_ATP_ID_
        atp_info_p.atp_id = msg->atp_id;
        #endif
        atp_info_p.atp_speed = msg->atp_speed;
        atp_info_p.atp_period_odom = msg->atp_period_odom;
        atp_info_p.atp_calc_odom = msg->atp_calc_odom;
        atp_info_p.beacon_id = msg->beacon_id;
        atp_info_p.beacon_odom = msg->beacon_odom;
        udp_recv_flag = 0;
    }

    void atp_info_publish(){
        ROS_INFO("atp_info_publish");
        vio_udp_->read_atp_info.lock();
        atp_info_p.header.stamp = ros::Time::now();
        #ifdef _ADD_ATP_ID_
        atp_info_p.atp_id = vio_udp_->atp_info_.atp_id;
        #endif
        atp_info_p.cur_station_id = vio_udp_->atp_info_.cur_station_id;
        atp_info_p.next_station_id = vio_udp_->atp_info_.next_station_id;
        atp_info_p.beacon_id = vio_udp_->atp_info_.beacon_id;
        atp_info_p.beacon_odom = vio_udp_->atp_info_.beacon_odom;
        atp_info_p.atp_speed = vio_udp_->atp_info_.atp_speed;
        atp_info_p.atp_period_odom = vio_udp_->atp_info_.atp_period_odom;
        atp_info_p.atp_calc_odom = vio_udp_->atp_info_.atp_calc_odom;
        atp_info_pub.publish(atp_info_p);
        vio_udp_->read_atp_info.unlock(); 
        udp_recv_flag = 0;
    }

    //200ms定时发送信息
    void UDP_send(const ros::TimerEvent &e){
        vio_udp_odom_read.lock();
        ROS_INFO("vio_udp_->atp_info_.atp_id = %d",vio_udp_->atp_info_.atp_id);
        error_calc(atp_info_p.atp_speed,atp_info_p.atp_period_odom,atp_info_p.atp_calc_odom,
                   vio_speed_calc,(uint16_t)period_odom,(uint32_t)calc_odom_result);
        if(vio_speed_calc > 3000) confidence_coefficient = 0;//2023-12-21
        if(confidence_coefficient < 1){//视觉数据无效
            ROS_INFO("============================confidence_coefficient error ==================================");
            vio_udp_->vio_udp_send(vio_speed_calc,period_odom,calc_odom_result,false);
            if(vio_udp_->atp_info_.atp_id == atp_id_num){//在这个方向跑才存日志
                change_ends_flag = false;
                is_forward = true;
                if(data_output_flag)
                    data_output->write_data(line_vio++,atp_info_p.atp_speed,atp_info_p.atp_period_odom,atp_info_p.atp_calc_odom,
                                            vio_speed_calc,(uint16_t)period_odom,(uint32_t)calc_odom_result,
                                            speed_error,peroid_odom_error,calc_odom_error,
                                            atp_info_p.beacon_id,atp_info_p.beacon_odom,0);
            }
            else{
                is_forward = false;
                if(change_ends_flag == false){
                    change_ends_flag = true;
                    data_output->change_ends();
                }
            }
        }
        else{//视觉数据有效
            if(vio_udp_->atp_info_.atp_id == atp_id_num){//在这个方向跑才存日志
                vio_udp_->vio_udp_send(vio_speed_calc,period_odom,calc_odom_result,true);
                change_ends_flag = false;
                is_forward = true;
                if(data_output_flag)
                    data_output->write_data(line_vio++,atp_info_p.atp_speed,atp_info_p.atp_period_odom,atp_info_p.atp_calc_odom,
                                            vio_speed_calc,(uint16_t)period_odom,(uint32_t)calc_odom_result,
                                            speed_error,peroid_odom_error,calc_odom_error,
                                            atp_info_p.beacon_id,atp_info_p.beacon_odom,1);
            }
            else{//反向跑直接置信位给0
                vio_udp_->vio_udp_send(vio_speed_calc,period_odom,calc_odom_result,false);
                is_forward = false;    
                if(change_ends_flag == false){
                    change_ends_flag = true;
                    data_output->change_ends();
                }
            }
        }
        if(data_display){
            printf("===================================\n");
            ROS_INFO("vio_speed = %d,vio_period_odom = %lf,vio_calc_odom = %lf",vio_speed_calc,period_odom,calc_odom_result);
            ROS_INFO("atp_speed = %d,atp_period_odom = %d,atp_calc_odom = %d",
                    atp_info_p.atp_speed,atp_info_p.atp_period_odom,atp_info_p.atp_calc_odom);   
            ROS_INFO("speed_error = %d,peroid_odom_error = %d,calc_odom_error = %d",
                    speed_error,peroid_odom_error,calc_odom_error);   
            ROS_INFO("speed_error_persent = %f%%,peroid_odom_error_persent = %f%%,calc_odom_error_persent = %f%%",
                    speed_error_persent * 100.0,peroid_odom_error_persent * 100.0,calc_odom_error_persent * 100.0);  
            printf("===================================\n"); 
        }
        else
            ROS_INFO("vio_speed = %d,vio_period_odom = %lf,vio_calc_odom = %lf",vio_speed_calc,period_odom,calc_odom_result);
        data_reset();
        vio_udp_odom_read.unlock();
        if(!bag_flag)
            if(vio_udp_->atp_info_flag){
                atp_info_publish();
                vio_udp_->atp_info_flag = false;
            }
    }

    void error_calc(uint16_t atp_speed,uint16_t atp_period_odom,uint32_t atp_calc_odom,
                    uint16_t vio_speed,uint16_t vio_period_odom,uint32_t vio_calc_odom){
        speed_error = atp_speed - vio_speed;
        peroid_odom_error = atp_period_odom - vio_period_odom;
        calc_odom_error = atp_calc_odom - vio_calc_odom;   
        if(atp_speed != 0) 
            speed_error_persent = (float)speed_error / (float)atp_speed;
        if (atp_period_odom != 0)
            peroid_odom_error_persent = (float)peroid_odom_error / (float)atp_period_odom;
        if (atp_calc_odom != 0)                
            calc_odom_error_persent = (float)calc_odom_error / (float)atp_calc_odom;
    }

    void data_reset(){
        if(odom_callback_flag > odom_times){
            vio_speed_calc = 0;
            period_odom = 0.0;
        }
        if(udp_recv_flag > atp_times){//没收到数据，将一些东西置零
            atp_info_p.atp_speed = 0;
            atp_info_p.atp_period_odom = 0;
            vio_udp_->atp_info_.atp_id = 0;
            atp_info_p.beacon_id = 0,atp_info_p.beacon_odom = 0;
        }
        speed_error_persent = 0.0;
        peroid_odom_error_persent = 0.0;
        calc_odom_error_persent = 0.0;
        odom_callback_flag ++;
        udp_recv_flag ++;
    }

private:
    std::shared_ptr<vio_udp> vio_udp_;
    std::shared_ptr<Data_output> data_output;
    int line_vio,line_atp;
    ros::Publisher atp_info_pub; 
    
    ros::Timer timer_UDP_send;
    atp_info::atp atp_info_p;
    ros::Subscriber sub_atp_info;

    std::mutex  vio_udp_odom_read;
    double period_odom = 0.0;
    double calc_odom_result = 0.0;
    uint16_t vio_speed_calc = 0;
    int speed_error = 0,peroid_odom_error = 0,calc_odom_error = 0;
    float speed_error_persent = 0.0,peroid_odom_error_persent = 0.0,calc_odom_error_persent = 0.0;
    bool data_output_flag,data_display;
    uint16_t speed_callback_sum;
    uint16_t callback_count;
    bool bag_flag,odom_calc_sec_flag;
    float confidence_coefficient;
    int odom_times,atp_times;
    int odom_callback_flag,udp_recv_flag;
    int atp_id_num;
    bool change_ends_flag;
    bool is_forward;
    TYamlIO yaml_;
};


