#pragma once 

#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>  
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <iomanip>

#pragma pack(1)
typedef struct{
    uint16_t info_type; //信息类型，暂时固定FFEE
    uint16_t vio_id; //VIO的ID
    uint32_t period_signal; //周期信号，每200ms加1
    uint16_t vio_speed; //VIO输出的速度
    uint16_t vio_period_odom; //这个周期内行走的距离 cm
    uint32_t vio_calc_odom; //累计行驶里程cm
    uint8_t  is_valid; //标记信息是否有效 有效1 无效0 无效原因见下字段
    uint32_t invalid_status; //信息无效状态，
    /*
        此无效状态值描述无效的原因，定义如下：
        0x00000001 地图校准前后的视觉惯导速度或距离相差超过设定阈值；
        0x00000002 惯导加速度超过设定阈值；
        0x00000004 程序迭代计算失败；
        0x00000008 其他后续可补充的失效信息。
    */
    uint16_t CRC; //CRC16
} vio_info;
#pragma pack()

typedef struct{
    uint16_t info_type; //信息类型，暂时固定EEFF
    uint16_t ATP_id; //ATP设备源ID 激活端ATP的ID 1：代表1端，2代表2端。
    uint16_t cur_station_id; //当前站号ID 在到达下一站并停稳后更新。出站运行在区间过程中此站号不变。
    uint16_t next_station_id; //下一站站号ID 在到达下一站并停稳后更新。出站运行在区间过程中此站号不变。但会基于运行计划调整。
    uint32_t beacon_id;//信标ID 视觉惯导检查信标ID变化，获取信标间里程数据。在未收到新的信标时发送上一个信标ID。
    uint32_t beacon_odom;//信标间里程
    uint16_t atp_speed;//ATP列车实时速度
    uint16_t atp_period_odom;//ATP本周期里程信息
    uint32_t atp_calc_odom;//ATP累计行驶里程
    // uint16_t CRC; //CRC16
}ATP_info;

class vio_udp{
public:
    vio_udp(const char * src_ip,const char * dest_ip){
        vio_id_ = 1; 
        period_signal_ = 0;
        calc_odom_ = 0.0;
        debug_mode = false;
        atp_info_init();
        vio_info_init();

        client_len = sizeof(client_addr);
        udp_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sockfd_ < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return ;
        }
        addr.sin_family = AF_INET;
        addr.sin_port = htons(50501);
        // addr.sin_addr.s_addr = INADDR_ANY;  
        addr.sin_addr.s_addr = inet_addr(src_ip); 
        if (bind(udp_sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cerr << "Error binding socket" << std::endl;
            return ;
        }  

        addr.sin_port = htons(50523);
        addr.sin_addr.s_addr = inet_addr(dest_ip);
        // addr.sin_addr.s_addr = inet_addr("10.21.0.217");
        udp_recv_thread = std::thread(&vio_udp::UDP_recv, this);
    }

    ~vio_udp(){
        close(udp_sockfd_);
    }

    void vio_udp_send(uint16_t speed,double odom,double *calc_odom){
        pack_info(speed,odom,calc_odom);
        if(debug_mode){
            unsigned char* bytePtr = reinterpret_cast<unsigned char*>(&vio_info_);
            std::cout << "send: ";
            for (int i = 0; i < sizeof(vio_info_); i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytePtr[i]) << " ";
            }
            std::cout << std::endl;
        }
        sendto(udp_sockfd_,&vio_info_,sizeof(vio_info_),0, (struct sockaddr *)&addr,sizeof(addr));
    }

    void vio_udp_send(uint16_t speed,double odom,double calc_odom,bool is_valid = true){
        if(is_valid)//有效，原因给0
            pack_info(speed,odom,calc_odom,0);
        else//无效，原因给4
            pack_info(speed,odom,calc_odom,4);
        if(debug_mode){
            unsigned char* bytePtr = reinterpret_cast<unsigned char*>(&vio_info_);
            std::cout << "send: ";
            for (int i = 0; i < sizeof(vio_info_); i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytePtr[i]) << " ";
            }
            std::cout << std::endl;
        }
        sendto(udp_sockfd_,&vio_info_,sizeof(vio_info_),0, (struct sockaddr *)&addr,sizeof(addr));
    }

private:
    void UDP_recv(){
        while(1){
            udp_recv();
            usleep(2000);
        }
    }

    void atp_info_init(){
        atp_info_.ATP_id = 0;
        atp_info_.atp_speed = 0;
        atp_info_.atp_period_odom = 0;
        atp_info_.atp_calc_odom = 0;
        atp_info_.beacon_odom = 0;
        atp_info_.beacon_id = 0;
    }

    void vio_info_init(){
        vio_info_.info_type = htons(0xFFEE);
        vio_info_.vio_id = htons(vio_id_);
        vio_info_.period_signal = htons(period_signal_);
        vio_info_.vio_speed = 0;
        vio_info_.vio_period_odom = 0;
        vio_info_.vio_calc_odom = 0;
        vio_info_.is_valid = 1;
        vio_info_.invalid_status = 0;
        vio_info_.CRC = 0;
    }

    uint16_t CRC(char* DATA, int len){
        
        return 0;
    }

    void unpack_atp_info(ATP_info info_){

    }
    // 解包ATP信息
    void udp_recv(){
        memset(rebuf,0,128);
        int recvfrom_len = recvfrom(udp_sockfd_, rebuf, sizeof(rebuf), 0, (struct sockaddr *)&client_addr, &client_len);
        if(debug_mode){
            if(recvfrom_len > 0){
                std::cout << "rec_msg:";
                for (int i = 0; i < recvfrom_len; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(rebuf[i])) << " ";
                }
                std::cout << std::endl;
            }   
        }
        
        if (recvfrom_len >= sizeof(ATP_info)) {
            std::cout << "rec_msg right" << std::endl;
            atp_info_flag = true;
            ATP_info* atp_info_ptr = (ATP_info*)rebuf;
            read_atp_info.lock();
            atp_info_.info_type = ntohs(atp_info_ptr->info_type);
            atp_info_.ATP_id = ntohs(atp_info_ptr->ATP_id);
            atp_info_.cur_station_id = ntohs(atp_info_ptr->cur_station_id);
            atp_info_.next_station_id = ntohs(atp_info_ptr->next_station_id);
            atp_info_.beacon_id = ntohl(atp_info_ptr->beacon_id);
            atp_info_.beacon_odom = ntohl(atp_info_ptr->beacon_odom);
            atp_info_.atp_speed = ntohs(atp_info_ptr->atp_speed);
            atp_info_.atp_period_odom = ntohs(atp_info_ptr->atp_period_odom);
            atp_info_.atp_calc_odom = ntohl(atp_info_ptr->atp_calc_odom);
            // atp_info_.CRC = ntohs(atp_info_ptr->CRC);  
            read_atp_info.unlock();  
        } else {
            // 错误处理，例如记录错误、断开连接等
        }
    }

    void pack_info(uint16_t speed,double odom,double *calc_odom){
        vio_info_.period_signal = htonl(++period_signal_);
        vio_info_.vio_speed = htons(speed);
        vio_info_.vio_period_odom = htons((uint16_t)odom); 
        calc_odom_ += odom;
        *calc_odom = calc_odom_;
        vio_info_.vio_calc_odom = htonl((uint32_t)calc_odom_); // 保存累计里程
        //printf("period_signal = %d,vio_speed = %d,vio_period_odom = %lf,vio_calc_odom = %lf\n",period_signal_,speed,odom,calc_odom_);
        // vio_info_.CRC = CRC();
    }

    void pack_info(uint16_t speed,double odom,double calc_odom,uint32_t valid){
        vio_info_.period_signal = htonl(++period_signal_);
        vio_info_.vio_speed = htons(speed);
        vio_info_.vio_period_odom = htons((uint16_t)odom); 
        vio_info_.vio_calc_odom = htonl((uint32_t)calc_odom); // 保存累计里程
        if(valid == 0)//有效
            vio_info_.is_valid = 1;//给1
        else
            vio_info_.is_valid = 0;//给0
        vio_info_.invalid_status = htonl(valid);
        //printf("period_signal = %d,vio_speed = %d,vio_period_odom = %lf,vio_calc_odom = %lf\n",period_signal_,speed,odom,calc_odom_);
        // vio_info_.CRC = CRC();
    }

private:
    int udp_sockfd_;
    sockaddr_in addr;
    char rebuf[128];
    sockaddr_in client_addr;
    socklen_t client_len;
    std::thread udp_recv_thread;
    double calc_odom_;

public:
    ATP_info atp_info_;
    std::mutex read_atp_info;
    bool atp_info_flag;
    uint16_t vio_id_;
    vio_info vio_info_;
    uint32_t period_signal_;
    bool debug_mode;
};

