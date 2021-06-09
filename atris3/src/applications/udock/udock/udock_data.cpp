/*
    Udock data by xiangbin.huang
*/
  
#include "log/log.h"
#include "udock_data.h"
#include "config/config.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#if USE_GSUDP_DATA
boost::asio::io_service service;
boost::asio::ip::udp::endpoint ep( boost::asio::ip::address::from_string("10.20.18.4"), 6968);
boost::asio::ip::udp::socket sock(service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 6968) );
#endif
UdockData::UdockData()
{
    yaw_imu = 0.0f;
    init();
}

UdockData::~UdockData()
{

}

void UdockData::init(void)
{
#if USE_GSUDP_DATA
    new boost::thread(boost::bind(&UdockData::udp_receive_thread, this));
#endif

    new boost::thread(boost::bind(&UdockData::data_state_thread, this));
}

void UdockData::caculate_yaw(void)
{
    static float yaw_old = 0.0f;

    float quat_w = odom_gauss.quat[0];
    float quat_x = odom_gauss.quat[1];
    float quat_y = odom_gauss.quat[2];
    float quat_z = odom_gauss.quat[3];
    float yaw = atan2( 2*(quat_x * quat_y + quat_w * quat_z), 1-2*(quat_y * quat_y + quat_z * quat_z) );
    float dyaw = yaw - yaw_old;
    yaw_old = yaw;
    if(dyaw >M_PI){
        dyaw -= M2_PI;
    }
    else if(dyaw < -M_PI){
        dyaw += M2_PI;
    }
    yaw_imu += dyaw;
}

void UdockData::laserScanCb(lidarData &scan)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_lock);
    clock_gettime(CLOCK_REALTIME, &receive_lidar_time);
    memcpy(&lidar_gauss, &scan, sizeof(lidarData));
}

void UdockData::laserScanCbAssistant(lidarData &scan)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_assistant_lock);
    clock_gettime(CLOCK_REALTIME, &receive_lidar_assistant_time);
    memcpy(&lidar_assistant, &scan, sizeof(lidarData));
}


void UdockData::laserScanCbFrontSlanted(lidarData &scan)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_front_slanted_lock);
    clock_gettime(CLOCK_REALTIME, &receive_lidar_front_slanted_time);
    memcpy(&lidar_front_slanted, &scan, sizeof(lidarData));
}

void UdockData::laserScanCbBottomLine(lidarData &scan)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_bottom_line_lock);
    clock_gettime(CLOCK_REALTIME, &receive_lidar_bottom_line_time);
    memcpy(&lidar_bottom_line, &scan, sizeof(lidarData));    
}

void UdockData::odometryCb(OdomData &odom)
{
    boost::lock_guard<boost::mutex> lock(odom_data_lock);
    clock_gettime(CLOCK_REALTIME, &receive_odom_time);
    memcpy(&odom_gauss, &odom, sizeof(OdomData));
    caculate_yaw();
}


bool UdockData::get_yaw(float &yaw)
{
    boost::lock_guard<boost::mutex> lock(odom_data_lock);
    if(is_odom_data_ok == false){
        return false;
    }
    yaw = yaw_imu;
    return true;
}

bool UdockData::get_lidar_data(LaserRaw &data)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_lock);
    if(is_lidar_data_ok == false){
        log_info("[udock data] lidar data(back of wheeled or velodyne horizontal line of tracked) not ok, get lidar data failed...");
        return false;
    }
    data.angle_min = lidar_gauss.angle_min;
    data.angle_max = lidar_gauss.angle_max;
    data.angle_increment = lidar_gauss.angle_increment;
    data.range_size = lidar_gauss.range_size;
    for(unsigned int i=0; i<data.range_size; i++){
        data.laser_range[i] = lidar_gauss.laser_range[i];
    }
    return true;
}

bool UdockData::get_lidar_data_assistant(LaserRaw &data)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_assistant_lock);
    if(is_lidar_data_assistant_ok == false){
        log_info("[udock data] lidar data (velodyne horizontal line)not ok, get lidar data failed...");
        return false;
    }
    data.angle_min = lidar_assistant.angle_min;
    data.angle_max = lidar_assistant.angle_max;
    data.angle_increment = lidar_assistant.angle_increment;
    data.range_size = lidar_assistant.range_size;
    for(unsigned int i=0; i<data.range_size; i++){
        data.laser_range[i] = lidar_assistant.laser_range[i];
    }
    return true;
}

bool UdockData::get_lidar_data_front_slanted(LaserRaw &data)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_front_slanted_lock);
    if(is_lidar_data_front_slanted_ok == false){
        log_info("[udock data] lidar data (front slanted)not ok, get lidar data failed...");
        return false;
    }
    data.angle_min = lidar_front_slanted.angle_min;
    data.angle_max = lidar_front_slanted.angle_max;
    data.angle_increment = lidar_front_slanted.angle_increment;
    data.range_size = lidar_front_slanted.range_size;
    for(unsigned int i=0; i<data.range_size; i++){
        data.laser_range[i] = lidar_front_slanted.laser_range[i];
    }
    return true;
}

bool UdockData::get_lidar_data_bottom_line(LaserRaw &data)
{
    boost::lock_guard<boost::mutex> lock(lidar_data_bottom_line_lock);
    if(is_lidar_data_bottom_line_ok == false){
        log_info("[udock data] lidar data (velodyne bottom line)not ok, get lidar data failed...");
        return false;
    }
    data.angle_min = lidar_bottom_line.angle_min;
    data.angle_max = lidar_bottom_line.angle_max;
    data.angle_increment = lidar_bottom_line.angle_increment;
    data.range_size = lidar_bottom_line.range_size;
    for(unsigned int i=0; i<data.range_size; i++){
        data.laser_range[i] = lidar_bottom_line.laser_range[i];
    }
    return true;
}



void UdockData::data_state_thread(void)
{
    timespec time_now;
    while(1)
    {
        //判断当前时间是否被同步导致小于收到数据时间
        clock_gettime(CLOCK_REALTIME, &time_now);
        if(time_now.tv_sec < receive_odom_time.tv_sec){
            log_info("[udock data] time now less than odom");
            boost::lock_guard<boost::mutex> lock(odom_data_lock);
            clock_gettime(CLOCK_REALTIME, &receive_odom_time);
        }
        if(time_now.tv_sec < receive_lidar_time.tv_sec){
            log_info("[udock data] time now less than lidar");
            boost::lock_guard<boost::mutex> lock(lidar_data_lock);
            clock_gettime(CLOCK_REALTIME, &receive_lidar_time);
        }
        if(time_now.tv_sec < receive_lidar_assistant_time.tv_sec){
            log_info("[udock data] time now less than assistant");
            boost::lock_guard<boost::mutex> lock(lidar_data_assistant_lock);
            clock_gettime(CLOCK_REALTIME, &receive_lidar_assistant_time);
        }

        if(time_now.tv_sec < receive_lidar_front_slanted_time.tv_sec){
            log_info("[udock data] time now less than slanted");
            boost::lock_guard<boost::mutex> lock(lidar_data_front_slanted_lock);
            clock_gettime(CLOCK_REALTIME, &receive_lidar_front_slanted_time);
        }

        if(time_now.tv_sec < receive_lidar_bottom_line_time.tv_sec){
            log_info("[udock data] time now less than bottom line");
            boost::lock_guard<boost::mutex> lock(lidar_data_bottom_line_lock);
            clock_gettime(CLOCK_REALTIME, &receive_lidar_bottom_line_time);
        }
        //odom
        if(time_now.tv_sec - receive_odom_time.tv_sec >2){
            is_odom_data_ok = false;
        }
        else{
            is_odom_data_ok = true;
        }
        //lidar
        if(time_now.tv_sec - receive_lidar_time.tv_sec >2){
            is_lidar_data_ok = false;
        }
        else{
            is_lidar_data_ok = true;
        }
        //assistant
        if(time_now.tv_sec - receive_lidar_assistant_time.tv_sec >2){
            is_lidar_data_assistant_ok = false;
        }
        else{
            is_lidar_data_assistant_ok = true;
        }
        //slanted
        if(time_now.tv_sec - receive_lidar_front_slanted_time.tv_sec >2){
            is_lidar_data_front_slanted_ok = false;
        }
        else{
            is_lidar_data_front_slanted_ok = true;
        }
        //bottom line
        if(time_now.tv_sec - receive_lidar_bottom_line_time.tv_sec >2){
            is_lidar_data_bottom_line_ok = false;
        }
        else{
            is_lidar_data_bottom_line_ok = true;
        }
        sleep(1);
    }
}
#if USE_GSUDP_DATA
void UdockData::udp_receive_thread(void)
{
    static char buff[1800];
    boost::asio::ip::udp::endpoint sender_ep;
    int bytes =0;

    while(1)
    {
        bytes = sock.receive_from(boost::asio::buffer(buff), sender_ep);
        switch(bytes)
        {
            case sizeof(OdomData):
            {
                boost::lock_guard<boost::mutex> lock(odom_data_lock);
                clock_gettime(CLOCK_REALTIME, &receive_odom_time);
                memcpy(&odom_gauss, buff, sizeof(OdomData));
                caculate_yaw();
            }
            break;
            case sizeof(lidarData):
            {
                boost::lock_guard<boost::mutex> lock(lidar_data_lock);
                clock_gettime(CLOCK_REALTIME, &receive_lidar_time);
                memcpy(&lidar_gauss, buff, sizeof(lidarData));
            }
            break;
            default:
            break;
        }
    }
}
#endif

/*
    END file Udock data by xiangbin.huang
*/
