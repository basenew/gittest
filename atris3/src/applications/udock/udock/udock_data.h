/*
 * @Description: file content
 * @Author: wenzhi.xu
 * @Date: 2019-08-31 10:56:28
 * @LastEditors: wenzhi.xu
 * @LastEditTime: 2019-08-31 11:13:15
 */
/*
    Udock data by xiangbin.huang
*/

#ifndef __UDOCK_DATA_H__
#define __UDOCK_DATA_H__
#include <string>
#include "udock_common.h"
#include "platform/gs/GsData.h"
#include <time.h>
#include <boost/thread.hpp>



#pragma pack(1)

typedef struct OdomData_{
    float x;
    float y;
    float quat[4];

}OdomData;

typedef struct lidarData_{
    int stamp;
    float angle_min, angle_max;
    float angle_increment, range_min, range_max;
    unsigned int range_size;
    float laser_range[2048];

}lidarData;

#pragma pack()

class UdockData
{
    public:
        ~UdockData();

        void init(void);
        void caculate_yaw(void);

        bool get_lidar_data(LaserRaw &data);
        bool get_lidar_data_assistant(LaserRaw &data);
        bool get_lidar_data_front_slanted(LaserRaw &data);
        bool get_lidar_data_bottom_line(LaserRaw &data);
        bool get_yaw(float &yaw);

        void laserScanCb(lidarData &scan);
        void laserScanCbAssistant(lidarData &scan);
        void laserScanCbFrontSlanted(lidarData &scan);
        void laserScanCbBottomLine(lidarData &scan);
        void odometryCb(OdomData &odom);

        static UdockData* get_instance(){
            static UdockData udock_data;
            return &udock_data;
        }

        OdomData            odom_gauss;
        lidarData           lidar_gauss;

        lidarData           lidar_assistant;
        lidarData           lidar_front_slanted;
        lidarData           lidar_bottom_line;

    private:
        UdockData();
#if USE_GSUDP_DATA
        void udp_receive_thread(void);
#endif
        void data_state_thread(void);

        float               yaw_imu;
        bool                is_lidar_data_ok;
        bool                is_lidar_data_assistant_ok;
        bool                is_lidar_data_front_slanted_ok;
        bool                is_lidar_data_bottom_line_ok;
        bool                is_odom_data_ok;

        boost::mutex        lidar_data_lock;
        boost::mutex        lidar_data_assistant_lock;
        boost::mutex        lidar_data_front_slanted_lock;
        boost::mutex        lidar_data_bottom_line_lock;
        boost::mutex        odom_data_lock;

        timespec receive_lidar_time;
        timespec receive_lidar_assistant_time;
        timespec receive_lidar_front_slanted_time;
        timespec receive_lidar_bottom_line_time;
        timespec receive_odom_time;
};


#endif
/*
    END file Udock data by xiangbin.huang
*/
