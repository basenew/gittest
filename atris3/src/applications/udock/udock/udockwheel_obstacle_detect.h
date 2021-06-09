/*
    Udock algrithm by xiangbin.huang
*/

#ifndef __UDOCKWHEEL_OBSTACLE_DETECT_H__
#define __UDOCKWHEEL_OBSTACLE_DETECT_H__


#include "udock_obstacle_detect.h"
#include "udock_common.h"
#include "udock_data.h"
#include <chrono>
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"

class UdockWheelObstacleDetect
{
    public:
        ~UdockWheelObstacleDetect(){
        }
        static UdockWheelObstacleDetect* get_instance(){
            static UdockWheelObstacleDetect udock_obstacle;
            return &udock_obstacle;
        }
        UdockData       *gs_udp;
        //    B----------------C
        //    |                |
        //    |                |
        //    |                |
        //    A----------------D
        void set_wheel_front_detect_area(float angle, float speed, quadrilateralPoint &area);
        void set_wheel_back_detect_area(float angle, float speed, quadrilateralPoint &area);
        void set_wheel_front_slanted_detect_area(float angle, float speed, quadrilateralPoint &area);
        
        // 获取雷达数据
        bool get_back_lidar_filter_data(void);
        bool get_front_lidar_filter_data(void);
        bool get_front_slanted_lidar_filter_data(void);

        //获取超声数据
        bool get_front_ultra_sound_data(void);
        bool get_back_ultra_sound_data(void);

        //回充避障函数
        bool need_back_move_stop(float angle);
        bool need_front_move_stop(float angle);
        //遥控避障函数
        float get_safe_front_speed(float linear_speed, float angle);
        float get_safe_back_speed(float linear_speed, float angle);

        float get_safe_front_speed_stategy(float linear_speed, float angle);
        float speed_control(float linear_speed);
        float speed_control(float linear_speed, bool &get_state);

        //void play_tts(int key, bool force = false);

    private:
        UdockWheelObstacleDetect() {
            aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
            gs_udp = UdockData::get_instance();
            forbid_move_front = false;
            msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
        }
        UdockObstacleDetect ob_det;
        vector<dataPoint>   ob_data_back;           //后置雷达数据
        vector<dataPoint>   ob_data_assistant;      //多线雷达数据
        vector<dataPoint>   ob_data_front_slanted;  //前置 倾斜数据
        vector<dataPoint>   ob_data_ultrosonic;     //超声数据

        //回充避障参数
        quadrilateralPoint move_back_linear_point_;
        quadrilateralPoint move_front_linear_point_;
        quadrilateralPoint move_front_slanted_linear_point_;
        //遥控避障参数
        quadrilateralPoint rc_move_front_point_;
        quadrilateralPoint rc_move_front_slanted_point_;
        quadrilateralPoint rc_move_back_point_;

        LaserRaw    scan;               //后置单线
        LaserRaw    scan_bottom_line;   //多线雷达最低一线
        LaserRaw    scan_assistant;     //多线雷达取单线
        LaserRaw    scan_front_slanted; //前置倾斜单线

        atris_msgs::AisoundTTS msg;
        
        bool        forbid_move_front;
        bool        flag_cliff;

        unsigned int    count_for_print_log = 0;
        bool            print_log_flag = false;

        // 延迟减速作用
        bool                                        switch_deceleration_memory_effect_ = true;
        std::chrono::steady_clock::time_point       time_point_deceleration_start_;
        float                                       speed_thresh_; 
        std::chrono::milliseconds                   duration_deceleration_;
        ros::NodeHandle nh_;
        ros::Publisher aisound_tts_pub_;
        void deceleration_record(float speed);
        void deceleration_affect(float& speed);


#ifdef _POLICE_
        float h_bottom_line = 1.126 - 0.2;    //height of bottom line detection range of police version
        float h_slanted_lidar = 0.779;    //height of front slanted lidardetection range of police version
        float h_cliff_detect = 1.341;    //height of cliff detect
        float multilidar_to_collisionbar = 0.417f + 0.1;
        float slanted_lidar_to_collisionbar = 0.463f + 0.1;
#else
        float h_bottom_line = 0.988 - 0.2;    //height of bottom line detection range of standard version
        float h_slanted_lidar = 0.645;    //height of front slanted lidar detection range of standard version
        float h_cliff_detect = 1.158;    //height of cliff detect
        float multilidar_to_collisionbar = 0.35f + 0.1;
        float slanted_lidar_to_collisionbar = 0.438f + 0.1;
#endif
};


#endif
/*
    END file Udock obstacle detect by xiangbin.huang
*/
