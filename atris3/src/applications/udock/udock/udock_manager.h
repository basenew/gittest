/*
    Udock manager by xiangbin.huang
*/


#ifndef __UDOCK_MANNAGER_H__
#define __UDOCK_MANNAGER_H__

#include "udock_location.h"
#include "udock_data.h"
#include "udock_common.h"
#include "can/can.h"
#include "udock_planning.h"
#include "udockwheel_obstacle_detect.h"
#include "udock_obstacle_avoid_under_given_control.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/ChargeInfo.h"
#include "atris_msgs/ChargeCmd.h"
#include "atris_msgs/VelCmd.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/GetNavPose.h"
#include "atris_msgs/GetGsLaserRawData.h"
#include "atris_msgs/DockSDKCmd.h"
#include "atris_msgs/TimeCalibrated.h"
#include "platform/udock/UdockData.h"
#include "imemory/atris_imemory_api.h"

/*-----------------------------------------
 -----------------------------------------*/
class UdockTankManager
{
    public:
        UdockTank       *udock;
        UdockData       *gsudp;
        ObstacleAvoidUnderGivenControl  *obstacle_avoid_under_given_control;
        UdockWheelObstacleDetect        *wheel_obstacle_detect;
#ifdef _CHASSIS_MARSHELL_
        moveControl     *move_control;
#endif
        ~UdockTankManager();

        void init(void);
        int  dock_sdk(dockSDKCMD cmd);      //sdk send cmd
        dockSDKState get_charge_state();    //sdk require info

#ifdef _CHASSIS_MARSHELL_
        bool move_to_dock_point(int step);
        bool localizition(robotPose *local, int point_num, int showd_num);
#endif

        bool get_move_state(void);

        bool is_moving(void);
        void set_moving_state(bool state);

        //single instance
        static UdockTankManager* get_instance(){
            static UdockTankManager udock_tank_manager;
            return &udock_tank_manager;
        }
    private:
        void report_charge_state_to_pc(UdockMsg state_code);
        void set_charge_state(dockSDKState state);
        void set_move_state(bool state);
        void on_charge_info(const atris_msgs::ChargeInfo &msg);
        void on_determine_safe_control(const atris_msgs::VelCmd &msg);
        void on_pc_cmd(const atris_msgs::SignalMessage &msg);
        void on_dock_sdk_cmd(const atris_msgs::DockSDKCmd &msg);

        void udock_thread(void);
        void udock_move_test_thread(void);
        void udock_test_thread(void);
        void play_tts_cycle_thread(void);

        float get_odom_yaw(void);
        int move_dist(const float dist, const int speed);
        int move_dist_ob(const float dist, const int speed); //cm cm/s
        int move_angle(const float dist, const int speed);

        bool data_transform(void);
        bool track_location(float d, int point_num, int showd_num);
        bool move_to_dock(void);
        bool blue_tooth_pare(void);
        bool send_test_cmd(void);
        bool move_to_dock_point(void);

        bool move_to_dock_last_point(float d);
        bool move_to_dock_last_step(void);

        bool move_away_from_dock(const float dist, const int speed, const int time_s);
#ifdef _CHASSIS_MARSHELL_
        bool move_away_from_dock_wheel(const float dist, const int speed);
#endif
        bool respons_leave_msg(void);
        bool notify_leave_msg(void);
        void on_recv_time_calibrated(const atris_msgs::TimeCalibrated &msg);

    private:
        UdockTankManager();

        TrackMovePara       track_move_para;
        dockSDKState        charge_state_;
        bool                move_state_;
        bool                battery_charging_state_;

        LaserRaw            lidar_data[10];
        LaserRaw            lidar_data_filter;
        LaserRaw            lidar_data_raw;

        boost::mutex        sdk_state_lock;
        boost::mutex        charge_state_lock;
        boost::mutex        move_state_lock;
        ros::NodeHandle nh_;
        ros::Publisher signal_resp_pub_;
        ros::Publisher charge_cmd_pub_;
        ros::Publisher set_vel_cmd_pub_;
        ros::Publisher aisound_tts_pub_;
        shm::TimeCalibrated tc_;
        ros::Subscriber dock_sdk_cmd_sub_;
        ros::Subscriber set_vel_to_udock_sub_;
        ros::Subscriber charge_info_sub_;
        ros::Subscriber signal_req_sub_;
        ros::Subscriber time_calibrated_sub_;
        ros::ServiceClient get_nav_pose_srv_client_;
        ros::ServiceClient get_gs_laser_raw_data_srv_client_;

        bool                is_moving_state;
    public:
        boost::mutex        pause_state_lock;
};

#endif
