#ifndef __CHASSIS_MANAGER_H__
#define __CHASSIS_MANAGER_H__

#include "time.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include "power_manager.h"
#include "chassis/chassis_control.h"
#include "imemory/atris_imemory_api.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/VelCmd.h"
#include "atris_msgs/TimeCalibrated.h"
#include "atris_msgs/GetChassisInfo.h"
#include "tiny_ros/ros.h"
#include "tiny_ros/geometry_msgs/Twist.h"
#include "chassis/ChassisDef.h"

enum ChassisType_
{
    CHASSIS_TYPE_TRACKED,
    CHASSIS_TYPE_WHEEL
};

class ChassisManager
{
    private:
        ChassisManager();
        ~ChassisManager();
        PowerManager    *power;
        ControlModule   *control_module;
        bool chassis_manager_proc_exit;
        shm::TimeCalibrated tc_;
        ros::NodeHandle nh_;
        ros::Publisher set_vel_to_udock_pub_;
        ros::Subscriber set_vel_to_chassis_sub_;
        ros::Subscriber time_calibrated_sub_;
        ros::Subscriber signal_req_sub_;
        ros::ServiceServer get_chassis_info_srv_;
        tinyros::Publisher robot_cmd_vel_pub_;
        bool doGetChassisInfo(atris_msgs::GetChassisInfo::Request& req, atris_msgs::GetChassisInfo::Response& res);
        void set_vel_cmd(const atris_msgs::VelCmd &msg);
        void on_recv_time_calibrated(const atris_msgs::TimeCalibrated &msg);
        void on_recv_chassis_ctrl(const atris_msgs::SignalMessage &msg);
        void speed_limit(float &x, float &y, float &z);
    public:
        void init();
        static ChassisManager* get_instance(){
            static ChassisManager singleton;
            return &singleton;
        }
};

#endif
