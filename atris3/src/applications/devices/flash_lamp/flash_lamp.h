#ifndef __FLASH_LAMP_H__
#define __FLASH_LAMP_H__

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include "can/can.h"
#include "can_service/can_service.h"
#include "list.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetLampStatus.h"
#include "atris_msgs/LampCmd.h"
#include "atris_msgs/GetGpsPos.h"

enum LampStatus
{
    FL_ERR = -1,
    FL_OFF = 0,
    FL_ON  = 1
};

class RedBlueLedSetCmdExtra{
    public:
        RedBlueLedSetCmdExtra(char status, float speed, int time, int source){
            this->status = status;
            this->speed = speed;
            this->time = time;
            this->source = source;
        }
        char get_status(){return status;}
        float get_speed(){return speed;}
        int get_delay_time(){return time;}
        int get_command_source(){return source;}

    private:
        char status;
        float speed;
        int time;
        int source;
};

class FlashLamp
{
    public:
        FlashLamp();
        int init();
        int init(CanService *can);
        bool check_state();
        bool get_state();
        int get_lamp_status(char& rb_status, char& w_status, char& alarm_status);
        int set_red_blue_flash_status(char status, int source);
        int set_red_blue_flash_status(char status, float speed, int source);
        int set_red_blue_flash_status(char status, float speed, int time, int source);

        void construct_light_evt_pack(int source, char status, int type);
        int set_white_lamp_status(char status, int source);
        int set_alarm_lamp_status(char status);
        int close_all_lamp();
        LampStatus get_white_lamp_status();
        LampStatus get_rb_lamp_status();
        LampStatus get_alarm_lamp_status();
        static FlashLamp* get_instance(){
            static FlashLamp singleton;
            return &singleton;
        }
        int flash_lamp_filter(void *data);

    private:
        bool lamp_state;
        std::string rb_serial_num_;
        std::string w_serial_num_;
        CanService *can_service;
        boost::mutex mutex;
        boost::condition_variable cond;
        LampStatus rb_status, w_status, alarm_status;
        CanPkg filter_pkg;
        MsgList<RedBlueLedSetCmdExtra *> cmd_list;
        void red_blue_flash_task(void);
        void control_red_blue_led(char status, float speed, int source);
        ros::NodeHandle nh_;
        ros::Subscriber signal_req_sub_;
        ros::Subscriber lamp_cmd_sub_;
        ros::ServiceServer get_lamp_status_srv_;
        ros::ServiceClient get_gps_position_srv_client_;
        void on_recv_lamp_ctrl1(const atris_msgs::SignalMessage &msg);
        void on_recv_lamp_ctrl(const atris_msgs::LampCmd &msg);
        bool do_get_lamp_status(atris_msgs::GetLampStatus::Request& req, atris_msgs::GetLampStatus::Response& res);
};

enum LampCmd
{
    FL_CMD_STATUS = 0x01,
    FL_CMD_RB   = 0x03,
    FL_CMD_W    = 0x05,
    FL_CMD_AL   = 0x07
};






#endif
