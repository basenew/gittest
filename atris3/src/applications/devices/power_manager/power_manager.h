#ifndef __POWER__MANAGER_H__
#define __POWER__MANAGER_H__
#include <json/json.h>
#include "can/can.h"
#include "config/config.h"
#include "chassis/chassis.h"
#include "tts_strings/tts_strings.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/ChargeInfo.h"
#include "atris_msgs/ChargeCmd.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "platform/pm/PMData.h"
#include "atris_msgs/NavReset.h"
#include "atris_msgs/GetGpsPos.h"

class PowerManager
{
    private:
        PowerManager();
#ifdef BATTERY_TEST
        std::string bat;
        int fd;
#endif
        boost::mutex poff_mutex;
        boost::condition_variable_any poff_cv;

        unsigned int soc;
        CanDevice *can;
        bool can_send;
        int report_count;
        bool is_power_off_;
        void proc_can_data();
        void do_can(CanPkg *pkg);
        void on_receive_charge_cmd(const atris_msgs::ChargeCmd &msg);
        int do_battery_data(CanPkg *pkg);
        int do_power_control(CanPkg *pkg);
        int do_monitor(CanPkg *pkg);
        int do_power(CanPkg *pkg);
        int do_cycle_task();
        void proc_poweroff(int status);
        void proc_poweroff_thread(int status);
        int send_poweroff_finish(int status);
        int power_off_face_x86();
        int power_off_nav_x86();
        void on_power_off(const std_msgs::Int32 &msg);
        double get_uptime();
        void reportPowerOffEventToPc(int val);
        int remote_status;
        int brake_state;
        ros::Time brake_time;
        double pwm_chassis_lock_time;
        int light_state;
        int flash_light_state;
        int ch_1;
        int ch_2;
        int ch_3;
        int ch_4;
        int ch_5;
        int ch_6;
        int ch_7;
        int ch_8;
        int ch_flag;
        ros::Time send_twish_time;
        std::string rbt_sn_;
        ros::NodeHandle nh_;
        ros::Publisher diag_info_pub_;
        ros::Publisher aisound_tts_pub_;
        ros::Publisher charge_info_pub_;
        ros::Publisher nav_reset_pub_;
        ros::Publisher lamp_cmd_pub_;
        ros::Subscriber charge_cmd_sub_;
        ros::Subscriber power_off_sub_;
        ros::ServiceClient get_gps_position_srv_client_;
        bool send_twish(int pwm_L,int pwm_R,int center,int max,bool send = true);
        volatile int brake_;

        int tts_key_;
        int tts_ext_key_;

        int brake_data1;
        int brake_data2;
        int bumper_data;
        int bumper_data1;
        int bumper_data2;
        int bumper_data3;
        int bumper_data4;
        float battery_voltage;
        float battery_current;
        std::string power_off_serial_num_;
    public:
        void sendTtstext(int key,int ext_key = TTSStrings::TTS_KEY_NULL);
        void sendTtsTextForce(int key);
        void notifyPowerOffEvent(std::string serial_num, Json::Value & reason);
        int init();
        RemoteControlMode remote_control_mode;
        int brake(BrakeState status);
        float getBatteryVoltage(void){return battery_voltage;}
        float getBatteryCurrent(void){return battery_current;}
        static PowerManager* get_instance(){
            static PowerManager singleton;
            return &singleton;
        }
};

#endif
