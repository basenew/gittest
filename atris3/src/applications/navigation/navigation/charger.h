#ifndef __CHARGE_H__
#define __CHARGE_H__

#include "gs_nav.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/DockSDKCmd.h"
#include "atris_msgs/PowerChargeCmd.h"
class Charger
{
    private:
        Charger();
        NavStateMonitor nav_monitor;
        GsNav *gsnav;
        GsApi *gsapi;
        int state_code, before_code, state_count;
        ros::NodeHandle nh_;
        ros::Publisher signal_resp_pub_;
        ros::Publisher dock_sdk_cmd_pub_;

        void on_recv_reached_charge_point();
        int nav_state_monitor(char *data);

        void report_pos_state(int state_code);
        bool del_monitor_task();


    public:
        void init();
        void start_charge(std::string map_name);
        void start_charge();
        void stop_charge();
        void notify_reached_point();

        static Charger* get_instance(){
            static Charger singleton;
            return &singleton;
        }
};









#endif
