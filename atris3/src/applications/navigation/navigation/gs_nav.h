#ifndef __GS_NAV__
#define __GS_NAV__

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include "libwebsockets.h"
#include "gs_api.h"
#include "config/config.h"
#include "kv_db.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/GetGpsPos.h"

typedef boost::function<int (char *)> nav_state_cb;
typedef struct NavStateMonitor_{
    int id;
    nav_state_cb cb;
}NavStateMonitor;

enum MapLoadingState{
    MLS_IDLE = 0,
    MLS_LOADING
};

enum ThreadState
{
    THREAD_IDLE,
    THREAD_RUNNING
};

class GsNav
{
    private:
        GsNav();
        enum MapLoadingState mls_state;
        volatile bool lws_srv_dev_state, lws_srv_health_state;
        volatile int lws_cli_count;
        bool gsnav_state;
        volatile ThreadState thread_dev_state, thread_health_state;
        int state_monitor_id;
        int lidar_master_state, lidar_slave_state,
            rgbd_state, gpsstatus,gyro_state, gyro_state_data, odom_delta_spd;
        char ultrasonic_state;
        std::vector<GsMap> map_list;
        int ws_cli_health_connect(const struct lws_protocols * protocols, struct lws *client, const char *path);
        int ws_cli_dev_connect(const struct lws_protocols * protocols, struct lws *client, const char *path);
        void update_gps_thread();
        void gps_data_publish();
        std::string gs_host;
        std::string dev_state_path;
        std::string state_path, health_state_path;
        Config *cfg;
        Utils *utils;
        GsApi *gsapi;
        boost::mutex mutex;
        boost::condition_variable cond;
        std::list<NavStateMonitor> state_monitor_list;

        boost::thread* gps_data_thread_;
        double lati_;
        double long_;
        std::string using_map;
        KVDB   nav_db;
        double alti_;
        ros::NodeHandle nh_;
        ros::Publisher diag_info_pub_;

    public:
        bool check_state();
        bool get_state();
        struct lws *ws_cli_dev_state, *ws_cli_state, *ws_cli_health_state;
        boost::thread *health_thread, *dev_status_thread;
        int init();
        void on_recv_dev_status(const char *data);
        void on_recv_status(char *data);
        void add_state_monitor(NavStateMonitor &state_monitor);
        int del_state_monitor(int id);
        int create_state_monitor(int type);
        const std::string& get_using_map(){return using_map;};
        static GsNav* get_instance(){
            static GsNav singleton;
            return &singleton;
        }

        bool doGetGpsPosition(atris_msgs::GetGpsPos::Request& req, atris_msgs::GetGpsPos::Response& res);
        ros::ServiceServer get_gps_position_srv_;


};



#endif

