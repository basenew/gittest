#ifndef DBCOM_SIGNAL_H__
#define DBCOM_SIGNAL_H__

#include "config/config.h"
#include "log/log.h"
#include "utils/utils.h"
#include "atris_defines.h"
#include "atris_msgs/RobotInfo.h"
#include <ros/ros.h>
#include <thread>
#include "sdk.h"
#include <mutex>
#include <condition_variable>

#define GET_SIG_FAIL_MAXIMUM 20

class DbComManager {
public:
    ~DbComManager();
    static DbComManager* get_instance()
    {
        static DbComManager singleton;
        return &singleton;
    }


private:
    DbComManager();
    int Init();
    int sendDbComSignalStrength(const std::string & signal_strength_str);
    int dbcom_login();
    int dbcom_read_signal();
    void dbcom_try_login_wait();
    //void sendDbComConnectStatus(bool connect_status);
    //void sendDbComLoginStatus(bool login_status);

private:
    ros::NodeHandle nh_;
    ros::Publisher diag_info_pub_;
    Config *cfg;
    Utils *utils;

    std::string device_ip, username, passwd;
    int login_timeout_;
    bool is_connected_;
    bool is_login_;
    bool dbcom_try_login_thread_exit_;
    std::mutex dbcom_login_mutex_;
    std::condition_variable dbcom_login_cv_;
    int get_signal_strength_failed_cnt_;
    bool need_reconnect_;

    boost::thread * connect_thread;
    boost::thread * get_sig_thread;
    struct sdk_connection *conn;
};

#endif
