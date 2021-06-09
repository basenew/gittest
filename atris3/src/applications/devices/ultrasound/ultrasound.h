#ifndef __ULTRASOUND_H__
#define __ULTRASOUND_H__

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include "can_service/can_service.h"
#include "list.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "task_manager/task_manager.h"

#define FILTER_BATTERY_CMD      0x3A
#define FILTER_SONIC_DISP       0x32

#define ULTRASOUND_TIMEOUT  1*1000 //1second
#define ULTRASOUND_INVALIDE 0xFFFF //未读取到超声数据

#define ULTRASOUND_NUM  8//超声波数量

class Ultrasound
{
public:
    ~Ultrasound() {}
    static Ultrasound *get_instance(){
        static Ultrasound instance;
        return &instance;
    }
    uint16_t ultra_data[ULTRASOUND_NUM];
    int init(CanService *can);
    void check_state(CanPkg &pkg);
    void send_diag(int diag);

private:
    Ultrasound();
    bool ultrasound_flag_;
    int ultrasound_interval_;
    boost::mutex ultrasound_mutex_;
    boost::condition_variable ultrasound_cond_;
    CanService *can_service;
    void notify_ultraSound();

    int ultrasonic_filter(void *data);
    unsigned int timeout[ULTRASOUND_NUM];
    int diag_status;
    ros::NodeHandle nh_;
    ros::Publisher diag_info_pub_;
    ros::Subscriber signal_req_sub_;
    void on_recv_signal(const atris_msgs::SignalMessage &msg);

};

#endif
