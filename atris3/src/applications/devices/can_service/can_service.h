#ifndef __CAN_SERVICE_H__
#define __CAN_SERVICE_H__
#include <list>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include "can/can.h"
#include "ros/ros.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/CanData.h"
#include "atris_defines.h"
#include "log/log.h"

typedef boost::function<int (void *)> filter_cb;

typedef struct
{
    int id;
    filter_cb filter;
}DevFilter;

class AckCanData
{
public:
    unsigned int seq;
    boost::mutex *mutex;
    boost::condition_variable_any *cv;
    CanPkg *pkg;
    unsigned char id;
};

#define ACK_MAX_TIMEOUT 2

class CanService
{
public:
    ~CanService() {}
    static CanService *get_instance(){
        static CanService instance;
        return &instance;
    }
    int send(void *data, int len);
    int send(CanPkg &pkg, int len);
    int send_for_ack(CanPkg *pkg, int len);
    int add_filter(DevFilter &filter);
    int del_filter(int id);

private:
    CanService();
    int filter_id;
    unsigned int seq;
    CanDevice *can_dev;
    std::list<DevFilter> filter_list;
    std::list<AckCanData> concurrent_list;
    boost::mutex concur_mutex;
    boost::mutex filter_mutex;
    boost::condition_variable_any concur_cond;
    ros::NodeHandle nh_;
    ros::Subscriber signal_req_sub_;
    ros::Subscriber can_data_sub_;
    void on_can_data_cb(const atris_msgs::CanData& msg);
    void proc_channel_data();
    void do_filter(void *data);
    int ack_filter(CanPkg &pkg);
    void on_recv_can_service(const atris_msgs::SignalMessage &msg);
};


#endif