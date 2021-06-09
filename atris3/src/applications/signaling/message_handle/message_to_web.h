#ifndef ATRIS_SIGNALING_MESSAGE_TO_WEB_H_
#define ATRIS_SIGNALING_MESSAGE_TO_WEB_H_

#include <boost/thread.hpp>
#include <json/json.h>
#include <ros/ros.h>
#include "tiny_ros/ros.h"
#include "log/log.h"
#include <boost/threadpool.hpp>
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetGpsPos.h"

class MessageToWeb
{
public:
    ~MessageToWeb() { post_thread_pool_.shutdown(); };
    static MessageToWeb* get_instance()
    {
        static MessageToWeb singleton;
        return &singleton;
    }
    int InitRemoteMqttTopic();

private:
    MessageToWeb();
    void ReceiveSingalMessage(const atris_msgs::SignalMessage &msg);
    void PostMessageToWeb(const std::string &content);
    void GetHttpHeader(std::vector<std::string> &http_header);
    void GetHttpHeaderSign(std::string &sign);
    void GetRandString(char *str);
    
private:
    int post_queue_count_;
    boost::mutex post_queue_mutex_;
    boost::threadpool::pool post_thread_pool_;

    ros::NodeHandle nh_;
    ros::Subscriber signal_req_sub_;
    ros::ServiceClient get_gps_position_srv_client_;
    const int kHttpTimeoutSec = 15;
};



#endif
