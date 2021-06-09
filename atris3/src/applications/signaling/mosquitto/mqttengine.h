/*
 * mqttengine.h
 *
 *  Created on: 2019-6-04
 *      Author: fupj
 */

#ifndef MQTTENGINE_H_
#define MQTTENGINE_H_
#include <string>
#include <algorithm>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include "log/log.h"
#include "mosquitto/mosquittopp.h"
#include "config/config.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/NetworkState.h"
#include "atris_msgs/AisoundTTS.h"
#include "tts_strings/tts_strings.h"

struct mqtt_params {
    std::string host;
    std::string username;
    std::string password;
    int port;
    int encrypt;
    int keepalive;
    std::string msg_type;
    std::string client_id; // can not set to ""
};

class MqttEngine: public mosqpp::mosquittopp {
public:
    MqttEngine(const mqtt_params &params);
    virtual ~MqttEngine();

protected:
    virtual void on_connect(int rc);
    virtual void on_connect_with_flags(int rc, int flags);
    virtual void on_disconnect(int rc);
    virtual void on_publish(int mid);
    virtual void on_message(const struct mosquitto_message * message);
    virtual void on_subscribe(int mid, int qos_count, const int * granted_qos);
    virtual void on_unsubscribe(int mid);
    virtual void on_log(int level, const char * str);
    virtual void on_error();

private:
    void loopForeverThread();
    void trySubThread();
    void notifyOnline(bool online);
    void set_mqtt_will();
    void messageInstantReceived(const mosquitto_message *mqttmsg);
    void messageInstantSend (const atris_msgs::SignalMessage& msg);
    void sendNetWorkState(bool state);

    static boost::mutex mutex_;
    static MqttEngine* engine_;

    boost::thread* loop_forever_thread_;
    std::string robot_topic_;
    std::string shadow_topic_;
    std::string task_topic_;
    boost::thread* try_sub_thread_;
    bool try_sub_flag_;
    bool engine_logined_;
    std::string msgType_;
    ros::NodeHandle nh_;
    ros::Publisher signal_req_pub_;
    ros::Publisher network_state_pub_;
    ros::Subscriber signal_resp_sub_;
    ros::Publisher aisound_tts_pub_;
};

#endif /* MQTTENGINE_H_ */
