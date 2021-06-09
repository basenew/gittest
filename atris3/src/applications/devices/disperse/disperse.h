/*
 * disperse.h
 *
 *  Created on: 2019-3-5
 *      Author: fupj
 */

#ifndef DISPERSE_H_
#define DISPERSE_H_
#include <boost/thread.hpp>
#include <json/json.h>
#include "libmediaplayer/mediaplayer.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetVoiceChatStatus.h"
#include "atris_msgs/StopVoiceChat.h"
#include "atris_msgs/GetDisperseStatus.h"
#include "atris_msgs/GetGpsPos.h"

class DisperseListener;
class Disperse {
public:
    virtual ~Disperse();
    static Disperse* get_instance();
private:
    Disperse();
    void playThread();
    void notifyDidsperseEvent(int status);
    void dispersePlay();
    void disperseStop();
    void messageInstantReceive(const atris_msgs::SignalMessage& msg);
    bool doGetDisperseStatus(atris_msgs::GetDisperseStatus::Request& req, atris_msgs::GetDisperseStatus::Response& res);
    std::string disperse_serial_num_;
    static boost::mutex mutex_;
    static Disperse* disperse_;
    friend class DisperseListener;
    boost::mutex play_mutex_;
    boost::condition_variable play_cond_;
    boost::thread* play_thread_;
    DisperseListener *listener_;
    MediaPlayer* player_;
    bool is_disperse_;
    ros::NodeHandle nh_;
    ros::Subscriber signal_req_sub_;
    ros::ServiceClient voice_chat_status_srv_client_;
    ros::ServiceClient stop_voice_chat_srv_client_;
    ros::ServiceServer get_disperse_status_srv_;
    ros::ServiceClient get_gps_position_srv_client_;
};
#endif /* DISPERSE_H_ */
