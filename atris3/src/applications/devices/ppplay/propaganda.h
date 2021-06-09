/*
 * propaganda.h
 *
 *  Created on: 2018-7-19
 *      Author: fupj
 */

#ifndef PROPAGANDA_H_
#define PROPAGANDA_H_
#include <boost/thread.hpp>
#include <json/json.h>
#include "libmediaplayer/mediaplayer.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetVoiceChatStatus.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/PPPlayerControl.h"
#include "atris_msgs/GetGpsPos.h"

// the current player owner of the media
enum PlayerOwner
{
    PC = 0,
    NAV = 1
};

class PPPlayerListener;
class PPPlayDownload;
class Propaganda {
public:
    virtual ~Propaganda();
    static Propaganda* get_instance();
    void play(const std::vector<std::string> &play_list, bool loop = true, int mode = 2);
    void pause();
    void resume();
    bool isPlaying();
    bool isPaused();
    void stop();
    inline void setPlayOwner(int owner){boost::unique_lock<boost::mutex> lock(pl_own_mutex_); play_owner_ = owner;};
    inline int getPlayOwner(){boost::unique_lock<boost::mutex> lock(pl_own_mutex_); return play_owner_;};
    std::string getCurPlay(){return current_play_;};
private:
    Propaganda();
    void messageInstantReceive(const atris_msgs::SignalMessage& msg);
    void doRequestMusicTransport(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doRequestMusicList(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doRequestMusicPlay(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doRequestMusicRemove(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doPlay(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doResume(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doRename(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doPause(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doStop(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    bool doGetPPPlayingStatus(atris_msgs::GetPPPlayingStatus::Request& req, atris_msgs::GetPPPlayingStatus::Response& res);
    bool doPPPlayerControl(atris_msgs::PPPlayerControl::Request& req, atris_msgs::PPPlayerControl::Response& res);
    void responseMusicPlay(std::string result, std::string action, std::string status, std::string msgID, int64_t timestamp, bool notify = false);
    void notifyPPPlayEvent(int ctrl_source, int action, std::string cur_play = "");
    void playNextThread();
    void doPlayNext();
    void randListGen(std::vector<std::string> &play_list_tmp, std::vector<std::string> &play_list_rand, int list_size);
    int randPickFunc(int list_size);
    inline void setPlayMode(int mode){play_mode_ = mode;};
    inline int getPlayMode(){return play_mode_;};
    //inline void setPlayOwner(int owner){boost::unique_lock<boost::mutex> lock(pl_own_mutex_); play_owner_ = owner;};
    //inline int getPlayOwner(){boost::unique_lock<boost::mutex> lock(pl_own_mutex_); return play_owner_;};

private:
    friend class PPPlayerListener;
    friend class PPPlayDownload;
    static boost::mutex mutex_;
    static Propaganda* propaganda_;
    static boost::mutex play_next_mutex_;
    static boost::condition_variable play_next_cond_;
    boost::thread* play_next_thread_;
    MediaPlayer* ppplayer_;
    boost::mutex list_mutex_;
    boost::mutex pl_own_mutex_;
    std::vector<std::string> play_list_;
    std::list<std::string> current_list_;
    std::string current_play_;
    atris_msgs::SignalMessage current_play_origin_;
    int64_t pts_;
    int64_t duration_;
    bool play_recycle_;
    bool started_;
    bool play_interval_flag_;
    bool play_interval_pause_;
    boost::mutex play_interval_mutex_;
    boost::condition_variable play_interval_cond_;
    PPPlayerListener *listener_;
    int play_times_;
    int play_interval_time_;
    ros::NodeHandle nh_;
    ros::Subscriber signal_req_sub_;
    ros::ServiceClient voice_chat_status_srv_client_;
    ros::ServiceServer get_ppplaying_status_srv_;
    ros::ServiceServer ppplayer_control_srv_;
    ros::ServiceClient get_gps_position_srv_client_;
    int play_mode_;
    int play_owner_;
    std::string pp_serial_num_;
};

#endif /* PROPAGANDA_H_ */
