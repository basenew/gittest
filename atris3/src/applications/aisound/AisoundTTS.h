/*
 * AisoundTTS.h
 *
 *  Created on: 2019-3-28
 *      Author: fupj
 */

#ifndef AISOUND_TTS_H_
#define AISOUND_TTS_H_
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ros/ros.h>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/AisoundTTS.h"
#include "libmediaplayer/mediaplayer.h"
#include "tts_strings/tts_strings.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/PPPlayerControl.h"
#include "atris_defines.h"

class TTSPlayerListener;
class AisoundTTS {
public:
    virtual ~AisoundTTS();
    static AisoundTTS* get_instance();

private:
    AisoundTTS();
    void aisoundTTSTopicCb (const atris_msgs::AisoundTTS& msg);
    void aisoundTTSThread();
    void ttsSpinThread();
    void ttsSpinStop();
    void messageInstantReceive (const atris_msgs::SignalMessage& msg);

private:
    static boost::mutex mutex_;
    static boost::mutex tts_interrupt_mutex_;
    static AisoundTTS* aisound_tts_;
    static boost::condition_variable tts_player_cond_;
    static boost::mutex tts_player_mutex_;
    bool tts_task_running_;
    bool msp_login_;
    bool lock_state_;
    static bool isPPPlayPaused;

    friend class TTSPlayerListener;
    MediaPlayer* tts_player_;
    int tts_priority_playing_;
    TTSPlayerListener *tts_player_listener_;
    boost::thread* tts_task_thread_;
    boost::condition_variable tts_task_cond_;
    std::list<atris_msgs::AisoundTTS> tts_low_queue_;
    std::list<atris_msgs::AisoundTTS> tts_mid_queue_;
    std::list<atris_msgs::AisoundTTS> tts_high_queue_;
    std::list<atris_msgs::AisoundTTS> tts_user_queue_;

    boost::thread* tts_spin_thread_;
    int tts_spin_interval_;
    std::string tts_spin_text_;
    bool tts_spin_loop_;
    bool tts_interrupt_flg_;
    boost::mutex tts_spin_mutex_;
    boost::condition_variable tts_spin_cond_;
    boost::mutex tts_interval_mutex_;
    boost::condition_variable tts_interval_cond_;
    ros::NodeHandle nh_;
    ros::Subscriber aisound_tts_sub_;
    ros::Subscriber signal_req_sub_;
    ros::ServiceClient get_ppplaying_status_srv_client_;
    ros::ServiceClient ppplayer_control_srv_client_;
};

#endif /* AISOUND_TTS_H_ */
