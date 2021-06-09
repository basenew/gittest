/*
 * robot.h
 *
 *  Created on: 2018-6-21
 *      Author: fupj
 */

#ifndef ROBOT_H_
#define ROBOT_H_
#include <boost/thread.hpp>
#include <algorithm>
#include <string>
#include <json/json.h>
#include "database/sqliteengine.h"
#include "propaganda.h"
#include "sound/sound.h"
#include "disperse.h"
#include "config/config.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/TimeCalibrated.h"
#include "atris_msgs/RobotShaked.h"
#include "atris_msgs/StopVoiceChat.h"
#include "atris_msgs/NavReset.h"
#include "std_msgs/String.h"
#include "imemory/atris_imemory_api.h"

class MqttEngine;

class Robot {
public:
    virtual ~Robot();
    static Robot* get_instance();
    static int64_t timeCalibrated();
    void init();
    void doResetRobotByAbi();

public:
    static int64_t calibration_value;
    static bool calibrated;

private:
    std::string getSnFromEeprom();

private:
    Robot();
    void initRobotTable();
    void initAppdataTable();
    void reinitAppDataDb(void);
    void messageInstantReceive(const atris_msgs::SignalMessage& msg);
    void doYunxinTokenUpdate(const std_msgs::String& msg);
    void doResetRobot(int clear = 0);
    void doRequestGetRobotState(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestChangeRobotState(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestLoginRobot(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestBoundRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestBoundRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestUnboundRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestUnboundRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestChangePwd(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestResetRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestResetRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestTimeCalibration(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestBoundInfo(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestRobotStatus(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestSetVolume(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestSetMute(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestGetVolume(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestGetMute(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestSetPlayInterval(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestGetPlayInterval(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestShakeDown(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestShakeUp(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestShakeDie(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestShakeStatus(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestStopAllTask(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestSetRobotSn(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestRecordFactoryFile(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestPlayFactoryFile(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doNoticeVoipCall();
    void doRequesSetVoipAttributes(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequesGetVoipAttributes(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequesHangupVoip(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    int closeMicAfterRecordFile();
    void covertAbiHttpErrToFailString(int abiHttpErr, std::string &string);

private:
    static boost::mutex mutex_;
    static Robot* robot_;
    bool abiEnable_;
    MediaPlayer* player_;
    shm::Robot shmrobot_;
    ros::NodeHandle nh_;
    ros::Publisher aisound_tts_pub_;
    ros::Publisher time_calibrated_pub_;
    ros::Publisher robot_shaked_pub_;
    ros::Publisher nav_reset_pub_;
    ros::Subscriber signal_req_sub_;
    ros::Subscriber yunxin_token_sub_;
    ros::ServiceClient stop_voice_chat_srv_client_;
};

#endif /* ROBOT_H_ */
