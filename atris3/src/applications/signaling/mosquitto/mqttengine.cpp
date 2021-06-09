/*
 * mqttengine.cpp
 *
 *  Created on: 2019-6-04
 *      Author: fupj
 */

#include "mqttengine.h"
#include <json/json.h>
#include "crypto/crypto.h"
#include "md5/md5.h"
#include "utils/utils.h"
#include <openssl/ssl.h>
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"

#define MQTTENGINE_TAG           "MqttEngine->"
#define MQTTENGINE_CRT_PATH      "/home/atris/atris_app/config/mosquitto/ca.crt"

/***************************************************************************/
void MqttEngine::on_connect(int rc) {
    log_info("%s Connected with code %d.", __PRETTY_FUNCTION__, rc);
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (try_sub_thread_) {
        try_sub_flag_ = false;
        try_sub_thread_->interrupt();
        try_sub_thread_->join();
        delete try_sub_thread_;
        try_sub_thread_ = NULL;
    }

    if(rc == 0) {
        /* Only attempt to subscribe on a successful connect. */
        if ((!robot_topic_.empty()) && (!shadow_topic_.empty() && (!task_topic_.empty()))) {
            if (!engine_logined_)
            {
                atris_msgs::AisoundTTS tts_msg;
                tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SERVER_ON_CONNECT);
                aisound_tts_pub_.publish(tts_msg);
            }

            engine_logined_ = true;
            sendNetWorkState(engine_logined_);
            subscribe(NULL, robot_topic_.c_str());
            subscribe(NULL, shadow_topic_.c_str());
            subscribe(NULL, task_topic_.c_str());
            notifyOnline(true);

        } else {
            try_sub_flag_ = true;
            try_sub_thread_ = new boost::thread(boost::bind(&MqttEngine::trySubThread, this));
        }
    } else {
        reconnect_async();
    }
}

void MqttEngine::on_connect_with_flags(int rc, int flags) {
    log_info("%s-rc:%d.", __PRETTY_FUNCTION__, rc);

}

void MqttEngine::on_disconnect(int rc) {
    log_warn("%src:%d.", __PRETTY_FUNCTION__, rc);
    engine_logined_ = false;
    sendNetWorkState(engine_logined_);
    reconnect_async();

    atris_msgs::AisoundTTS tts_msg;
    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SERVER_DISCONNECT);
    aisound_tts_pub_.publish(tts_msg);
}

void MqttEngine::sendNetWorkState(bool state)
{
    log_info("%s",__PRETTY_FUNCTION__);
    atris_msgs::NetworkState network_state;
    network_state.network_state = state?1:0;
    network_state_pub_.publish(network_state);
}

void MqttEngine::on_publish(int mid) {
    log_info("%s",__PRETTY_FUNCTION__);
}

void MqttEngine::on_message(const mosquitto_message *message) {
        log_info("%s:topic:%s",__PRETTY_FUNCTION__,message->topic);
    messageInstantReceived(message);
}

void MqttEngine::on_subscribe(int mid, int qos_count, const int *granted_qos) {
    log_info("%s-mid%d,qos_count:%d,%d", __PRETTY_FUNCTION__,mid,qos_count,granted_qos[0]);
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (try_sub_thread_) {
        try_sub_flag_ = false;
        try_sub_thread_->interrupt();
        try_sub_thread_->join();
        delete try_sub_thread_;
        try_sub_thread_ = NULL;
    }
}

void MqttEngine::on_unsubscribe(int mid) {
    log_info("%s-mid:%d", __PRETTY_FUNCTION__,mid);
}

void MqttEngine::on_log(int level, const char *str) {
    switch (level) {
    case MOSQ_LOG_INFO:
    case MOSQ_LOG_NOTICE:
    case MOSQ_LOG_SUBSCRIBE:
    case MOSQ_LOG_UNSUBSCRIBE:
    case MOSQ_LOG_WEBSOCKETS:
    case MOSQ_LOG_INTERNAL:    
    case MOSQ_LOG_WARNING:     
    case MOSQ_LOG_ERR: 
    case MOSQ_LOG_DEBUG:
    default:
        //log_info("%s:level:%d:str:%s",__PRETTY_FUNCTION__,level,str);
        break;
    }
}

void MqttEngine::on_error() {
    log_info("%s",__PRETTY_FUNCTION__);
}
/*****************************************************************************/

boost::mutex MqttEngine::mutex_;
MqttEngine* MqttEngine::engine_ = NULL;

MqttEngine::MqttEngine(const mqtt_params &params)
    : mosquittopp(params.client_id.c_str(), true),
      robot_topic_(""),
      shadow_topic_(""),
      task_topic_(""),
      try_sub_thread_(NULL),
      try_sub_flag_(false),
      engine_logined_(false),
      msgType_(params.msg_type) {
    signal_resp_sub_ = nh_.subscribe(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100, &MqttEngine::messageInstantSend, this);
    signal_req_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
    network_state_pub_ = nh_.advertise<atris_msgs::NetworkState>(TOPIC_NETWORK_STATE, 100);
    loop_forever_thread_ = new boost::thread(boost::bind(&MqttEngine::loopForeverThread, this));
    nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string sn = shmrbt.robot.sn;
    if (!sn.empty()) {
       robot_topic_ = "Power-Atris/" + sn + "/+";
       shadow_topic_ ="shadow/operation/push/control/Power-Atris/" + sn;
       task_topic_ ="shadow/operation/result/inspection/Power-Atris/" +sn;
    }

    if (params.encrypt) {
        tls_set(MQTTENGINE_CRT_PATH);
    }

    set_mqtt_will();
    int ret = username_pw_set(params.username.c_str(), params.password.c_str());
    if(0 == ret){
            ret =connect_async(params.host.c_str(), params.port, params.keepalive);
            log_info("%s:%d\n",__PRETTY_FUNCTION__,ret);
    }
}

void MqttEngine::loopForeverThread() {
    log_info(MQTTENGINE_TAG"%s", __FUNCTION__);
    loop_forever();
}

MqttEngine::~MqttEngine() {
    log_info(MQTTENGINE_TAG"%s", __FUNCTION__);
    if (engine_) {
        delete engine_;
        engine_ = nullptr;
    }
}

void MqttEngine::trySubThread() {
    try_sub_flag_ = true;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string sn = shmrbt.robot.sn;
    while (try_sub_flag_) {
        if (!sn.empty()) {
            engine_logined_ = true;
            robot_topic_ = "Power-Atris/" + sn + "/+";
            shadow_topic_ = "shadow/operation/push/control/Power-Atris/" + sn;
            task_topic_ = "shadow/operation/result/inspection/Power-Atris/" + sn;
            subscribe(NULL, robot_topic_.c_str());
            subscribe(NULL, shadow_topic_.c_str());
            subscribe(NULL, task_topic_.c_str());
            notifyOnline(true);
            break;
        }
        sleep(1);
    }
}

void MqttEngine::notifyOnline(bool online) {
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string topic = "shadow/operation/push/pc/poweratris";
    Json::Value root;
    Json::FastWriter writer;
    root["linetime"]=ros::Time::now().toSec();
    root["sn"]=shmrbt.robot.sn;
    root["state"]=online?1:0;
    root["tilte"]="atris";
    std::string msg = writer.write(root);
    log_debug("%s topic: %s, msg: %s", __PRETTY_FUNCTION__, topic.c_str(),  msg.c_str());
    publish(NULL, topic.c_str(), msg.length(), (const void*)msg.c_str(), 1, true);
}

void MqttEngine::set_mqtt_will() {
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string topic = "shadow/operation/push/pc/poweratris";
    Json::Value root;
    Json::FastWriter writer;
    root["linetime"]=ros::Time::now().toSec();
    root["sn"]=shmrbt.robot.sn;
    root["state"]=0;
    root["tilte"]="atris";
    std::string msg = writer.write(root);
    int ret =will_set(topic.c_str(), msg.length(), (const void*)msg.c_str(), 1, true);
}

void MqttEngine::messageInstantReceived(const mosquitto_message *mqttmsg) {
    Json::Reader reader;
    std::string title, accid, sender;
    Json::Value root, response;
    atris_msgs::SignalMessage message;
    std::string topic = mqttmsg->topic;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    
    std::string sn = std::string(shmrbt.robot.sn);
    if (topic.find("Power-Atris") != std::string::npos
      && topic.find(sn) != std::string::npos
      && mqttmsg->payload && mqttmsg->payloadlen > 0) {
        do {
            std::string playload((const char*)mqttmsg->payload, mqttmsg->payloadlen);
            if (playload.empty()) {
                log_error("%s playload is empty.", __PRETTY_FUNCTION__);
                break;
            }

            if (!reader.parse(playload, root)) {
                log_error("%s parse error: %s.", __PRETTY_FUNCTION__, playload.c_str());
                break;
            }

            std::string::size_type pos1 = topic.find_first_of("/");;
            std::string::size_type pos2 = topic.find_first_of("/", pos1+1);
            if (pos1 != std::string::npos && pos2 != std::string::npos) {
              sender = topic.substr(pos1+1, pos2-pos1-1);
            }

            if (sender.empty()) {
              log_error("%s sender is empty, check topic: %s.", __PRETTY_FUNCTION__, topic.c_str());
              break;
            }

            title = root["title"].isNull() ? "" : root["title"].asString();
            accid = root["accid"].isNull() ? "" : root["accid"].asString();
            message.type = msgType_;
            message.title = title;
            message.msg = playload;
            message.account = sender;
            
            if (root["title"].isNull() || root["accid"].isNull()
                    || root["content"].isNull() || root["content"]["id"].isNull()
                    || root["content"]["timestamp"].isNull() || title.length() < 8) {
                log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                response["json"] = message.msg;
                response["error_msg"] = "Json invalid";
                Utils::get_instance()->responseResult(message, response, "response_json_error");
            } else {
                #if 0 //stop use author except for the first generation
                    // filter invalid instruction
                    log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                    response["id"] = root["content"]["id"];
                    response["timestamp"] = root["content"]["timestamp"];
                    if ((title != "request_bound_robot")
                            && (title != "request_reset_robot")
                            && (title != "request_change_pwd")
                            && (title != "request_robot_status")
                            && (title != "request_bound_info")
                            && (title != "request_robot_info")
                            && (title != "request_login_robot")) {
                        log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                        if (shmrbt.robot.binded) {
                            std::string accid = root["accid"].isNull() ? "" :root["accid"].asString();
                            if (accid != std::string(shmrbt.robot.accid)) {
                                response["result"] = "fail_other_bound";
                                log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                                Utils::get_instance()->responseResult(message, response, ("response" + title.substr(7)));
                                break;
                            }
                        } else {
                            response["result"] = "fail_no_bound";
                            log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                            Utils::get_instance()->responseResult(message, response, ("response" + title.substr(7)));
                            break;
                        }
                    }
                    log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                    //shaked down
                    if (shmrbt.appdata.shaked) {
                        if ((title != "request_robot_status") && (title != "request_robot_info") && (title != "request_shake_up")
                                && (title != "request_shake_die") && (title != "request_shake_down")) {
                            response["result"] = "fail_shake_down";
                            Utils::get_instance()->responseResult(message, response, ("response" + title.substr(7)));
                            log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                            break;
                        }else{
                            log_info("%s:%d\n",__PRETTY_FUNCTION__,__LINE__);
                        }
                    }
                #endif
                message.msgID = root["content"]["id"].asString();
                message.timestamp = root["content"]["timestamp"].asInt64();
                signal_req_pub_.publish(message);
            }
        } while(0);
    } else {
        log_error("%s topic: %s, payload: %p, payloadlen: %d", 
          __PRETTY_FUNCTION__, topic.c_str(), mqttmsg->payload, mqttmsg->payloadlen);
    }
}

void MqttEngine::messageInstantSend(const atris_msgs::SignalMessage& msg) {
    Json::Reader reader;
    Json::Value root;
    if (this->engine_logined_ && (msg.type == msgType_ || msg.type.empty())) {
        if (!reader.parse(msg.msg, root) || root.isNull()) {
            log_error("%s msgID: %s, msg: %s", __PRETTY_FUNCTION__, msg.msgID.c_str(), msg.msg.c_str());
        } else {
            string topic;
            shm::Robot shmrbt;
            shm::iMemory_read_Robot(&shmrbt);
            std::string sn = std::string(shmrbt.robot.sn);
            string title = root["title"].asString();
            if(title.find("response_new_task")  != std::string::npos 
            || title.find("notify_task_status") != std::string::npos 
            || title.find("notify_optimal_path") != std::string::npos 
            || title.find("notify_robot_pose") != std::string::npos 
            || title.find("notify_task_point_status") != std::string::npos) { //todo better solution ?
                topic = "shadow/operation/inspection/Power-Atris/" + sn;
            }else if(title.find("notify_robot_info")!=std::string::npos) {
                topic = "shadow/operation/setting/Atris/" + sn;
            }else{
                topic = "shadow/operation/control/Power-Atris/" + sn;
            }
            publish(NULL, topic.c_str(), msg.msg.length(), (const void*)msg.msg.c_str());
            log_debug("%s topic: %s, msgID: %s, msg: %s", __PRETTY_FUNCTION__, topic.c_str(), msg.msgID.c_str(), msg.msg.c_str());

			if (title.find("notify_robot_info") == std::string::npos) { // for debug
                topic = "Atris/" + sn + "/" + root["title"].asString();
                publish(NULL, topic.c_str(), msg.msg.length(), (const void*)msg.msg.c_str());
            }
        }
    } else {
        if (!this->engine_logined_) {
            log_warn("%s mqtt: %s, engine_logined: %d", __PRETTY_FUNCTION__, msgType_.c_str(), this->engine_logined_);
        }
    }
}

