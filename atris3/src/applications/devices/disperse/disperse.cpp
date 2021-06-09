/*
 * disperse.cpp
 *
 *  Created on: 2019-3-5
 *      Author: fupj
 */
#include "disperse.h"
#include "propaganda.h"
#include "utils/utils.h"
#include "database/sqliteengine.h"
#include "sound/sound.h"
#include "imemory/atris_imemory_api.h"
#include "log/log.h"

#define DISPERSE_TAG    "Disperse->"
#define DISPERSE_FILE    "/home/atris/atris_app/data/1.mp3"
#define DISPERSE_VOLUME    100

Disperse* Disperse::disperse_ = NULL;
boost::mutex Disperse::mutex_;

class DisperseListener: public MediaPlayerListener
{
  Disperse *disperse_;
  virtual void notify(GstMessageType msg, int64_t ext1 = 0, int64_t ext2 = 0) {
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    switch(msg) {
    case GST_MESSAGE_EOS: {
      log_error(DISPERSE_TAG"%s GST_MESSAGE_EOS", __FUNCTION__);
      boost::unique_lock<boost::mutex> lock(disperse_->play_mutex_);
      disperse_->play_cond_.notify_one();
      break;
    }
    case GST_MESSAGE_ERROR: {
      log_error(DISPERSE_TAG"%s GST_MESSAGE_ERROR", __FUNCTION__);
      boost::unique_lock<boost::mutex> lock(disperse_->play_mutex_);
      disperse_->disperseStop();
      break;
    }
    default: { break; }
    }
  }

public:
  DisperseListener(Disperse *disperse): disperse_(disperse)  {  }
  virtual ~DisperseListener() { }
};

Disperse::Disperse() 
  : is_disperse_(false) {
  log_info(DISPERSE_TAG"%s", __FUNCTION__);
  signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Disperse::messageInstantReceive, this);
  get_disperse_status_srv_ = nh_.advertiseService(SRV_GET_DISPERSE_STATUS, &Disperse::doGetDisperseStatus, this);
  voice_chat_status_srv_client_ = nh_.serviceClient<atris_msgs::GetVoiceChatStatus>(SRV_GET_VOICE_CHAT_STATUS);
  stop_voice_chat_srv_client_ = nh_.serviceClient<atris_msgs::StopVoiceChat>(SRV_STOP_VOICE_CHAT);
  get_gps_position_srv_client_ = nh_.serviceClient<atris_msgs::StopVoiceChat>(SRV_GET_GPS_POSITION);
  listener_ = new DisperseListener(this);
  player_ = MediaPlayer::createMediaPlayer();
  player_->setListener(listener_);
  play_thread_ = new boost::thread(boost::bind(&Disperse::playThread, this));
  disperse_serial_num_ = "";
}

Disperse::~Disperse() {
    log_info(DISPERSE_TAG"%s", __FUNCTION__);
}

Disperse* Disperse::get_instance() {
  if (!disperse_) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (!disperse_) {
      disperse_ = new Disperse();
    }
  }
  return disperse_;
}

void Disperse::playThread() {
  boost::unique_lock<boost::mutex> lock(play_mutex_, boost::defer_lock);
  while (true) {
    lock.lock();
    play_cond_.wait(lock);
    dispersePlay();
    lock.unlock();
  }
}

void Disperse::notifyDidsperseEvent(int status)
{
  ros::Time now = ros::Time::now();
  Json::Value content;
  Json::Value evt_arr;
  Json::Value disperse_event_msg;
  Json::FastWriter writer;
  std::string event_content = "";

  shm::Robot shmrbt;
  shm::iMemory_read_Robot(&shmrbt);
  std::string accid = std::string(shmrbt.robot.accid);

  content["category"] = "custom_event"; // custom event

  // eventId according to message type
  content["eventId"] = "dispersion_event";

  // accid
  content["userId"] = accid;
  content["recordedAt"] = (uint64_t)(now.toSec());
  content["segmentation"] = "";

  content["customSegmentation"]["status"] = status;

  content["customSegmentation"]["serialNum"] = disperse_serial_num_;

  // reason
  content["customSegmentation"]["reason"]["type"] = "default";
  content["customSegmentation"]["reason"]["value"] = "";

  //source
  // can only be triggered by remote control
  content["customSegmentation"]["source"] = 2;
  content["customSegmentation"]["lati"] = "";
  content["customSegmentation"]["longi"] = "";
  atris_msgs::GetGpsPos gpspos;
  if (get_gps_position_srv_client_.call(gpspos)) 
  {
      if(gpspos.response.status == 0)
      {
          content["customSegmentation"]["lati"] = gpspos.response.latitude;
          content["customSegmentation"]["longi"] = gpspos.response.longitude;
      }
  } 
  else 
  {
      log_warn("%s get_gps_position_srv_client_ call fail.", __FUNCTION__);
      //return;
  }
  evt_arr.append(content);
  event_content = writer.write(evt_arr);
  log_info("%s event_content is %s",__FUNCTION__, event_content.c_str());

  disperse_event_msg["event_message"] = evt_arr;

  // publish to http
  Utils::get_instance()->NotifyRobotStatus("notify_dispersion_event", disperse_event_msg);
}

void Disperse::dispersePlay() {
  if (!is_disperse_) {
    is_disperse_ = true;
    
    atris_msgs::GetVoiceChatStatus status;
    voice_chat_status_srv_client_.call(status);
    if (status.response.chated) {
        atris_msgs::StopVoiceChat stop_chat;
        stop_voice_chat_srv_client_.call(stop_chat);
    }

    if (Propaganda::get_instance()->isPlaying()) {
      Propaganda::get_instance()->stop();
    }

    std::string url = DISPERSE_FILE;
    player_->setDataSource(url);
    player_->prepare();
    player_->start();

    Sound::get_instance()->setVolume(DISPERSE_VOLUME);
    Sound::get_instance()->setMuted(false);

    char rand_str[16] = {0};
    Utils::get_instance()->getRandStr(rand_str);
    disperse_serial_num_ = rand_str;
    notifyDidsperseEvent(1);
  }
}

void Disperse::disperseStop() {
  if(is_disperse_)
  {
    player_->stop();
    is_disperse_ = false;
    
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    Sound::get_instance()->setVolume(shmrbt.appdata.volume);
    Sound::get_instance()->setMuted(shmrbt.appdata.muted);
    notifyDidsperseEvent(0);
  }
}


void Disperse::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
  Json::Reader reader;
  Json::Value root, response;
  if (msg.title == "request_sonic_disperse") {
    reader.parse(msg.msg, root);

    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["switch"].isNull()) {
      response["content"]["switch"] = root["content"]["switch"];
      boost::unique_lock<boost::mutex> lock(play_mutex_);
      if (root["content"]["switch"].asInt()) {
        dispersePlay();
      } else {
        disperseStop();
      }
    } else {
      response["result"] = "fail_invalid_data";
    }
    Utils::get_instance()->responseResult(msg, response, "response_sonic_disperse");
  } else if (msg.title == "request_get_sonic_disperse_status") {
    boost::unique_lock<boost::mutex> lock(play_mutex_);
    reader.parse(msg.msg, root);
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["switch"] = is_disperse_ ? 1 : 0;
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_get_sonic_disperse_status");
  }
}

bool Disperse::doGetDisperseStatus(atris_msgs::GetDisperseStatus::Request& req,
  atris_msgs::GetDisperseStatus::Response& res) {
  res.dispersing = is_disperse_;
  return true;
}

