/*
 * propaganda.cpp
 *
 *  Created on: 2018-7-19
 *      Author: fupj
 */

#include "propaganda.h"
#include <libgen.h>
#include "database/sqliteengine.h"
#include "transferfile/transferfile.h"
#include "sound/sound.h"
#include "utils/utils.h"
#include "imemory/atris_imemory_api.h"

#define PPPLAY_TAG    "Propaganda->"
#define PPPLAY_DIR    "/userdata/impdata/media/"
#define DOWNLOAD_PROGRESS_INTERVAL 1   // seconds
#define PPPLAY_TABLE_COLUMN  5
#define PPPLAY_OUT_LIMITS  20
#define PPPLAY_TABLE "CREATE TABLE IF NOT EXISTS [ppplay] (" \
    "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
    "[name] TEXT," \
    "[type] TEXT," \
    "[path] TEXT," \
    "[url] TEXT)"

Propaganda* Propaganda::propaganda_ = NULL;
boost::mutex Propaganda::play_next_mutex_;
boost::condition_variable Propaganda::play_next_cond_;
boost::mutex Propaganda::mutex_;

class PPPlayerListener: public MediaPlayerListener
{
public:
  int64_t last_pts_;
  Propaganda *ppgd_;
  bool need_resp_;
  atris_msgs::SignalMessage origin_;

  virtual void notify(GstMessageType msg, int64_t ext1 = 0, int64_t ext2 = 0) {
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    switch(msg) {
    case GST_MESSAGE_EOS: {
      ppgd_->pts_ = ppgd_->duration_;
      ppgd_->notifyPPPlayEvent(ppgd_->getPlayOwner(),0,ppgd_->getCurPlay());
      ppgd_->responseMusicPlay("success", "play", "finished", origin_.msgID, origin_.timestamp, (need_resp_ ? false : true));
      need_resp_ = false;
      boost::unique_lock<boost::mutex> lock(Propaganda::play_next_mutex_);
      Propaganda::play_next_cond_.notify_one();
      break;
    }
    case GST_MESSAGE_ERROR: {
      ppgd_->pts_ = ppgd_->duration_;
      ppgd_->responseMusicPlay("fail_play_error", "play", "", origin_.msgID, origin_.timestamp, (need_resp_ ? false : true));
      need_resp_ = false;
      boost::unique_lock<boost::mutex> lock(Propaganda::play_next_mutex_);
      Propaganda::play_next_cond_.notify_one();
      break;
    }
    case GST_MESSAGE_PROGRESS: {
      ppgd_->pts_ = ext1;
      ppgd_->duration_ = ext2;

      boost::unique_lock<boost::mutex> lock(ppgd_->play_interval_mutex_);
      if (ppgd_->play_interval_pause_) {
        ppgd_->play_interval_pause_ = false;
        ppgd_->pause();
      }

      if (!ppgd_->started_) {
        ppgd_->started_ = true;
        last_pts_ = ppgd_->pts_;
        if (ppgd_->play_interval_pause_) {
          ppgd_->responseMusicPlay("success", "pause", "", origin_.msgID, origin_.timestamp, (need_resp_ ? false : true));
          need_resp_ = false;
        } else {
          ppgd_->responseMusicPlay("success", "play", "started", origin_.msgID, origin_.timestamp, (need_resp_ ? false : true));
          need_resp_ = false;
        }
      } else {
        if(origin_.type != "yunxin")
        { 
          if ((ppgd_->pts_ - last_pts_) >= 1000) {
            last_pts_ = ppgd_->pts_;
            ppgd_->responseMusicPlay("success", "play", "playing", origin_.msgID, origin_.timestamp, (need_resp_ ? false : true));
            need_resp_ = false;
          }
        }

      }
      break;
    }
    default: { break; }
    }
  }

public:
  PPPlayerListener(Propaganda *ppgd): ppgd_(ppgd) { }
  virtual ~PPPlayerListener() { }
};

class PPPlayDownload: public FileObject {
private:
  uint32_t last_sec_;

  void responseMusicTransport(std::string result, std::string status, double progress ) {
    Json::Value response;
    response["id"] = origin.msgID;
    response["timestamp"] = origin.timestamp;
    response["url"] = remote_path;
    response["name"] = name;
    response["progress"] = progress;
    response["status"] = status;
    response["result"] = result;
    Utils::get_instance()->responseResult(origin, response, "response_music_transport");
  }

  virtual void progress(double progress)  {
    uint32_t now = ros::Time::now().sec;
    if ((now - last_sec_) >= DOWNLOAD_PROGRESS_INTERVAL) {
#if 0
      responseMusicTransport("success", "downloading", progress);
#endif
      last_sec_ = now;
    }
    if (now < last_sec_) {
      last_sec_ = now;
    }
  }

  virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
    if (state == TRANSFER_FILE_ERROR) {
      responseMusicTransport("fail_download_error", "", 0);
    } else if (state == TRANSFER_FILE_STARTED) {
      responseMusicTransport("success", "started", 0);
    } else if (state == TRANSFER_FILE_COMPLETED) {
      char **result, **result1;
      int row, column, row1, column1;

      boost::unique_lock<boost::mutex> lock(Propaganda::mutex_);
      SqliteEngine::query("SELECT * FROM ppplay", &result, &row, &column);
      if (row < PPPLAY_OUT_LIMITS) {
        SqliteEngine::query("SELECT * FROM ppplay WHERE name='"+name+"'", &result1, &row1, &column1);
        if (row1 <= 0) {
          SqliteEngine::execSQL("INSERT INTO ppplay(name, type, path, url) "
              "VALUES('"+name+"', 'music', '"+local_path+"', '"+remote_path+"')");
        } else {
          SqliteEngine::execSQL("UPDATE ppplay SET type='music' WHERE name='"+name+"'");
          SqliteEngine::execSQL("UPDATE ppplay SET path='"+local_path+"' WHERE name='"+name+"'");
          SqliteEngine::execSQL("UPDATE ppplay SET url='"+remote_path+"' WHERE name='"+name+"'");
        }

        SqliteEngine::freeQuery(result1);

        responseMusicTransport("success", "finished", 100);
      } else {
        remove(local_path.c_str());
        responseMusicTransport("fail_out_limits", "finished", 100);
      }

      SqliteEngine::freeQuery(result);
    }
  }

public:
    PPPlayDownload(): last_sec_(0) { }
    atris_msgs::SignalMessage origin;
  std::string name;
};

Propaganda::Propaganda() {
  log_info(PPPLAY_TAG"%s", __FUNCTION__);
  voice_chat_status_srv_client_ = nh_.serviceClient<atris_msgs::GetVoiceChatStatus>(SRV_GET_VOICE_CHAT_STATUS);
  get_gps_position_srv_client_ = nh_.serviceClient<atris_msgs::GetGpsPos>(SRV_GET_GPS_POSITION);
  signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Propaganda::messageInstantReceive, this);
  ppplayer_control_srv_ = nh_.advertiseService(SRV_PPPLAYER_CONTROL, &Propaganda::doPPPlayerControl, this);
  get_ppplaying_status_srv_ = nh_.advertiseService(SRV_GET_PPPLAYING_STATUS, &Propaganda::doGetPPPlayingStatus, this);

  char **result;
  int row, column;

  retry:
  SqliteEngine::execSQL(PPPLAY_TABLE);
  SqliteEngine::query("SELECT * FROM ppplay", &result, &row, &column);
  if (row > 0 && column != PPPLAY_TABLE_COLUMN) {
    SqliteEngine::execSQL("DROP TABLE ppplay");
    SqliteEngine::freeQuery(result);
    goto retry;
  }
  SqliteEngine::freeQuery(result);

  shm::Robot shmrbt;
  shm::iMemory_read_Robot(&shmrbt);
  listener_ = new PPPlayerListener(this);
  ppplayer_ = MediaPlayer::createMediaPlayer();
  ppplayer_->setListener(listener_);
  current_play_ = "";
  pts_ = duration_ = 0;
  play_recycle_ = true;
  started_ = false;
  play_interval_flag_ = false;
  play_interval_pause_ = false;
  play_times_ = -1;
  play_interval_time_ = shmrbt.appdata.play_interval;
  play_next_thread_ = new boost::thread(boost::bind(&Propaganda::playNextThread, this));
  play_mode_ = 1;
  play_owner_ = PC; // default owner = PC
  pp_serial_num_ = "";
}

Propaganda::~Propaganda() {
  log_info(PPPLAY_TAG"%s", __FUNCTION__);
  if (ppplayer_) {
    delete ppplayer_;
  }
  if (listener_) {
    delete listener_;
  }
  if (play_next_thread_) {
    play_next_thread_->interrupt();
    play_next_thread_->join();
    delete play_next_thread_;
  }
}

Propaganda* Propaganda::get_instance() {
  if (!propaganda_) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (!propaganda_) {
      propaganda_ = new Propaganda();
    }
  }
  return propaganda_;
}

void Propaganda::playNextThread() {
  boost::unique_lock<boost::mutex> play_next_lock(Propaganda::play_next_mutex_, boost::defer_lock);
  boost::unique_lock<boost::mutex> play_interval_lock(Propaganda::play_interval_mutex_, boost::defer_lock);
  while (true) {
    play_next_lock.lock();
    Propaganda::play_next_cond_.wait(play_next_lock);
    play_next_lock.unlock();

    play_interval_lock.lock();
    //log_info("%s(%d):play_interval_time_=%d", __FUNCTION__, __LINE__, play_interval_time_);
    if (play_interval_time_ > 0 && play_interval_flag_ && !play_interval_pause_) {
      play_interval_cond_.timed_wait(play_interval_lock, boost::get_system_time() + boost::posix_time::seconds(play_interval_time_));
    }
    if (play_interval_flag_ && play_times_ != 0) {
      if (play_times_ > 0) {
        play_times_--;
      }
      doPlayNext();
    }
    play_interval_lock.unlock();
  }
}

void Propaganda::doRequestMusicTransport(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  Json::Value response;
  response["id"] = values["content"]["id"];
  response["timestamp"] = values["content"]["timestamp"];
  response["result"] = "fail_invalid_data";

  if (!values["content"]["list"].isNull() && values["content"]["list"].isArray()) {
    int size = values["content"]["list"].size();
    if (size > 0) {
      for (int i = 0; i < size; i++) {
        Json::Value value = values["content"]["list"][i];
        if (!value["name"].isNull() && !value["url"].isNull() ) {
          char **result, **result1;
          int row, column, row1, column1;

          boost::unique_lock<boost::mutex> lock(Propaganda::mutex_);
          boost::shared_ptr<PPPlayDownload> obj = boost::shared_ptr<PPPlayDownload> (new PPPlayDownload());
          obj->local_path = PPPLAY_DIR + value["name"].asString();
          obj->remote_path = value["url"].asString();
          obj->name = value["name"].asString();
          obj->origin = msg;
          
          response["url"] = value["url"];
          response["name"] = value["name"];
          response["progress"] = 100;
          response["status"] = "finished";
          SqliteEngine::query("SELECT name, path FROM ppplay WHERE name='"+value["name"].asString()
            +"' AND url='"+value["url"].asString() +"'", &result, &row, &column);
          if (row > 0) {
            std::string path = result[column + 1];
            if(access(path.c_str(), F_OK) !=0 ) {
              TransferFile::download(obj);
            } else {
              response["result"] = "success";
              Utils::get_instance()->responseResult(msg, response, "response_music_transport");
            }
          } else {
            SqliteEngine::query("SELECT * FROM ppplay", &result1, &row1, &column1);
            if (row1 < PPPLAY_OUT_LIMITS) {
              TransferFile::download(obj);
            } else {
              SqliteEngine::freeQuery(result1);
              response["result"] = "fail_out_limits";
              Utils::get_instance()->responseResult(msg, response, "response_music_transport");
            }
          }
          SqliteEngine::freeQuery(result);
        } else {
          Utils::get_instance()->responseResult(msg, response, "response_music_transport");
        }
      }
    } else {
      Utils::get_instance()->responseResult(msg, response, "response_music_transport");
    }
  } else {
    Utils::get_instance()->responseResult(msg, response, "response_music_transport");
  }
}

void Propaganda::doRequestMusicList(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  char **result;
  int row, column, columnIdx;
  Json::Value list, response;

  boost::unique_lock<boost::mutex> lock(Propaganda::mutex_);
  SqliteEngine::query("SELECT name, url FROM ppplay", &result, &row, &column);
  columnIdx = column;
  list.resize(0);
  for (int i = 0; i < row; i++) {
    Json::Value ppgd;
    ppgd["name"] = result[columnIdx];
    ppgd["url"] = result[columnIdx + 1];
    list.append(ppgd);
    columnIdx += column;
  }

  response["id"] = values["content"]["id"];
  response["timestamp"] = values["content"]["timestamp"];
  response["list"] = list;
  response["result"] = "success";
  Utils::get_instance()->responseResult(msg, response, "response_music_list");
  SqliteEngine::freeQuery(result);
}

void Propaganda::doRequestMusicPlay(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  Json::Value response;
  response["id"] = values["content"]["id"];
  response["timestamp"] = values["content"]["timestamp"];
  response["result"] = "fail_invalid_data";

  atris_msgs::GetVoiceChatStatus status;
  voice_chat_status_srv_client_.call(status);

  if (!status.response.chated && !values["content"]["action"].isNull()) {
    std::string action = values["content"]["action"].asString();
    if (action == "play") {
      doPlay(msg, values);
    } else if (action == "pause") {
      doPause(msg, values);
    } else if (action == "resume") {
      doResume(msg, values);
    } else if (action == "stop") {
      doStop(msg, values);
    } else {
      Utils::get_instance()->responseResult(msg, response, "response_music_play");
    }
  } else {
    if (status.response.chated) {
      response["result"] = "fail_voice_chated";
      Utils::get_instance()->responseResult(msg, response, "response_music_play");
    } else {
      Utils::get_instance()->responseResult(msg, response, "response_music_play");
    }
  }
}

void Propaganda::responseMusicPlay(std::string result, std::string action, 
  std::string status, std::string msgID, int64_t timestamp, bool notify) {
  Json::Value response;
  response["id"] = msgID;
  response["timestamp"] = timestamp;
  response["name"] = current_play_;
  response["action"] = action;
  response["status"] = status;
  response["position"] = pts_;
  response["duration"] = duration_;
  response["result"] = result;
  Utils::get_instance()->responseResult(current_play_origin_, response, (notify ? "notify_music_play" :  "response_music_play"));
}

void Propaganda::doRename(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  Json::Value response;
  response["id"] = values["content"]["id"];
  response["timestamp"] = values["content"]["timestamp"];
  response["result"] = "fail_invalid_data";
  if (!values["content"]["name"].isNull() && !values["content"]["rename"].isNull()
      && !values["content"]["rename"].asString().empty()) {
    SqliteEngine::execSQL("UPDATE ppplay SET name='"+values["content"]["rename"].asString()
        +"' WHERE name='"+values["content"]["name"].asString()+"'");
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_music_rename");
  } else {
    Utils::get_instance()->responseResult(msg, response, "response_music_rename");
  }
}

int Propaganda::randPickFunc(int list_size)
{
  int size = list_size;
  int tmp;

  srand((int)time(0));  // generate randome seed
  tmp = rand()%size;
  return tmp;
} 


void Propaganda::doPlayNext() 
{
  if(getPlayOwner() == PC)
  {
    if (play_recycle_ && current_list_.size() <= 0) 
    {
      for (std::size_t i = 0; i < play_list_.size(); i++) 
      {
        current_list_.push_back(play_list_[i]);
      }
    }
  }
  else if(getPlayOwner() == NAV)
  {
    int nav_play_mode = getPlayMode();
    if (play_recycle_ && current_list_.size() <= 0 && (nav_play_mode == 1 || nav_play_mode == 2)) 
    {
      for (std::size_t i = 0; i < play_list_.size(); i++) 
      {
        current_list_.push_back(play_list_[i]);
      }
    }
    else if(play_recycle_ && current_list_.size() <= 0 && nav_play_mode == 3)
    {
      int size = play_list_.size();
      int j = randPickFunc(size);
      current_list_.push_back(play_list_[j]);
    }
  }


  if (current_list_.size() > 0) {
    std::string url = current_list_.front();
    current_list_.pop_front();
    current_play_ = basename((char*)url.c_str());

    started_ = false;

    ppplayer_->setDataSource(url);
    ppplayer_->prepare();
    ppplayer_->start();
    notifyPPPlayEvent(getPlayOwner(),1,getCurPlay());

  }
}

void Propaganda::doPlay(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  shm::Robot shmrbt;
  shm::iMemory_read_Robot(&shmrbt);
  bool play_next = true;
  Json::Value response;
  response["id"] = values["content"]["id"];
  response["timestamp"] = values["content"]["timestamp"];
  response["result"] = "fail_invalid_data";

  if (values["content"]["play_list"].isArray() && values["content"]["play_list"].size() > 0) {
    boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
    std::string mode = values["content"]["mode"].asString();
    if (mode == "recycle") {
      play_recycle_ = true;
    } else if (mode == "recycle_no"){
      play_recycle_ = false;
    } else {
      Utils::get_instance()->responseResult(msg, response, "response_music_play");
      return;
    }

    if (!values["content"]["play_interval"].isNull()) {
      play_interval_time_ = values["content"]["play_interval"].asInt();
      log_info("play interval time = %d\r\n",play_interval_time_);
    } else {
      play_interval_time_ = shmrbt.appdata.play_interval;
    }

    if (!values["content"]["times"].isNull()) {
      play_times_ = values["content"]["times"].asInt();
      log_info("play times = %d\r\n",play_times_);
    } else {
      play_times_ = -1;
    }

    int size = values["content"]["play_list"].size();
    log_info("---------- %s, size = %d\r\n",__FUNCTION__, size);
    play_list_.clear();
    current_list_.clear();
    for (int i = 0; i < size; i++) {
      char **result;
      int row, column;
      std::string name = values["content"]["play_list"][i].asString();

      boost::unique_lock<boost::mutex> lock(Propaganda::mutex_);
      SqliteEngine::query("SELECT path FROM ppplay WHERE name='"+name+"'", &result, &row, &column);
      if (row > 0) {
        std::string path = result[column];
        play_list_.push_back(path);

        if (i > 0) {
          current_list_.push_back(path);
        } else {
          if (ppplayer_->isPlaying() && (current_play_ == name)) {
            play_next = false;
          } else {
            current_list_.push_back(path);
          }
        }
      }
      SqliteEngine::freeQuery(result);
    }

    setPlayOwner(PC);

    if (play_list_.size() > 0) {
      play_interval_flag_ = true;
      play_interval_pause_ = false;
      if (play_next) 
      {
        if(ppplayer_->isPlaying())
        {
          ppplayer_->stop();
          notifyPPPlayEvent(PC,0, getCurPlay());
        }

        if (play_times_ > 0) {
          play_times_--;
        }
        listener_->need_resp_ = true;
        listener_->origin_ = msg;
        doPlayNext();
      } else {
        listener_->origin_ = msg;
        responseMusicPlay("success", "play", "playing", values["content"]["id"].asString(), values["content"]["timestamp"].asInt64());
      }
    } else {
      response["result"] = "fail_no_found";
      Utils::get_instance()->responseResult(msg, response, "response_music_play");
    }


  } else {
    Utils::get_instance()->responseResult(msg, response, "response_music_play");
  }

  
}

void Propaganda::doResume(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  std::string result = "success";

  boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
  if (ppplayer_->isPlaying() ||  play_interval_flag_) {
    if (ppplayer_->isPlaying()) {
      ppplayer_->resume();
    } else {
      play_interval_pause_ = false;
    }
  } else {
    result = "fail_no_play";
  }
  responseMusicPlay(result, "resume", "", values["content"]["id"].asString(), values["content"]["timestamp"].asInt64());
}

void Propaganda::doPause(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  std::string result = "success";

  boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
  if (ppplayer_->isPlaying() || play_interval_flag_) {
    if (ppplayer_->isPlaying()) {
      ppplayer_->pause();
    } else {
      play_interval_pause_ = true;
      play_interval_cond_.notify_all();
    }
  } else {
    result = "fail_no_play";
  }
  responseMusicPlay(result, "pause", "", values["content"]["id"].asString(), values["content"]["timestamp"].asInt64());
}

void Propaganda::doStop(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  std::string result = "success";

  {
    boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
    play_interval_flag_ = false;
    play_interval_pause_ = false;
    play_interval_cond_.notify_all();
  }

  if (ppplayer_->isPlaying()) {
    ppplayer_->stop();
    play_list_.clear();
    current_list_.clear();
    notifyPPPlayEvent(PC,0,getCurPlay());
  } else {
    result = "fail_no_play";
  }
  responseMusicPlay(result, "stop", "", values["content"]["id"].asString(), values["content"]["timestamp"].asInt64());
}

void Propaganda::doRequestMusicRemove(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
  Json::Value response;
  response["id"] = msg.msgID;
  response["timestamp"] = msg.timestamp;
  response["result"] = "success";

  if (values["content"]["list"].isArray()) {
    char **result;
    int row, column, size;
    std::string name;

    size = values["content"]["list"].size();

    boost::unique_lock<boost::mutex> lock(Propaganda::mutex_);
    for (int i = 0; i < size; i++) {
      name = values["content"]["list"][i].asString();
      SqliteEngine::query("SELECT path FROM ppplay WHERE name='"+name+"'", &result, &row, &column);
      if (row > 0) {
        remove(result[column]);
        SqliteEngine::execSQL("DELETE FROM ppplay WHERE name='"+name+"'");
      }
      SqliteEngine::freeQuery(result);

    }
    Utils::get_instance()->responseResult(msg, response, "response_music_remove");
  } else {
    response["result"] = "fail_invalid_data";
    Utils::get_instance()->responseResult(msg, response, "response_music_remove");
  }
}

void Propaganda::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
  Json::Reader reader;
  Json::Value root;
  if (msg.title == "request_music_transport") {
    current_play_origin_ = msg;
    reader.parse(msg.msg, root);
    doRequestMusicTransport(msg, root);
  } else if (msg.title == "request_music_list") {
    current_play_origin_ = msg;
    reader.parse(msg.msg, root);
    doRequestMusicList(msg, root);
  } else if (msg.title == "request_music_play") {
    current_play_origin_ = msg;
    reader.parse(msg.msg, root);
    doRequestMusicPlay(msg, root);
  } else if (msg.title == "request_music_remove") {
    current_play_origin_ = msg;
    reader.parse(msg.msg, root);
    doRequestMusicRemove(msg, root);
  } else if (msg.title == "request_music_rename") {
    current_play_origin_ = msg;
    reader.parse(msg.msg, root);
    doRename(msg, root);
  }
}

bool Propaganda::doGetPPPlayingStatus(atris_msgs::GetPPPlayingStatus::Request& req,
  atris_msgs::GetPPPlayingStatus::Response& res) {
    boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
    res.status = 0; // idle status
    res.pts = 0;
    res.duration = 0;
    if (play_interval_flag_) {
        res.pts = pts_;
        res.duration = duration_;
        if (ppplayer_->isPlaying() || play_interval_pause_) {
            std::string url = (current_list_.size()>0?current_list_.front():(play_list_.size()>0?play_list_.front():""));
            res.status = (ppplayer_->isPaused() || play_interval_pause_) ? 2 : 1;
            res.name = ppplayer_->isPlaying() ? current_play_.c_str() : basename((char*)url.c_str());
        } else {
            // waitting status
            res.status = 3;
            res.name = current_play_.c_str();
        }
    } else {
        res.name = "";
    }
    res.owner = getPlayOwner();

    return true;
}

bool Propaganda::doPPPlayerControl(atris_msgs::PPPlayerControl::Request& req, 
  atris_msgs::PPPlayerControl::Response& res) {
  ros::Time now = ros::Time::now();
  std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
  res.result = true;
  
  switch(req.play_control) {
    case atris_msgs::PPPlayerControl::Request::PPPLAYER_PLAY: {
      res.result = false;
      if (!req.play_list_json.empty()) {
        Json::Reader reader;
        Json::Value root;
        if (reader.parse(req.play_list_json, root)) {
          if (root.isArray() && root.size() > 0) {
            std::vector<std::string> play_list;
            for (int i = 0; i < root.size(); i++) {
              std::string name = root[i].asString();
              play_list.push_back(name);
            }
            
            play(play_list, req.play_loop, req.play_mode);
            res.result = true;
          } else {
            log_error(PPPLAY_TAG"%s play_list invalid: %s", req.play_list_json.c_str());
          }
        } else {
          log_error(PPPLAY_TAG"%s play_list parse failed: %s", req.play_list_json.c_str());
        }
      } else {
        log_error(PPPLAY_TAG"%s play_list is empty");
      }
      break;
    }
    case atris_msgs::PPPlayerControl::Request::PPPLAYER_PAUSE: {
      pause();
      log_warn("****************************** pause **************************************\r\n");
      responseMusicPlay("success", "pause", "", listener_->origin_.msgID, listener_->origin_.timestamp, true);
      break;
    }
    case atris_msgs::PPPlayerControl::Request::PPPLAYER_RESUME: {
      resume();
      log_warn("****************************** resume **************************************\r\n");
      responseMusicPlay("success", "resume", "",  listener_->origin_.msgID, listener_->origin_.timestamp, true);
      break;
    }
    case atris_msgs::PPPlayerControl::Request::PPPLAYER_STOP: {
      stop();
      break;
    }

    default: {
      res.result = false;
      break;
    }
  }

  return true;
}

void Propaganda::randListGen(std::vector<std::string> &play_list_tmp, std::vector<std::string> &play_list_rand, int list_size)
{
  int size = list_size;
  int tmp = 0;
  //std::size_t i = 0;
  log_info("list size = %d",size);
  srand((int)time(0));  // generate randome seed
  while(size!=0)
  {
    log_info("rand list gen");
    tmp = rand()%size;
    log_info("rand num gen , num = %d , size = %d",tmp, size);
    play_list_rand.push_back(play_list_tmp[tmp]);
    std::vector<std::string>::iterator itr = play_list_tmp.begin()+tmp;
    play_list_tmp.erase(itr);
    size--;
    //i++;
  }
  
} 

/****************************************************************/

void Propaganda::play(const std::vector<std::string> &play_list, bool loop, int mode) 
{
  size_t list_size = -1;
  std::vector<std::string> play_list_tmp;
  std::vector<std::string> play_list_rand;

  //{
  boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
  play_interval_flag_ = true;
  play_interval_pause_ = false;
  play_times_ = -1;
  play_interval_time_ = 0;
  play_interval_cond_.notify_all();
  //}

  log_info("------------------------- %s play list size = %d , mode = %d\r\n",__FUNCTION__, play_list.size(), mode);

  if (ppplayer_->isPlaying()) {
    ppplayer_->stop();
    notifyPPPlayEvent(NAV,0,getCurPlay());
  }

  if (loop) {
    play_recycle_ = true;
  } else {
    play_recycle_ = false;
  }

  if(mode == 1)
  {
    log_info("+++++++++++++++++++++ %s mode = 1, single song recycle",__FUNCTION__);
    list_size = 1;
  }
  else if(mode == 2)
  {
    log_info("++++++++++++++++++++ %s mode = 2, in order recycle", __FUNCTION__);
    list_size = play_list.size();
  }
  else
  {
    log_info("%s other mode : %d, list size = %d",__FUNCTION__, mode, play_list.size());
    list_size = play_list.size();
    play_list_tmp = play_list;
    randListGen(play_list_tmp, play_list_rand, list_size);
    log_info("rand list size = %d",play_list_rand.size());
  }

  setPlayMode(mode);
  setPlayOwner(NAV);

  play_list_.clear(); current_list_.clear();

  if (play_list.size() > 0) 
  {
      //for (std::size_t i = 0; i < play_list.size(); i++)
      for (std::size_t i = 0; i < list_size; i++)
      {
        std::string uri;
        if(mode == 1 || mode == 2)
        {
          uri = play_list[i];
        }
        else
        {
          uri = play_list_rand[i];
        }

        if (*(uri.begin()) != '/') 
        {
          uri = PPPLAY_DIR + uri;
        }
        play_list_.push_back(uri);
        current_list_.push_back(uri);
      }
    

    if (play_list_.size() > 0) {
      doPlayNext();
    }
  } else {
    log_error(PPPLAY_TAG"%s play_list is empty");
  }
}

bool Propaganda::isPlaying() {
  return ppplayer_->isPlaying();
}

bool Propaganda::isPaused() {
  return ppplayer_->isPaused();
}

void Propaganda::stop() {
  {
    boost::unique_lock<boost::mutex> lock(play_interval_mutex_);
    play_interval_flag_ = false;
    play_interval_pause_ = false;
    play_interval_cond_.notify_all();
  }
  if (ppplayer_->isPlaying()) 
  {
    ppplayer_->stop();
    notifyPPPlayEvent(NAV,0,getCurPlay());
  }
}

void Propaganda::pause() {
  ppplayer_->pause();
}

void Propaganda::resume() {
  ppplayer_->resume();
}

void Propaganda::notifyPPPlayEvent(int ctrl_source, int action, std::string cur_play)
{
  ros::Time now = ros::Time::now();
  Json::Value content;
  Json::Value evt_arr;
  Json::Value pp_event_msg;
  Json::FastWriter writer;
  std::string event_content = "";

  shm::Robot shmrbt;
  shm::iMemory_read_Robot(&shmrbt);
  std::string accid = std::string(shmrbt.robot.accid);

  if(action == 1)
  {
    char rand_str[16] = {0};
    Utils::get_instance()->getRandStr(rand_str);
    pp_serial_num_ = rand_str;
  }

  content["category"] = "custom_event"; // custom event

  // eventId according to message type
  content["eventId"] = "propaganda_event";

  // accid
  content["userId"] = accid;
  content["recordedAt"] = (uint64_t)(now.toSec());
  content["segmentation"] = "";

  content["customSegmentation"]["status"] = action;

  content["customSegmentation"]["serialNum"] = pp_serial_num_;
  //content["customSegmentation"]["cur_play"] = cur_play;
  if(ctrl_source == PC)
  {
    content["customSegmentation"]["source"] = 2;
  }
  else
  {
    content["customSegmentation"]["source"] = 0;
  }
  // reason
  content["customSegmentation"]["reason"]["type"] = "plain";
  content["customSegmentation"]["reason"]["value"] = cur_play;
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

  pp_event_msg["event_message"] = evt_arr;

  // publish to http
  Utils::get_instance()->NotifyRobotStatus("notify_propaganda_event", pp_event_msg);

}
