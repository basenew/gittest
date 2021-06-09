#include "AisoundTTS.h"
#include <sys/stat.h>
#include "sound/sound.h"
#include "utils/utils.h"
#include "database/sqliteengine.h"
#include "tts_strings/tts_strings.h"
#include "imemory/atris_imemory_api.h"
#include "log/log.h"

#define AISOUND_TTS_TAG           "AisoundTTS->"

/*
* rdn:           合成音频数字发音方式
* volume:        合成音频的音量
* pitch:         合成音频的音调
* speed:         合成音频对应的语速
* voice_name:    合成发音人
* sample_rate:   合成音频采样率
* text_encoding: 合成文本编码格式
*
*/
#define AISOUND_PARAMS_XIAOFENG "engine_type = local,voice_name=xiaofeng, text_encoding = UTF8, " \
    "tts_res_path = fo|/home/atris/atris_app/bin/msc/res/tts/xiaofeng.jet;fo|/home/atris/atris_app/bin/msc/res/tts/common.jet, " \
    "sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 2"
    
#define AISOUND_PARAMS_XIAOYAN "engine_type = local,voice_name=xiaoyan, text_encoding = UTF8, " \
    "tts_res_path = fo|/home/atris/atris_app/bin/msc/res/tts/xiaoyan.jet;fo|/home/atris/atris_app/bin/msc/res/tts/common.jet, " \
    "sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 2"

//合成的语音文件名称
#define AISOUND_OUTPUT_FILE "/userdata/impdata/aisound/tts_sample.wav"

//登录参数,appid与msc库绑定,请勿随意改动
#define AISOUND_LOGIN_PARAMS "appid = 5c936e37, work_dir = /userdata/impdata/aisound"

enum {TTS_SPEAKER_XIAOFENG = 0 };
enum {TTS_SPEAKER_XIAOYAN = 1 };

typedef int SR_DWORD;
typedef short int SR_WORD ;

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int				size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int				fmt_size;				// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int				samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int				avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int				data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0
};

/* 文本合成 */
static int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path)
    {
        log_error("%s params is error!\n", __FUNCTION__);
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp)
    {
        log_error("%s open %s error.\n", __FUNCTION__, des_path);
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        log_error("%s QTTSSessionBegin failed, error code: %d.\n", __FUNCTION__, ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
        log_error("%s QTTSTextPut failed, error code: %d.\n", __FUNCTION__, ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }

    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000

    while (1)
    {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
    }

    if (MSP_SUCCESS != ret)
    {
        log_error("%s QTTSAudioGet failed, error code: %d.\n", __FUNCTION__, ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        log_error("%s QTTSSessionEnd failed, error code: %d.\n", __FUNCTION__, ret);
    }

    return ret;
}



class TTSPlayerListener: public MediaPlayerListener
{
    //friend class AisoundTTS;
    virtual void notify(GstMessageType msg, int64_t ext1 = 0, int64_t ext2 = 0) {
        if (msg == GST_MESSAGE_EOS || msg == GST_MESSAGE_ERROR) {
            boost::unique_lock<boost::mutex> lock(AisoundTTS::tts_player_mutex_);
            if(AisoundTTS::isPPPlayPaused) {
                atris_msgs::PPPlayerControl ctrl;
                ctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_RESUME;
                log_info("!!!!!!!!!!!!!!!!!!!!!! aisound send resume to ppplayer module !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
                AisoundTTS::get_instance()->ppplayer_control_srv_client_.call(ctrl);
                AisoundTTS::isPPPlayPaused = false;
            }
            AisoundTTS::tts_player_cond_.notify_all();
        }
    }
};

boost::mutex AisoundTTS::mutex_;
boost::mutex AisoundTTS::tts_interrupt_mutex_;
AisoundTTS* AisoundTTS::aisound_tts_ = NULL;

boost::mutex AisoundTTS::tts_player_mutex_;
boost::condition_variable AisoundTTS::tts_player_cond_;
bool AisoundTTS::isPPPlayPaused = false;

AisoundTTS::AisoundTTS()
    : tts_priority_playing_(-1)
    , tts_spin_loop_(false)
    , lock_state_(false)
    , tts_spin_text_("")
    , tts_spin_interval_(0){
    log_info(AISOUND_TTS_TAG"%s", __FUNCTION__);
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &AisoundTTS::messageInstantReceive, this);
    aisound_tts_sub_ = nh_.subscribe(TOPIC_AISOUND_TTS_MESSAGE, 100, &AisoundTTS::aisoundTTSTopicCb, this);
    get_ppplaying_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPPPlayingStatus>(SRV_GET_PPPLAYING_STATUS);
    ppplayer_control_srv_client_ = nh_.serviceClient<atris_msgs::PPPlayerControl>(SRV_PPPLAYER_CONTROL);
    // mk work dir
    if(access("/userdata/impdata", 0) == -1) {
        mkdir("/userdata/impdata", 0777);
    }
    if(access("/userdata/impdata/aisound", 0) == -1) {
        mkdir("/userdata/impdata/aisound", 0777);
    }

    if(access("/userdata/impdata/aisound", 0) == -1) {
        log_error(AISOUND_TTS_TAG"%s Err work_dir: /userdata/impdata/aisound", __FUNCTION__);
    }

    // MSPLogin
    int ret = MSPLogin(NULL, NULL, AISOUND_LOGIN_PARAMS);
    msp_login_ = (ret == MSP_SUCCESS ? true : false);
    if (!msp_login_) {
        log_error(AISOUND_TTS_TAG"%s MSPLogin failed, error code: %d.\n", __FUNCTION__, ret);
    }

    tts_player_listener_ = new TTSPlayerListener();
    tts_player_ = MediaPlayer::createMediaPlayer();
    tts_player_->setListener(tts_player_listener_);
    tts_task_thread_ = new boost::thread(boost::bind(&AisoundTTS::aisoundTTSThread, this));
    tts_spin_thread_ = new boost::thread(boost::bind(&AisoundTTS::ttsSpinThread, this));
}

AisoundTTS::~AisoundTTS() {
    if (tts_task_thread_) {
        tts_task_running_ = false;
        tts_task_thread_->interrupt();
        tts_task_thread_->join();
        delete tts_task_thread_;
    }

    if (msp_login_) {
        MSPLogout();
    }
}

AisoundTTS* AisoundTTS::get_instance() {
    if (!aisound_tts_) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!aisound_tts_) {
            aisound_tts_ = new AisoundTTS();
        }
    }
    return aisound_tts_;
}


void AisoundTTS::ttsSpinThread() {
  	boost::unique_lock<boost::mutex> tts_spin_lock(tts_spin_mutex_, boost::defer_lock);
  	boost::unique_lock<boost::mutex> tts_interval_lock(tts_interval_mutex_, boost::defer_lock);
  	while (true) {
    		tts_spin_lock.lock();
    		tts_spin_cond_.wait(tts_spin_lock);

    		tts_interval_lock.lock();
    		if (tts_spin_loop_ && tts_spin_interval_ > 0) {
    			tts_interval_cond_.timed_wait(tts_interval_lock, boost::get_system_time() + boost::posix_time::seconds(tts_spin_interval_));
    		}
        if (tts_spin_loop_ && !tts_spin_text_.empty()) {
            boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_);
            atris_msgs::AisoundTTS tts;
            tts.text = tts_spin_text_;
            tts.priority = atris_msgs::AisoundTTS::PRIORITY_USER;
            tts_user_queue_.clear();
            tts_user_queue_.push_back(tts);
            tts_task_cond_.notify_all();
        }
    		tts_interval_lock.unlock();
    		tts_spin_lock.unlock();
  	}
}

void AisoundTTS::ttsSpinStop() {
    {
      boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_);
      if (tts_player_->isPlaying()) {
          boost::unique_lock<boost::mutex> lock(AisoundTTS::tts_player_mutex_);
          tts_player_->stop();
          AisoundTTS::tts_player_cond_.notify_all();
      }
      tts_low_queue_.clear();
      tts_mid_queue_.clear();
      tts_high_queue_.clear();
      tts_user_queue_.clear();
    }
    tts_spin_loop_ = false;
    tts_spin_text_ = "";
    tts_spin_interval_ = 0;
    tts_interval_cond_.notify_all();
    boost::unique_lock<boost::mutex> lock(tts_spin_mutex_);
}

void AisoundTTS::aisoundTTSTopicCb (const atris_msgs::AisoundTTS& msg) {
    boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_);
    atris_msgs::AisoundTTS tts = msg;
    
    if (tts.clear && tts_player_->isPlaying() 
        && !(tts_priority_playing_ > msg.priority)) {
        boost::unique_lock<boost::mutex> lock(AisoundTTS::tts_player_mutex_);
        tts_player_->stop();
        AisoundTTS::tts_player_cond_.notify_all();
    }
	
    if(msg.key == TTSStrings::TTS_KEY_TELECONTROL_LOCKING)
    {
        lock_state_ = true;
    }
    else if(msg.key == TTSStrings::TTS_KEY_TELECONTROL_UNLOCK)
    {
        lock_state_ = false;
    }
    
    switch (tts.priority) {
    case atris_msgs::AisoundTTS::PRIORITY_LOW: {
        if (tts.clear)
            tts_low_queue_.clear();
        tts_low_queue_.push_back(tts);
        break;
    }
    case atris_msgs::AisoundTTS::PRIORITY_MID: {
        if (tts.clear)
            tts_mid_queue_.clear();
        tts_mid_queue_.push_back(tts);
        break;
    }
    case atris_msgs::AisoundTTS::PRIORITY_HIGH: {
        if (tts.clear)
            tts_high_queue_.clear();
        tts_high_queue_.push_back(tts);
        break;
    }
    default: break;
    }

    if (tts.clear) {
        boost::unique_lock<boost::mutex> interrupt_lock(tts_interrupt_mutex_);
        tts_interrupt_flg_ = false;
    }

    tts_task_cond_.notify_all();
}

void AisoundTTS::aisoundTTSThread() {
    boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_, boost::defer_lock);
    boost::unique_lock<boost::mutex> interrupt_lock(AisoundTTS::tts_interrupt_mutex_,boost::defer_lock);
    tts_task_running_ = true;
    while(tts_task_running_) {
        lock.lock();
        if (!tts_user_queue_.size() && !tts_high_queue_.size() 
          && !tts_mid_queue_.size() && !tts_low_queue_.size()) {
            tts_task_cond_.wait(lock);
        }

        atris_msgs::AisoundTTS tts;
        if (tts_user_queue_.size() > 0) {
            tts_priority_playing_ = atris_msgs::AisoundTTS::PRIORITY_USER;
            tts = tts_user_queue_.front();
            tts_user_queue_.pop_front();
        } else if (tts_high_queue_.size() > 0) {
            tts_priority_playing_ = atris_msgs::AisoundTTS::PRIORITY_HIGH;
            tts = tts_high_queue_.front();
            tts_high_queue_.pop_front();
        } else if (tts_mid_queue_.size() > 0) {
            tts_priority_playing_ = atris_msgs::AisoundTTS::PRIORITY_MID;
            tts = tts_mid_queue_.front();
            tts_mid_queue_.pop_front();
        } else if (tts_low_queue_.size() > 0) {
            tts_priority_playing_ = atris_msgs::AisoundTTS::PRIORITY_LOW;
            tts = tts_low_queue_.front();
            tts_low_queue_.pop_front();
        }
        
        if(tts_priority_playing_ == atris_msgs::AisoundTTS::PRIORITY_LOW && lock_state_ == true)
        {
            lock.unlock();
            continue;
        }
        
        
        interrupt_lock.lock();
        tts_interrupt_flg_ = true;
        interrupt_lock.unlock();

        lock.unlock();

        shm::Robot shmrbt;
        shm::iMemory_read_Robot(&shmrbt);
        if (!tts.text.empty() && shmrbt.appdata.tts_enable) {
            if (!msp_login_) {
                int ret = MSPLogin(NULL, NULL, AISOUND_LOGIN_PARAMS);
                msp_login_ = (ret == MSP_SUCCESS ? true : false);
                if (!msp_login_) {
                    log_error(AISOUND_TTS_TAG"%s MSPLogin failed, error code: %d.\n", __FUNCTION__, ret);
                }
            }

            if (msp_login_) {
                int ret = 0;
                int tts_speaker = shmrbt.appdata.tts_speaker;
            
                log_info(AISOUND_TTS_TAG"%s 正在合成 ...", __FUNCTION__);
                log_debug(AISOUND_TTS_TAG"%s tts.text: %s, tts.speaker: %d", __FUNCTION__, tts.text.c_str(), tts_speaker);
                
                if (tts_speaker == TTS_SPEAKER_XIAOFENG) {
                  ret = text_to_speech(tts.text.c_str(), AISOUND_OUTPUT_FILE, AISOUND_PARAMS_XIAOFENG);
                } else if (tts_speaker == TTS_SPEAKER_XIAOYAN) {
                  ret = text_to_speech(tts.text.c_str(), AISOUND_OUTPUT_FILE, AISOUND_PARAMS_XIAOYAN);
                } else {
                  ret = text_to_speech(tts.text.c_str(), AISOUND_OUTPUT_FILE, AISOUND_PARAMS_XIAOFENG);
                }
                
                
                //boost::unique_lock<boost::mutex> lock(tts_interrupt_mutex_,boost::defer_lock);
                interrupt_lock.lock();
                if(tts_interrupt_flg_ == false && tts_priority_playing_ != atris_msgs::AisoundTTS::PRIORITY_HIGH && tts_priority_playing_ != atris_msgs::AisoundTTS::PRIORITY_USER)
                {
                    log_warn("tts interrupt , text is %s",tts.text.c_str());
                    interrupt_lock.unlock();
                    continue;
                }
                else
                {
                    log_info("tts not interrupted, text is %s , priority is %d",tts.text.c_str(), tts_priority_playing_);
                }
                                

                if (ret == MSP_SUCCESS) {
                    boost::unique_lock<boost::mutex> lock(AisoundTTS::tts_player_mutex_);
                    atris_msgs::GetPPPlayingStatus status;
                    get_ppplaying_status_srv_client_.call(status);
                    if(status.response.status == 1) {
                        atris_msgs::PPPlayerControl ctrl;
                        ctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_PAUSE;
                        log_info("!!!!!!!!!!!!!!!!!!!!!!!! aisound send pause to ppplayer!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
                        ppplayer_control_srv_client_.call(ctrl);
                        if (ctrl.response.result) {
                          AisoundTTS::isPPPlayPaused = true;
                        }
                    }
                    
                    tts_player_->setDataSource(AISOUND_OUTPUT_FILE);
                    tts_player_->prepare();
                    tts_player_->start();
                    interrupt_lock.unlock();
                    AisoundTTS::tts_player_cond_.wait(lock);
                } else {
                    log_error(AISOUND_TTS_TAG"%s text_to_speech failed, error code: %d.\n", __FUNCTION__, ret);
                    interrupt_lock.unlock();
                }
                log_info(AISOUND_TTS_TAG"%s 合成完毕\n\n", __FUNCTION__);
            }

            if (tts_priority_playing_ == atris_msgs::AisoundTTS::PRIORITY_USER) {
                if (tts_spin_loop_) {
                    boost::unique_lock<boost::mutex> lock(tts_spin_mutex_);
                    tts_spin_cond_.notify_all();
                } else {
                    tts_spin_text_ = "";
                }
            }

            tts_priority_playing_ = -1;
        }
    }
}

void AisoundTTS::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
    Json::Reader reader;
    Json::Value root, response;
    if (msg.title == "request_tts_play_start") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "fail_invalid_data";
        
        shm::Robot shmrbt;
        shm::iMemory_read_Robot(&shmrbt);
        if (shmrbt.appdata.tts_enable) {
            if (!root["content"]["text"].isNull() && !root["content"]["interval"].isNull()
                && !root["content"]["loop"].isNull()) {
                {
                    ttsSpinStop();
                }
                
                int interval = root["content"]["interval"].asInt();
                tts_spin_text_ = root["content"]["text"].asString();
                tts_spin_interval_ = interval >= 0 ? interval : 0;
                tts_spin_loop_ = root["content"]["loop"].asInt() != 0;
                if (!tts_spin_text_.empty()) {
                    boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_);
                    atris_msgs::AisoundTTS tts;
                    tts.text = tts_spin_text_;
                    tts.priority = atris_msgs::AisoundTTS::PRIORITY_USER;
                    tts_user_queue_.clear();
                    tts_user_queue_.push_back(tts);
                    tts_task_cond_.notify_all();
                    response["result"] = "success";
                } else {
                    response["result"] = "fail_text_empty";
                }
            }
        } else {
            response["result"] = "fail_tts_disable";
        }
        
        Utils::get_instance()->responseResult(msg, response, "response_tts_play_start");
    } else if (msg.title == "request_tts_play_stop") {
        {
            ttsSpinStop();
        }
    
        boost::unique_lock<boost::mutex> lock(AisoundTTS::mutex_);
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_tts_play_stop");
    } else if (msg.title == "request_tts_play_status") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["status"] = tts_spin_text_.empty() ? 0 : 1;
        response["text"] = tts_spin_text_;
        response["interval"] = tts_spin_text_.empty() ? 0 : tts_spin_interval_;
        response["loop"] = tts_spin_text_.empty() ? 0 : tts_spin_loop_;
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_tts_play_status");
    } else if (msg.title == "request_tts_enable") {
        reader.parse(msg.msg, root);
        if (!root["content"]["enable"].isNull()) {
            int enable = root["content"]["enable"].asInt();
            if (!enable) {
                ttsSpinStop();
            }
        }
    }
}

