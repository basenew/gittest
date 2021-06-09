/*
 * mediaplayer.cpp
 *
 *  Created on: 2018-7-3
 *      Author: fupj
 */

#include "mediaplayer.h"
#include "sound/sound.h"
#include "log/log.h"

#define MEDIAPLAYER_TAG   "MediaPlayer->"
#define VOLUME_MAX_DOUBLE 1.0

const char* MediaPlayer::stateToString() {
    switch(state_) {
    case MEDIA_PLAYER_STATE_ERROR: return "MEDIA_PLAYER_STATE_ERROR";
    case MEDIA_PLAYER_IDLE: return "MEDIA_PLAYER_IDLE";
    case MEDIA_PLAYER_INITIALIZED: return "MEDIA_PLAYER_INITIALIZED";
    case MEDIA_PLAYER_PREPARING: return "MEDIA_PLAYER_PREPARING";
    case MEDIA_PLAYER_PREPARED: return "MEDIA_PLAYER_PREPARED";
    case MEDIA_PLAYER_STARTED: return "MEDIA_PLAYER_STARTED";
    case MEDIA_PLAYER_PAUSED: return "MEDIA_PLAYER_PAUSED";
    case MEDIA_PLAYER_STOPPED: return "MEDIA_PLAYER_STOPPED";
    case MEDIA_PLAYER_PLAYBACK_COMPLETE: return "MEDIA_PLAYER_PLAYBACK_COMPLETE";
    default: return "UNKNOW_STATE";
    }
}

gboolean MediaPlayer::playerBusMsg(GstBus * bus, GstMessage * msg, gpointer user_data) {
    MediaPlayer *player = (MediaPlayer *)user_data;
    switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS: {
        log_info(MEDIAPLAYER_TAG"%s End Of Streamer...", __FUNCTION__);
        boost::unique_lock<boost::mutex> lock(player->mutex_);
        player->state_ = MEDIA_PLAYER_PLAYBACK_COMPLETE;
        gst_element_set_state(player->play_pipe_, GST_STATE_NULL);
        break;
    }
    case GST_MESSAGE_ERROR: {
        GError *error;
        gst_message_parse_error(msg, &error, NULL);
        if (error && error->message) {
            log_error(MEDIAPLAYER_TAG"%s %s", __FUNCTION__, error->message);
            g_clear_error (&error);
        }
        boost::unique_lock<boost::mutex> lock(player->mutex_);
        gst_element_set_state(player->play_pipe_, GST_STATE_NULL);
        player->state_ = MEDIA_PLAYER_IDLE;
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError *error;
        gst_message_parse_warning (msg, &error, NULL);
        if (error && error->message) {
            log_warn(MEDIAPLAYER_TAG"%s %s", __FUNCTION__, error->message);
            g_clear_error (&error);
        }
        break;
    }
    case GST_MESSAGE_STATE_CHANGED: {
        /* we only care about pipeline state change messages */
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT_CAST (player->play_pipe_)) {
            GstState old, current, pending;
            gst_message_parse_state_changed (msg, &old, &current, &pending);
            log_info(MEDIAPLAYER_TAG"%s GST_MESSAGE_STATE_CHANGED: %s->%s", __FUNCTION__,
                     gst_element_state_get_name (old), gst_element_state_get_name (current));
        }
        break;
    }

    default: { break; }
    }

    // ----------------------------------------------------------------------------
    // callbacks for application
    if (player->listener_) {
        boost::unique_lock<boost::mutex> lock(player->listener_mutex_);
        if (player->listener_) {
            player->listener_->notify(GST_MESSAGE_TYPE (msg));
        }
    }

    return TRUE;
}

gboolean MediaPlayer::positionNotify(gpointer user_data) {
    MediaPlayer *player = (MediaPlayer *)user_data;
    if (player->state_ == MEDIA_PLAYER_STARTED) {
        gint64 pos,len;
        GstFormat fm = GST_FORMAT_TIME;
        gst_element_query_position(player->play_pipe_, fm, &pos);
        gst_element_query_duration(player->play_pipe_, fm, &len);
        if (player->listener_ && len > 0 && pos > 0 && len >= pos) {
            boost::unique_lock<boost::mutex> lock(player->listener_mutex_);
            if (player->listener_) {
                player->listener_->notify(GST_MESSAGE_PROGRESS, (int64_t)(pos / GST_MSECOND),  (int64_t)(len / GST_MSECOND));
            }
        }
    }
    return TRUE;
}

MediaPlayer* MediaPlayer::createMediaPlayer() {
    log_debug(MEDIAPLAYER_TAG"%s ", __FUNCTION__);
    return new MediaPlayer();
}

void MediaPlayer::startMainLoop() {
    g_main_loop_run(main_loop_);
    log_warn(MEDIAPLAYER_TAG"%s g_main_loop_run quit!", __FUNCTION__);
}

MediaPlayer::MediaPlayer()
    : listener_(NULL), duration_(0), pts_(0),
      main_loop_(NULL), main_loop_thread_(NULL), play_pipe_(NULL) {
    log_debug(MEDIAPLAYER_TAG"%s", __FUNCTION__);
    gst_init(NULL, NULL);
}

MediaPlayer::~MediaPlayer() {
    log_debug(MEDIAPLAYER_TAG"%s begin", __FUNCTION__);
    boost::unique_lock<boost::mutex> lock(mutex_);
    destoryMediaPlayer();
    log_debug(MEDIAPLAYER_TAG"%s end", __FUNCTION__);
}


bool MediaPlayer::initMediaPlayer() {
    log_debug(MEDIAPLAYER_TAG"%s begin", __FUNCTION__);

    duration_ = pts_ = 0;

    destoryMediaPlayer();

    main_loop_ = g_main_loop_new(NULL,FALSE);
    if (main_loop_) {
        main_loop_thread_ = new boost::thread(boost::bind(&MediaPlayer::startMainLoop, this));
        play_pipe_ = gst_element_factory_make("playbin","play-pipe");
        if (play_pipe_) {
            bus_watch_ = gst_bus_add_watch (GST_ELEMENT_BUS (play_pipe_), &MediaPlayer::playerBusMsg, this);
            timeout_ = g_timeout_add(100, &MediaPlayer::positionNotify, this);
            g_object_set(play_pipe_, "volume", 1.0, NULL);
            gst_element_set_state (play_pipe_, GST_STATE_NULL);
            state_ = MEDIA_PLAYER_INITIALIZED;
        } else {
            log_error(MEDIAPLAYER_TAG"%s couldn't create pipeline", __FUNCTION__);
            state_ = MEDIA_PLAYER_STATE_ERROR;
        }
    } else {
        log_error(MEDIAPLAYER_TAG"%s couldn't create mainloop", __FUNCTION__);
        state_ = MEDIA_PLAYER_STATE_ERROR;
    }
    log_debug(MEDIAPLAYER_TAG"%s end", __FUNCTION__);

    return ((state_ != MEDIA_PLAYER_STATE_ERROR) ? true : false);
}

void MediaPlayer::destoryMediaPlayer() {
    if (!main_loop_ && !main_loop_thread_ && !play_pipe_ && !main_loop_) {
        return;
    }

    log_info(MEDIAPLAYER_TAG"%s step1: quit main loop", __FUNCTION__);
    if (main_loop_) {
        g_main_loop_quit(main_loop_);
    }

    log_info(MEDIAPLAYER_TAG"%s step2: quit main thread", __FUNCTION__);
    if (main_loop_thread_) {
        main_loop_thread_->interrupt();
        main_loop_thread_->timed_join(boost::posix_time::milliseconds(500));
        delete main_loop_thread_;
        main_loop_thread_ = NULL;
    }

    log_info(MEDIAPLAYER_TAG"%s step3: release play pipe", __FUNCTION__);
    if (play_pipe_) {
        gst_element_set_state(play_pipe_, GST_STATE_NULL);
        g_source_remove(bus_watch_);
        g_source_remove(timeout_);
        gst_object_unref(play_pipe_);
        play_pipe_ = NULL;
    }

    if (main_loop_) {
        g_main_loop_unref(main_loop_);
        main_loop_ = NULL;
    }
    log_info(MEDIAPLAYER_TAG"%s step4: destoryMediaPlayer end!", __FUNCTION__);
}

void MediaPlayer::setDataSource(const std::string& url) {
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (initMediaPlayer()) {
        std::string uri = url;
        if (*(uri.begin()) == '/') {
            uri = "file://" + uri;
        }
        gst_element_set_state (play_pipe_, GST_STATE_READY);
        g_object_set (play_pipe_, "uri", uri.c_str(), NULL);
        state_ = MEDIA_PLAYER_PREPARING;
    }
}

void MediaPlayer::start() {
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (state_ == MEDIA_PLAYER_PREPARED) {
            gst_element_set_state (play_pipe_, GST_STATE_PLAYING);
            state_ = MEDIA_PLAYER_STARTED;
        }
    }
}

void MediaPlayer::stop() {
    log_info(MEDIAPLAYER_TAG"%s begin state: %s", __FUNCTION__, MediaPlayer::stateToString());
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (state_ != MEDIA_PLAYER_STOPPED) {
        destoryMediaPlayer();
        state_ = MEDIA_PLAYER_STOPPED;
    }
    log_info(MEDIAPLAYER_TAG"%s end state: %s", __FUNCTION__, MediaPlayer::stateToString());
}

void MediaPlayer::pause() {
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (state_ == MEDIA_PLAYER_STARTED) {
            gst_element_set_state(play_pipe_, GST_STATE_PAUSED);
            state_ = MEDIA_PLAYER_PAUSED;
        }
    }
}

bool MediaPlayer::isPlaying() {
    //log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (state_ >= MEDIA_PLAYER_PREPARED && state_ <= MEDIA_PLAYER_PAUSED) {
            return true;
        }
    }
    return false;
}

bool MediaPlayer::isPaused() {
    //log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        return (state_ != MEDIA_PLAYER_PAUSED ? false : true);
    }
    return false;
}

int64_t MediaPlayer::getCurrentPosition() {
    GstFormat fm = GST_FORMAT_TIME;
    int64_t pos = 0;
    gst_element_query_position(play_pipe_, fm, &pos);
    return (int64_t)(pos / GST_MSECOND);
}

int64_t MediaPlayer::getDuration() {
    GstFormat fm = GST_FORMAT_TIME;
    int64_t len = 0;
    gst_element_query_duration(play_pipe_, fm, &len);
    return (int64_t)(len / GST_MSECOND);
}

void MediaPlayer::prepare() {
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (state_ == MEDIA_PLAYER_PREPARING) {
            gst_element_set_state(play_pipe_, GST_STATE_PAUSED);
            state_ = MEDIA_PLAYER_PREPARED;
        }
    }
}

void MediaPlayer::resume() {
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (state_ == MEDIA_PLAYER_PAUSED) {
            gst_element_set_state(play_pipe_, GST_STATE_PLAYING);
            state_ = MEDIA_PLAYER_STARTED;
        }
    }
}

void MediaPlayer::setListener(MediaPlayerListener* listener) {
    boost::unique_lock<boost::mutex> lock(listener_mutex_);
    listener_ = listener;
}

MediaPlayerStates MediaPlayer::getMediaPlayerState() {
    return state_;
}

void MediaPlayer::setVolume(int volume) {
    if (volume >= 0 && volume <= 100) {
        g_object_set(play_pipe_, "volume", (double)(volume*VOLUME_MAX_DOUBLE/100), NULL);
    }
}

void MediaPlayer::setMute(bool mute) {
    g_object_set(play_pipe_, "mute", mute, NULL);
}

int MediaPlayer::getVolume() {
    double volume = 0;
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        g_object_get(play_pipe_, "volume", &volume, NULL);
    }
    return (int)(volume*100/VOLUME_MAX_DOUBLE);
}

bool MediaPlayer::getMute() {
    bool mute = FALSE;
    log_debug(MEDIAPLAYER_TAG"%s state: %s", __FUNCTION__, MediaPlayer::stateToString());
    if (state_ != MEDIA_PLAYER_STATE_ERROR) {
        g_object_get(play_pipe_, "mute", &mute, NULL);
    }
    return mute;
}

std::string MediaPlayer::getCurrentPlayUri() {
    gchar *tmp = NULL;
    std::string uri = "";
    g_object_get (play_pipe_, "current-uri", &tmp, NULL);
    if (tmp) {
        uri = tmp; g_free(tmp);
    }
    return uri;
}
