/*
 * mediaplayer.h
 *
 *  Created on: 2018-7-3
 *      Author: fupj
 */

#ifndef MEDIAPLAYER_H_
#define MEDIAPLAYER_H_
#include <boost/thread.hpp>
#include <glib-2.0/glib.h>
#include <gstreamer-1.0/gst/gst.h>

enum MediaPlayerStates {
    MEDIA_PLAYER_STATE_ERROR        = 0,
    MEDIA_PLAYER_IDLE               = 1 << 0,
    MEDIA_PLAYER_INITIALIZED        = 1 << 1,
    MEDIA_PLAYER_PREPARING          = 1 << 2,
    MEDIA_PLAYER_PREPARED           = 1 << 3,
    MEDIA_PLAYER_STARTED            = 1 << 4,
    MEDIA_PLAYER_PAUSED             = 1 << 5,
    MEDIA_PLAYER_STOPPED            = 1 << 6,
    MEDIA_PLAYER_PLAYBACK_COMPLETE  = 1 << 7
};

// ----------------------------------------------------------------------------
// callbacks for application
class MediaPlayerListener
{
public:
    virtual void notify(GstMessageType msg, int64_t ext1 = 0, int64_t ext2 = 0) = 0;
};

class MediaPlayer {
public:
    virtual ~MediaPlayer();
    static MediaPlayer* createMediaPlayer();
    void setDataSource(const std::string& url);
    void start();
    void stop();
    void pause();
    bool isPlaying();
    bool isPaused();
    int64_t getCurrentPosition();
    int64_t getDuration();
    void setVolume(int volume);
    int getVolume();
    void setMute(bool mute);
    bool getMute();
    void prepare();
    void resume();
    void  setListener(MediaPlayerListener* listener);
    MediaPlayerStates  getMediaPlayerState();
    std::string getCurrentPlayUri();

private:
    static gboolean playerBusMsg(GstBus * bus, GstMessage * msg, gpointer user_data);
    static gboolean positionNotify(gpointer user_data);
    const char* stateToString();
    void startMainLoop();
    bool initMediaPlayer();
    void destoryMediaPlayer();
    MediaPlayer();

    GMainLoop* main_loop_;
    boost::thread* main_loop_thread_;
    GstElement *play_pipe_;
    guint bus_watch_;
    guint timeout_;
    int duration_;
    int pts_;
    boost::mutex mutex_;
    boost::mutex listener_mutex_;
    MediaPlayerStates state_;
    MediaPlayerListener* listener_;
};

#endif /* MEDIAPLAYER_H_ */
