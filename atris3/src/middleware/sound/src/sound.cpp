/*
 * sound.cpp
 *
 *  Created on: 2018-8-17
 *      Author: fupj
 */

#include "sound/sound.h"
#include <libgen.h>
#include "log/log.h"

#define SOUND_TAG  "Sound->"

boost::mutex Sound::mutex_;
Sound* Sound::sound_ = NULL;

Sound* Sound::get_instance() {
    if (!sound_) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!sound_) {
            sound_ = new Sound();
        }
    }
    return sound_;
}

Sound::Sound()
  : muted_(false),
    volume_(0) {
    
}

Sound::~Sound() {
    
}

void Sound::setVolume(int volume) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    char strcmd[100] = {0};
    FILE *fp = NULL;
    volume_ = volume;
    snprintf(strcmd, sizeof(strcmd)-1, "amixer set Master %d%%", volume_);
    if((fp = popen(strcmd, "r"))) {
        log_info(SOUND_TAG"%s setVolume(%s)", __FUNCTION__, strcmd);
        pclose(fp);
    } else {
        log_error(SOUND_TAG"%s setVolume(%s) failded", __FUNCTION__, strcmd);
    }
}

void Sound::setMuted(bool muted) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    muted_ = muted;
    FILE *fp = NULL;
    std::string strcmd = muted_ ? "amixer sset Master mute" : "amixer sset Master unmute";
    if((fp = popen(strcmd.c_str(), "r"))) {
        log_info(SOUND_TAG"%s setMute(%d)", __FUNCTION__, muted_);
        pclose(fp);
    } else {
        log_error(SOUND_TAG"%s setMute(%d) failded", __FUNCTION__, muted_);
    }
}

