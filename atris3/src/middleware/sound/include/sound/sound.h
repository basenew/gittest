/*
 * sound.h
 *
 *  Created on: 2018-8-17
 *      Author: fupj
 */

#ifndef SOUND_H_
#define SOUND_H_
#include <boost/thread.hpp>
#include <string>

class Sound {
public:
    virtual ~Sound();
    static Sound* get_instance();
    void setVolume(int volume);
    void setMuted(bool muted);
private:
    Sound();
    static boost::mutex mutex_;
    static Sound* sound_;
    bool muted_;
    int volume_;
};

#endif /* SOUND_H_ */
