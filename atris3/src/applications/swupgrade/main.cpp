#include <signal.h>
#include <curl/curl.h>
#include "swupgrade.h"
#include <libgen.h>
#include "tiny_ros/ros/time.h"
#include "sound/sound.h"
#include "firmware.h"
#include "database/sqliteengine.h"
#include "sound/sound.h"
#include "tts_strings/tts_strings.h"
#include "transferfile/transferfile.h"
#include "task_manager/task_manager.h"
#include "imemory/atris_imemory_api.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "swupgrade");
    tinyros::init("swupgrade");
    ros::Time::init();
    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);
    shm::iMemory_init();
    SWUpgrade::get_instance();
    ros::MultiThreadedSpinner s(10);
    ros::spin(s);
    curl_global_cleanup();

    return 0;
}