#include <ros/ros.h>
#include <tiny_ros/ros.h>
#include "visionability.h"
#include <signal.h>
#include <curl/curl.h>
#include "imemory/atris_imemory_api.h"
#include "log/log.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "visionability");
    tinyros::init("visionability");
    shm::iMemory_init();

    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);
    
    VisionAbility::get_instance();
  	ros::MultiThreadedSpinner s(10);
  	ros::spin(s);
    
    curl_global_cleanup();

    return 0;
}
