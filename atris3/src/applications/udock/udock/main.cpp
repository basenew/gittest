#include "ros/ros.h"
#include "tiny_ros/ros.h"
#include "log/log.h"
#include <signal.h>
#include <curl/curl.h>
#include "config/config.h"
#include "udock_manager.h"
#include "imemory/atris_imemory_api.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udock");
    tinyros::init("udock");
    int ret = 0;
    
    shm::iMemory_init();

    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);

    Config *cfg = Config::get_instance();
    
    log_info("UdockTankManager");
    UdockTankManager *udock_manager = UdockTankManager::get_instance();
    udock_manager->init();
#ifdef _CHASSIS_MARSHELL_
    udock_manager->move_control->init();
#endif
    log_info("udock module init sucess-----------------!");
    log_info("wall width config is %f !", udock_manager->udock->wall_width_);

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    curl_global_cleanup();

    return 0;
}


