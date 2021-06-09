#include <signal.h>
#include "gaussian.h"
#include <curl/curl.h>
#include "map_manager.h"
#include "scheme_manager.h"
#include "log/log.h"
#include "imemory/atris_imemory_api.h"

int main(int argc, char *argv[]) {
    int ret = 0;
    ros::init(argc, argv, "navigation");
    tinyros::init("navigation");

    if (!Config::get_instance()->nav_enable){
        log_info("nav is disable");
        return ret;
    }
    
    shm::iMemory_init();

    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);

    log_info("Init Gaussian api");
    ret = GsApi::get_instance()->init();
    if(ret != ERR_OK){
        log_error("gaussian api init fail.");
    }else{
        log_error("gaussian api init success.");
    }

    log_info("Init Gaussian");
    Gaussian *gs = Gaussian::get_instance();
    gs->init();
    if(ret < 0){
        log_error("gaussian module init fail.");
    }

    log_info("Init Charger");
    Charger::get_instance()->init();
    nav::MapManager::get_instance().init();
    nav::Navigation::get_instance().init();
    nav::SchemeManager::get_instance().init();
    if(ret < 0){
        log_error("patrol module init fail.");
    }
    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    curl_global_cleanup();
    return ret;
}

