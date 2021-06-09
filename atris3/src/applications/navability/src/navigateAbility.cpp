#include "navigateAbility.h"
#include <json/json.h>
#include "ros/ros.h"
#include "log/log.h"

#undef TOPIC_NAV_ABILITY_REQUEST_MESSAGE
#undef TOPIC_NAV_ABILITY_RESPONSE_MESSAGE
#define TOPIC_NAV_ABILITY_REQUEST_MESSAGE  "/topic/nav/ability/request/message"
#define TOPIC_NAV_ABILITY_RESPONSE_MESSAGE  "/topic/nav/ability/response/message"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_abt]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_abt]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_abt]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_abt]"#format, ##args)

static std::map<std::string, int> abirity_type_map = {
    {"request_start_mapping", AbilityType::request_start_mapping},
    {"request_pause_mapping", AbilityType::request_pause_mapping},
    {"request_resume_mapping", AbilityType::request_resume_mapping},
    {"request_stop_mapping", AbilityType::request_stop_mapping},
    {"request_get_map_list", AbilityType::request_get_map_list},
    {"request_set_map", AbilityType::request_set_map},
    {"request_cancel_map", AbilityType::request_cancel_map},
    {"request_rename_map", AbilityType::request_rename_map},
    {"request_del_map", AbilityType::request_del_map},
    {"request_del_all_map", AbilityType::request_del_all_map},
    {"request_upload_map", AbilityType::request_upload_map},
    {"request_download_map", AbilityType::request_download_map},
    {"request_thransfer_map_status", AbilityType::request_thransfer_map_status},
    {"request_get_mapStatus", AbilityType::request_get_mapStatus},
    {"request_start_locating", AbilityType::request_start_locating},
    {"request_pause_locating", AbilityType::request_pause_locating},
    {"request_resume_locating", AbilityType::request_resume_locating},
    {"request_stop_locating", AbilityType::request_stop_locating},
    {"request_get_locating_status", AbilityType::request_get_locating_status},
    {"request_get_locating_result", AbilityType::request_get_locating_result},
    {"request_get_position", AbilityType::request_get_position},
    {"request_start_navigating", AbilityType::request_start_navigating},
    {"request_pause_navigating", AbilityType::request_pause_navigating},
    {"request_resume_navigating", AbilityType::request_resume_navigating},
    {"request_stop_navigating", AbilityType::request_stop_navigating},
    {"request_get_navigating_status", AbilityType::request_get_navigating_status},
    {"request_get_speed", AbilityType::request_get_speed},
    {"request_set_speed", AbilityType::request_set_speed},
    {"request_apply_site", AbilityType::request_apply_site},
    {"request_get_version", AbilityType::request_get_version},
    {"request_get_point_list", AbilityType::request_get_point_list},
    {"request_tsp_navigating", AbilityType::request_tsp_navigating},
    {"request_query_route", AbilityType::request_query_route},
    {"request_get_using_map", AbilityType::request_get_using_map},
};


std::string NavigateAbility::messageAbility(IN const atris_msgs::NavAbilityMessage& msg) {
    std::string resp;
    Linfo("NavigateAbility::messageAbility title: %s", msg.title.c_str());
    switch(NavigateAbility::getAbilityType(msg.title)) {
#if 0
        case AbilityType::request_start_mapping: {
            Linfo("start mapping");
            nav_api_.startMapping(msg.msg, resp);
            Linfo("start mapping ok");
        }
        break;
        
        case AbilityType::request_pause_mapping: {
            Linfo("pause mapping");
            nav_api_.pauseMapping(msg.msg, resp);
            Linfo("pause mapping ok");
        }
        break;
        
        case AbilityType::request_resume_mapping: {
            Linfo("resume mapping");
            nav_api_.resumeMapping(msg.msg, resp);
            Linfo("resume mapping ok");
        }
        break;

        case AbilityType::request_stop_mapping: {
            Linfo("stop mapping");
            nav_api_.stopMapping(msg.msg, resp);
            Linfo("stop mapping ok");
        }
        break;
#endif

        case AbilityType::request_get_map_list: {
            Linfo("get maps");
            nav_api_.getMapLists(msg.msg, resp);
            Linfo("get maps ok");
        }
        break;

        case AbilityType::request_set_map: {
            Linfo("set map");
            nav_api_.setMap(msg.msg, resp);
            Linfo("set map ok");
        }
        break;

        case AbilityType::request_cancel_map: {
            Linfo("cancel map");
            nav_api_.cancelMap(msg.msg, resp);
            Linfo("cancel map ok");
        }
        break;

        case AbilityType::request_rename_map: {
            Linfo("rename map");
            nav_api_.renameMap(msg.msg, resp);
            Linfo("rename map ok");
        }
        break;

        case AbilityType::request_del_map: {
            Linfo("del map");
            nav_api_.delMap(msg.msg, resp);
            Linfo("del map ok");
        }
        break;

        case AbilityType::request_del_all_map: {
            Linfo("del all map");
            nav_api_.delAllMap(msg.msg, resp);
            Linfo("del all map ok");
        }
        break;

        case AbilityType::request_upload_map: {
            Linfo("upload map");
            nav_api_.upLoadMap(msg.msg, resp);
            Linfo("upload map ok");
        }
        break;

        case AbilityType::request_download_map: {
            Linfo("download map");
            nav_api_.downloadMap(msg.msg, resp);
            Linfo("download map ok");
        }
        break;

        case AbilityType::request_thransfer_map_status: {
            Linfo("transfer map");
            nav_api_.getTransferMapStatus(msg.msg, resp);
            Linfo("transfer map ok");
        }
        break;

        case AbilityType::request_get_mapStatus: {
            Linfo("get map status");
            nav_api_.getMapStatus(msg.msg, resp);
            Linfo("get map status ok");
        }
        break;

        case AbilityType::request_start_locating: {
            Linfo("start locating");
            nav_api_.startRelocating(msg.msg, resp);
            Linfo("start locating ok");
        }
        break;

        case AbilityType::request_pause_locating: {
            Linfo("pause locating");
            nav_api_.pauseReLocating(msg.msg, resp);
            Linfo("pause locating ok");
        }
        break;

        case AbilityType::request_resume_locating: {
            Linfo("resume locating");
            nav_api_.resumeReLocating(msg.msg, resp);
            Linfo("resume locating ok");
        }
        break;

        case AbilityType::request_stop_locating: {
            Linfo("stop locating");
            nav_api_.stopReLocating(msg.msg, resp);
            Linfo("stop locating ok");
        }
        break;

        case AbilityType::request_get_locating_status: {
            Linfo("get locating status");
            nav_api_.getRelocatingStatus(msg.msg, resp);
            Linfo("get locating status ok");
        }
        break;

        case AbilityType::request_get_locating_result: {
            Linfo("get locating result");
            nav_api_.getRelocatingResult(msg.msg, resp);
            Linfo("get locating result ok");
        }
        break;

        case AbilityType::request_get_position: {
            Linfo("get position");
            nav_api_.getRobotPosition(msg.msg, resp);
            Linfo("get position ok");
        }
        break;

        case AbilityType::request_start_navigating: {
            Linfo("start navigating");
            nav_api_.startNavigation(msg.msg, resp);
            Linfo("start navigating ok");
        }
        break;

        case AbilityType::request_pause_navigating: {
            Linfo("pause navigating");
            nav_api_.pauseNavigating(msg.msg, resp);
            Linfo("pause navigating ok");
        }
        break;

        case AbilityType::request_resume_navigating: {
            Linfo("resume navigating");
            nav_api_.resumeNavigating(msg.msg, resp);
            Linfo("resume navigating ok");
        }
        break;

        case AbilityType::request_stop_navigating: {
            Linfo("stop navigating");
            nav_api_.stopNavigating(msg.msg, resp);
            Linfo("stop navigating ok");
        }
        break;

        case AbilityType::request_get_navigating_status: {
            Linfo("get navigating status");
            nav_api_.getNavistatus(msg.msg, resp);
            Linfo("get navigating status ok");
        }
        break;

        case AbilityType::request_get_speed: {
            Linfo("get speed");
            nav_api_.getSpeed(msg.msg, resp);
            Linfo("get speed ok");
        }
        break;

        case AbilityType::request_set_speed: {
            Linfo("set speed");
            nav_api_.setSpeed(msg.msg, resp);
            Linfo("set speed ok");
        }
        break;

        case AbilityType::request_apply_site: {
            nav_api_.applySite(msg.msg, resp);
        }
        break;

        case AbilityType::request_get_version: {
            nav_api_.getVersion(msg.msg, resp);
        }
        break;

        case AbilityType::request_get_point_list: {
            nav_api_.getPointLists(msg.msg, resp);
        }
        break;

        case AbilityType::request_tsp_navigating: {
            nav_api_.getTspNavigating(msg.msg, resp);
        }
        break;

        case AbilityType::request_query_route: {
            nav_api_.queryRoute(msg.msg, resp);
        }
        break;

        case AbilityType::request_get_using_map: {
            nav_api_.getUsingMap(msg.msg, resp);
        }
        break;

        case AbilityType::request_unknow: {
            Lwarn("NavigateAbility::messageAbility request_unknow!!!");
        }
        break;

        default: break;
    }

    return resp;
}


//////////////////////////////////////////////////////////////////////////////////////////////////

NavigateAbility::NavigateAbility() {
    navability_sub_ = nh_.subscribe(TOPIC_NAV_ABILITY_REQUEST_MESSAGE, 100, &NavigateAbility::messageInstantReceive, this);
    navability_pub_ = nh_.advertise<atris_msgs::NavAbilityMessage>(TOPIC_NAV_ABILITY_RESPONSE_MESSAGE, 100);

    int ret = nav_api_.init();
    if (ret){
        Lerror("Init navigateApi err: %d", ret);
    }
}

NavigateAbility::~NavigateAbility() {
  
}

void NavigateAbility::messageInstantReceive (const atris_msgs::NavAbilityMessage& msg) {
    std::string resp = messageAbility(msg);
    if (!resp.empty()) {
        Json::Reader reader;
        Json::Value root;
        if (reader.parse(resp, root)) {
            atris_msgs::NavAbilityMessage respmsg;
            respmsg.title = (!root["title"].isNull()) ? root["title"].asString() : "";
            respmsg.id = (!root["content"].isNull() && !root["content"]["id"].isNull()) ? root["content"]["id"].asString() : "";
            respmsg.timestamp = (!root["content"].isNull() && !root["content"]["timestamp"].isNull()) ? root["content"]["timestamp"].asLargestInt() : 0;
            respmsg.msg = resp;
            navability_pub_.publish(respmsg);
        } else {
            Lerror("NavigateAbility::messageInstantReceive0 response json parse failed!");
        }
    } else {
        Lwarn("NavigateAbility::messageInstantReceive0 response message empty!");
    }
    
}

void NavigateAbility::messageInstantReceive(const std::string& req, std::string &resp) {
    if (!req.empty()) {
        Json::Reader reader;
        Json::Value root;
        if (reader.parse(req, root)) {
            atris_msgs::NavAbilityMessage reqmsg;
            reqmsg.title = (!root["title"].isNull()) ? root["title"].asString() : "";
            reqmsg.id = (!root["content"].isNull() && !root["content"]["id"].isNull()) ? root["content"]["id"].asString() : "";
            reqmsg.timestamp = (!root["content"].isNull() && !root["content"]["timestamp"].isNull()) ? root["content"]["timestamp"].asLargestInt() : 0;
            reqmsg.msg = req;
            resp = messageAbility(reqmsg);
            if (resp.empty()) {
                Lwarn("NavigateAbility::messageInstantReceive1 response message empty!");
            }
        } else {
            Lerror("NavigateAbility::messageInstantReceive1 Json parse failed!");
        }
    } else {
        Lwarn("NavigateAbility::messageInstantReceive1 empty message!");
    }
}

void NavigateAbility::notifyMessage(const std::string &msg) {
    if (!msg.empty()) {
        Json::Reader reader;
        Json::Value root;
        if (reader.parse(msg, root)) {
            atris_msgs::NavAbilityMessage ntfmsg;
            ntfmsg.title = (!root["title"].isNull()) ? root["title"].asString() : "";
            ntfmsg.id = (!root["content"].isNull() && !root["content"]["id"].isNull()) ? root["content"]["id"].asString() : "";
            ntfmsg.timestamp = (!root["content"].isNull() && !root["content"]["timestamp"].isNull()) ? root["content"]["timestamp"].asLargestInt() : 0;
            ntfmsg.msg = msg;
            navability_pub_.publish(ntfmsg);
        
            std::unique_lock<std::mutex> lock(ws_session_mutex_);
            for(auto it : ws_session_map_) {
                it.second.send((char*)msg.data(), msg.size(), uWS::TEXT);
            }
        } else {
            Lerror("NavigateAbility::notifyMessage Json parse failed!");
        }
    } else {
        Lwarn("NavigateAbility::notifyMessage empty message!");
    }
    
}

int NavigateAbility::getAbilityType(std::string title) {
    return (abirity_type_map.count(title) ? abirity_type_map[title] : AbilityType::request_unknow);
}

int64_t NavigateAbility::addWSSession(const uWS::WebSocket &socket) {
    std::unique_lock<std::mutex> lock(ws_session_mutex_);
    ws_session_map_[++ws_session_id_] = socket;
    return ws_session_id_;
}

void NavigateAbility::removeWSSession(int64_t id) {
    std::unique_lock<std::mutex> lock(ws_session_mutex_);
    if (ws_session_map_.count(id) > 0) {
        ws_session_map_.erase(id);
    }
}

uWS::WebSocket* NavigateAbility::getWSSession(int64_t id) const {
    std::unique_lock<std::mutex> lock(ws_session_mutex_);
    if (ws_session_map_.count(id) > 0) {
        return &ws_session_map_[id];
    }
    return nullptr;
}


