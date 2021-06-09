#ifndef NAVIGATE_ABIRITY_H_
#define NAVIGATE_ABIRITY_H_
#include <ros/ros.h>
#include "uWS.h"
#include "navigateApi.h"
#include "atris_msgs/NavAbilityMessage.h"

enum AbilityType : int {
    request_unknow = -1,
    request_start_mapping = 0,
    request_pause_mapping,
    request_resume_mapping,
    request_stop_mapping,
    request_get_map_list,
    request_set_map,
    request_cancel_map,
    request_rename_map,
    request_del_map,
    request_del_all_map,
    request_upload_map,
    request_download_map,
    request_thransfer_map_status,
    request_get_mapStatus,
    request_start_locating,
    request_pause_locating,
    request_resume_locating,
    request_stop_locating,
    request_get_locating_status,
    request_get_locating_result,
    request_get_position,
    request_start_navigating,
    request_pause_navigating,
    request_resume_navigating,
    request_stop_navigating,
    request_get_navigating_status,
    request_get_speed,
    request_set_speed,
    request_apply_site,
    request_get_version,
    request_get_point_list,
    request_tsp_navigating,
    request_query_route,
    request_get_using_map,
};

class NavigateAbility {
public:
    virtual ~NavigateAbility();
    static NavigateAbility* get_instance() {
        static NavigateAbility singleton;
        return &singleton;
    }
    int64_t addWSSession(const uWS::WebSocket &socket);
    void removeWSSession(int64_t id);
    uWS::WebSocket* getWSSession(int64_t id) const;
    int getAbilityType(std::string title);
    void notifyMessage(const std::string &msg);
    void messageInstantReceive(const atris_msgs::NavAbilityMessage& msg);
    void messageInstantReceive(const std::string& req, std::string &resp);

private:
    NavigateApi nav_api_;
    ros::NodeHandle nh_;
    ros::Publisher navability_pub_;
    ros::Subscriber navability_sub_;
    mutable int ws_session_id_ = -1;
    mutable std::mutex ws_session_mutex_;
    mutable std::map<int64_t, uWS::WebSocket> ws_session_map_;

private:
    NavigateAbility();
    std::string messageAbility(const atris_msgs::NavAbilityMessage& msg);
};

#endif /* AISOUND_TTS_H_ */

