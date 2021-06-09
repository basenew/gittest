#ifndef __GS_API__
#define __GS_API__

#include <json/json.h>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <sstream>
#include <random>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "config/config.h"
#include "utils/utils.h"
#include "platform/gs/GsData.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/NavToPoint.h"
#include "atris_msgs/NavToPoint2.h"
#include "atris_msgs/GetNavPointList.h"
#include "atris_msgs/GetNavPoint2List.h"
#include "atris_msgs/NavPoint.h"
#include "atris_msgs/GetNavPose.h"
#include "atris_msgs/GetGsLaserRawData.h"
#include "atris_msgs/NavAbilityMessage.h"

struct NavabilityResp {
  std::mutex mutex;
  atris_msgs::NavAbilityMessage resp;
  std::shared_ptr<std::condition_variable> cond = nullptr;
};
typedef std::shared_ptr<NavabilityResp> NavabilityRespPtr;
typedef std::shared_ptr<std::condition_variable> NavabilityCondPtr;

class GsApi
{
    private:
        GsApi();

        std::string host;
        Config *cfg;
        Utils *utils;
        boost::mutex map_mutex;
        int req_get_data(std::string &url, Json::Reader &reader,
                    Json::Value &root, std::string &resp);
        int req_get_data_only(std::string &url, Json::Reader &reader,
                    Json::Value &root, std::string &resp);
        int req_get_file(std::string &url, std::string &path);
        int req_post_data(std::string &url, std::string &param, Json::Reader &reader,
                    Json::Value &root, std::string &resp);
        int req_post_file(std::string &url, std::string &filepath, Json::Reader &reader, 
                    Json::Value &root, std::string &resp);
        int req_get_response(std::string& url);
        int req_get_response(std::string& url, Json::Value& root);
        int req_post_response(std::string& url, Json::Value& root);
        int proc_response(Json::Reader &reader, Json::Value &root, std::string &resp);
        bool do_nav_to_point_cb(atris_msgs::NavToPoint::Request& req, atris_msgs::NavToPoint::Response& res);
        bool do_nav_to_point_cb2(atris_msgs::NavToPoint2::Request& req, atris_msgs::NavToPoint2::Response& res);
        bool do_get_nav_point_list_cb(atris_msgs::GetNavPointList::Request& req, atris_msgs::GetNavPointList::Response& res);
        bool do_get_nav_point_list_cb2(atris_msgs::GetNavPoint2List::Request& req, atris_msgs::GetNavPoint2List::Response& res);
        void do_navability_sub_cb(const atris_msgs::NavAbilityMessage& msg);
        int get_navability_respresult(const atris_msgs::NavAbilityMessage &response);
        int random_char();
        std::string generate_uuid();
        
        std::map<std::string, NavabilityRespPtr> navability_resp_map_;
        std::mutex navability_resp_map_mutex_;
        ros::NodeHandle nh_;
        ros::Publisher navability_pub_;
        ros::Subscriber navability_sub_;
        ros::ServiceServer nav_to_point_srv_;
        ros::ServiceServer nav_to_point_srv2_;
        ros::ServiceServer get_nav_point_list_srv_;
        ros::ServiceServer get_nav_pose_srv_;
        ros::ServiceServer get_laser_raw_data_srv_;
    public:
        int init();
        int reset_setting();
        int get_robot_pos();
        int get_gps_data(double &latitude, double &longitude, double &altitude, int &status);
        int get_version(std::string &version);
        int move(float &linear, float &angular);
#ifdef _ATRIS_NAVX_
		int map_get(std::string &map);
#endif
        int map_get_list(std::vector<GsMap> &list);
        int map_get_path_list(const std::string& map_name, std::vector<GsPath> &path_list);
        int map_get_point_list(std::string map_name, std::vector<std::string> &point_list, int type);
        int map_get_point_list(const std::string& map_name, std::vector<GsNamePoint>& point_list, int type);
        int map_get_point_list(const std::string& map_name, std::vector<StationInfo> &point_list, int type);
        int map_get_point_list(const std::string& map_name, std::vector<GsNavPoint> &point_list, int type);
        int map_get_route_points(std::vector<GsNavPoint> &points);

        int map_get_png(std::string map);
        int map_del(std::string map);
        int map_all_del();
        int map_set(std::string map);
        int map_load(std::string map);
        int map_upload(std::string map);
        int map_download(const std::string &map, const std::string &path);
        int map_rename(std::string old_map, std::string new_map);
        int nav_task_get();
        Json::Value nav_new_path_task(std::string map_name, std::string path_name);
        Json::Value nav_new_nav_task(std::string map_name, std::string point);
        Json::Value nav_new_nav_path_task(std::string map_name, GsPath path, GsPos pos);
        int nav_task_new(std::string task_name, std::string map_name,
                std::string map_id, std::vector<Json::Value> &task_list);
        int nav_task_pause();
        int nav_relocate(double x, double y, double theta, enum GsLocateType type, const std::string &map_name, const std::string &point);
        int nav_relocate_stop();
        int nav_check_relocate_state(int& locate_status);
        int nav_task_test();
        int nav_task_resume();
        int nav_task_start(std::string &task_name, Json::Value task, std::string &map_name);
        int nav_merge_path_task_start(std::string &task_name, Json::Value task, std::string map_name, std::string map_id);
        int nav_task_stop_all();
        int nav_task_stop();
        int nav_task_is_finished(bool &is_task_finished);
        int nav_set_nav_speed(int level);
        int nav_set_path_speed(int level);
        int nav_get_pos(GsPos &pos);
        int nav_merge_path(std::string map, std::string path_name, std::vector<GsPath> path_list);
        int nav_upgrade();
        int nav_to(std::string map_name, std::string pos_name);
        //int get_tsp_navigating(std::vector<StationInfo> &point_list);
        int get_tsp_navigating(std::vector<GsNavPoint> &point_list);
        int auto_nav_to(GsPos &pos);
        int auto_nav_to(GsNavPoint &nav_pos);
        int nav_auto_charge(std::string map_name, std::string charger_name);
        int path_get(std::string map);
        int path_del(std::string map, std::string path);
        int path_info(std::string map, std::string path);
        int path_test();
        int power_off();
        int reboot();
        int get_laser_grid_data(LaserGrid &laser_grid);
        int get_laser_raw_data(LaserRaw &laser_data);
        int get_current_init_status(std::string &currMap, std::string &currInitPoint);
        int get_device_status(Json::Value &status);
        int get_init_status(const std::string& map);
        int set_time();
        bool get_laser_raw_data(atris_msgs::GetGsLaserRawData::Request& req, atris_msgs::GetGsLaserRawData::Response& res);
        bool nav_get_pos(atris_msgs::GetNavPose::Request& req, atris_msgs::GetNavPose::Response& res);
        bool is_initialize_success();
        int check_nav_conditions();
        bool check_network(std::string dest_ip, int timeout = 30);
        static GsApi* get_instance(){
            static GsApi singleton;
            return &singleton;
        }
};

#endif
