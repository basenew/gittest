#include <boost/thread/thread.hpp>
#include "gs_api.h"
#include "nav_error_code.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <string>
#include <time.h>
#include <chrono>

#define ATRIS_LIDAR_IP "192.168.1.200"
#define ATRIS_CHASSIS_IP "10.20.18.10"

#define NAVABILITY_PACKAGE_BEGIN(_title_) \
{ \
    Json::FastWriter jwriter; \
    Json::Value root; \
    request.title = _title_; \
    request.id = generate_uuid(); \
    request.timestamp = (uint64_t)(ros::Time::now().toSec() * 1000); \
    root["title"] = request.title; \
    root["content"]["id"] = request.id; \
    root["content"]["timestamp"] = request.timestamp;     
#define NAVABILITY_PACKAGE_END(_duration_) \
    request.msg = jwriter.write(root); \
    std::unique_lock<std::mutex> map_lock(navability_resp_map_mutex_); \
    navability_resp_map_[request.id] = NavabilityRespPtr(new NavabilityResp()); \
    navability_resp_map_[request.id]->cond = NavabilityCondPtr(new std::condition_variable()); \
    map_lock.unlock(); \
    std::unique_lock<std::mutex> resp_lock(navability_resp_map_[request.id]->mutex); \
    navability_pub_.publish(request); \
    if (_duration_ > 0) { \
        if (navability_resp_map_[request.id]->cond->wait_until(resp_lock, std::chrono::system_clock::now() + \
            std::chrono::milliseconds(_duration_ * 1000)) == std::cv_status::timeout) { \
            log_warn("GsApi[%s] timeout!!!", request.title.c_str()); \
        } \
    } else { \
        navability_resp_map_[request.id]->cond->wait(resp_lock); \
    } \
    if (navability_resp_map_[request.id]->resp.id == request.id) { \
        response = navability_resp_map_[request.id]->resp; \
    } \
    resp_lock.unlock(); \
    map_lock.lock(); \
    navability_resp_map_.erase(request.id); \
    map_lock.unlock(); \
}


GsApi::GsApi() {
   // nav_to_point_srv_ = nh_.advertiseService(SRV_NAV_TO_POINT, &GsApi::do_nav_to_point_cb, this);
    nav_to_point_srv2_ = nh_.advertiseService(SRV_NAV_TO_POINT, &GsApi::do_nav_to_point_cb2, this);
    get_nav_point_list_srv_ = nh_.advertiseService(SRV_GET_NAV_POINT_LIST, &GsApi::do_get_nav_point_list_cb, this);
    get_nav_pose_srv_ = nh_.advertiseService(SRV_GET_NAV_POSE, &GsApi::nav_get_pos, this);
    get_laser_raw_data_srv_ = nh_.advertiseService(SRV_GET_GS_LASER_RAW_DATA, &GsApi::get_laser_raw_data, this);
    navability_pub_ = nh_.advertise<atris_msgs::NavAbilityMessage>(TOPIC_NAV_ABILITY_REQUEST_MESSAGE, 100);
    navability_sub_ = nh_.subscribe(TOPIC_NAV_ABILITY_RESPONSE_MESSAGE, 100, &GsApi::do_navability_sub_cb, this);
    cfg = Config::get_instance();
    utils = Utils::get_instance();
    host = "http://" + cfg->gs_ip + ":" + std::to_string(cfg->gs_web_port);
}

int GsApi::init()
{
    log_info("%s", __FUNCTION__);

#ifndef _ATRIS_NAVX_
    if (cfg->nav_enable){
        int ret;
        int try_times;
        std::string version;

        try_times = Config::get_instance()->nav_init_timeout;
        if (try_times <= 0) try_times = 180;

        log_info("%s is enable init timeout:%d", __FUNCTION__, try_times);
        while ((ret = GsApi::get_instance()->get_version(version)) != ERR_OK && --try_times){
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (ret != ERR_OK){
            log_error("gs service is not ready to run!!!");
        }

        return ret;
    }else{
        log_error("nav is disable!!!");
        return ERR_OK;
    }
#endif
    return ERR_OK;
}

int GsApi::random_char() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    return dis(gen);
}

std::string GsApi::generate_uuid() {
    std::stringstream ss;
    for (int i = 0; i < 16; i++) {
        int rc = random_char();
        std::stringstream hexstream;
        hexstream << std::hex << rc;
        std::string hex = hexstream.str();
        ss << (hex.length() < 2 ? std::string("0") + hex : hex);
    }
    return ss.str();
}

void GsApi::do_navability_sub_cb(const atris_msgs::NavAbilityMessage& msg) {
    std::unique_lock<std::mutex> map_lock(navability_resp_map_mutex_);
    if (navability_resp_map_.count(msg.id) > 0) {
        std::unique_lock<std::mutex> resp_lock(navability_resp_map_[msg.id]->mutex);
        navability_resp_map_[msg.id]->resp = msg;
        navability_resp_map_[msg.id]->cond->notify_all();
    }
}


bool GsApi::do_nav_to_point_cb2(atris_msgs::NavToPoint2::Request& req,
  atris_msgs::NavToPoint2::Response& res) {
    GsNavPoint nav_pos;
    nav_pos.id = req.id;
    nav_pos.np.name = req.name;
    nav_pos.mode = req.mode;
    nav_pos.np.pos.x = req.x;
    nav_pos.np.pos.y = req.y;
    nav_pos.np.pos.angle = req.yaw;
    auto_nav_to(nav_pos);
    res.result = true;
    return true;
}

bool GsApi::do_nav_to_point_cb(atris_msgs::NavToPoint::Request& req,
  atris_msgs::NavToPoint::Response& res) {
    std::string posName = "";
    std::vector<GsMap> list;
    std::string mapName = "";
    std::string initPointName = "";
    bool ret = false;
    ret = get_current_init_status(mapName, initPointName);
    if (ret) {
        log_info("get mapName failed");
        return false;
    }
    
    log_info("mapName is %s", mapName.c_str());
    std::vector<GsNamePoint> pointList;
    ret = map_get_point_list(mapName, pointList, GS_NAV_POINT);
    if (ret) {
        log_info("map_get_point_list failed\n");
        return false;
    }

    unsigned int i = 0;
    int xPos = req.x;
    int yPos = req.y;
    const char *delim = ",";
    std::string::size_type idx = req.name.find(delim);
    if (idx != std::string::npos) {
      for (i = 0; i < pointList.size(); i++) {
          if (pointList[i].pos.x == xPos && pointList[i].pos.y == yPos) {
              posName = pointList[i].name;
              break;
          }
      }
      if (i == pointList.size()) {
          log_info("location pos is not right");
          return false;
      }
    } else {
        for (i = 0; i < pointList.size(); i++) {
            if (pointList[i].name == req.name) {
                posName = pointList[i].name;
                break;
            }
        }
        if (i == pointList.size()) {
            log_info("location name is not right");
            return false;
        } 
    }
    
    log_info("mapName[%s],posName[%s]", mapName.c_str(), posName.c_str());
    nav_to(mapName, posName);
    res.result = true;
    return true;
}

bool GsApi::do_get_nav_point_list_cb2(atris_msgs::GetNavPoint2List::Request& req,
    atris_msgs::GetNavPoint2List::Response& res) {
    std::vector<GsNavPoint> pointList;
    std::string mapName = "";
    std::string initPointName = "";
    bool ret = false;

    get_current_init_status(mapName, initPointName);
    
    if (ret) {
        log_info("get mapName failed");
    }

    if (map_get_point_list(mapName, pointList, StationInfo::StationType::EXACT_STATION)) {
        if (pointList.size() > 0) {
            res.points.resize(pointList.size());
            for (unsigned int i = 0; i < pointList.size(); i++) {
                res.points[i].name = pointList[i].np.name;
                res.points[i].id = pointList[i].id;
                res.points[i].x = pointList[i].np.pos.x;
                res.points[i].y = pointList[i].np.pos.y;
                res.points[i].yaw = pointList[i].np.pos.angle;
            }
        }
    } else {
        log_info("map_get_point_list failed\n");
    }

    return true;
}

bool GsApi::do_get_nav_point_list_cb(atris_msgs::GetNavPointList::Request& req,
  atris_msgs::GetNavPointList::Response& res) {
  std::vector<GsMap> list;
  std::vector<GsNamePoint> pointList;
  std::string mapName = "";
  std::string initPointName = "";
  bool ret = false;

  get_current_init_status(mapName, initPointName);
  
  if (ret) {
      log_info("get mapName failed");
  }
  
  if (!map_get_point_list(mapName, pointList, GS_NAV_POINT)) {
      if (pointList.size() > 0) {
          res.points.resize(pointList.size());
          for (unsigned int i = 0; i < pointList.size(); i++) {
              res.points[i].name =  pointList[i].name;
              res.points[i].x = pointList[i].pos.x;
              res.points[i].y = pointList[i].pos.y;
              res.points[i].z = pointList[i].pos.angle;
          }
      }
  } else {
      log_info("map_get_point_list failed\n");
  }

  return true;
}

int GsApi::req_get_file(std::string &url, std::string &path)
{
    if(utils->http_get_file(url, path) != 0){
        log_error("http get file fail.");
        return ERR_HTTP_GET_FILE_FAIL;
    }

    return ERR_OK;
}

int GsApi::proc_response(Json::Reader &reader, Json::Value &root, std::string &resp)
{
    if(!reader.parse(resp.c_str(), root)){
        log_error("%s:fail parse resp json", __FUNCTION__);
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    }

    if(!root["successed"].isNull() && false == root["successed"].asBool()){
        if (!root["errorCode"].isNull())
        {
            log_error("%s:%s", root["errorCode"].asString().c_str(), root["msg"].asString().c_str());
            return get_gs_err_code(root["errorCode"].asString());
        }
        return ERR_RSP_FAIL;
    }

    return ERR_OK;

}

int GsApi::req_get_data(std::string &url, Json::Reader &reader, Json::Value &root, std::string &resp)
{
    int ret = req_get_data_only(url, reader, root, resp);
	if (ret == ERR_OK)
        return proc_response(reader, root, resp);

    return ret;
}

int GsApi::req_get_data_only(std::string &url, Json::Reader &reader, Json::Value &root, std::string &resp)
{
    //log_info("%s url:%s", __FUNCTION__, url.c_str());
    int times = 3;
    while (utils->http_get(url, resp) != 0 && times--);

    if(times == 0)
    {
        log_error("%s url:%s fail", __FUNCTION__, url.c_str());
        return ERR_HTTP_GET_FAIL;
    }

    return ERR_OK;
}

int GsApi::req_post_data(std::string &url, std::string &param,
                        Json::Reader &reader, Json::Value &root, std::string &resp)
{
    if ( utils->http_post(url, param, resp) < 0) {
        log_info("%s url:%s fail", __FUNCTION__, url.c_str());
        return ERR_HTTP_POST_FAIL;
    }

    return proc_response(reader, root, resp);
}


int GsApi::req_post_file(std::string &url, std::string &filepath, Json::Reader &reader,
                         Json::Value &root, std::string &resp)
{
    if ( utils->http_post_file(url, filepath, resp) != 0) {
        log_error("http get req fail.");
        return ERR_HTTP_POST_FILE_FAIL;
    }

    return proc_response(reader, root, resp);
}

int GsApi::get_version(std::string &version)
{
    std::string url = host + "/gs-robot/info";

    std::string resp;
    Json::Reader reader;
    Json::Value root;

    int ret = req_get_data(url, reader, root, resp);
    if (ret == ERR_OK){
        version = root["data"]["version"].asString();
    }
    return ret;
}

int GsApi::req_get_response(std::string& url, Json::Value& root)
{
    std::string resp;
    Json::Reader reader;

    return req_get_data(url, reader, root, resp);
}

int GsApi::req_get_response(std::string& url)
{
    Json::Value root;
    std::string resp;
    Json::Reader reader;

    return req_get_data(url, reader, root, resp);
}

int GsApi::get_navability_respresult(const atris_msgs::NavAbilityMessage &response) {
    Json::Value root;
    Json::Reader reader;
    if (response.msg.empty()) {
        return -1;
    }
    if (reader.parse(response.msg, root)) {
        if (!root["content"].isNull() && !root["content"]["result"].isNull()) {
            std::string result = root["content"]["result"].asString();
            return (result == "success" ? 0 : -2);
        } else {
            return -3;
        }
    } else {
        return -4;
    }
}

int GsApi::map_set(std::string map)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_set_map") { 
        root["content"]["data"] = map;
    } NAVABILITY_PACKAGE_END(15)
    
    log_info("%s resp:%s", __FUNCTION__, response.msg.c_str());
	return get_navability_respresult(response);
#else
    std::string enc_map = utils->url_encoder(map.c_str(), map.size());
    std::string url = host + "/gs-robot/cmd/use_map?map_name=" + enc_map;
	return req_get_response(url);
#endif
}

#ifdef _ATRIS_NAVX_
int GsApi::map_get(std::string &map)
{
    log_info("%s", __FUNCTION__);
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_mapStatus") {
    } NAVABILITY_PACKAGE_END(15)

	std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	if (ret != ERR_OK)
		return ret;
    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)){
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code
    }
	return ERR_OK;
}
#endif

int GsApi::map_get_list(std::vector<GsMap> &list)
{
    log_info("%s", __FUNCTION__);
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_map_list") {
    } NAVABILITY_PACKAGE_END(15)
   
	std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	if (ret != ERR_OK)
		return ret;
    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)){
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code
    }
    Json::Value map_list = root["content"]["data"]["maps"];
    Json::Value m;
    int map_size = map_list.size();
    for(int i = 0; i < map_size; i ++){
        GsMap map;
        m = map_list[i];
        map.name = m.asString();
        list.push_back(map);
        log_info("%s map:%s", __FUNCTION__, map.name.c_str());
	}
	return ret;

#else
    std::string url = host + "/gs-robot/data/maps";
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s", __FUNCTION__);
    int ret = req_get_data(url, reader, root, resp);
    if (ret != ERR_OK){
        return ret;
    }

    Json::Value map_list = root["data"];
    Json::Value m;
    int map_size = map_list.size();
    for(int i = 0; i < map_size; i ++){
        GsMap map;
        m = map_list[i];
        map.create_date = m["createAt"].asString();
        map.id = m["id"].asString();
        map.grid_h = m["mapInfo"]["gridHeight"].asInt();
        map.grid_w = m["mapInfo"]["gridWidth"].asInt();
        map.origin_x = m["mapInfo"]["originX"].asFloat();
        map.origin_y = m["mapInfo"]["originY"].asFloat();
        map.resolution = m["mapInfo"]["resolution"].asDouble();
        map.name = m["name"].asString();
        list.push_back(map);
#if 0
        log_info("date:%s name:%s id:%s reso:%lf",
                map.create_date.c_str(), map.name.c_str(), map.id.c_str(), map.resolution);
        log_info("grid_h:%d  grid_w:%d", map.grid_h, map.grid_w);
        log_info("org_x:%f org_y:%f", map.origin_x, map.origin_y);
#endif
    }

    return ERR_OK;
#endif
}

int GsApi::map_get_point_list(std::string map_name, std::vector<std::string> &point_list, int type)
{
    std::string enc_map_name = utils->url_encoder(map_name.c_str(), map_name.size());
    std::string url = host + "/gs-robot/data/positions?map_name=" + enc_map_name + "&type=" + std::to_string(type);
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s", __FUNCTION__);
    int ret = req_get_data(url, reader, root, resp);
    if (ret != ERR_OK){
        return ret;
    }

    int point_size = root["data"].size();
    for(int i = 0; i < point_size; i ++){
        std::string p_name = root["data"][i]["name"].asString();
        log_info("init point name:%s", p_name.c_str());
        point_list.push_back(p_name);
    }

    return ERR_OK;
}

int GsApi::map_del(std::string map)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_del_map") {
        root["content"]["data"] = map;
    } NAVABILITY_PACKAGE_END(15)
    
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    std::string enc_map = utils->url_encoder(map.c_str(), map.size());
    std::string url = host + "/gs-robot/cmd/delete_map?map_name=" + enc_map;
	return req_get_response(url);
#endif
}

int GsApi::map_all_del()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_del_all_map") {
    } NAVABILITY_PACKAGE_END(15)
    
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    std::string url = host + "/gs-robot/cmd/clear";
    return req_get_response(url);
#endif
}

int GsApi::reset_setting()
{
    std::string url = host + "/gs-robot/cmd/reset_robot_setting";
    return req_get_response(url);
}

int GsApi::map_rename(std::string old_name, std::string new_name)
{
    log_info("%s %s %s", __FUNCTION__, old_name.c_str(), new_name.c_str());
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_rename_map") {
        root["content"]["origin_name"] = old_name;
        root["content"]["new_name"] = new_name;
    } NAVABILITY_PACKAGE_END(15)

    std::string resp = response.msg;
    int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    std::string enc_old_name = utils->url_encoder(old_name.c_str(), old_name.size());
    std::string enc_new_name = utils->url_encoder(new_name.c_str(), new_name.size());
    std::string url = host + "/gs-robot/cmd/rename_map?origin_map_name=" + enc_old_name
        + "&new_map_name=" + enc_new_name;
	return req_get_response(url);
#endif
}

int GsApi::map_load(std::string name)
{
    log_info("%s %s", __FUNCTION__, name.c_str());
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_set_map") {
        root["content"]["data"] = name;
    } NAVABILITY_PACKAGE_END(15)

    log_info("%s resp:%s", __FUNCTION__, response.msg.c_str());
	return get_navability_respresult(response);
#else
    std::string enc_map = utils->url_encoder(name.c_str(), name.size());
    std::string url = host + "/gs-robot/cmd/load_map?map_name=" + enc_map;
	return req_get_response(url);
#endif
}

int GsApi::map_upload(std::string name)
{
    log_info("map name:%s", name.c_str());
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_upload_map") {
        root["content"]["data"] = name;
    } NAVABILITY_PACKAGE_END(15)
    
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    std::string enc_map = utils->url_encoder(name.c_str(), name.size());
    std::string url = host + "/gs-robot/data/upload_map?map_name=" + enc_map;

    std::string resp;
    Json::Reader reader;
    Json::Value root;

    std::string filepath = cfg->maps_dir + name + ".tar.gz";

    log_info("%s: %s", __FUNCTION__, url.c_str());

    struct stat st;
    log_info("map:%s", filepath.c_str());
    if(stat(filepath.c_str(), &st) != 0){
        log_error("stat file fail:%s err:%d", filepath.c_str(), errno);
        return ERR_STAT_FILE;
    }

    return req_post_file(url,filepath,reader,root,resp);
#endif

}

int GsApi::map_download(const std::string &map, const std::string &path)
{
#ifdef _ATRIS_NAVX_
    log_info("%s map:%s", __FUNCTION__, map.c_str());
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_download_map") {
        root["content"]["data"] = map;
    } NAVABILITY_PACKAGE_END(300)
    
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    boost::lock_guard<boost::mutex> lock(map_mutex);
    std::string enc_map = utils->url_encoder(map.c_str(), map.size());
    std::string url = host + "/gs-robot/data/download_map_data?map_name=" + enc_map;

    log_info("%s:%s", __FUNCTION__, url.c_str());
    std::string map_file = path + map + ".tar.gz";
    if(0 == access(map_file.c_str(), F_OK)){
        remove(map_file.c_str());
    }
    int ret = req_get_file(url, map_file);

    return ret;
#endif
}

int GsApi::path_get(std::string map)
{
    std::string url = host + "/gs-robot/data/paths?map_name=" + map;
	return req_get_response(url);
}

int GsApi::path_del(std::string map, std::string path)
{
    std::string url = host + "/gs-robot/cmd/delete_path?map_name=" + map + "&path_name=" + path;
	return req_get_response(url);
}

int GsApi::path_info(std::string map, std::string path)
{
    std::string url = host + "/gs-robot/data/path_data_list?map_name=" + map + "&path_name=" + path;
	return req_get_response(url);
}

int GsApi::path_test()
{
    bool ret;
    std::vector<GsMap> maps;
    GsMap m;

    map_get_list(maps);
    unsigned int i = 0;
    for(; i < maps.size(); i ++){
        m = maps.at(i);
        log_info("map name:%s", m.name.c_str());
        if(m.name == "atris"){
            log_info("got map");
            break;
        }
    }

    if(maps.size() == i){
        log_info("map not found");
    }


    log_info("got map:%s", m.name.c_str());
    ret = map_load(m.name);

    ret = map_set(m.name);
    if(!ret){
        log_error("set map fail.");
        return ERR_FAIL;
    }

    path_get(m.name);

    path_info(m.name, "main_path");

    return ERR_OK;
}

int GsApi::get_gps_data(double &latitude, double &longitude, double &altitude, int &status)
{
    Json::Value root;
    std::string url = host + "/gs-robot/real_time_data/gps_raw";
	int ret = req_get_response(url, root);
	if (ret != ERR_OK)
	  return ret;

    if(root["data"]["latitude"].isNull() || root["data"]["longitude"].isNull() ||
        root["data"]["altitude"].isNull() || root["data"]["status"].isNull())
        return ERR_NONE_GPS;

    latitude = root["data"]["latitude"].asDouble();
    longitude = root["data"]["longitude"].asDouble();
    altitude = root["data"]["altitude"].asDouble();
    status = root["data"]["status"].asInt();

    return ERR_OK;
}

Json::Value GsApi::nav_new_path_task(std::string map_name, std::string path_name)
{
    Json::Value root;

    root["name"] = Json::Value("PlayPathTask");
    root["start_param"]["map_name"] = Json::Value(map_name);
    root["start_param"]["path_name"] = Json::Value(path_name);

    return  root;
}

/**
 * @brief nav_new_nav_path_task 轨道导航任务，导航到指定点
 *
 * @param map_name
 * @param path
 * @param pos
 *
 * @return
 */
Json::Value GsApi::nav_new_nav_path_task(std::string map_name, GsPath path, GsPos pos)
{
    Json::Value root;

    root["name"] = Json::Value("NavigationTask");
    root["start_param"]["map_name"] = Json::Value(map_name);
    root["start_param"]["path_name"] = Json::Value(path.name);
    root["start_param"]["path_type"] = Json::Value(0);//默认使用0
    root["start_param"]["destination"]["angle"] = Json::Value(pos.angle);
    root["start_param"]["destination"]["gridPosition"]["x"] = Json::Value(pos.x);
    root["start_param"]["destination"]["gridPosition"]["y"] = Json::Value(pos.y);

    return root;
}

int GsApi::req_post_response(std::string& url, Json::Value& root)
{
    Json::FastWriter fw;
    Json::Reader resp_reader;
    std::string resp;
    std::string req_data = fw.write(root);

    return req_post_data(url, req_data, resp_reader, root, resp);
}

int GsApi::nav_task_new(std::string task_name, std::string map_name,
        std::string map_id, std::vector<Json::Value> &task_list)
{
    std::string url = host + "/gs-robot/cmd/save_task_queue";

    //任务参数
    Json::Value root;
    root["name"] = Json::Value(task_name);
    root["map_name"] = Json::Value(map_name);
    root["map_id"] = Json::Value(map_id);
    root["loop"] = Json::Value(false);

    Json::Value t;
    int size = task_list.size();
    for(int i = 0; i < size; i ++){
        t = task_list.at(i);
        root["tasks"].append(t);
    }

	return req_post_response(url, root);
}

int GsApi::nav_relocate_stop()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_stop_locating") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
	return ret;
#else
    std::string url = host + "/gs-robot/cmd/stop_initialize";
	return req_get_response(url);
#endif
}

int GsApi::nav_relocate(double x, double y, double theta, enum GsLocateType type, const std::string &map, const std::string &point)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_start_locating") {
        root["content"]["data"]["orientation"]["yaw"] = theta;
        root["content"]["data"]["position"]["x"] = x;
        root["content"]["data"]["position"]["y"] = y;
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    if (map.empty()) {
        log_error("map is empty");
        return ERR_MAP_NOT_FOUND;
    }

    std::string map_name = map;//utils->url_encoder(map.c_str(), map.size());
    /*
    if(GS_LOCATE_NORMAL == type){
        std::string url = host + "/gs-robot/cmd/initialize?map_name=" + map_name + "&init_point_name=" + point;
		return req_get_response(url);
    }
    else*/
    {
        std::string url = host + "/gs-robot/cmd/initialize_customized";
        if(GS_LOCATE_DIRECT_CUSTMOIZED == type)
            url = host + "/gs-robot/cmd/initialize_customized_directly";
        std::string resp;
        Json::Reader reader;
        Json::Value root;

        root["mapName"] = Json::Value(map_name);
        root["point"]["angle"] = Json::Value(theta);
        root["point"]["gridPosition"]["x"] = Json::Value(x);
        root["point"]["gridPosition"]["y"] = Json::Value(y);

		return req_post_response(url, root);
    }

    return ERR_INVALID_LOCATE_TYPE;
#endif
}

int GsApi::nav_check_relocate_state(int& locate_status)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_locating_status") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());

    Json::Value root;
    Json::Reader reader;
    if(resp.empty() || !reader.parse(resp, root)){
		//todo deal with code
		return ERR_FAIL;
	}

    if (!root["content"].isNull() && !root["content"]["code"].isNull()) {
	    locate_status = root["content"]["code"].asInt();
    }
	return ret;

#else
    std::string url = host + "/gs-robot/cmd/is_initialize_finished";
    Json::Value root;
	int ret = req_get_response(url, root);
	if (ret != ERR_OK)
	  return ret;

    if(root["data"].asString() == "true"){
        locate_status = 1;
    }
    else if(root["data"].asString() == "failed"){
        locate_status = -1;
    }
    else
        locate_status = 0;

    return ERR_OK;
#endif
}

int GsApi::nav_task_start(std::string &task_name, Json::Value task, std::string &map_name)
{
    std::string url = host + "/gs-robot/cmd/start_task_queue";
    Json::Value root;
    root["name"] = Json::Value(task_name);
    root["map_name"] = Json::Value("");
    root["loop"] = Json::Value(false);
    root["loop_count"] = Json::Value(0);
    root["tasks"].append(task);
    return req_post_response(url, root);
}

int GsApi::nav_merge_path_task_start(std::string &task_name, Json::Value task, std::string map_name, std::string map_id)
{
    std::string url = host + "/gs-robot/cmd/start_task_queue";
    Json::Value root;
    root["name"] = Json::Value(task_name);
    root["map_name"] = Json::Value(map_name);
    root["map_id"] = Json::Value("");
    root["loop"] = Json::Value(false);
    root["loop_count"] = Json::Value(0);
    root["tasks"].append(task);

    return req_post_response(url, root);
}
int GsApi::nav_task_test()
{
    std::string url = host + "/gs-robot/cmd/navigate";
    Json::Value root;
    root["destination"]["angle"] = Json::Value(100);
    root["destination"]["gridPosition"]["x"] = Json::Value(100);
    root["destination"]["gridPosition"]["y"] = Json::Value(100);
    return req_post_response(url, root);
}

int GsApi::nav_task_pause()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_pause_navigating") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/pause_task_queue";
    return req_get_response(url);
#endif
}

int GsApi::nav_task_stop()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_stop_navigating") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/stop_current_task";
    return req_get_response(url);
#endif
}

int GsApi::nav_task_stop_all()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_stop_navigating") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
\
    return ERR_OK;
#else
    std::string url = host + "/gs-robot/cmd/stop_task_queue";
    return req_get_response(url);
#endif
}

int GsApi::nav_task_resume()
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_resume_navigating") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/resume_task_queue";
    return req_get_response(url);
#endif
}

int GsApi::nav_task_is_finished(bool &is_task_finished)
{
    std::string url = host + "/gs-robot/cmd/is_task_queue_finished";
    Json::Value root;
	int ret = req_get_response(url, root);
	if (ret != ERR_OK)
	  return ret;

    if(root["data"].asString() == "true") {
        is_task_finished = true;
    }
    else
        is_task_finished = false;

    return ERR_OK;
}

int GsApi::nav_upgrade()
{
    std::string url = host + "/gs-robot/system/update_system_form";
    return req_get_response(url);
}


bool GsApi::check_network(std::string dest_ip, int timeout) {
    int count = 0;
    bool bRet = false;
    while (count < timeout) {
        if (!utils->check_network_state(dest_ip.c_str())) {
            log_error("%s network unreachable",__FUNCTION__);
            count++;
        } else {
            log_info("%s network ok", __FUNCTION__);
            bRet = true;
            break;
        }
        usleep(1000*1000);
    }

    if (count == timeout) {
        log_error("%s timeout : %d", __FUNCTION__, count);
    }

    return bRet;
}

int GsApi::check_nav_conditions(){
    std::string dest_ip;
    // check lidar
    dest_ip = ATRIS_LIDAR_IP;
    check_network(dest_ip.c_str());
    if (!check_network(dest_ip.c_str())) {
        log_error("lidar ip network err.");
        return ERR_GS_LIDAR_ERR;
    }

    // check chassis
    dest_ip = ATRIS_CHASSIS_IP;
    check_network(dest_ip.c_str());
    if (!check_network(dest_ip.c_str())) {
        log_error("chassis ip network err.");
        return ERR_GS_CHASSIS_ERR;
    }
    log_debug("nav condition ready ^_^!");
    return 0;
}

int GsApi::auto_nav_to(GsNavPoint &nav_pos){
    int check_ret = check_nav_conditions();
    if (check_ret) {
		log_info("nav conditions not ready maybe chassis or lidar not ready");
		return check_ret;
	}
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_start_navigating") {
        root["content"]["data"]["goal"][0]["position"]["x"] = nav_pos.np.pos.x;
        root["content"]["data"]["goal"][0]["position"]["y"] = nav_pos.np.pos.y;
        root["content"]["data"]["goal"][0]["orientation"]["yaw"] = nav_pos.np.pos.angle;
        root["content"]["data"]["point_id"] = nav_pos.id;
        root["content"]["data"]["mode"] = nav_pos.mode;
    } NAVABILITY_PACKAGE_END(60)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("GsNavPoint %s req id: %s, ret:%d resp:%s", __FUNCTION__, nav_pos.id.c_str(), ret, resp.c_str());
    return ret;
}

int GsApi::auto_nav_to(GsPos &pos){
#ifdef _ATRIS_NAVX_
    int check_ret = check_nav_conditions();
    if (!check_ret) {return check_ret;}
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_start_navigating") {
        root["content"]["data"]["goal"][0]["position"]["x"] = pos.x;
        root["content"]["data"]["goal"][0]["position"]["y"] = pos.y;
        root["content"]["data"]["goal"][0]["orientation"]["yaw"] = pos.angle;   
    } NAVABILITY_PACKAGE_END(60)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/navigate";
    Json::Value root;
    root["destination"]["angle"] = Json::Value(pos.angle);
    root["destination"]["gridPosition"]["x"] = Json::Value(pos.x);
    root["destination"]["gridPosition"]["y"] = Json::Value(pos.y);
    return req_post_response(url, root);
#endif
}

int GsApi::nav_to(std::string map_name, std::string pos_name)
{
#ifdef _ATRIS_NAVX_
    return ERR_OK;
#else
    std::string enc_map_name = utils->url_encoder(map_name.c_str(), map_name.size());
    std::string enc_pos_name = utils->url_encoder(pos_name.c_str(), pos_name.size());
    std::string url = host + "/gs-robot/cmd/position/navigate?map_name=" + enc_map_name + "&position_name=" + enc_pos_name;
    return req_get_response(url);
#endif
}

int GsApi::nav_set_path_speed(int level)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_set_speed") {
        root["content"]["data"]["level"] = level;
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/set_speed_level?level=" + std::to_string(level);
    return req_get_response(url);
#endif
}

int GsApi::nav_set_nav_speed(int level)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_set_speed") {
        root["content"]["data"]["level"] = level;
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    return ret;
#else
    std::string url = host + "/gs-robot/cmd/set_navigation_speed_level?level=" + std::to_string(level);
    return req_get_response(url);
#endif
}

bool GsApi::nav_get_pos(atris_msgs::GetNavPose::Request& req, 
  atris_msgs::GetNavPose::Response& res) {
  GsPos pos;
  if (nav_get_pos(pos) == ERR_OK) {
    res.pos.position.x = pos.x;
    res.pos.position.y = pos.y;
    res.pos.position.z = pos.angle;
  }

  return true;
}

int GsApi::nav_get_pos(GsPos &pos)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_position") {
    } NAVABILITY_PACKAGE_END(30)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret == ERR_OK){
        Json::Reader reader;
        Json::Value root;
        if(reader.parse(resp, root)){
            if (!root["content"].isNull() && !root["content"]["data"].isNull()
            &&  !root["content"]["data"]["yaw"].isNull()
            &&  !root["content"]["data"]["x"].isNull()
            &&  !root["content"]["data"]["y"].isNull()){
                Json::Value js_pos = root["content"]["data"];
                pos.angle = js_pos["yaw"].asDouble();
                pos.x = js_pos["x"].asDouble();
                pos.y = js_pos["y"].asDouble();
            }else{
                log_error("pos json format error");
            }
        }
    }
    return ret;
#else
    std::string url = host + "/gs-robot/real_time_data/position";
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    //log_info("%s", __FUNCTION__);
    if( utils->http_get(url, resp) < 0){
        log_error("http get req fail.");
        return ERR_HTTP_GET_FAIL;
    }

    if(!reader.parse(resp.c_str(), root)){
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    }

    if(root["angle"].isNull() || root["gridPosition"]["x"].isNull() || root["gridPosition"]["y"].isNull()){
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    }
    else{
        pos.angle = root["angle"].asDouble();
        pos.x = root["gridPosition"]["x"].asInt();
        pos.y = root["gridPosition"]["y"].asInt();
    }

    return ERR_OK;
#endif
}

int GsApi::move(float &linear, float &angular)
{
    std::string url = host + "/gs-robot/cmd/move";
    Json::Value root;
    root["speed"]["linearSpeed"] = Json::Value(linear);
    root["speed"]["angularSpeed"] = Json::Value(angular);
    return req_post_response(url, root);
}


/**
 * @brief nav_merge_path 路径融合，在跑路径前，如果是多条路径，需要调用接口先把
 * 多条路径融合成新的路径，然后再执行跑路径任务
 *
 * @param map
 * @param path_name
 * @param path_list
 *
 * @return
 */
int GsApi::nav_merge_path(std::string map, std::string path_name, std::vector<GsPath> path_list)
{
    std::string url = host + "/gs-robot/cmd/merge_path";
    Json::Value root;
    root["mapName"] = Json::Value(map);
    root["pathName"] = Json::Value(path_name);

    for(unsigned int i = 0; i < path_list.size(); i ++){
        GsPath path = path_list.at(i);
        Json::Value v;
        v["type"] = Json::Value(path.type);
        v["pathName"] = Json::Value(path.name);
        root["paths"].append(v);
    }
    return req_post_response(url, root);
}

int GsApi::power_off()
{
#ifdef _ATRIS_NAVX_
    return ERR_OK;
#else
    std::string url = host + "/gs-robot/cmd/power_off";
    return req_get_response(url);
#endif
}

int GsApi::reboot()
{
#ifdef _ATRIS_NAVX_
    return ERR_OK;
#else   
    std::string url = host + "/gs-robot/cmd/reboot";
    return req_get_response(url);
#endif
}

bool GsApi::get_laser_raw_data(atris_msgs::GetGsLaserRawData::Request& req, 
  atris_msgs::GetGsLaserRawData::Response& res) {
  LaserRaw laser_data;
  if (get_laser_raw_data(laser_data) == ERR_OK) {
      res.frame_id = laser_data.frame_id;
      res.stamp = laser_data.stamp;
      res.angle_min = laser_data.angle_min;
      res.angle_max = laser_data.angle_max;
      res.range_min = laser_data.range_min;
      res.range_max = laser_data.range_max;
      res.range_size = laser_data.range_size;
      res.intens_size = laser_data.intens_size;
      res.angle_increment = laser_data.angle_increment;
      for (uint32_t i = 0; i < 2048; i++) {
          res.laser_range[i] = laser_data.laser_range[i];
      }
      for (uint32_t i = 0; i < 2048; i++) {
          res.intensities[i] = laser_data.intensities[i];
      }
  }

  return true;
}

int GsApi::get_laser_raw_data(LaserRaw &laser_data)
{
    std::string url = host + "/gs-robot/real_time_data/laser_raw";
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s", __FUNCTION__);
    if( utils->http_get(url, resp) < 0){
        log_error("http get req fail.");
        return ERR_HTTP_GET_FAIL;
    }

    if(!reader.parse(resp.c_str(), root)){
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    }

    laser_data.stamp = root["header"]["stamp"].asInt();
    laser_data.frame_id = root["header"]["frame_id"].asString();

    laser_data.angle_min = root["angle_min"].asFloat();
    laser_data.angle_max = root["angle_max"].asFloat();

    laser_data.angle_increment = root["angle_increment"].asFloat();
    laser_data.range_min = root["range_min"].asFloat();
    laser_data.range_max = root["range_max"].asFloat();

    laser_data.range_size = root["ranges"].size();
    laser_data.intens_size = root["intensities"].size();

    for(unsigned int i = 0; i < laser_data.range_size; i ++){
        laser_data.laser_range[i] = root["ranges"][i].asFloat();
    }

    for(unsigned int i = 0; i < laser_data.range_size; i ++){
        laser_data.intensities[i] = root["intensities"][i].asFloat();
    }

    return ERR_OK;
}

int GsApi::get_laser_grid_data(LaserGrid &laser_grid)
{
    std::string url = host + "/gs-robot/real_time_data/laser_phit";
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s", __FUNCTION__);
    if( utils->http_get(url, resp) < 0){
        log_error("http get req fail.");
        return ERR_HTTP_GET_FAIL;
    }

    if(!reader.parse(resp.c_str(), root)){
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    }

    laser_grid.stamp = root["header"]["stamp"].asInt();
    laser_grid.frame_id = root["header"]["frame_id"].asString();
    laser_grid.resolution = root["resolution"].asFloat();

    laser_grid.grid_data_size = root["gridPhits"].size();

    laser_grid.org_x = root["originX"].asFloat();
    laser_grid.org_y = root["originY"].asFloat();

    laser_grid.grid_width = root["gridWidth"].asInt();
    laser_grid.grid_height = root["gridHeight"].asInt();

    for(unsigned int i = 0; i < laser_grid.grid_data_size; i ++){
        laser_grid.grid_data[i].x = root["gridPhits"][i]["x"].asInt();
        laser_grid.grid_data[i].y = root["gridPhits"][i]["y"].asInt();
    }

    return ERR_OK;
}

int GsApi::nav_auto_charge(std::string map_name, std::string charger_name)
{
    std::string enc_map_name = utils->url_encoder(map_name.c_str(), map_name.size());
    std::string enc_charger_name = utils->url_encoder(charger_name.c_str(), charger_name.size());
    std::string url = host + "/gs-robot/cmd/position/navigate?map_name=" + enc_map_name + "&position_name=" + enc_charger_name;
	return req_get_response(url);
}

int GsApi::get_current_init_status(std::string &currMap, std::string &currInitPoint)
{
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_using_map") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret == ERR_OK){
        Json::Reader reader;
        Json::Value root;
        if(!reader.parse(resp, root)){
            log_error("%s", resp.c_str());
            return ERR_GS_INVALID_JSON;
        }

        if (!root["content"].isNull() && !root["content"]["data"].isNull()
        &&  !root["content"]["data"]["map"].isNull()){
            Json::Value js_pos = root["content"]["data"];
            currMap = js_pos["map"].asString();
            currInitPoint = "Origin";
        }else{
            log_error("pos json format error");
        }
    }
    
    return ret;
#else
    std::string url = host + "/gs-robot/real_time_data/current_initialize_status";

    std::string resp;
    Json::Reader reader;
    Json::Value root;

    int ret = req_get_data(url, reader, root, resp);
    if (ret != ERR_OK){
        log_error("%s req get data only err:%d", __FUNCTION__, ret);
        return ret;
    }

    if (root["currentMap"].isNull() || root["currentInitPoint"].isNull()) {
        log_error("%s", resp.c_str());
        return ERR_GS_INVALID_JSON;
    } else {
        currMap = root["currentMap"].asString();
        currInitPoint = root["currentInitPoint"].asString();
    }

    return ERR_OK;
#endif
}

int GsApi::get_device_status(Json::Value &status)
{
#ifdef _ATRIS_NAVX_
    return ERR_OK;
#else
    std::string url = host + "/gs-robot/data/device_status";

    std::string resp;
    Json::Reader reader;
    Json::StyledWriter writer;
    Json::Value root;

    int ret = req_get_data(url, reader, root, resp); 
    if (ret == ERR_OK)
        status = root["data"];

    return ret;
#endif
}

int GsApi::get_init_status(const std::string& map)
{
log_info("%s", __FUNCTION__);
#ifdef _ATRIS_NAVX_
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_using_map") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    std::string currMap, currInitPoint;
    if (ret == ERR_OK){
        Json::Reader reader;
        Json::Value root;
        if(reader.parse(resp, root)){

            if (!root["content"].isNull() && !root["content"]["data"].isNull()
            &&  !root["content"]["data"]["map"].isNull()){
                Json::Value js_pos = root["content"]["data"];
                currMap = js_pos["map"].asString();
                currInitPoint = "Origin";
            }else{
                log_error("pos json format error");
            }
        }

        if(currMap.empty()){
            log_warn("robot is not init.");
            return ERR_NOT_LOCATED;
        }

        if (currMap != map){
            log_warn("robot inited by %s not %s", currMap.c_str(), map.c_str());
            return ERR_NOT_LOCATED_BY_SPC_MAP;
        }

        return ERR_OK;
    }
    return ret;
#else
    std::string url = host + "/gs-robot/real_time_data/current_initialize_status";

    std::string resp;
    Json::Reader reader;
    Json::Value root;

    int ret = req_get_data(url, reader, root, resp);
    if(ret != ERR_OK){
        log_error("%s req get data only err:%d", __FUNCTION__, ret);
        return ret;
    }

    if(root["currentInitPoint"].isNull() || root["currentMap"].isNull())
        return ERR_INVALID_FIELD;

    std::string p = root["currentInitPoint"].asString();
    std::string m = root["currentMap"].asString();

    if(p.empty() || m.empty()){
        log_warn("robot is not init.");
        return ERR_NOT_LOCATED;
    }

	if (m != map){
        log_warn("robot inited by %s not %s", m.c_str(), map.c_str());
		return ERR_NOT_LOCATED_BY_SPC_MAP;
	}
	
    return ERR_OK;
#endif
}

bool GsApi::is_initialize_success()
{
    log_info("%s", __FUNCTION__);

#ifdef _ATRIS_NAVX_

    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_locating_status") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret == ERR_OK){
        Json::Reader reader;
        Json::Value root;
        if(reader.parse(resp.c_str(), root)){
            if (!root["content"].isNull() && !root["content"]["msg"].isNull()
            &&  !root["content"]["result"].isNull()
            &&  !root["content"]["code"].isNull()){
                if (root["content"]["result"].asString() == "success") {
                    return true;
                }
            }else{
                log_error("locating status format error");
            }
        }
    }
    return false;        
#else
    Json::Value status;
    get_device_status(status);

    bool s = false;
    if(!status["locationStatus"].isNull()){
        s = status["locationStatus"].asBool();
    }

    return s;
#endif
}

int GsApi::map_get_path_list(const std::string& map_name, std::vector<GsPath> &path_list){
    std::string enc_map_name = utils->url_encoder(map_name.c_str(), map_name.size());
	std::string url = host + "/gs-robot/data/paths?map_name=" + enc_map_name;
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s", __FUNCTION__);
    int ret = req_get_data(url, reader, root, resp);
    if (ret != ERR_OK){
        return ret;
    }

    int path_size = root["data"].size();
    for(int i = 0; i < path_size; i ++){
        GsPath path;
        path.name  = root["data"][i]["name"].asString();
        path.type = root["data"][i]["type"].asInt();
        log_info("path name:%s", path.name.c_str());
        path_list.push_back(path);
    }

    return ERR_OK;
}

int GsApi::set_time(){
    std::string url = host + "/gs-robot/cmd/set_time?time=";
    time_t now;
    time (&now);
    char str_now[64];
    strftime(str_now, sizeof(str_now), "%Y-%m-%d %H:%M:%S", localtime(&now) );
    url += str_now;
    log_info("set time url:%s", url.c_str());
    return req_get_response(url);
}

#if 1 
int GsApi::map_get_point_list(const std::string& map_name, std::vector<StationInfo> &point_list, int type)
{
    log_info("%s StationInfo", __FUNCTION__);
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_get_point_list") {
    } NAVABILITY_PACKAGE_END(15)
    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d ", __FUNCTION__, ret);
    //log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret != ERR_OK) {
        log_error("get err.");
        return ret;
    }

    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)) {
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code       
    }

#if 0
    if(root["content"].isNull() || root["content"]["msg"].isNull() || 
       root["content"]["result"].isNull() || root["content"]["code"].isNull()) {
        log_error("%s: return parm err", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code 
    }

    if (root["content"]["result"].asString() != "success") {
         log_error("%s: return is fail err", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code        
    }
#endif

    Json::Value p_list = root["content"]["data"];
    int p_size = p_list.size();
    if (p_size <= 0) {
        log_error("no point found");
        return ERR_GS_NO_POINT;
    }

    for(int i = 0; i < p_size; i++) {
        StationInfo sta;
        switch(p_list[i]["type"].asInt()) {
            case 0: sta.type = StationInfo::StationType::STATION; break;
            case 1: sta.type = StationInfo::StationType::CHARGE; break;
            case 3: sta.type = StationInfo::StationType::EXACT_STATION; break;
        }
        sta.station_id = p_list[i]["station_id"].isNull() ? "" : p_list[i]["station_id"].asString();
        sta.station_name = p_list[i]["station_name"].isNull() ? "": p_list[i]["station_name"].asString();
        //sta.longitude = p_list[i]["longitude"].isNull() ? -1 : p_list[i]["longitude"].asDouble() ;
        //sta.latitude = p_list[i]["latitude"].isNull() ? -1 : p_list[i]["latitude"].asDouble();
        //sta.azimuth = p_list[i]["azimuth"].isNull() ? -1 : p_list[i]["azimuth"].asDouble();
        //sta.building = p_list[i]["building"].isNull() ? "" : p_list[i]["building"].asString();
        //sta.uint = p_list[i]["uint"].isNull() ? "" : p_list[i]["uint"].asString();                           
        //sta.floor = p_list[i]["floor"].isNull() ? "" : p_list[i]["floor"].asString();
        sta.association_id = p_list[i]["association_id"].isNull() ? -1 : p_list[i]["association_id"].asInt();
        sta.gauss_x = p_list[i]["gauss_x"].isNull() ? -1 : p_list[i]["gauss_x"].asDouble();
        sta.gauss_y = p_list[i]["gauss_y"].isNull() ? -1 : p_list[i]["gauss_y"].asDouble();
        //sta.region_id = p_list[i]["region_id"].isNull() ? -1 : p_list[i]["region_id"].asInt();
        log_info("%s point:%s type:%d ass_id:%d", __FUNCTION__, sta.station_id.c_str(), sta.type, sta.association_id);
        if (!p_list[i]["type"].isNull() && p_list[i]["type"].asInt() == type) {
            point_list.push_back(sta);
        }
    }

    if (point_list.size() <= 0) {
        log_error("no type point found");
        return ERR_GS_NO_TYPE_POINT;       
    }

    return ERR_OK;
}
#endif

int GsApi::map_get_point_list(const std::string& map_name, std::vector<GsNavPoint> &point_list, int type)
{
    log_info("%s GsNavPoint", __FUNCTION__);
    int ret = ERR_OK;
    std::vector<StationInfo> sta_list;
    if (ERR_OK != (ret = map_get_point_list(map_name, sta_list, type))){
        return ret;
    }
    for (auto sta:sta_list) {
        GsNavPoint p;
        p.id = sta.station_id;
        p.type = (int)sta.type;
        p.ass_id = sta.association_id;
        p.np.name = sta.station_name;
        p.np.pos.x = sta.gauss_x;
        p.np.pos.y = sta.gauss_y;
        p.np.pos.angle = sta.azimuth;
        point_list.push_back(p);
		log_info("%s point id:%s name:%s type:%d x:%f y:%f",
				 __FUNCTION__, p.id.c_str(), p.np.name.c_str(), p.type, p.np.pos.x, p.np.pos.y);
    }
    return ERR_OK;
}

int GsApi::map_get_point_list(const std::string& map_name, std::vector<GsNamePoint> &point_list, int type)
{
    std::string enc_map_name = utils->url_encoder(map_name.c_str(), map_name.size());
    std::string url = host + "/gs-robot/data/positions?map_name=" + enc_map_name + "&type=" + std::to_string(type);
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    log_info("%s GsNamePoint", __FUNCTION__);
    int ret = req_get_data(url, reader, root, resp);
    if (ret != ERR_OK){
        return ret;
    }

    int point_size = root["data"].size();
    for(int i = 0; i < point_size; i ++){
        GsNamePoint point;
        point.name  = root["data"][i]["name"].asString();
        point.pos.x = root["data"][i]["gridX"].asInt();
        point.pos.y = root["data"][i]["gridY"].asInt();
        point.pos.angle = root["data"][i]["angle"].asDouble();
        log_info("init point name:%s", point.name.c_str());
        point_list.push_back(point);
    }

    return ERR_OK;
}

int GsApi::map_get_route_points(std::vector<GsNavPoint> &points)
{
    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_query_route") {
    } NAVABILITY_PACKAGE_END(30)

    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret != ERR_OK) {
        return ret;
    }

    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)) {
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code       
    }

    Json::Value p_list = root["content"]["data"];
	if (p_list.isNull() || !p_list.isArray()){
        log_error("%s:resp json is not points array", __FUNCTION__);
		return ERR_FAIL;
	}

    int p_size = p_list.size();
    if (p_size <= 0) {
        log_error("no point found");
        return ERR_GS_NO_POINT;
    }

	points.clear();
    for(int i = 0; i < p_size; i++) {
		GsNavPoint pt;
        pt.np.pos.x = p_list[i]["gauss_x"].asDouble();
        pt.np.pos.y = p_list[i]["gauss_y"].asDouble();
		points.push_back(pt);
    	log_info("route point:%s name:%s x:%f y:%f", pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y);
	}

	return ERR_OK;
}

int GsApi::get_tsp_navigating(std::vector<GsNavPoint> &points){
	if (points.empty()){
    	log_info("%s points empty", __FUNCTION__);
		return ERR_OK;
	}

	std::vector<StationInfo> stations;
	int ret = GsApi::get_instance()->map_get_point_list("", stations, StationInfo::StationType::EXACT_STATION);
	if (ret != ERR_OK){
    	log_info("%s map_get_point_list err:%s", __FUNCTION__, get_err_msg(ret));
		return ret;
	}

	std::vector<StationInfo> real_stations;
	for (auto pt:points){
		for (auto stn:stations){
			log_info("%s station:%s assid:%d", __FUNCTION__, stn.station_id.c_str(), stn.association_id);
			if (pt.id == stn.station_id){
				real_stations.push_back(stn);
				log_info("%s real station:%s", __FUNCTION__, stn.station_id.c_str());
			}
		}
	}

	if (real_stations.empty()){
    	log_info("%s map_get_point_list err:%s", __FUNCTION__, get_err_msg(ERR_GS_NO_POINT));
		return ERR_GS_NO_POINT;
	}

    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_tsp_navigating") {
		for(auto pt:real_stations){
            Json::Value js_pt;
            js_pt["station_id"]     = pt.station_id;    
            js_pt["association_id"] = pt.association_id;       
            js_pt["station_name"]   = pt.station_name; 
            js_pt["type"]           = pt.type; 
            js_pt["gauss_x"]        = pt.gauss_x;       
            js_pt["gauss_y"]        = pt.gauss_y;       
			root["content"]["data"].append(js_pt);
			log_info("pre tsp point id:%s name:%s ass_id:%d", pt.station_id.c_str(), pt.station_name.c_str(), pt.association_id);
        }
    } NAVABILITY_PACKAGE_END(30)

    std::string resp = response.msg;
	ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret != ERR_OK) {
        return ret;
    }

    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)) {
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code       
    }

    Json::Value p_list = root["content"]["data"];
	if (p_list.isNull() || !p_list.isArray()){
        log_error("%s:resp json is not points array", __FUNCTION__);
		return ERR_FAIL;
	}

    int p_size = p_list.size();
    if (p_size <= 0) {
        log_error("no point found");
        return ERR_GS_NO_POINT;
    }

	points.clear();
    for(int i = 0; i < p_size; i++) {
		GsNavPoint pt;
        pt.type     = p_list[i]["type"].asInt();
        pt.id       = p_list[i]["station_id"].asString();
        pt.ass_id   = p_list[i]["ssociation_id"].asInt();
        pt.np.name  = p_list[i]["station_name"].asString();
        pt.np.pos.x = p_list[i]["gauss_x"].asDouble();
        pt.np.pos.y = p_list[i]["gauss_y"].asDouble();
		points.push_back(pt);
    	log_info("tsped point id:%s name:%s x:%f y:%f", pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y);
	}

    return ERR_OK;
}

#if 0
int GsApi::get_tsp_navigating(std::vector<StationInfo> &points)
{
	if (points.empty()){
    	log_info("%s points empty", __FUNCTION__);
		return ERR_OK;
	}

    atris_msgs::NavAbilityMessage request, response;
    NAVABILITY_PACKAGE_BEGIN("request_tsp_navigating") {
		for(auto pt:points){
            Json::Value js_pt;
            js_pt["station_id"]     = pt.station_id;    
            js_pt["station_name"]   = pt.station_name;  
            js_pt["longitude"]      = pt.longitude;     
            js_pt["latitude"]       = pt.latitude;      
            js_pt["azimuth"]        = pt.azimuth;       
            js_pt["building"]       = pt.building;      
            js_pt["uint"]           = pt.uint;          
            js_pt["floor"]          = pt.floor;         
            js_pt["gauss_x"]        = pt.gauss_x;       
            js_pt["gauss_y"]        = pt.gauss_y;       
            js_pt["region_id"]      = pt.region_id;     
            js_pt["association_id"] = pt.association_id;
			root["content"]["data"].append(js_pt);
    		log_info("pre tsp point:%s", __FUNCTION__, pt.station_id.c_str());
        }
    } NAVABILITY_PACKAGE_END(30)

    std::string resp = response.msg;
	int ret = get_navability_respresult(response);
    log_info("%s ret:%d resp:%s", __FUNCTION__, ret, resp.c_str());
    if (ret != ERR_OK) {
        return ret;
    }

    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(resp.c_str(), root)) {
        log_error("%s:fail parse resp json", __FUNCTION__);
        return ERR_FAIL;//todo format gs and xs error code       
    }

    Json::Value p_list = root["content"]["data"];
	if (p_list.isNull() || !p_list.isArray()){
        log_error("%s:resp json is not points array", __FUNCTION__);
		return ERR_FAIL;
	}

    int p_size = p_list.size();
    if (p_size <= 0) {
        log_error("no point found");
        return ERR_GS_NO_POINT;
    }

	points.clear();
    for(int i = 0; i < p_size; i++) {
        StationInfo sta;
        sta.station_id = p_list[i]["station_id"].isNull() ? "" : p_list[i]["station_id"].asString();
        sta.station_name = p_list[i]["station_name"].isNull() ? "": p_list[i]["station_name"].asString();
        sta.longitude = p_list[i]["longitude"].isNull() ? -1 : p_list[i]["longitude"].asDouble() ;
        sta.latitude = p_list[i]["latitude"].isNull() ? -1 : p_list[i]["latitude"].asDouble();
        sta.azimuth = p_list[i]["azimuth"].isNull() ? -1 : p_list[i]["azimuth"].asDouble();
        sta.building = p_list[i]["building"].isNull() ? "" : p_list[i]["building"].asString();
        sta.uint = p_list[i]["uint"].isNull() ? "" : p_list[i]["uint"].asString();                           
        sta.floor = p_list[i]["floor"].isNull() ? "" : p_list[i]["floor"].asString();
        sta.association_id = p_list[i]["association_id"].isNull() ? -1 : p_list[i]["association_id"].asInt();
        sta.gauss_x = p_list[i]["gauss_x"].isNull() ? -1 : p_list[i]["gauss_x"].asDouble();
        sta.gauss_y = p_list[i]["gauss_y"].isNull() ? -1 : p_list[i]["gauss_y"].asDouble();
        sta.region_id = p_list[i]["region_id"].isNull() ? -1 : p_list[i]["region_id"].asInt();
        log_info("tsped point:%s", sta.station_id.c_str());
        points.push_back(sta);
    }

    return ERR_OK;
}
#endif

