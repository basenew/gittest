#pragma once
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <json/json.h>
#include "nav_error_code.h"
#include "kv_db.h"
#include "gs_api.h"
#include "ros/ros.h"
#include "trans_notifier.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[map_mgr]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[map_mgr]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[map_mgr]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[map_mgr]"#format, ##args)

#define DB_MOD_NAV_NAME            "nav"
#define DB_KEY_USING_MAP           "using_map"
#define GET_USING_MAP_FROM_DB()    _db.get(DB_KEY_USING_MAP)
#define SET_USING_MAP_TO_DB(name)  _db.set(DB_KEY_USING_MAP, name)


static const string MAP_UPLOAD_TMP_PATH("/userdata/tmp/maps/");

using namespace std;
using namespace Json;
using namespace atris_msgs;

namespace nav{

class MapManager
{
public:

    inline const string& get_using_map(){return _using_map;};
    inline string get_using_map_from_db(){return GET_USING_MAP_FROM_DB();};
    inline bool is_set_using_map(){return !_using_map.empty();};
    inline bool init(){
        Linfo("%s nav v2 enable", __FUNCTION__);
        _db.select(DB_MOD_NAV_NAME);
        // TODO : get using map name from web server.
        // string using_map = GET_USING_MAP_FROM_DB();
        string using_map, currInitPoint;
        GsApi::get_instance()->get_current_init_status(using_map, currInitPoint);
        Linfo("current robot using map:%s", using_map.c_str());
        if (using_map.empty()) {
            using_map = GET_USING_MAP_FROM_DB();
            Linfo("using_map empty...");
        }

        _loading_map = false;
 
        _using_map = using_map;


        if (!using_map.empty()){
            //_load_map(using_map);
            thread t(bind(&MapManager::_load_map, this, using_map));
            t.detach();
        }
    
        Utils::createDir(MAP_UPLOAD_TMP_PATH);
        _inited = true;
        return true;
    };

    int load_map(){
        Linfo("%s", __FUNCTION__);
        if (!_inited){
            Lerror("map manager is not inited");
            return ERR_SERVICE_NOT_READY;
        }

        string using_map = GET_USING_MAP_FROM_DB();
        if (!using_map.empty()){
            return _load_map(using_map); 
        }
        Lerror("using map is not set");
        return ERR_MAP_NOT_USING;
    };
    int load_map(const string map_name);
    static MapManager& get_instance(){
        static MapManager singleton;
        return singleton;
    };
private:
    int _hdl_load_map(Value &req, Value &resp, string& result, const SignalMessage &msg);
    int _upload_map(Value &req, Value &resp, string& result, const SignalMessage &msg);
    int _download_map(Value &req, Value &resp, string& result, const SignalMessage &msg, vector<GsMap> &maps);
    int _del_map(Value &req, Value &resp, string& result);
    int _set_using_map(Value &req, Value &resp, string& result);
    int _get_maps(Value &req, Value &resp, string& result);
    int _rename_map(Value &req, Value &resp, string& result);
    int _load_map(const string map_name);
    int _is_exit(const string map_name);
	int _get_points(Value &req, Value &resp, string& result);

    inline static int _download_cb(void* name){
        Linfo("download %s to client", ((string*)name)->c_str());
        return GsApi::get_instance()->map_download(*(string*)name, MAP_UPLOAD_TMP_PATH);
    };
    void _handle_msg(const SignalMessage &msg);
private:
    MapManager()
    : _inited(false) {
		Linfo("%s", __FUNCTION__);
    	_signal_req_sub = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &MapManager::_handle_msg, this);
		Config &cfg = *Config::get_instance();
		_sw_upload_map_api = Utils::get_instance()->GenFullApiPath(cfg.swp_host, cfg.swp_port, cfg.swp_https, cfg.swp_upload_map_api);
		Linfo("sw platform upload map api:%s", _sw_upload_map_api.c_str());
	}

    string _using_map;
    KVDB   _db;
    bool   _inited;
    bool   _loading_map;
    mutex  _mt;
    ros::NodeHandle nh_;
    ros::Subscriber _signal_req_sub;
    string _sw_upload_map_api;
};

} 
