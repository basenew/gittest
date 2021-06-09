#include <thread>
#include "gs_api.h"
#include "navigation.h"
#include "scheme_manager.h"
#include "map_manager.h"
#include "nav_error_code.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[map_mgr]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[map_mgr]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[map_mgr]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[map_mgr]"#format, ##args)

#define PARSE_JSON()    if(!reader.parse(msg.msg, req)){\
                            Lerror("parse json fail.");\
                            ret = ERR_INVALID_JSON;\
                            break;\
                        }

#define QINIU_BUCKET_NAME "http://video.ubtrobot.com/"

namespace nav{

void MapManager::_handle_msg(const SignalMessage &msg){
    Reader reader;
    Value  req, rsp;
    string result("success");
    int    ret = ERR_OK;

    do
    {
        if (!_inited){
            if (msg.title == "request_sync_map"
             || msg.title == "request_del_map"
             || msg.title == "request_rename_map"
             || msg.title == "request_set_map"
             || msg.title == "request_get_maps"
             || msg.title == "request_load_map"
             || msg.title == "request_get_points"){
                Lwarn("service not ready");
                ret = ERR_SERVICE_NOT_READY;
                break;
            } else return;
        }
        else if (msg.title == "request_sync_map"){
            PARSE_JSON();
            ret = _upload_map(req, rsp, result, msg);
            if (ret == ERR_OK) return;
        }
        else if (msg.title == "request_del_map"){
            PARSE_JSON();
            ret = _del_map(req, rsp, result);
        }
        else if (msg.title == "request_rename_map"){
            PARSE_JSON();
            ret = _rename_map(req, rsp, result);
        }
        else if (msg.title == "request_set_map"){
            PARSE_JSON();
            ret = _set_using_map(req, rsp, result);
        }
        else if (msg.title == "request_get_maps"){
            PARSE_JSON();
            ret = _get_maps(req, rsp, result);
        }
        else if(msg.title == "request_load_map"){
            PARSE_JSON();
            ret = _hdl_load_map(req, rsp, result, msg);
            if (ret == ERR_OK){
                return;
            }
        }
        else if (msg.title == "request_get_points"){
            Linfo("request_get_points");
            PARSE_JSON();
            ret = _get_points(req, rsp, result);
        }
        else
            return;

    }while(0);

    if (ret != ERR_OK && (result == "success" || result.empty()))
        result = get_err_msg(ret); 
    rsp["id"] = req["content"]["id"];
    rsp["timestamp"] = req["content"]["timestamp"];
    rsp["result"] = result;
    rsp["result_code"] = ret;
    Utils::get_instance()->responseResult(msg, rsp, "response" + msg.title.substr(7));
}
int MapManager::_get_points(Value &req, Value &resp, string& result){
    if (req["content"]["map"].isNull()
    ||  req["content"]["type"].isNull()){
        result = "fail_invalid_data";
        return ERR_INVALID_FIELD;
    }

    resp["type"] = req["content"]["type"];
    resp["mapname"] = req["content"]["map"];
    string map_name = req["content"]["map"].asString(); 
    int type = req["content"]["type"].asInt();
    vector<GsNavPoint> points;
	int ret = GsApi::get_instance()->map_get_point_list(map_name, points, type);
    if (ret == ERR_OK){
        for(auto pt:points){
            Value js_pt;
            js_pt["id"] = pt.id;
            js_pt["name"] = pt.np.name;
            js_pt["x"] = pt.np.pos.x;
            js_pt["y"] = pt.np.pos.y;
            js_pt["angle"] = pt.np.pos.angle;
            resp["points"].append(js_pt);
        }
    }
    else
        resp["points"] = "";

    return ret;
}

int MapManager::_upload_map(Value &req, Value &rsp, string& result, const SignalMessage &msg){
    Linfo("%s", __FUNCTION__);
    int sz = 0;
    if (req["content"]["stations"].isNull() || !req["content"]["stations"].isArray()
        || (sz = req["content"]["stations"].size()) <= 0) {
        result = "fail_invalid_data";
        Lerror("stations field not set or not array");
        return ERR_INVALID_FIELD;
    }

    for(int i = 0; i < sz; i ++){
        Value value = req["content"]["stations"][i];
        boost::shared_ptr<TransNotifyer> obj = boost::shared_ptr<TransNotifyer> (new TransNotifyer(true));
        string m = value["name"].asString() + ".zip";
        //string m = value["name"].asString() + ".tar.gz";
        obj->local_path = Config::get_instance()->maps_dir + m;
        obj->remote_path = value["url"].asString();
        obj->name = value["name"].asString();
        obj->station_id = value["station"].asString();
        obj->origin = msg;
        //obj->param = obj.get();
		function<int(string)> post_web = [=](string)->int{
        	Linfo("upload %s to xs", obj->name.c_str());
        	int ret = GsApi::get_instance()->map_upload(obj->name);
			if (ret == ERR_OK){
				string url = "https://test79.ubtrobot.com/irms-service/swagger-ui.html#/PcToolController/uploadMapUsingPOST";
				Value root;
				root["mapName"] = obj->name;
				root["mapUrl"] = obj->remote_path;
				root["substationId"] = obj->station_id;
				FastWriter fw;
				string rsp;
			    string req = fw.write(root);
				ret = Utils::get_instance()->http_post_with_header(url, req, rsp);
			}
			return ret;
		};
        obj->cb = post_web;//&MapManager::_upload_cb;
        Linfo("map name:%s local path:%s cb:%p", obj->name.c_str(), obj->local_path.c_str(), obj->cb);
        TransferFile::download(obj);
    }

    return ERR_OK;
}

int MapManager::_hdl_load_map(Value &req, Value &resp, string& result, const SignalMessage &msg){
    Linfo("%s", __FUNCTION__);
    int size = 0;
    if (req["content"]["stations"].isNull() || !req["content"]["stations"].isArray()
        || (size = req["content"]["stations"].size()) <= 0) {
        result = "fail_invalid_data";
        Lerror("stations field not set or not array");
        return ERR_INVALID_FIELD;
    }
    
    vector<GsMap> maps;
    int ret = GsApi::get_instance()->map_get_list(maps);
    if(ret != ERR_OK)
        return ret;

    if(maps.size() <= 0)
        return ERR_NONE_MAP;

    boost::shared_ptr<boost::thread> map_task(new boost::thread(boost::bind(&MapManager::_download_map,
                                              this, req, resp, result, msg, maps)));
    return ERR_OK;
}

int MapManager::_download_map(Value &req, Value &resp, string& result, const SignalMessage &msg, vector<GsMap> &maps){
    Linfo("%s", __FUNCTION__);
    int ret = ERR_OK;
    string map_path = MAP_UPLOAD_TMP_PATH;  
    Utils::createDir(map_path);
    bool map_not_found = true;
    Value &js_maps = req["content"]["stations"];
    resp["id"] = req["content"]["id"];
    int sz = js_maps.size();
    for (int i = 0; i < sz; i++){
		Value &js_map = js_maps[i];
		if (js_map["name"].isNull() || js_map["station"].isNull()){
			Lerror("invalid map json data");
			continue;
		}

        string map_name = js_maps[i]["name"].asString();
        string station= js_maps[i]["station"].asString();
		if (map_name.empty() || station.empty()){
			Lerror("map or station is empty");
			continue;
		}

        for (auto m:maps){
            if (m.name == map_name){
                //string map_path_name = map_path + map_name + ".tar.gz";
                string map_path_name = map_path + map_name + ".zip";
                Linfo("start download map:%s form gs box", map_path_name.c_str());

                resp["status"]   = Value("start_download");
                resp["name"]     = map_name;
                resp["result"]   = Value("success");
                resp["progress"] = Value(0.0);
                Utils::get_instance()->responseResult(msg, resp, "response_load_map");
                
                ret =GsApi::get_instance()->map_download(map_name, map_path);
                if (ret != ERR_OK)
                {
                    Linfo("download map:%s from gs err:%d", map_path_name.c_str(), ret);
                    //resp["status"]   = Value("finish_download");
                    //resp["result"]   = Value("success");
                    //resp["progress"] = Value(100.0);
                    //Utils::get_instance()->responseResult(msg, resp, "response_load_map");
                    continue;
                }

                Linfo("start upload map:%s to cloud", map_path_name.c_str());
                resp["status"]   = Value("finish_download");
                resp["name"]     = map_name;
                resp["result"]   = Value("success");
                resp["progress"] = Value(100.0);
                Utils::get_instance()->responseResult(msg, resp, "response_load_map");

                char md5[128] = {0};
                Utils::get_instance()->gen_md5(map_path_name.c_str(), md5);
                string map_md5(md5);
                //map_md5 += ".tar.gz";
                map_md5 += ".zip";

                Linfo("md5 map:%s", map_md5.c_str());

                //upload file to cloud server
                boost::shared_ptr<TransNotifyer> upload_file = boost::shared_ptr<TransNotifyer> (new TransNotifyer(false));
                if(Config::get_instance()->hfs_type == "qiniu"){
                    Linfo("qiniu fs");
                    if(TransferFile::uploaded(map_md5)){
                        Linfo("file exist, don't upload again.");
                        resp["url"]      = QINIU_BUCKET_NAME + map_md5;
                        resp["name"]     = map_name;
                        resp["progress"] = Value(100.0);
                        resp["status"]   = "finish_upload";
                        resp["result"]   = Value("success");
                        Utils::get_instance()->responseResult(msg, resp, "response_load_map");

                        std::string cmd = "/bin/rm " + map_path_name;
                        system(cmd.c_str());
                        map_not_found = false;
                        continue;
                    }
                    upload_file->remote_path = QINIU_BUCKET_NAME + map_md5;
                }
                else{
                    Linfo("not qiniu fs");

                    if (*(Config::get_instance()->hfs_url.end()-1) != '/') {
                        upload_file->remote_path = Config::get_instance()->hfs_url + "/" + map_name + ".zip";
                        //upload_file->remote_path = Config::get_instance()->hfs_url + "/" + map_name + ".tar.gz";
                    } else {
                        upload_file->remote_path = Config::get_instance()->hfs_url + map_name + ".zip";
                        //upload_file->remote_path = Config::get_instance()->hfs_url + map_name + ".tar.gz";
                    }
                }
                upload_file->local_path = map_path_name;
                upload_file->deleteDays = 1;
                upload_file->name = map_name;
                upload_file->origin = msg;
                upload_file->started_str = "start_upload";
                function<int(string)> post_web = [=](string map_url)->int{
                    Linfo("map url:%s", map_url.c_str());
                    Linfo("upload %s to fs", map_name.c_str());
                    Linfo("sw upload map api:%s", _sw_upload_map_api.c_str());
                    //string url = "https://local.ubtrobot.com/irms-service/irms/pc/uploadMap";
                    Value root;
                    root["mapName"] = map_name;
                    root["mapUrl"] = map_url;
                    root["substationId"] = station;
                    FastWriter fw;
                    string rsp;
                    string req = fw.write(root);
                    return Utils::get_instance()->http_post_with_header(_sw_upload_map_api, req, rsp);
                };
				upload_file->cb = post_web;//&MapManager::_upload_cb;
                TransferFile::upload(upload_file);
                map_not_found = false;
                             
                //Value mm;
                while(true){
                    if(upload_file->state == TRANSFER_FILE_COMPLETED){
                        if(Config::get_instance()->hfs_url.find("upload") != std::string::npos){
                            Linfo("use go-fastdfs...");
                            upload_file->remote_path  = Utils::get_instance()->get_fileurl();
                        }
                        Linfo("upload local:%s , remote:%s", upload_file->local_path.c_str(), upload_file->remote_path.c_str());
                        //mm["name"] = Value(map_name);
                        //mm["url"] = Value(upload_file->remote_path);
                        //resp["maps"].append(mm);
                        Linfo("success upload map:%s", map_name.c_str());
                        //result = "success";
                        std::string cmd = "/bin/rm " + map_path_name;
                        //todo zy system(cmd.c_str());
                        break;
                    }
                    else if(upload_file->state == TRANSFER_FILE_ERROR){
                        Lerror("upload files fail");
                        ret = ERR_TRANSFER_FILE_FAIL;
                        break;
                    }
                    this_thread::sleep_for(chrono::milliseconds(500));
                }
                break;
            }
        }
    }

    return map_not_found? ERR_MAP_NOT_FOUND:ERR_OK;
}

int MapManager::_del_map(Value &req, Value &rsp, string& result)
{
    Linfo("%s", __FUNCTION__);
    Value map_list;
    map_list = req["content"]["name"];
    Value del_map_list;
    int size = map_list.size();
    if(size <= 0){
        result = "fail_invalid_data";
        return ERR_INVALID_FIELD;
    }
    else{
        int ret;
        for(int i = 0; i < size; i ++){
            string map = map_list[i].asString();
            if (_using_map == map){//todo test case
                Lerror("can not remove using map:%s", map.c_str());
                ret = ERR_REMOVE_USING_MAP;
                continue;
            }
            ret = GsApi::get_instance()->map_del(map); //删除地图
            if(ret == ERR_OK)
                rsp["name"].append(Value(map));
        }
        return rsp["name"].size()?ERR_OK:ret;
    }
}

int MapManager::_rename_map(Value &req, Value &rsp, string& result)
{
    Linfo("%s", __FUNCTION__);
    if(req["content"]["name"].isNull() || req["content"]["newname"].isNull()){
        result = "fail_invalid_data";
        return ERR_INVALID_FIELD;
    }

    string name = req["content"]["name"].asString();
    string newname = req["content"]["newname"].asString();
    if (name.empty()||newname.empty()){
        result = "fail_invalid_data";
        return ERR_INVALID_VALUE;
    }

    if (_using_map == name){//todo test case
        Lerror("can not rename using map:%s", name.c_str());
        return ERR_RENAME_USING_MAP;
    }

    return GsApi::get_instance()->map_rename(name, newname);
}

int MapManager::_is_exit(const string map_name){
    Linfo("%s: %s ?", __FUNCTION__, map_name.c_str());
    vector<GsMap> maps;
    int ret = GsApi::get_instance()->map_get_list(maps);
    if(ret != ERR_OK)
        return ret;

    if(maps.size() <= 0)
        return ERR_NONE_MAP;

    for (auto m:maps){
        if (m.name == map_name)
            return ERR_OK;
    }

    return ERR_MAP_NOT_FOUND;
}

int MapManager::load_map(const string map_name){
    Linfo("%s", __FUNCTION__);
    return _load_map(map_name);
}

int MapManager::_load_map(const string map_name){
    Linfo("%s", __FUNCTION__);
    if (map_name.empty()){
        Lerror("load map is empty");
        return ERR_INVALID_VALUE;
    }

    int ret = _is_exit(map_name);//todo test case
    if (ret != ERR_OK){
        Linfo("%s not exist:%s", map_name.c_str(), get_err_msg(ret));
        return ret;
    }

    if (_loading_map) return ERR_MAP_LOADING;
    unique_lock<mutex> lck(_mt);
    if (_loading_map) return ERR_MAP_LOADING;
    _loading_map = true;

    string last_using_map = _using_map;
    //_using_map = "";
    if (ERR_OK != GsApi::get_instance()->get_init_status(map_name)){//todo test case
        Linfo("load %s replace %s", map_name.c_str(), last_using_map.c_str());
        ret = GsApi::get_instance()->map_load(map_name);
        if(ret != ERR_OK){
            //todo zy
            _loading_map = false;
            Lerror("load %s fail", map_name.c_str());
            return ret;
        }
    }

    if (ret == ERR_OK){
        _using_map = map_name;
        SET_USING_MAP_TO_DB(_using_map);
    }

    _loading_map = false;
    return ret;
}

int MapManager::_set_using_map(Value &req, Value &rsp, string& result)
{
    Linfo("%s", __FUNCTION__);
    if(req["content"]["name"].isNull()){
        result = "fail_invalid_data";
        return ERR_INVALID_FIELD;
    }
    
    rsp["name"] = req["content"]["name"];
    if (Navigation::get_instance().is_running()
    ||  SchemeManager::get_instance().is_running()){
        return ERR_ON_RUNNING; 
    }

    string map_name = req["content"]["name"].asString();
    //thread t(bind(&MapManager::_load_map, this, map_name));    
    //t.detach();
    return _load_map(map_name);
}

int MapManager::_get_maps(Value &req, Value &rsp, string& result){
    Linfo("%s", __FUNCTION__);
    vector<GsMap> maps;
    int ret = GsApi::get_instance()->map_get_list(maps);
    if(ret != ERR_OK)
        return ret;

    if(maps.empty())
        rsp["maps"] = "";
    else{
        for(auto m:maps)
            rsp["maps"].append(Value(m.name));
    }

    if (_using_map.empty()){
        string using_map = GET_USING_MAP_FROM_DB();
        _load_map(using_map);
    }
    rsp["map_using"] = _using_map;//todo
    return ERR_OK;
}

}

