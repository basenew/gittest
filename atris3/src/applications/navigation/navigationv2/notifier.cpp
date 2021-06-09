#include <thread>
#include "notifier.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_ntf]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_ntf]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_ntf]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_ntf]"#format, ##args)


#define WS_NAV_STATUS_PROTOCOL_NAME "gs-state-protocol"
#define WS_DEV_HEALTH_PROTOCOL_NAME "gs-health-state-protocol"

#define WS_NAV_STATUS_PATH "/gs-robot/notice/status"
#define WS_DEV_HEALTH_PATH "/gs-robot/notice/system_health_status"

#define WS_CONNECT_TIMEOUT    (5)

using namespace std;

namespace nav{

bool Notifier::connect_ws_status()
{
    Linfo("%s", __FUNCTION__);
    if (!_ws_status.isConnected()){
        Linfo("try connect status ws");
        wsClientParam param;
        param.path = WS_NAV_STATUS_PATH;
        param.protocolsName = WS_NAV_STATUS_PROTOCOL_NAME;
        param.addr = Config::get_instance()->gs_ip;
        param.port = Config::get_instance()->gs_ws_port;

        _ws_status.setParams(param);
        _ws_status.setSsl(false);
        _ws_status.setReadCallback(boost::bind(&Notifier::_status_cb, this, _1));
        _ws_status.start();
    }
    else{
        Linfo("try reconnect status ws");
        _ws_status.restart();
    }

    int timeout = WS_CONNECT_TIMEOUT;     
    while (timeout-- && !_ws_status.isConnected()){
        Linfo("try connect status ws timeout:%d...", timeout);
        this_thread::sleep_for(chrono::seconds(1));
    }

    Linfo("connect status ws:%s", _ws_status.isConnected() ? "success":"fail");
    return _ws_status.isConnected();
}

bool Notifier::connect_ws_health()
{
    Linfo("%s", __FUNCTION__);
    if (!_ws_health.isConnected()){
        wsClientParam param;
        param.path = WS_DEV_HEALTH_PATH;
        param.protocolsName = WS_DEV_HEALTH_PROTOCOL_NAME;
        param.addr = Config::get_instance()->gs_ip;
        param.port = Config::get_instance()->gs_ws_port;

        _ws_health.setParams(param);
        _ws_health.setSsl(false);
        _ws_health.setReadCallback(boost::bind(&Notifier::_health_cb, this, _1));
        _ws_health.start();
    }
    else{
        Linfo("try reconnect status ws");
        _ws_health.restart();
    }

    int timeout = WS_CONNECT_TIMEOUT;     
    while (timeout-- && !_ws_health.isConnected()){
        Linfo("try connect health ws timeout:%d...", timeout);
        this_thread::sleep_for(chrono::seconds(1));
    }

    return _ws_health.isConnected();
}
    
void Notifier::do_navability_sub_cb(const atris_msgs::NavAbilityMessage& msg) {
    if (msg.title == "notify_nav_state"){
        Linfo("notify_nav_state:%s", msg.msg.c_str());
        _status_cb(msg.msg);
	}else if (msg.title == "notify_robot_pose"){
		publish_position(msg.msg);
    }else{
        Linfo("ignore:%s", msg.msg.c_str());
    }
}

void Notifier::_status_cb(const string& data)
{
    unique_lock<recursive_mutex> lock(_mt);
    if (_st_cb){
       Linfo("%s 111 %s",  __FUNCTION__, data.c_str());
        _st_cb(const_cast<string&>(data));
       Linfo("%s 222",  __FUNCTION__);
    }
    else{
        Json::Reader reader;
        Json::Value root;

        if(!reader.parse(data.c_str(), root)){
            Lerror("parse gs status js data fail js:%s", data.c_str());
            return;
        }

        int code = root["statusCode"].asInt();
        if (code != 701 && code != 1006){
            //publish_pos_status(code, 0);
            Linfo("code:%d", code);
            //Linfo("%s js:%s", __FUNCTION__, data.c_str());
        }
    }
}

int Notifier::publish_event(){
    Json::Value root;
    Json::Value evt_charge, evt_lamp, evt_leaking, evt_flashing;
    Json::Value evt_shutdown, evt_impacted, evt_avoiding, evt_emergency;

    evt_lamp["lamp"]         = 0;//0:关 1:开
    evt_leaking["leaking"]   = 0;//1:进水
    evt_charge["charge"]     = 0;//0:充电断开 1:充电中
    evt_flashing["flashing"] = 0;//0:爆闪关闭 1:爆闪打开
    evt_shutdown["shutdown"] = 0;//1:正在关机
    evt_impacted["impacted"] = 0;//1:发生碰撞
    evt_avoiding["avoiding"] = 0;//1:避障中
    evt_emergency["emergency"] = 0;//1紧急制动:

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    ros::Time now = ros::Time::now();
    root["title"] = "notify_event";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    root["content"]["result"] = "success";
    root["content"]["result_code"] = ERR_OK;

    root["content"]["events"].append(evt_lamp);
    root["content"]["events"].append(evt_charge);
    root["content"]["events"].append(evt_leaking);
    root["content"]["events"].append(evt_flashing);
    root["content"]["events"].append(evt_shutdown);
    root["content"]["events"].append(evt_impacted);
    root["content"]["events"].append(evt_avoiding);
    root["content"]["events"].append(evt_emergency);

    static int i = 0;
    if (i >= 7){ 
        i = 0;
    }   
    else
        i++;

    Value &evts = root["content"]["events"];
    evts[i] = 1;

    publish_msg(root);
    return ERR_OK;
}

int Notifier::publish_pos(){
    Linfo("%s", __FUNCTION__);
    ros::Time now = ros::Time::now();
    Json::Value root;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "response_robot_pose";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);

    GsPos cur_pos;
    int ret = GsApi::get_instance()->nav_get_pos(cur_pos);

    if (ret != ERR_OK) {
        Lerror("get pos err:%d", ret);
        root["content"]["result"] = get_err_msg(ret);
        root["content"]["result_code"] = ret;
    } else {
        root["content"]["x"] = Json::Value(cur_pos.x);
        root["content"]["y"] = Json::Value(cur_pos.y);
        root["content"]["theta"] = Json::Value((float)(cur_pos.angle));
        root["content"]["result"] = "success";
        root["content"]["result_code"] = ERR_OK;
    }

    publish_msg(root);

    return ret;

}

void Notifier::publish_msg(const Value& root){
    int64_t now = (int64_t)(ros::Time::now().toSec() * 1000);
    Json::FastWriter fw;
    std::string js_data = fw.write(root);
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    atris_msgs::SignalMessage resp;
    resp.account = shmrbt.robot.receiver;
    resp.msgID = std::to_string(now);
    resp.msg = js_data;
    if(!root["title"].isNull()) {
        resp.title = root["title"].asString();
    }
    Notifier::get_instance().signal_resp_pub_.publish(resp);
    Linfo("publish:%s", resp.msg.c_str());
}

void Notifier::publish_recharge_battery(int level){
    Linfo("%s %d", __FUNCTION__, level);
    Json::Value root;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["title"] = "response_recharge_battery";
    root["content"]["battery"] = level;
    root["content"]["result"] = "success";
    root["content"]["result_code"] = ERR_OK;
    publish_msg(root);
}

int Notifier::publish_pos_status(int status_code, int task_type)
{
    //Linfo("report pos and status");
    //report state and position
    GsPos cur_pos;
    int ret = GsApi::get_instance()->nav_get_pos(cur_pos);
    if(ret != ERR_OK){
        Lerror("get pos err:%d", ret);
        return ret;
    }

    Json::Value root;

    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["title"] = "response_nav_state";
    root["content"]["x"] = Json::Value(cur_pos.x);
    root["content"]["y"] = Json::Value(cur_pos.y);
    root["content"]["navtype"] = Json::Value((int)task_type);;
    root["content"]["angle"] = Json::Value((float)(cur_pos.angle));
    root["content"]["state"] = Json::Value(status_code);
    publish_msg(root);
    return ERR_OK;
}

void Notifier::publish_status(int status_code, int task_type, const std::string& reason){
    Linfo("report state");

    Json::Value root;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["title"] = "response_state";
    root["content"]["navtype"] = Json::Value(task_type);
    root["content"]["reason"] = Json::Value(reason);
    root["content"]["state"] = Json::Value(status_code);
    publish_msg(root);
}

void Notifier::publish_position(const string& msg){
	Linfo("%s %s", __FUNCTION__, msg.c_str());	
    Json::Reader reader;
	Json::Value root, content, pos;
	if (reader.parse(msg.c_str(), pos)){
		ros::Time now = ros::Time::now();
    	std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    	shm::Robot shmrbt;
    	shm::iMemory_read_Robot(&shmrbt);
    	root["accid"] = shmrbt.robot.sn;
    	root["title"] = "notify_robot_pose";
    	content["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    	content["id"] = uid.str();
    	content["location_x"] = pos["location_x"].asDouble();
    	content["location_y"] = pos["location_y"].asDouble();
		root["content"] = content;
    	publish_msg(root);
	}else{
		Lerror("%s invalid pos json:%s", __FUNCTION__, msg.c_str());
	}
}

void Notifier::publish_route_points(std::string schemeName, std::string taskType, std::string taskMouldId,
			                        std::string taskTimestamp, vector<GsNavPoint> &pos_points,
									vector<GsNavPoint> &route_points){
    Json::Value root, content, path, js_pos_points, js_route_points, point;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["accid"] = shmrbt.robot.sn;
    root["title"] = "notify_optimal_path";
    content["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    content["id"] = uid.str();
    content["schemeName"] = schemeName;
    content["operationType"] = taskType;
    content["taskMouldId"] = taskMouldId;
    content["taskTimestamp"] = taskTimestamp;

#if 0
	for(auto pt:pos_points){
		Json::Value point;
		point["location_x"] = pt.np.pos.x;
		point["location_y"] = pt.np.pos.y;
		point["map_point_id"] = pt.id;
		//js_pos_points.append(point);
		path.append(point);
	}
#endif

	for(auto pt:route_points){
		Json::Value point;
		point["location_x"] = pt.np.pos.x;
		point["location_y"] = pt.np.pos.y;
		point["map_point_id"] = pt.id;
		//point["map_point_name"] = pt.np.name;
		//js_route_points.append(point);
		path.append(point);
	}
	//path["position_points"] = js_pos_points;
	//path["route_points"] = js_route_points;
	content["path"] = path;
	root["content"] = content;
	
    Json::FastWriter fw;
	std::string str_json = fw.write(root);
	Linfo("publish route points:%s", str_json.c_str());
    publish_msg(root);
}

void Notifier::publish_task_status(std::string mapName, std::string schemeName, std::string taskType,
                                   std::string taskMouldId, std::string taskTimestamp, std::string taskStatus){
    Linfo("report task status");

    Json::Value root;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["accid"] = shmrbt.robot.sn;
    root["title"] = "notify_task_status";
    root["content"]["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    root["content"]["id"] = uid.str();
    root["content"]["schemeName"] = schemeName;
    root["content"]["operationType"] = taskType;
    root["content"]["taskMouldId"] = taskMouldId;
    root["content"]["taskTimestamp"] = taskTimestamp;
    root["content"]["taskStatus"] = taskStatus;
    publish_msg(root);
}

void Notifier::publish_task_point_status(PointInfo pinfo) {
    Linfo("report task point status");

    Json::Reader reader;
    Json::Value root, resp;

    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    shm::TaskInfo taskinfo;
    shm::iMemory_read_TaskInfo(&taskinfo);
    resp["accid"] = shmrbt.robot.sn;
    resp["title"] = "notify_task_point_status";
    resp["content"]["id"] = uid.str();
    resp["content"]["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    resp["content"]["schemeName"] = taskinfo.scheme_name;
    resp["content"]["operationType"] = taskinfo.operation_type;
    resp["content"]["taskMouldId"] = taskinfo.task_mould_id;
    resp["content"]["taskTimestamp"] = taskinfo.task_timestamp;

    resp["content"]["pointBaseId"] = pinfo.pointBaseId;
    resp["content"]["recognitionType"] = pinfo.recognitionType;
    resp["content"]["visiblePicUrl"] = pinfo.visiblePicUrl;
    resp["content"]["thermometryPicUrl"] = pinfo.thermometryPicUrl;
    resp["content"]["audioUrl"] = pinfo.audioUrl;
    resp["content"]["videoUrl"] = pinfo.videoUrl;
    resp["content"]["meterResult"] = pinfo.meterResult;
    resp["content"]["meterType"] = pinfo.meterType;
    resp["content"]["meterIndex"] = pinfo.meterIndex;

/*
    if(!root["pointName"].isNull()){
        resp["content"]["pointName"] = root["pointName"].asString();
    }

    if(!root["recognitionType"].isNull()){
        resp["content"]["recognitionType"] = root["recognitionType"].asString();
    }

    if(!root["meterType"].isNull()){
        resp["content"]["meterType"] = root["meterType"].asString();
    }

    if(!root["audioStatus"].isNull()){
        resp["content"]["audioStatus"] = root["audioStatus"].asBool();
    }

    if(!root["visiblePicUrl"].isNull()){
        resp["content"]["visiblePicUrl"] = root["visiblePicUrl"].asString();
    }

    if(!root["thermometryPicUrl"].isNull()){
        resp["content"]["thermometryPicUrl"] = root["thermometryPicUrl"].asString();
    }

    if(!root["audioUrl"].isNull()){
        resp["content"]["audioUrl"] = root["audioUrl"].asString();
    }

    if(!root["videoUrl"].isNull()){
        resp["content"]["videoUrl"] = root["videoUrl"].asString();
    }
*/
    publish_msg(resp);
}

void Notifier::publish_udock_status(int status) {
    Linfo("report udock status");

    Json::Value root;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["accid"] = shmrbt.robot.sn;
    root["title"] = "notify_udock_state";
    root["content"]["timestamp"] = (uint64_t) (now.toSec() * 1000000000ull);
    root["content"]["id"] = uid.str();
    root["content"]["state"] = status;
    publish_msg(root);
}

void Notifier::_health_cb(const string& data){
    //Linfo("health ws js:%s", data.c_str());
    Json::Reader reader;
    Json::Value root, rbtInfoValue;
    Json::FastWriter fw;
    std::string strRbtInfo = "";
    atris_msgs::RobotInfo rbtInfo;

    if(!reader.parse(data.c_str(), root)){
        Lerror("parse device health state fail");
        return ;
    }

    if(!root["laserTopic"].isNull()){
        lidar_master_state = root["laserTopic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["main_lidar"]["error"] = lidar_master_state;
    }

    if(!root["laser2Topic"].isNull()){
        lidar_slave_state = root["laser2Topic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["slave_lidar"]["error"] = lidar_slave_state;
    }

    if(!root["imuTopic"].isNull()) {
        rbtInfoValue["robot_info"]["gyro"]["error"] = root["imuTopic"].asBool() ? 0 : 1;
    }

    if(!root["odomDeltaSpeed"].isNull()){
        odom_delta_spd = root["odomDeltaSpeed"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["odom"]["error"] = odom_delta_spd;
    }

    if(!root["pointcloudTopic"].isNull()){
        rgbd_state = root["pointcloudTopic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["rgbd"]["error"] = rgbd_state;
    }

    rbtInfoValue["robot_info"]["gaussian_status"] = root;

    rbtInfo.json = fw.write(rbtInfoValue);
    diag_info_pub_.publish(rbtInfo);
}

#if 0
void Notifier::_on_status_action(int code){
    switch(code){
    case 200:
        Lerror("code 200 stopped");
        break;
    case 300:
    case 305:
    case 401:
    case 402:
    case 404:
    case 407:
    case 408:
        _navigator.stop();
        break;
    case 1006:
        _locator.stop();
        break;
    default:
        Linfo("ignore %d", code);
        break;
    }
}

void Notifier::_on_status(int code, const Value& json){
    switch(code){
        case 100:
            _on_pause();
            break;
        case 200:
            //on_stopped();
            break;
        case 300:
            _on_path_invalid();
            break;
        case 301:
            _on_path_to_start_point();//导航到轨道起点处
            break;
        case 302://轨道行走中
            _on_path_running(json);
            break;
        case 303://轨道中行走，遇到障碍物，等待
            _on_path_block();
            break;
        case 304:
            _on_path_avoiding_obstacle();
            break;
        case 305://不能到达
            _on_path_unreachable();
            break;
        case 306://轨道行走结束
            _on_path_finish();
            break;
        case 401://localization error
            _on_localization_error();
            break;
        case 402://goal not safe
            _on_goal_point_not_safe();
            break;
        case 403://前方遇到障碍物
            _on_nav_block();
            break;
        case 404:
            _on_nav_unreached();
            break;
        case 405: //navigating
            _on_nav_navigating();
            break;
        case 406: //planning
            _on_nav_planning();
            break;
        case 407:
            _on_nav_reached();
            break;
        case 408:
            _on_nav_unreachable();
            break;
        case 1006:
            _on_locate_proc(code);
            break;
        default:
            break;
    }
    _on_status_action(code);
}

void Notifier::_on_pause(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_stopped(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_locate_proc(const int code){
    Linfo("%s %d", __FUNCTION__, code);
    if(code == 1006){
    }
}

void Notifier::_on_patrol_finished(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_localization_error(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_goal_point_not_safe(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_avoiding_obstacle(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_invalid(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_running(Json::Value json){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_block(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_unreachable(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_to_start_point(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_path_finish(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_navigating(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_block(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_planning(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_unreachable(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_unreached(){
    Linfo("%s", __FUNCTION__);
}

void Notifier::_on_nav_reached(){
    Linfo("%s", __FUNCTION__);
}
#endif

}

