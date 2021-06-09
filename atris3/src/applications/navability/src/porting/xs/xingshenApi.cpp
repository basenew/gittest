/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
 File Name     : xingshenApi.cpp
 Version       : Initial Draft
 Author        : marty.gong@ubtrobot.com
 Created       : 2020/3/2
 Last Modified :
 Description   : xingshenApi.cpp

 ******************************************************************************/

#include "xingshenApi.h"
#include <sstream>
#include <random>
#include <thread>

#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/NavAbilityMessage.h"

//using namespace xs_sdk;
//using namespace std;


#define ONE_MIN 60
#define XS_API_PORT 42002
#define XS_IP_ADDR "10.20.18.2"

#define RELOCATE_THREAD 75
#define DEFAULT_ZONE -1 
//周期是100ms    modified by marty.gong@ubtrobot.com:2020-3-27-11:5:28 
#define ASYN_TO_SYN_TIME_1S 10
#define ASYN_TO_SYN_TIME_3S 30
#define ASYN_TO_SYN_TIME_5S 10*3*60
#define ASYN_TO_SYN_TIME_10S 100
#define XS_LOG_PATH "/userdata/atris_app/logs/"
#define XS_MAP_PATH "/userdata/tmp/maps/"

#define XS_NAVI_SUB_REGSIS_CHECK ((regisiter_bit_&1) == 1)
#define XS_NAVI_SUB_REGSIS_SET ((regisiter_bit_ = regisiter_bit_| (1<<0)))
#define XS_FAULT_SUB_CHECK (((regisiter_bit_>>1)&1) == 1)
#define XS_FAULT_SUB_SET ((regisiter_bit_ = regisiter_bit_| (1<<1)))
#define XS_ALL_SUB_CHECK ((regisiter_bit_&0x11) == 0x11)

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[xsh_api]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[xsh_api]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[xsh_api]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[xsh_api]"#format, ##args)

int random_char() {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, 255);
    return dis(gen);
}

string generate_uuid() {
    stringstream ss;
    for (int i = 0; i < 16; i++) {
        int rc = random_char();
        stringstream hexstream;
        hexstream << std::hex << rc;
        string hex = hexstream.str();
        ss << (hex.length() < 2 ? string("0") + hex : hex);
    }
    return ss.str();
}

NaviMove::NaviMove():interface_(make_shared<RobotInterface>()){
    if(interface_){
        EnableLog();
        navability_pub_ = nh_.advertise<atris_msgs::NavAbilityMessage>(TOPIC_NAV_ABILITY_RESPONSE_MESSAGE, 100);

        interface_->sig_register_return.connect(this, &NaviMove::ReisterReturn);
        interface_->sig_disconnect.connect(this, &NaviMove::Disconnect);
        interface_->sig_unregister_return.connect(this, &NaviMove::UnregisterReturn);

        interface_->sig_apply_site_return.connect(this,&NaviMove::RespCallback);

        interface_->sig_transfer_map_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_apply_map_return.connect (this,&NaviMove::RespCallback);
        interface_->sig_delete_map_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_rename_map_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_qto_map_info_return.connect(this,&NaviMove::MapListgetRsp);
        interface_->sig_map_transfer_progress.connect(this,&NaviMove::MapTransferProgress);
        interface_->sig_query_map_return.connect(this,&NaviMove::QueryMapInfoRsp);

        interface_->sig_query_localisation_result_return.connect(this,&NaviMove::RobotPoseRsp);
        interface_->sig_control_localisation_ex_return.connect(this,&NaviMove::RelocateRsp);
        interface_->sig_control_localisating_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_start_station_localisation.connect(this,&NaviMove::RespCallback);

        interface_->sig_navigate_ex_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_control_task_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_task_real_status.connect(this,&NaviMove::NavigateStatusRsp);

        interface_->sig_tsp_navigation_return.connect(this,&NaviMove::NaviTspgetRsp);

        interface_->sig_modify_navigation_settting_return.connect(this,&NaviMove::RespCallback);
        interface_->sig_query_navigation_setting_return.connect(this,&NaviMove::NaviConfigGetRsp);

        interface_->sig_navigation_state.connect(this,&NaviMove::RobotNaviStateRsp);

        interface_->sig_fault.connect(this,&NaviMove::RobotFaultStateRsp);

        interface_->sig_qto_station_info_return.connect(this,&NaviMove::PointsListgetRsp);

        interface_->sig_query_version_return.connect(this,&NaviMove::VersiongetRsp);
        interface_->sig_commom_state.connect(this,&NaviMove::StateNotify);
		interface_->sig_query_route_return.connect(this,&NaviMove::RouteRsp);
    }
}

void NaviMove::Init(){
    is_connect = false;
    is_register = false;
    is_running_ = false;
    is_functionregister_ = false;
    func_flag = false;
    mapName_ = mapTemp_ = "";
    ApiMonitorThread_ = new thread(bind( &NaviMove::ApiMonitorThread, this));
    ApiThread_ = new thread(bind( &NaviMove::ApiSlotThread, this));
}

void NaviMove::SubNotifyMsg(int topic, bool sub, int cycle){
    Subscribe param;
    param.topic = topic;
    param.subscribe_type = sub?Subscribe::SubscribeType::subscribe:Subscribe::SubscribeType::unSubscribe;
    param.cycle = cycle;
    int ret = interface_->SubscribeInfomation(param);
    if (ret < 0){
        Linfo("fail : %s topic:%d cycle:%d err:%d", sub?"sub":"unsub", topic, cycle, ret);
    } else {
        Linfo("success : %s topic:%d cycle:%d ", sub?"sub":"unsub", topic, cycle);
    }
}

#define SUB_NAV_STATUS() SubNotifyMsg(27, true, 1000)
#define UNSUB_NAV_STATUS() SubNotifyMsg(27, false, 1000)

void NaviMove::ApiMonitorThread(){
    while(true){
        if(!is_connect || !func_flag){
            if(!is_connect){
                ApiConnect();
            }

            if(!func_flag){
                if(FunctionInit()){
                    func_flag = true;
                }
            }

            if(is_connect && func_flag){
                is_running_ = true;
                Linfo(" api has established.");
                SUB_NAV_STATUS();
            }else{
                is_running_ = false;
                Linfo(" api has not established yet...,please wait.");
            }
        }

		this_thread::sleep_for(chrono::seconds(1));
    }
}

void NaviMove::ApiSlotThread(){
    int i = 0;
    while(true){
        if(interface_){
            //lock_guard<mutex> lock(mutex_interface_);
            interface_->ProcessEvent();
        }
		this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void NaviMove::ApiGetRobotPoseThread(){
    while(true){
        if(interface_){
            //lock_guard<mutex> lock(mutex_interface_);
            interface_->QueryLocalisationResult();
        }
		this_thread::sleep_for(chrono::milliseconds(10));
    }
}

bool NaviMove::FunctionInit(){
    Linfo("%s", __FUNCTION__);
    bool flag_ = true;

    if(!interface_){ 
        Linfo("interface is not ok.");
		return false;
	}

    if(!is_connect) return false;

    if(!is_register){
        if(0 < interface_->RegisterMe()){
            int count_num = 0;
            while(!is_register){
                if(count_num++ > 3){
                    flag_ = false;
                    Linfo("register fail: timeout.\n");
                    break;
                }
                sleep(1);
            }
        }else{
            Linfo("register fail:%s.\n", RobotInterface::ErrorString(interface_->LastError()));
            flag_ =  false;
        }
    }

    if(!is_functionregister_){
        if(!(XS_NAVI_SUB_REGSIS_CHECK)){
            if(!TopicSubscribe(13,1000)){
                Linfo("sub navi topic failed.");
                flag_ = false;
            }else{
                Linfo("sub navi topic successful.");
                XS_NAVI_SUB_REGSIS_SET;
            }
        }
        XS_FAULT_SUB_SET;
        if(XS_ALL_SUB_CHECK){
            is_functionregister_ = true;
        }
    }
    return flag_;
}

void NaviMove::ApiConnect(){
    //lock_guard<mutex> lock(mutex_interface_);
    if(is_connect) return;
		
    if(interface_){
        if (interface_->Connect(XS_IP_ADDR, XS_API_PORT)){
            is_connect = true;
            Linfo("%s ok", __FUNCTION__);
        }else{
            is_connect = false;
            Linfo("%s err:%s", __FUNCTION__, RobotInterface::ErrorString(interface_->LastError()));
        }
    }
}

bool NaviMove::ReqToReqJson(const string &req, ReqJson &reqJson, ReqType type){
    Reader reader;
    FastWriter fw;
    Value val;

    if(reader.parse(req,val)){
        reqJson.type = type;
        reqJson.title = (val["title"].isNull())?(""):(val["title"].asString());
        reqJson.id = (val["content"]["id"].isNull())?(""):(val["content"]["id"].asString());
        reqJson.timestamp =(val["content"]["timestamp"].isNull())? 0:(val["content"]["timestamp"].asLargestInt());
        if (val["content"]["data"].isNull()) {
            reqJson.data = "";
        } else {
            if (val["content"]["data"].isString()) {
               reqJson.data = val["content"]["data"].asString();
            } else {
               reqJson.data = fw.write(val["content"]["data"]);
            }
        }
    }else{
        Linfo("invalid request:%s", req.c_str());
        return false;
    }

    if (type == REQ_NON)return true;

    function<void()> innerParseFun;
    switch(type){
    case REQ_MAP:
        innerParseFun = [&](){ 
        RenameMapReqJson &jsonMap = (RenameMapReqJson&)reqJson;
        jsonMap.origin_map = (val["content"]["origin_name"].isNull())?(""):(val["content"]["origin_name"].asString());
        jsonMap.new_map = (val["content"]["new_name"].isNull())?(""):(val["content"]["new_name"].asString());
        };
        break;
    case REQ_LOC:
        innerParseFun = [&](){ 
        LocReqJson &jsonLoc = (LocReqJson&)reqJson;
        auto &jsonData = val["content"]["data"];
        jsonLoc.x =   (jsonData["position"]["x"].isNull())?(-1):     (jsonData["position"]["x"].asDouble());
        jsonLoc.y =   (jsonData["position"]["y"].isNull())?(-1):     (jsonData["position"]["y"].asDouble());
        jsonLoc.yaw = (jsonData["orientation"]["yaw"].isNull())?(-1):(jsonData["orientation"]["yaw"].asDouble());
        };
        break;
    case REQ_NAV:
        innerParseFun = [&](){ 
        NavReqJson &jsonNav = (NavReqJson&)reqJson;
        jsonNav.mode = (val["content"]["data"]["mode"].isNull())?(-1):(val["content"]["data"]["mode"].asInt());
       // jsonNav.path_name = (val["content"]["data"]["path_name"].isNull())?(""):(val["content"]["data"]["path_name"].asString());
       // jsonNav.point_name= (val["content"]["data"]["point_name"].isNull())?(""):(val["content"]["data"]["point_name"].asString());
        jsonNav.point_id = (val["content"]["data"]["point_id"].isNull()?(""):(val["content"]["data"]["point_id"].asString()));
        Pose2D pose_;
        for(unsigned int i = 0; i< val["content"]["data"]["goal"].size();i++){
            auto &jsonGoal = val["content"]["data"]["goal"];
            pose_.x   = jsonGoal[i]["position"]["x"].isNull()?
                   (-1):jsonGoal[i]["position"]["x"].asDouble();
            pose_.y   = jsonGoal[i]["position"]["y"].isNull()?
                   (-1):jsonGoal[i]["position"]["y"].asDouble();
            pose_.raw = jsonGoal[i]["orientation"]["yaw"].isNull()?
                   (-1):jsonGoal[i]["orientation"]["yaw"].asDouble();

            jsonNav.dest_points.push_back(pose_);
        }
        };
        break;
    case REQ_TSP:
        innerParseFun = [&](){ 
        TspNavReqJson &jsonTspNav = (TspNavReqJson&)reqJson;
        auto &p_list = val["content"]["data"];
    	Linfo("REQ_TSP points sz:%d", p_list.size());
    	for(int i = 0; i < p_list.size(); i++) {
    	    StationInfo sta; 
			//sta.type = StationInfo::StationType::EXACT_STATION;//todo zy
			sta.position_types.push_back(StationInfo::StationPositionType::GAUSS);
    	    sta.station_id = p_list[i]["station_id"].isNull() ? "" : p_list[i]["station_id"].asString();
    	    sta.station_name = p_list[i]["station_name"].isNull() ? "": p_list[i]["station_name"].asString();
    	    //sta.longitude = p_list[i]["longitude"].isNull() ? -1 : p_list[i]["longitude"].asDouble() ;
    	    //sta.latitude = p_list[i]["latitude"].isNull() ? -1 : p_list[i]["latitude"].asDouble();
    	    //sta.azimuth = p_list[i]["azimuth"].isNull() ? -1 : p_list[i]["azimuth"].asDouble();
    	    //sta.building = p_list[i]["building"].isNull() ? "" : p_list[i]["building"].asString();
    	    //sta.uint = p_list[i]["uint"].isNull() ? "" : p_list[i]["uint"].asString();     
    	    //sta.floor = p_list[i]["floor"].isNull() ? "" : p_list[i]["floor"].asString();
    	    sta.type = static_cast<StationInfo::StationType>(p_list[i]["type"].isNull() ? -1 : p_list[i]["type"].asInt());
    	    sta.association_id = p_list[i]["association_id"].isNull() ? -1 : p_list[i]["association_id"].asInt();
    	    sta.gauss_x = p_list[i]["gauss_x"].isNull() ? -1 : p_list[i]["gauss_x"].asDouble();
    	    sta.gauss_y = p_list[i]["gauss_y"].isNull() ? -1 : p_list[i]["gauss_y"].asDouble();
    	    //sta.region_id = p_list[i]["region_id"].isNull() ? -1 : p_list[i]["region_id"].asInt();
    	    Linfo("REQ_TSP point:%s type:%d x:%f y:%f ass_id:%d", sta.station_id.c_str(), sta.type, sta.gauss_x, sta.gauss_y, sta.association_id);
    	    jsonTspNav.dest_points.push_back(sta);
    	} 
        };
		break;
    case REQ_SPD:
        innerParseFun = [&](){ 
        SpdReqJson &jsonSpd = (SpdReqJson&)reqJson;
        jsonSpd.level = (val["content"]["data"]["level"].isNull())?(-1):(val["content"]["data"]["level"].asInt());
        jsonSpd.mode= 1;
        };
        break;
    default:
        Linfo("invalid request:%d", type);
        break;
    }
    innerParseFun();
    return true;
}

const static string PROTO_REQ_PREFIX("request");
const static string PROTO_RSP_PREFIX("response");
const static long unsigned int PROTO_REQ_PREFIX_LEN{PROTO_REQ_PREFIX.size()};

int NaviMove::ParseJson(const string &req, ReqJson &reqJson, RspJson &rspJson, ReqType type, ParseFun fun){
    Linfo("recv %s\n fun:%p",req.c_str(), &fun);
    if(!fun(req, reqJson, type)){
        rspJson.msg = "invaild json";
        Linfo("%s err:%s", rspJson.title.c_str(), rspJson.msg.c_str());
        return -1;
    }

    if (reqJson.title.length() >= PROTO_REQ_PREFIX_LEN){
        rspJson.title = PROTO_RSP_PREFIX + reqJson.title.substr(PROTO_REQ_PREFIX_LEN);    
        rspJson.id = reqJson.id;
        //rspJson.result = "failure";
        //rspJson.code = -1;
        return 0;
    }

    return -1;
}

int NaviMove::Function(string &rsp, ReqJson &reqJson, RspJson &rspJson, ApiFun fun, int timeout){
    int ret = -1;
    do{
        if(interface_ == nullptr){
            rspJson.msg = "the interface init error";
            break;
        }else if(!is_running_){
            rspJson.msg = "the  api can not be used,please wait";
            break;
        }

        int resp_id = 0;
        //mutex_interface_.lock();
        void *resp = nullptr;
        RspType type = RSP_NON;
        if(0 <= (resp_id = fun(resp, type))){
            rspJson.type = type;
            if (reqJson.title == "request_start_locating") {  //todo more elegant solution ?
                rspJson.msg = "start ok";
                rspJson.result = "success";
                rspJson.code = 0;
                ret = 0;
                break;
            }

            if (reqJson.title == "request_get_locating_result") {  //todo more elegant solution ?
                rspJson.msg = "quality: "+ to_string(relocate_result_.qulity);
                rspJson.result = "success";
                rspJson.code = 0;
                ret = 0;
                break;
            }

            if (type == RSP_NON){
                Linfo("normal rsp");
                if (resp == nullptr){
                    rspJson.msg = "inner error:resp is null";
                    break;
                }
                SigRsp *resp_tmp = (SigRsp*)resp;
                //mutex_interface_.unlock();
                resp_tmp->get_ok = false;
                resp_tmp->call_id = resp_id;

                {
                    //lock_guard<mutex> lock(resp_map_lck_);
                    resp_map_[resp_id] = resp;
                }

                if(AsynToSyn(resp_tmp->get_ok, 15)){
                    rspJson.msg = resp_tmp->message;
                    rspJson.result = (0 == resp_tmp->result)? "success":"failure";
                    ret = rspJson.code = resp_tmp->result;
                }else{
                    rspJson.msg = "wait rsp timeout";
                }

                {
                    //lock_guard<mutex> lock(resp_map_lck_);
                    resp_map_.erase(resp_id);
                    delete resp_tmp;
                }
            }else{
                Linfo("unnormal rsp:%d", type);
                *(bool*)resp = false;
                if(AsynToSyn(*(bool*)resp, timeout)){
                    ret = 0;
                }else{
                    rspJson.msg = "wait rsp timeout";
                }
            }
        }
        
        if (resp_id < 0){
            //mutex_interface_.unlock();
            rspJson.msg = RobotInterface::ErrorString(interface_->LastError());
        }

    }while(0);

    rspJson.code = ret;
    if (ret == 0){
        rspJson.result = "success";
    }else{
        Linfo("%s err:%s", rspJson.title.c_str(), rspJson.msg.c_str());
    }
    rsp = JsonToStr(rspJson);

    return ret;
}

int NaviMove::check_valid(const string &req, ReqJson &reqJson, RspJson &rspJson, string &rsp){
    Linfo("%s %s", __FUNCTION__, req.c_str());                
                                                                  
    if(!ReqToReqJson(req, reqJson)){            
        rsp = JsonToStr(rspJson);            
        return -1;                                                
    }                                                             
                                                                  
    rspJson.title = "response" + reqJson.title.substr(7);      
    rspJson.id = reqJson.id;                                   
    rspJson.result = "success";                                  
    rspJson.code = 0;                                            
                                                                  
    if(!is_running_){                                             
        Linfo("the  api can not be used,please wait.");         
    	rspJson.result = "failure";                              
    	rspJson.code = -1;                                       
        rspJson.msg = "the  api can not be used,please wait";  
        rsp = JsonToStr(rspJson);            
        return -1;                                                
    }
    return 0;
}

#define CHECK_VALID()\
    ReqJson reqJson;\
    RspJson rspJson;\
	int ret;\
    ret = check_valid(req, reqJson, rspJson, rsp);\
    if (ret != 0) return ret

int NaviMove::ApplySite(const string &req, string &rsp){
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
    	    SiteInfo site;
    	    site.name = reqJson.data;
        	resp = new SigRsp;
    	    return interface_->ApplySite(site);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::GetMapLists(const string &req, string &rsp){
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            type = RSP_MAP;
    	    GetTaskObject param;
    	    param.object_type = GetTaskObject::MAP_INFO;
        	resp = &(mapsSigRsp_.get_ok);
    	    int ret = interface_->QueryTaskObject(param);
            return ret;
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToHeadStr(rspJson);
	return -1;
}

int NaviMove::GetMapInfo(const string &req, string &rsp){
	CHECK_VALID();
    if(mapInfo_response_.get_ok){
        Linfo("failed to get map info.");
        rspJson.msg = "failed to get map info";
    	rspJson.result = "failure";
    	rspJson.code = -1;
    }
    rsp = MapInfoToStr(rspJson, mapInfo_response_);

    return rspJson.code;
}

int NaviMove::GetTransferMapStatus(const string &req, string &rsp){
    CHECK_VALID();
    if(transmapprogress_response_.current < transmapprogress_response_.total){
        stringstream str_progress;
        str_progress << transmapprogress_response_.current;
        string progress_full = str_progress.str()+"%";
        rspJson.msg = progress_full;
        rsp = JsonToStr(rspJson);
    	Linfo("transfer progress:%s", str_progress.str().c_str());
    }
    rsp = JsonToStr(rspJson);

    return 0;
}

int NaviMove::TransferMap(const string &req, string &rsp, bool isUploadMap){
    Linfo("%s", __FUNCTION__);
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            type = RSP_TRS;
        	MapRequestEx param;
        	param.address = XS_MAP_PATH;
        	param.control_type = (isUploadMap == true)? MapRequestEx::UPLOAD:MapRequestEx::DOWNLOAD;
        	param.map_info.name = reqJson.data;
			param.entities.push_back(MapRequestEx::ETY_ALL);
			transMapSigRsp_.map_name = reqJson.data;
			resp = &transMapSigRsp_.get_ok;
    	    return interface_->TransferMapEx(param);
    	};
        return Function(rsp, reqJson, rspJson, fun, 3*60);
	}
    rsp = JsonToStr(rspJson);//zy todo
	return -1;
}

int NaviMove::UploadMap(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    return  TransferMap(req, rsp, true);
}

int NaviMove::DownloadMap(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    return  TransferMap(req,  rsp, false);
}

int NaviMove::SetMap(const string &req, string &rsp){ 
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
			MapInfo param;
            param.name = reqJson.data;
        	resp = new SigRsp;
    	    return interface_->ApplyMap(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::CancelMap(const string &req, string &rsp){
	//zy todo
	CHECK_VALID();
    rsp = JsonToStr(rspJson);
    return 0;
}

int NaviMove::RenameMap(const string &req, string &rsp){
    RenameMapReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_MAP,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
        	MapRename param;
        	param.new_name = reqJson.new_map;
        	param.old_name = reqJson.origin_map;
        	resp = new SigRsp;
    	    return interface_->RenameMap(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::DelMap(const string &req, string &rsp, bool all){
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
    		MapInfo param;
			param.name = reqJson.data;
        	resp = new SigRsp;
    	    return interface_->DeleteMap(param, all);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::DelAllMap(const string &req, string &rsp){
	return DelMap(req, rsp, true);
}

int NaviMove::getMapStatus(const string &req, string &rsp){
	CHECK_VALID();
    if(mapSet_response_.get_ok){
        rspJson.msg = mapSet_response_.message;
        rspJson.result = (0 == mapSet_response_.result)? "success":"failure";
        rspJson.code = (0 == mapSet_response_.result)? 0:-1;
        if(0 == rspJson.code){
            mapName_ = mapTemp_;
            robot_info_.get_zone_ok = false;
            if(false == QueryMapInfo()){
                Linfo("get map info error.");
                rspJson.msg ="get mapInfo failure";
            }else{
                is_relocate_finished = false;
                Linfo("get map info success.");
            }
        }
        rsp = JsonToStr(rspJson);
        return 0;
    }else{
        rspJson.msg = "map setting";
        rspJson.result = "success";
        rspJson.code = 0;
        rsp = JsonToStr(rspJson);
        return 0;
    }
}

int NaviMove::getUsingMap(const string &req, string &rsp){
	CHECK_VALID();
    rsp = UsingMapToStr(rspJson);
    return 0;
}

//relocate
int NaviMove::StartRelocate(const string &req, string &rsp){
    LocReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_LOC,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
        	StationLocalisation param;
            //param.point_name = "";
        	param.point.x = reqJson.x;
        	param.point.y = reqJson.y;
        	param.point.z = 0;
        	resp = new SigRsp;
    	    return interface_->StartStationLocalisation(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::ControlRelocate(const string &req, string &rsp, LocalisatingControl::ControlType type){
    LocReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_LOC,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &rspType)->int{
        	LocalisatingControl param;
        	param.control_type = type;
        	resp = new SigRsp;
    	    return interface_->ControlLocalisating(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::PauseReLocate(const string &req, string &rsp){
	return ControlRelocate(req, rsp, LocalisatingControl::LOCAL_PAUSE);
}

int NaviMove::ResumeReLocate(const string &req, string &rsp){
	return ControlRelocate(req, rsp, LocalisatingControl::LOCAL_RESUME);
}

int NaviMove::StopReLocate(const string &req, string &rsp){
	return ControlRelocate(req, rsp, LocalisatingControl::LOCAL_STOP);
}

int NaviMove::GetRelocateStatus(const string &req, string &rsp){
	CHECK_VALID();
    if(relocate_result_.get_ok){
        if(RELOCATE_THREAD <= relocate_result_.qulity){
            is_relocate_finished = true;
            Linfo("localisation  success");
            ret = 1;
        }else{
            rspJson.msg = "localisation failed";
            Linfo("localisation failed");
            ret = -1;
        }
    }else{
        rspJson.msg = "localisating";
        Linfo("localisating");
        ret = -1;
    }

    rspJson.code = ret;
	if(ret < 0){
    	rspJson.result = "failure";
	}
    rsp = JsonToStr(rspJson);
	return ret;
}


int NaviMove::GetRelocateResult(const string &req, string &rsp){
    LocReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            LocalisationControlEx param;
            param.control_type = LocalisationControlEx::LOCAL_CHECK;
            resp = new SigRsp;
            return interface_->ControlLocalisationEx(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}


int NaviMove::GetRobotPosition(const string &req, string &rsp){
	CHECK_VALID();
    rsp = PoseToStr(rspJson);
    return 0;
}

//navigate
int NaviMove::StartNavigate(const string &req, string &rsp){
    NavReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NAV,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
        	NavigationPointEx param;
            
           // Linfo("NAV Param:  mode: %d, path_name: %s.",
            //         reqJson.mode, reqJson.path_name.c_str());
            param.point_id = reqJson.point_id; 
        	param.dest_points.assign(reqJson.dest_points.begin(),reqJson.dest_points.end());
        	//param.mode = reqJson.mode;
            param.mode = 1; // 停障模式
            //param.point_name = reqJson.point_name;
         	//param.path_name = reqJson.path_name;
        	param.zone = -1;//(true == robot_info_.get_zone_ok)? robot_info_.navi_state_.zone:DEFAULT_ZONE;
        	resp = new SigRsp;
    	    return interface_->NavigateEx(param);
    	};
        if(0 == Function(rsp, reqJson, rspJson, fun)){
            return RunNavigate(req, rsp);// NaviControlStart
        }else{
            Linfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }

	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::NavigateControl(const string &req, TaskControl::ControlType type, RspJson &rspJson, string &rsp){
    NavReqJson reqJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NAV,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &rspType)->int{
            TaskControl param;
            param.control_type = type;
        	resp = new SigRsp;
    	    return interface_->ControlTask(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::RunNavigate(const string &req, string &rsp){
    CHECK_VALID();
    return NavigateControl(req, TaskControl::ControlType::RUN,rspJson, rsp);   
}

int NaviMove::PauseNavigate(const string &req, string &rsp){
    CHECK_VALID();
    return NavigateControl(req, TaskControl::ControlType::SUSPEND,rspJson, rsp);
}

int NaviMove::ResumeNavigate(const string &req, string &rsp){//zy todo
    CHECK_VALID();
    return NavigateControl(req, TaskControl::ControlType::RESUME,rspJson, rsp);
}

int NaviMove::StopNavigate(const string &req, string &rsp){
    CHECK_VALID();
    return NavigateControl(req, TaskControl::ControlType::CANCEL,rspJson, rsp);
}

int NaviMove::GetNavistatus(const string &req, string &rsp){
	CHECK_VALID();
    rspJson.msg = navi_status_.msg;
    rspJson.code = navi_status_.code;

    rsp = JsonToStr(rspJson);
    return 0;
}

int NaviMove::QueryRoute(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            type = RSP_RUT;
        	resp = &(rutSigRsp_.get_ok);
    	    return interface_->QueryRoute();
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::GetTspNavigate(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    TspNavReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_TSP,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
        	TspNavigationRequest param;
        	memset(&param.start,0,sizeof(param.start));
        	param.zone = (true == robot_info_.get_zone_ok)? robot_info_.navi_state_.zone:DEFAULT_ZONE;
        	param.dest_points = reqJson.dest_points;
            type = RSP_TSP;
        	resp = &(tspSigRsp_.get_ok);
    	    return interface_->TspNavigate(param);
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToStr(rspJson);
	return -1;
}

int NaviMove::GetTspNavigateStatus(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
	CHECK_VALID();
    if(!tsp_result_.get_ok){
        rsp =TspPointsToStr(rspJson);
    }else{
        rspJson.msg =  "running";
        rsp = JsonToStr(rspJson);
    }

    return 0;
}

int NaviMove::GetSpeed(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
        	resp = new SigRsp;//zy todo NaviControlStart
    	    return interface_->QueryNavigationSettting();
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = SpeedToStr(rspJson);//zy todo
	return -1;
}

int NaviMove::SetSpeed(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    SpdReqJson reqJson;
    RspJson rspJson;
    if(!ReqToReqJson(req, reqJson, REQ_SPD)){
        return -1;
    }

    rspJson.title="response_set_speed";    
    rspJson.id = reqJson.id;
    rspJson.result = "failure";
    rspJson.code = -1;

    NavigationSetting::Key key_;

    switch (reqJson.level){
        case 0:
            key_ = NavigationSetting::CURRENT_SPEED_LOW;
            break;
        case 1:
            key_=  NavigationSetting::CURRENT_SPEED_NORMAL;
            break;
        case 2:
            key_ = NavigationSetting::CURRENT_SPEED_HIGH;
            break;
        default :
            key_ = NavigationSetting::CURRENT_SPEED_LOW;
            break;
    }

    int ret = 0;
    if(interface_){
        if(true == ConfigSetting(rspJson, key_,0,rsp)){
            rspJson.result = "success";
        }else{
            rspJson.msg = "ConfigSetting error";
            ret = -1;
        }
    }else{
        Linfo("the interface init error.");
        rspJson.msg = "the interface init error";
        ret = -1;
    }

    if (ret != 0)
        Linfo("%s err:%s", __FUNCTION__, rspJson.msg.c_str());
    rspJson.code = ret;
    rsp = JsonToStr(rspJson);
    return ret;
}

int NaviMove::GetVersion(const string &req, string &rsp){

    Linfo("%s", __FUNCTION__);
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            type = RSP_VERSION;
        	resp = &(versionSigRsp_.get_ok);
    	    int ret = interface_->QueryVersion();
            return ret;
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToHeadStr(rspJson);
	return -1;
}

int NaviMove::GetPointLists(const string &req, string &rsp){
    Linfo("%s", __FUNCTION__);
    ReqJson reqJson;
    RspJson rspJson;
    if (0 == ParseJson(req, reqJson, rspJson, REQ_NON,
                         bind(&NaviMove::ReqToReqJson,
                         this, _1, _2, _3))){
    	auto fun = [&](void *&resp, RspType &type)->int{
            type = RSP_STATION;
    	    GetTaskObject param;
    	    param.object_type = GetTaskObject::STATION_INFO;
        	resp = &(pointsSigRsp_.get_ok);
    	    int ret = interface_->QueryTaskObject(param);
            return ret;
    	};
        return Function(rsp, reqJson, rspJson, fun);
	}
    rsp = JsonToHeadStr(rspJson);
	return -1;
}

void NaviMove::RobotPoseRsp(const LocalisationResult *resp){
    memcpy(&robot_pose_.pose,&(resp->pose),sizeof(struct Pose2D));
    robot_pose_.result = resp->result;
    FastWriter fw;
    string js_resp = fw.write(resp);
    Linfo("%s %s", __FUNCTION__, js_resp.c_str());
}


void NaviMove::RobotFaultStateRsp(const Fault *resp){
    robotFault_.date_time = resp->date_time;
    robotFault_.fault_level= (int)resp->fault_level;
    robotFault_.fault_type = (int)resp->fault_type;
    robotFault_.fault_module = resp->fault_module;

    robotFault_.fault_message = resp->fault_message;
    robotFault_.fault_code = resp->fault_code;
    Linfo("%s code:%s", __FUNCTION__, robotFault_.fault_code.c_str());
}

void NaviMove::RelocateRsp(const LocalisationResponse *resp){
    relocate_result_.qulity = resp->quality;
    relocate_result_.get_ok = true;
    Linfo("%s quality:%d, pos.x : %lf, pos.y : %lf, pos.raw : %lf.", __FUNCTION__, resp->quality, resp->pos.x , resp->pos.y, resp->pos.raw);
}

void NaviMove::NavigateStatusRsp(const TaskRealStatus*resp){
    Linfo("%s", __FUNCTION__);
    switch(resp->status.status){
        case TaskRealStatus::RUNNING:
            navi_status_.code = NAV_ERR_NAVI_NAVIGATING;
            navi_status_.msg = "running";
            break;
        case TaskRealStatus::PAUSING:
            navi_status_.code = NAV_ERR_NAVI_PAUSING;
            navi_status_.msg = "pausing";
            break;
        case TaskRealStatus::STOPPED:
            if(0 == resp->status.code){
                navi_status_.code = NAV_ERR_NAVI_REACHED;
                navi_status_.msg = "finished.";
            }else{
                navi_status_.code = NAV_ERR_NAVI_GOAL_UNREACHED;
                navi_status_.msg = "failed.";
            }
            break;
        default:
            break;
    }
}

void NaviMove::NaviTspgetRsp(const TspNavigationResponse *resp){
    Linfo("%s points sz:%d", __FUNCTION__, resp->ordered_dests.size());
    tspSigRsp_.get_ok = true;
    tspSigRsp_.points = resp->ordered_dests;
	if (tspSigRsp_.points.empty())
	  Lerror("get tsp error:%s", RobotInterface::ErrorString(interface_->LastError()));
}

void NaviMove::RouteRsp(const TaskRoute *resp){
    Linfo("%s route points sz:%d", __FUNCTION__, resp->route_points.size());
    rutSigRsp_.get_ok = true;
    rutSigRsp_.route = *resp;
}

void NaviMove::NaviConfigGetRsp(const NavigationSetting*resp){
    Linfo("%s", __FUNCTION__);
    vector<NavigationSetting::Setting>::const_iterator it ;
    for(it = resp->setting.begin();it != resp->setting.end(); ++it){ 
        if(NavigationSetting::SAFE_DISTACNE == it->key){
            current_config_.safe_dis = it->value;
        }
        if(NavigationSetting::URGENCY_STOP_DISTANCE == it->key){
            current_config_.urge_stop_dis = it->value;
        }
        if(NavigationSetting::OBSTACLE_RADIUS == it->key){
            current_config_.obstacle_radius = it->key;
        }
        if(NavigationSetting::HEADBACK_ALLOW == it->key){
            current_config_.backward_allowd = true;
        }
        if(NavigationSetting::HEADBACK_NOT_ALLOW == it->key){
            current_config_.backward_allowd = false;
        }
        if(NavigationSetting::OBSTACLE_AVOID == it->key){
            current_config_.obstacle_enable = true;
        }
        if(NavigationSetting::OBSTACLE_STOP == it->key){
            current_config_.obstacle_enable = false;
        }   
        if(NavigationSetting::CURRENT_SPEED_LOW == it->key){
            current_config_.level = LOW;
        }
        if(NavigationSetting::CURRENT_SPEED_NORMAL == it->key){
            current_config_.level = NORMAL;
        }
        if(NavigationSetting::CURRENT_SPEED_HIGH == it->key){
            current_config_.level = HIGH;
        }
    }

    current_config_.get_ok = true;

}

void NaviMove::RobotNaviStateRsp(const NavigationState *resp){
    robot_info_.navi_state_.altitude = resp->altitude;
    robot_info_.navi_state_.azimuth = resp->azimuth;
    robot_info_.navi_state_.date_time = resp->date_time;
    robot_info_.navi_state_.global_x = resp->global_x;
    robot_info_.navi_state_.global_y = resp->global_y;
    robot_info_.navi_state_.latitude = resp->latitude;
    robot_info_.navi_state_.longitude = resp->longitude;
    robot_info_.navi_state_.map_name = resp->map_name;
    robot_info_.navi_state_.site_name = resp->site_name;
    robot_info_.navi_state_.speed = resp->speed;
    robot_info_.navi_state_.zone = resp->zone;
    robot_info_.navi_state_.map_info = resp->map_info;
    robot_info_.get_zone_ok = true;

    //robot_info_.navi_state_ = *resp;
    if (!robot_info_.navi_state_.map_name.empty() && robot_info_.navi_state_.map_name != mapName_){
		MapInfo &map_info = robot_info_.navi_state_.map_info;
		Linfo("!!using map:%s gh:%d gw:%d ox:%d oy:%d",
			  map_info.name.c_str(), map_info.grid_info.grid_height,
			  map_info.grid_info.grid_width, map_info.grid_info.origin_x, map_info.grid_info.origin_y);

    	thread t([&](){
    		Linfo("entry query map info thread");
    	    mapName_ = "";
    	   	if (QueryMapInfo()){
    	    	mapName_ = robot_info_.navi_state_.map_name;
    	    }
    	});

    	t.detach();
    }

	PositionNotify(resp->grid_x, resp->grid_y);
    Linfo("RobotNaviStateRsp map:%s z:%d x:%.3f y:%.3f cur_gx:%.3f cur_gy:%.3f gw:%d gh:%d",
          resp->map_name.c_str(), resp->zone, resp->global_x, resp->global_y, resp->grid_x, resp->grid_y,
		  resp->map_info.grid_info.grid_width, resp->map_info.grid_info.grid_height);
}

void NaviMove::PositionNotify(int grid_x, int grid_y){
    Linfo("%s gx:%d gy:%d", __FUNCTION__, grid_x, grid_y);
    atris_msgs::NavAbilityMessage msg;
    msg.title = "notify_robot_pose";
    msg.id = generate_uuid();
    msg.timestamp = (uint64_t)(ros::Time::now().toSec() * 1000);
    stringstream ss;
    ss << "{\"location_x\":" << double(grid_x) << ",\"location_y\":" << double(grid_y) << "}";
    msg.msg = ss.str();
    navability_pub_.publish(msg);
}

void NaviMove::QueryMapInfoRsp(const MapperResponse *resp){
    Linfo("%s", __FUNCTION__);
    mapInfo_response_.range.max_x = resp->range.max_x;
    mapInfo_response_.range.max_y = resp->range.max_y;
    mapInfo_response_.range.min_x = resp->range.min_x;
    mapInfo_response_.range.min_y = resp->range.min_y;
    mapInfo_response_.range.grid_size = resp->range.grid_size;
    mapInfo_response_.get_ok = true;   
	map_ratio_.gauss_w = mapInfo_response_.range.max_x - mapInfo_response_.range.min_x;
	map_ratio_.gauss_h = mapInfo_response_.range.max_y - mapInfo_response_.range.min_y;
	map_ratio_.ratio_w = robot_info_.navi_state_.map_info.grid_info.grid_width/map_ratio_.gauss_w;
	map_ratio_.ratio_h = robot_info_.navi_state_.map_info.grid_info.grid_height/map_ratio_.gauss_h;

    Linfo("map info:(xmin:%.3f,ymin:%.3f),(xmax:%.3f,ymax:%.3f),(gridsize:%d),(gauss_w:%.3f,gauss_h:%.3f),(ratiow:%.3f,ratioh:%.3f)", mapInfo_response_.range.min_x,mapInfo_response_.range.min_y,mapInfo_response_.range.max_x, mapInfo_response_.range.max_y,mapInfo_response_.range.grid_size, map_ratio_.gauss_w, map_ratio_.gauss_h, map_ratio_.ratio_w, map_ratio_.ratio_h); 
} 

void NaviMove::gauss2grid(double gauss_x, double gauss_y, double &grid_x, double &grid_y){
	grid_x = (gauss_x - mapInfo_response_.range.min_x) * map_ratio_.ratio_w;
	grid_y = (gauss_y - mapInfo_response_.range.min_y) * map_ratio_.ratio_h;
}

string NaviMove::RoutePointsToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;
	Linfo("route points sz:%d", rutSigRsp_.route.route_points.size());
	for (auto p:rutSigRsp_.route.route_points){
		gauss2grid(p.gauss_x, p.gauss_y, p.grid_x, p.grid_y);
        Value point;
        point["gauss_x"] = p.grid_x;//todo zy rm gauss_x & gauss_y
        point["gauss_y"] = p.grid_y;
        point["grid_x"]  = p.grid_x;
        point["grid_y"]  = p.grid_y;
		dataJson.append(point);
		Linfo("route point x:%.3f y:%.3f gx:%.3f gy:%.3f", p.gauss_x, p.gauss_y, p.grid_x, p.grid_y);
	}
	rutSigRsp_.route.route_points.clear();
	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::PointListToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;

	vector<StationInfo> *points;
	if (rspJson.type == RSP_TSP){
		Linfo("points type:TSP");
	    points = &(tspSigRsp_.points);
	}else if(rspJson.type == RSP_STATION){
		Linfo("points type:STATION");
		points = &(pointsSigRsp_.points);
	}else{
		Lerror("%s invalid rsp type", __FUNCTION__);
		return "";
	}

	Linfo("point sz:%d", points->size());
	for (auto p:*points){
        Value point;
        point["type"]           = p.type;
        point["station_id"]     = p.station_id;
        point["association_id"] = p.association_id;
        point["station_name"]   = p.station_name;
		if (rspJson.type == RSP_TSP){
			double grid_x, grid_y;
			gauss2grid(p.gauss_x, p.gauss_y, grid_x, grid_y);
			Linfo("tsped gauss_x:%1lf => grid_x:%1lf, gauss_y:%1lf => grid_y:%1lf", p.gauss_x, grid_x, p.gauss_y, grid_y);
			point["gauss_x"]    = grid_x;
			point["gauss_y"]    = grid_y;
		}else{
			point["gauss_x"]    = p.gauss_x;
			point["gauss_y"]    = p.gauss_y;
		}
    	Linfo("%s point id:%s name:%s type:%d gauss_x:%.3f gauss_y:%.3f",
			  __FUNCTION__, p.station_id.c_str(), p.station_name.c_str(), p.type, p.gauss_x, p.gauss_y);

        dataJson.append(point);
    }
    points->clear();

	return JsonToHeadStr(rspJson, dataJson);
}

void NaviMove::MapListgetRsp(const MapInfoResponse *resp){
    Linfo("%s", __FUNCTION__);
    Linfo("!!!!!!!!!!!!!map size : %d", resp->map.size());
    mapsSigRsp_.maps = resp->map;
    mapsSigRsp_.get_ok = true;
}

void NaviMove::PointsListgetRsp(const StationInfoResponse *resp){
    Linfo("%s", __FUNCTION__);

    pointsSigRsp_.points = resp->station;
    pointsSigRsp_.get_ok = true;
    
}

void NaviMove::VersiongetRsp(const VersionResponse *resp){
    Linfo("%s", __FUNCTION__);
    if(resp->remote_version.empty() && resp->local_version.empty()) {
        Linfo("VERSION EMPYT");
    }
    versionSigRsp_.rsp.local_version = resp->local_version;
    versionSigRsp_.rsp.remote_version = resp->remote_version;    
    versionSigRsp_.get_ok = true;
    
}


void NaviMove::StateNotify(const CommonState *state){
    Linfo("nav state:%d", state->state_code);
    atris_msgs::NavAbilityMessage msg;
    msg.title = "notify_nav_state";
    msg.id = generate_uuid();
    msg.timestamp = (uint64_t)(ros::Time::now().toSec() * 1000);
    stringstream ss;
    ss << "{\"statusCode\":" << state->state_code << "}";
    msg.msg = ss.str();
    navability_pub_.publish(msg);
}

void NaviMove::MapTransferProgress(const SyncProgress*resp){
    Linfo("%s total:%f current:%f", __FUNCTION__, resp->total, resp->current);
	if (resp->current >= resp->total){
		string tar_path_name = XS_MAP_PATH + transMapSigRsp_.map_name + ".tar.gz";
		string zip_path_name = XS_MAP_PATH + transMapSigRsp_.map_name + ".zip";
		//string zip_path_name = XS_MAP_PATH + transMapSigRsp_.map_name;
		int ret = interface_->FormatMap("UBX", tar_path_name, zip_path_name);
		Linfo("%s format %s to %s ret:%d %s", __FUNCTION__, tar_path_name.c_str(), zip_path_name.c_str(), ret, RobotInterface::ErrorString(interface_->LastError()));
	    transMapSigRsp_.get_ok = true;
	}
    //transmapprogress_response_.total = resp->total;
    //transmapprogress_response_.current = resp->current;
}

void NaviMove::RespCallback(const SimpleResponse *resp){
    Linfo("%s", __FUNCTION__);
    if (resp){
        //lock_guard<mutex> lock(resp_map_lck_);
        auto it = resp_map_.find(resp->call_id);
        if (it != resp_map_.end()){
            Linfo("response->id:%d,resut:%d,messge:%s",resp->call_id,resp->result,resp->message.c_str());
            SigRsp *sig_resp = static_cast<SigRsp*>(it->second);
            sig_resp->get_ok = true;
            sig_resp->result  = resp->result;
            sig_resp->message = resp->message;
        }else{
            Linfo("timeout response->id:%d,resut:%d,messge:%s",resp->call_id,resp->result,resp->message.c_str());
        }
    }else{
        Linfo("resp is null");
    }
}

void NaviMove::ReisterReturn(int err, const string &info, const string& vehicle_code){
    Linfo("%s err:%d info:%s v_code:%s", __FUNCTION__, err, info.c_str(), vehicle_code.c_str());
    is_register = true;
}

void NaviMove::Disconnect(){
    Linfo("%s", __FUNCTION__);
    is_functionregister_ = false;
    is_register = false;
    func_flag = false;
    is_connect = false;
    regisiter_bit_ = 0;
}

void NaviMove::UnregisterReturn(const SimpleResponse *resp){
    Linfo("%s", __FUNCTION__);
    is_register = false;
}

bool NaviMove::AsynToSyn(const bool &ack_flag, int timeout){
    int times = timeout * 10;
    while(times--) {
        //lock_guard<mutex> lock(mutex_interface_);
        if (ack_flag) return true;
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    return false;
}

bool NaviMove::TopicSubscribe(int topic_,int timer_){
    struct Subscribe param;
    param.topic = topic_;
    param.subscribe_type = Subscribe::subscribe;
    param.cycle = timer_;//ms

    if(0 < interface_->SubscribeInfomation(param)){
        return true;
    }else{
        Linfo("subscribe topic %d fail:%s.\n", topic_,RobotInterface::ErrorString(interface_->LastError()));
        return false;
    }

}


string NaviMove::JsonToStr(RspJson &rspJson){
    if (rspJson.code != 0) return JsonToHeadStr(rspJson);

    switch(rspJson.type){
    case RSP_NON:return JsonToHeadStr(rspJson);
    case RSP_TRS:return JsonToHeadStr(rspJson);
    case RSP_MAP:return MapListToStr(rspJson);
    case RSP_TSP:return PointListToStr(rspJson);
    case RSP_RUT:return RoutePointsToStr(rspJson);//todo zy
    case RSP_STATION:return PointListToStr(rspJson);
    case RSP_VERSION:return VersionToStr(rspJson);
    default:     return "";
    }
}

string NaviMove::JsonToHeadStr(RspJson &rspJson, Value &data){
    Linfo("%s", __FUNCTION__);
    FastWriter fw;                                       
    Value strJson;                          

    strJson["content"]["id"]=rspJson.id;                     
    strJson["content"]["msg"]=rspJson.msg;                   
    strJson["content"]["code"]= rspJson.code;                
    strJson["content"]["result"]=rspJson.result;             
    strJson["content"]["timestamp"]=(uint64_t)(ros::Time::now().toSec() * 1000);                       
    strJson["title"] = rspJson.title;

    if(!data.isNull()){
    	strJson["content"]["data"] = data;        
    }

    return fw.write(strJson);
}

string NaviMove::MapListToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;

    int i=0;
	for (auto m:mapsSigRsp_.maps){
        //dataJson[i++]["mapname"]= m.name;
        dataJson["maps"][i++] = m.name;
    }
    mapsSigRsp_.maps.clear();

	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::VersionToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;
    map<int, string> class_map = {
                        { 0, "RELEASE" },
                        { 1, "BETA" },
                        { 2, "INSIDE" },
                        { 3, "DEBUG" }
                     };

    map<int, string> type_map = {
                        { 0, "PROGRAM" },
                        { 1, "PARAMETER" },
                        { 2, "FIRMWARE" },
                     };
    for (auto lv: versionSigRsp_.rsp.remote_version){
        Linfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Value local_version;
        local_version["class"] = class_map.at(lv.class_);
        local_version["type"] =  type_map.at(lv.type);
        local_version["module_name"] = lv.module_name;
        local_version["version"] = lv.version;
        Linfo("version %s", lv.version.c_str());
        local_version["version_id"] = lv.version_id;
        Linfo("version_id %d", lv.version.c_str());
        dataJson["local_version"].append(local_version);
    }

    for (auto rv: versionSigRsp_.rsp.remote_version){
        Linfo("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        Value remote_version;
        remote_version["class"] = class_map.at(rv.class_);
        remote_version["type"] =  type_map.at(rv.type);
        remote_version["module_name"] = rv.module_name;
        remote_version["version"] = rv.version;        
        remote_version["version_id"] = rv.version_id;
        dataJson["remote_version"].append(remote_version);
    }

    versionSigRsp_.rsp.local_version.clear();
    versionSigRsp_.rsp.remote_version.clear();

	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::TspPointsToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;                          
    int  i = 0;
    for(auto pos:tsp_result_.points){
        dataJson[i]["orientation"]["roll"] = 0;
        dataJson[i]["orientation"]["pith"] = 0;
        dataJson[i]["orientation"]["yaw"]  = pos.azimuth;
        dataJson[i]["position"]["x"] = pos.gauss_x;
        dataJson[i]["position"]["y"] = pos.gauss_y;
        dataJson[i]["position"]["z"] = 0;//zy todo
        dataJson[i]["goal_name"] = pos.station_name;
        i++;
    }
	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::MapInfoToStr(RspJson &rspJson, MapInfoSigRsp &mapInfo){
    Linfo("%s", __FUNCTION__);
    Value dataJson;

    dataJson["minx"]=mapInfo.range.min_x;
    dataJson["miny"]=mapInfo.range.min_y;
    dataJson["maxx"]=mapInfo.range.max_x;
    dataJson["maxy"]=mapInfo.range.max_y;
    dataJson["gridsize"]=mapInfo.range.grid_size;

	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::PoseToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;

  //  if (is_relocate_finished){
        dataJson["x"]   = robot_info_.navi_state_.global_x;
        dataJson["y"]   = robot_info_.navi_state_.global_y;
        dataJson["yaw"] = robot_info_.navi_state_.azimuth;
  //  }else{
  //      rspJson.code   = -1;
  //      rspJson.result = "failure";
  //  }

	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::UsingMapToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson;
    dataJson["map"] = robot_info_.navi_state_.map_name;
	return JsonToHeadStr(rspJson, dataJson);
}

string NaviMove::SpeedToStr(RspJson &rspJson){
    Linfo("%s", __FUNCTION__);
    Value dataJson,angular,linear;

    angular["yaw"]=Value(0);
    angular["roll"]=Value(0);
    angular["pitch"]=Value(0);

    linear["x"]=Value(0);
    linear["y"]=Value(0);
    linear["z"]=Value(0);

    dataJson["angular"]=angular;
    dataJson["linear"]=linear;
    dataJson["mode"]=Value(1);
    dataJson["level"]=Value(current_config_.level);

	return JsonToHeadStr(rspJson, dataJson);
}

bool NaviMove::QueryMapInfo(){
    ReqJson reqJson;
    RspJson rspJson;
    auto fun = [&](void *&resp, RspType &type)->int{
        resp = new SigRsp;
    	return interface_->QueryMap();
    };
    string req;
    return Function(req, reqJson, rspJson, fun);
}

bool NaviMove::ConfigSetting(RspJson &rspJson, NavigationSetting::Key key, double value, string& rsp){
    auto fun = [&](void *&resp, RspType &type)->int{
   	    NavigationSetting param;
   	    NavigationSetting::Setting setting;

   	    setting.key = key;
   	    setting.value = value;
        param.setting.push_back(setting);
   	    resp = new SigRsp;//zy todo NaviControlStart
        return interface_->ModifyNavigationSettting(param);
    };
    ReqJson reqParse;
    return Function(rsp, reqParse, rspJson, fun);
}

void NaviMove::EnableLog(){
    if(interface_){
        interface_->EnableLog((RobotInterface::LogType)(RobotInterface::LT_CONSOLE_LOG | RobotInterface::LT_FILE_LOG),XS_LOG_PATH);
    }
}

