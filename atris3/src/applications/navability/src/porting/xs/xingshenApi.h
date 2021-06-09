#pragma once
/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
 File Name     : xinshenApi.h
Version       : Initial Draft
Author        : marty.gong@ubtrobot.com
Created       : 2020/3/2
Last Modified :
Description   : xingshenApi.h

 ******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/
//#include <ros.h>
#include <string>
#include <map>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <json/json.h>
#include <thread>
#include <ros/ros.h>

#include "navigateRobotCommon.h"
#include "xs/sdkcommom.h"
#include "xs/robotinterface.h"

using namespace std;
using namespace xs_sdk;
using namespace Json;

enum ReqType{
    REQ_NON,
    REQ_LOC,
    REQ_NAV,
    REQ_SPD,
    REQ_MAP,
    REQ_TSP,
};

enum RspType{
    RSP_NON,
    RSP_LOC,
    RSP_NAV,
    RSP_SPD,
    RSP_MAP,
    RSP_TSP,
    RSP_RUT,
	RSP_TRS,
    RSP_STATION,
    RSP_VERSION,
};

struct ReqJson{
    ReqType type;
    string title;
    string id;
    string data;
    long timestamp;
};

struct MapRatio{
	double gauss_w;
	double gauss_h;
	double ratio_w;
	double ratio_h;
};

struct LocReqJson:public ReqJson{
    double x;
    double y;
    double yaw;
};

struct NavReqJson:public ReqJson{
    int mode;
    string path_name;
    vector<Pose2D> dest_points;
    string point_name;
    string point_id;
};

struct TspNavReqJson:public ReqJson{
	std::vector<StationInfo> dest_points;
};

struct SpdReqJson:public ReqJson{
    int mode;
    int level;
};

struct RenameMapReqJson:public ReqJson{
    string origin_map;
    string new_map;
};

struct RspJson{
    RspType type;
    string title;
    string id;
    long timestamp;
    string msg;
    int code;
    string result;
};

struct SigRsp:public SimpleResponse{
    bool get_ok;
};

struct MapInfoSigRsp:public SigRsp{
    MapperResponse::MapRange range;
};

struct SigRelcateResult:public SigRsp{
    int32_t qulity;
};

struct MapsSigRsp:public SigRsp{
    vector<MapInfo> maps;
};

struct PointsSigRsp:public SigRsp{
    vector<StationInfo> points;
};

struct RutSigRsp:public SigRsp{
	TaskRoute route;
};

struct TransSigRsp:public SigRsp{
	string map_name;
};

using TspSigRsp = PointsSigRsp;

struct VersionSigRsp:public SigRsp{
    VersionResponse rsp;
};

typedef enum{
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
}SpeedLevel;

struct SigNaviConfigResp{
    double safe_dis;
    double urge_stop_dis;
    double obstacle_radius;
    bool backward_allowd;
    bool obstacle_enable;
    bool get_ok;
    SpeedLevel level;
};

struct RobotInfo{
    NavigationState navi_state_;
    bool get_zone_ok;
};

struct RobotFault{
    int64_t date_time;//timer
    string fault_code;
    string fault_message;
    int fault_module;//0:sensor 1:chassis 2:app
    int fault_type; //0:occour 1:remove
    int fault_level; //0-7
};

struct NaviError{
    ROBOT_ERR_CODE code;
    string msg;
};

class NaviMove:public sigslot::has_slots<>{
public:
    NaviMove();
    ~NaviMove(){
        if(interface_){
            interface_->UnregisterMe();
        }
    }

    void Init();

    using ApiFun = function<int(void*&, RspType&)>;
    using ParseFun = function<bool(const string&, ReqJson&, ReqType)>;

    int Function(string &rep, ReqJson &reqJson, RspJson &rspJson, ApiFun fun, int timeout = 60);
    int ParseJson(const string &req, ReqJson &reqJson, RspJson &rspJson, ReqType type, ParseFun fun);
#if 0
    int StartMapping(const string &req, string &rsp);
    int PauseMapping(const string &req, string &rsp);
    int ResumeMapping(const string &req, string &rsp);
    int StopMapping(const string &req, string &rsp);
#endif

    int ApplySite(const string &req, string &rsp);
    int GetMapLists(const string &req, string &rsp);
    int UploadMap(const string &req, string &rsp);
    int DownloadMap(const string &req, string &rsp);
    int GetTransferMapStatus(const string &req, string &rsp);
    int GetMapInfo(const string &req, string &rsp);
    int SetMap(const string &req, string &rsp);
    int RenameMap(const string &req, string &rsp);
    int CancelMap(const string &req, string &rsp);
    int DelMap(const string &req, string &rep, bool all = false);
    int DelAllMap(const string &req, string &rsp);
    int getMapStatus(const string &req, string &rsp);    
    int getUsingMap(const string &req, string &rsp);
    //relocate
    int StartRelocate(const string &req, string &rsp);
    int PauseReLocate(const string &req, string &rsp);
    int ResumeReLocate(const string &req, string &rsp);
    int StopReLocate(const string &req, string &rsp);
    int GetRelocateStatus(const string &req, string &rsp);
    int GetRobotPosition(const string &req, string &rsp);
    int ControlRelocate(const string &req, string &rep, LocalisatingControl::ControlType);
    int GetRelocateResult(const string &req, string &rsp);
    //navigate
    int StartNavigate(const string &req, string &rsp);
    int RunNavigate(const string &req, string &rsp);
    int PauseNavigate(const string &req, string &rsp);
    int ResumeNavigate(const string &req, string &rsp);
    int StopNavigate(const string &req, string &rsp);
    int GetNavistatus(const string &req, string &rsp);
    int GetTspNavigate(const string &req, string &rsp);
    int GetTspNavigateStatus(const string &req, string &rsp);
    int QueryRoute(const string &req, string &rsp);

    //speed
    int SetSpeed(const string &req, string &rsp);
    int GetSpeed(const string &req, string &rsp);

    int GetVersion(const string &req, string &rsp);

    // point
    int GetPointLists(const string &req, string &rsp);
private:
    int NaviControlStart(RspJson &rspJson);
    int NavigateControl(const string &req, TaskControl::ControlType type,RspJson &rspJson, string &rsp);
    void RespCallback(const SimpleResponse *resp);
    void UnregisterReturn(const SimpleResponse *resp);
    void RobotPoseRsp(const LocalisationResult *resp);
    void RobotNaviStateRsp(const NavigationState *resp);
    void MapTransferProgress(const SyncProgress *resp);
    void RelocateRsp(const LocalisationResponse *resp);
    void NavigateStatusRsp(const TaskRealStatus *resp);
    void NaviTspgetRsp(const TspNavigationResponse *resp);
	void RouteRsp(const TaskRoute *resp);
    void NaviConfigGetRsp(const NavigationSetting *resp);
    void QueryMapInfoRsp(const MapperResponse *resp);
    void MapListgetRsp(const MapInfoResponse *resp);
    void PointsListgetRsp(const StationInfoResponse *resp);
    void VersiongetRsp(const VersionResponse *resp);
	void StateNotify(const CommonState *state);
	void PositionNotify(int grid_x, int grid_y);
    void RobotFaultStateRsp(const Fault *resp);
    void ReisterReturn(int err, const string &info, const string& vehicle_code);
    void Disconnect();
    void ApiSlotThread();
    void ApiMonitorThread();
    void ApiGetRobotPoseThread();
    void ApiConnect();
	void SubNotifyMsg(int topic, bool sub, int cycle = 100);

    int  check_valid(const string &req, ReqJson &reqJson, RspJson &rspJson, string &rsp);

    bool ReqToReqJson(const string &req, ReqJson &reqJson, ReqType type = REQ_NON);
    bool AsynToSyn(const bool &ack_flag, int timeout = 30);

    string JsonToStr(RspJson &rspJson);
    string JsonToHeadStr(RspJson &rspJson, Value &data = const_cast<Value&>(Value::null));
    string TspPointsToStr(RspJson &rspJson);
    string MapInfoToStr(RspJson &rspJson, MapInfoSigRsp &mapInfo);
    string PoseToStr(RspJson &rspJson);
    string UsingMapToStr(RspJson &rspJson);
    string SpeedToStr(RspJson &rspJson);
    string MapListToStr(RspJson &rspJson);
    string PointListToStr(RspJson &rspJson);
    string VersionToStr(RspJson &rspJson);
	string RoutePointsToStr(RspJson &rspJson);

    //other
    int TransferMap(const string &req, string &rsp, bool isUploadMap);
	void gauss2grid(double gauss_x, double gauss_y, double &grid_x, double &grid_y);
    bool QueryMapInfo(); 
    int GetCurrentZone(){
        return robot_info_.get_zone_ok?robot_info_.navi_state_.zone:-1;
    };
    bool ConfigSetting(RspJson& rspJson, NavigationSetting::Key key, double value_,string& rsp);
    bool TopicSubscribe(int topic,int timer);
    bool FunctionInit();
    void EnableLog();

    void InitConfig();

private:
    thread *ApiThread_,*ApiMonitorThread_,*ApiRobotPoseThread_,*ApiConfigInitThread_;

    shared_ptr<RobotInterface> interface_;
    bool is_connect,is_register,func_flag,is_functionregister_,is_running_,is_relocate_finished;

    map<int, void*> resp_map_;

    SigRsp mapSet_response_;
	TransSigRsp transMapSigRsp_;
    MapInfoSigRsp mapInfo_response_;
    SigRelcateResult relocate_result_;
    LocalisationResult robot_pose_;
    MapsSigRsp mapsSigRsp_;
    PointsSigRsp pointsSigRsp_;
    TspSigRsp tspSigRsp_;
	TspSigRsp tsp_result_;
	RutSigRsp rutSigRsp_;
    VersionSigRsp versionSigRsp_;


    string mapName_,mapTemp_;
    mutex mutex_interface_;
    mutex resp_map_lck_;
    SigNaviConfigResp current_config_;
    RobotInfo robot_info_;
    NaviError navi_status_;
    RobotFault robotFault_;

    int regisiter_bit_;
    int32_t current_zone_;

    SyncProgress transmapprogress_response_;

	MapRatio map_ratio_;
    ros::NodeHandle nh_;
    ros::Publisher navability_pub_;
};

