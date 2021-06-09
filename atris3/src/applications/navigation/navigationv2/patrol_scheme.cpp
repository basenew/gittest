#include <string>
#include "patrol_scheme.h"
#include "imemory/atris_imemory_api.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[sch_pat]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[sch_pat]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[sch_pat]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[sch_pat]"#format, ##args)

namespace nav{

int Patrol::plan_route(string schemeName, string taskType, string taskMouldId, std::string taskTimestamp){
	Linfo("%s", __FUNCTION__);
	if (_points.size() <= 1) return ERR_OK;

	vector<GsNavPoint> points;
	//get scheme obs point
	for(auto obs_pt:_points){
		GsNavPoint pt;
		pt.id = obs_pt.id;
		pt.np.name  = obs_pt.name;
		pt.np.pos = obs_pt.pt;
		points.push_back(pt);
		Linfo("before tsp : point id:%s name:%s x:%.3f y:%.3f ang:%.3f",
			  pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y, pt.np.pos.angle);
	}

	//get tsp points
    points.erase(unique(points.begin(), points.end()), points.end()); //去重

	for(auto pt:points){
		ObsPoint obs_pt;
		obs_pt.id = pt.id;
		obs_pt.name = pt.np.name;
		obs_pt.pt = pt.np.pos;
		Linfo("after unique: point id:%s name:%s x:%.3f y:%.3f ang:%.3f",
			  pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y, pt.np.pos.angle);
	}

	int ret = GsApi::get_instance()->get_tsp_navigating(points);
	//notify route points to sw plat form
	if (ret == ERR_OK){
		Linfo("get_tsp_navigating ok");
		vector<GsNavPoint> route_points;
		ret = GsApi::get_instance()->map_get_route_points(route_points);
		if (ret == ERR_OK){
			Linfo("query_route ok");
			Notifier::publish_route_points(schemeName, taskType, taskMouldId, taskTimestamp, points, route_points);
            list<ObsPoint>  obs_temp(_points.begin(),_points.end());
			_points.clear();
			for(auto pt:points){
				ObsPoint obs_pt;
				obs_pt.id = pt.id;
				obs_pt.name = pt.np.name;
				obs_pt.pt = pt.np.pos;
				Linfo("tsped : point id:%s name:%s x:%.3f y:%.3f ang:%.3f",
					  pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y, pt.np.pos.angle);
			}

            for(auto pt:points){
                for(auto obs_pt:obs_temp) {
                    if (pt.id == obs_pt.id) {
                        _points.push_back(obs_pt);
                        Linfo("obs point id:%s name:%s x:%.3f y:%.3f ang:%.3f",
                            pt.id.c_str(), pt.np.name.c_str(), pt.np.pos.x, pt.np.pos.y, pt.np.pos.angle);
                    }
                }
			}
		}
	}

	return ret;
}

int Patrol::parse(const string& scheme_name, Value& scheme)//todo call this function at running?
{
    _sch_name = scheme_name;
    Linfo("parse scheme:%s to reflesh", _sch_name.c_str());
    _clear();

    int ret = _check_json(scheme);
    if (ret != ERR_OK) {
        Linfo("invalid scheme err:%d", ret);
        return ret;
    }

    int pt_sz = scheme["pointInfo"].size();
    Linfo("PointInfo size : %d.", pt_sz);
    _points.clear();
    for (int i = 0; i < pt_sz; i++) {
        Value &js_pt = scheme["pointInfo"][i];
		Json::FastWriter jw;
		string pinfo = jw.write(js_pt);
        ObsPoint pt;
        //pt.name = js_pt["locationName"].asString();
		Linfo("pinfo %d:%s", i, pinfo.c_str());
		if (js_pt["locationX"].asString().empty()
		||  js_pt["locationY"].asString().empty()
		||	js_pt["locationOrientation"].asString().empty()){
			Lerror("invalid point info json");
			continue;
		}

        pt.pt.x = stod(js_pt["locationX"].asString());
        pt.pt.y = stod(js_pt["locationY"].asString());
        pt.pt.angle = stod(js_pt["locationOrientation"].asString());
        pt.id = js_pt["mapPointId"].asString();
        
        pt.op.swh = SW_ENABLE; // todo pass by scheme ? //TODO kevin chen
        pt.op.ptz.swh = SW_ENABLE; //todo pass by scheme ?  //TODO kevin chen
        pt.op.ptz.h = atof(js_pt["cameraHangle"].asString().c_str());
        pt.op.ptz.v = atof(js_pt["cameraVangle"].asString().c_str());
        pt.op.ptz.z = atof(js_pt["cameraZoom"].asString().c_str());
        pt.op.ptz.f = atof(js_pt["cameraFoucs"].asString().c_str());
        pt.op.ptz.capi = js_pt["captureInfrared"].asBool() ? 1 : 0;
        pt.op.ptz.capv = js_pt["captureVisibleLight"].asBool() ? 1 : 0;
        pt.op.ptz.reci = js_pt["recordVideo"].asBool()? 1: 0;
        pt.op.ptz.recv = js_pt["recordVideo"].asBool()? 1: 0;
        pt.op.ptz.reca = js_pt["recordAudio"].asBool()? 1: 0;
        pt.point_base_id = js_pt["pointBaseId"].asInt();
		if (!js_pt["meterIndex"].isNull() && !js_pt["meterIndex"].asString().empty()){
			pt.op.vision.meterIndex = js_pt["meterIndex"].asString();
		}
        pt.op.vision.meterType = js_pt["meterType"].asString();
        pt.op.vision.meterModel = js_pt["meterModel"].asString();
        pt.op.vision.recognitionType = js_pt["recognitionType"].asString();

		if (pt.op.vision.recognitionType == "thermometry_equipment_view_data"){
			pt.op.vision.recognitionType = "infrared_thermometry";
			pt.op.ptz.capv = 1;
			pt.op.ptz.capi = 1;
		}

        pt.op.vision.temperatureFramePoint = js_pt["temperatureFramePoint"].asString();
        pt.op.vision.deviceFramePoint = js_pt["deviceFramePoint"].asString();        
        _points.push_back(pt);
    }

    //todo push task??
    TASK_LOCK();
    _updated = true;

    _valid = (ret == ERR_OK);
    _tf.name("tf_" + _sch_name);
    return ret;
}

int Patrol::_task_action(int op){

/*
    atris_msgs::LampCmd msg;
    if (op)
      msg.status = atris_msgs::LampCmd::STATUS_ON;
    else
      msg.status = atris_msgs::LampCmd::STATUS_OFF;
    
    if (_sch_op.swh == SW_ENABLE){
        Linfo("action task enable");
        int start = time(nullptr);
        if (_sch_op.bc.swh == SW_ENABLE){
            Linfo("exec bc task");
            if (op){
                atris_msgs::GetPPPlayingStatus status;
                get_ppplaying_status_srv_client_.call(status);
                if(!status.response.status  || status.response.owner != atris_msgs::GetPPPlayingStatus::Response::OWNER_NAV) {
                    Linfo("NAV start play music");
                    if (_sch_op.bc.lst.size() > 0) {
                    
                    Json::Value root;
                    Json::FastWriter jw;
                    for (std::size_t i = 0; i < _sch_op.bc.lst.size(); i++) {
                      root.append(_sch_op.bc.lst[i]);
                    }
                    atris_msgs::PPPlayerControl pppctrl;
                    pppctrl.request.play_list_json = jw.write(root);
                    pppctrl.request.play_mode = _sch_op.bc.md;
                    pppctrl.request.play_loop = true;
                    pppctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_PLAY;
                    ppplayer_control_srv_client_.call(pppctrl);
                    if (!pppctrl.response.result) {
                        log_warn("Patrol::_task_action(): ppplayer_control_srv_client play failed.");
                    }
                    } else {
                        log_warn("Patrol::_task_action(): ppplayer list is emtpy.");
                    }
                }
                else
                    Linfo("NAV continue play music");
            
            }
            else{
                Linfo("stop play music"publish_task_point_statusient_.call(pppctrl);
                if (!pppctrl.response.result) {
                    log_warn("Patrol::_task_action(): ppplayer_control_srv_client stop failed.");
                }
            }
        }

        if (_sch_op.lamp.swh == SW_ENABLE){
            Linfo("exec lamptask");
            if (_sch_op.lamp.white){
                Linfo("set lamp white:%d", op);
                msg.type = atris_msgs::LampCmd::LAMP_TYPE_WHITE;
                msg.source = atris_msgs::LampCmd::LAMP_SOURCE_NAV;
                lamp_cmd_pub_.publish(msg);
            }
            if (_sch_op.lamp.rb){
                Linfo("set lamp rb:%d", op);
                msg.type = atris_msgs::LampCmd::LAMP_TYPE_FLASH;
                msg.source = atris_msgs::LampCmd::LAMP_SOURCE_NAV;
                lamp_cmd_pub_.publish(msg);
            }
        }
    }
    else
        Linfo("task action disable");
*/

    return ERR_OK;
}

bool Patrol::_reload(){
    if (_points.empty()){
        Lerror("_points is empty!!");
        return false;
    }

    if (_updated){
        _tf.clear();
        _construct_task();
        _updated = false;
    }

    return true;
}

bool Patrol::_construct_task() {
    Linfo("$$$$$$$$$$$$$$$$point list 0 task size:%d$$$$$$$$$$$$$$$$", _tasks.size());
    //if (_tasks.empty()){
        list<Task*> tasks = _tasks;
        list<Task*> ptz_tasks = _ptz_tasks;
        Linfo("$$$$$$$$$$$$$$$$$$$$point list 0$$$$$$$$$$$$$$$$$$$$");
        int pt_cnt = 0;
        NavPointTask *nav_t;
        TaskNode     *pre_t = nullptr;
        PTZTask      *ptz_t;
        //Operation& op = _sch_op;
        for(auto pt: _points) {
            if (!tasks.empty()){
                nav_t = (NavPointTask*)tasks.front();
                tasks.pop_front();
                nav_t->clear();
            }
            else{
                nav_t = new NavPointTask();
                _tasks.push_back(nav_t);
            }

            GsNavPoint gs_pt;
            gs_pt.id = pt.id;
            Linfo("###################################point id : %s", pt.id.c_str());
            gs_pt.np.pos  = pt.pt;
            gs_pt.np.name = pt.name;
            nav_t->type(_type);
            nav_t->point(gs_pt);
            nav_t->name("nav_tsk_" + pt.name);
            _tf.push(nav_t);

            if (pt.op.ptz.swh) {
                Linfo("append ptz task");
                if (!ptz_tasks.empty()){
                    ptz_t = (PTZTask*)ptz_tasks.front();
                    ptz_tasks.pop_front();
                    ptz_t->clear();
                }
                else{
                    ptz_t = new PTZTask();

                    _ptz_tasks.push_back(ptz_t);
                }
                ptz_t->name("ptz_tsk_"+ pt.name);    
                ptz_t->op(pt.op.ptz);
                ptz_t->behind(nav_t);
                PointInfo pinfo;
                pinfo.pointBaseId = std::to_string(pt.point_base_id);
                pinfo.meterIndex =  pt.op.vision.meterIndex;
                pinfo.meterType =  pt.op.vision.meterType;
                pinfo.recognitionType = pt.op.vision.recognitionType;
                pinfo.meterModel = pt.op.vision.meterModel;
                pinfo.temperatureFramePoint = pt.op.vision.temperatureFramePoint;
                pinfo.deviceFramePoint = pt.op.vision.deviceFramePoint;
                ptz_t->Pinfo(pinfo);
                _tf.push(ptz_t);
            }

            if (pre_t){
                list<TaskNode*> &behinds = pre_t->behinds();
                if (behinds.empty()){
                    Linfo("%s behind1 %s", nav_t->name().c_str(), pre_t->name().c_str());
                    nav_t->behind(pre_t);
                }
                else{
                    for(auto t:behinds){
                        Linfo("%s behind2 %s", nav_t->name().c_str(), t->name().c_str());
                        nav_t->behind(t);
                    }
                }
            }

            pre_t = nav_t;
            Linfo("point %d:%s", ++pt_cnt, nav_t->name().c_str());

        }
        _append_action();
        Linfo("$$$$$$$$$$$$$$$$$$$$ point list task size:%d $$$$$$$$$$$$$$$$$$$$", _tasks.size());
        Linfo("$$$$$$$$$$$$$$$$$$$$ ptz task size:%d $$$$$$$$$$$$$$$$$$$$", _ptz_tasks.size());
    //}

    return !_tf.empty();
}

bool Patrol::_append_action(){
    for (size_t i = 0; i < _tf.size(); ++i){
        if (_tf[i]->type() == Task::TASK_PATROL){
            Linfo("%s append action", _tf[i]->name().c_str());
            _tf[i]->set_action(AFT_FIN, [this](){
                return _task_action(SW_ENABLE);
            }); 
        }
    }
    
    return true;
}


void Patrol::on_running(){
    if (!_reload()){
        _nxt_st = FIN;
        return;
    }
    
/*
    atris_msgs::PtzControl ptzctrl;
    ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_MOVE_DEFAULT_POINT;
    ptz_control_srv_client_.call(ptzctrl);
*/
    Linfo("!!!!!!!!!!!!!!!!!---%s %s start exec ---!!!!!!!!!!!!!!!", 
          _name.c_str(), _tf.name().c_str());

    NavPointTask* start = (NavPointTask*)_tf.begin();
    start->auto_nav_enable(false);
    _thd.push(&_tf);
    _tf.run();
    Linfo("run task:%s ok", _tf.name().c_str());

    //char rand_str[16] = {0};
   // Utils::get_instance()->getRandStr(rand_str);
    //_patrol_serial_num = rand_str;
   // notifyPatrolStatus(1);
}

void Patrol::on_finished(){
    Scheme::on_finished();
    Linfo("!!!!!!!!!!!!!!!!!!+++ %s %s %d %d %d finish execed  +++!!!!!!!!!!!!!",
         _name.c_str(), _tf.name().c_str(), _type, _tf.result().ret, _tf.is_stop_flow());
    //notifyPatrolStatus(0);
    if (_type == Task::TASK_PATROL) {
        Ldebug("_tf.type() = Task::TASK_PATROL finished.");
        shm::TaskInfo shmtask;
        shm::iMemory_read_TaskInfo(&shmtask);
        sched::Date dtstart;
        dtstart.ConvertTimestampToDate(atoi(shmtask.task_timestamp));
        if (!_tf.result().ret) {
            Ldebug("normal stop patrol!");
            scheduler.update_event_instance(dtstart, "status", std::to_string(sched::FINISH));
            Notifier::publish_task_status(std::string(shmtask.map_name), std::string(shmtask.scheme_name),
                                        std::string(shmtask.operation_type), std::string(shmtask.task_mould_id),
                                        std::string(shmtask.task_timestamp), "finish");
        } else {
            Ldebug("force stop patrol!");
            scheduler.update_event_instance(dtstart, "status", std::to_string(sched::TERMINATION));
            Notifier::publish_task_status(std::string(shmtask.map_name), std::string(shmtask.scheme_name),
                                        std::string(shmtask.operation_type), std::string(shmtask.task_mould_id),
                                        std::string(shmtask.task_timestamp), "termination");
        }

        Linfo("[Patrol::on_finished] sent request_recharge");
        Json::Value charge_req;
        Json::FastWriter fw;
        charge_req["title"] = "request_recharge";
        charge_req["content"]["switch"] = 1;
        atris_msgs::SignalMessage req;
        std::string charge_data = fw.write(charge_req);
        req.title = "request_recharge";
        req.msg = charge_data.c_str();
        signal_msg_pub_.publish(req);
    }
    Linfo("000000000000000000000000000000000");
}


void Patrol::on_loop(){
    Linfo("%s on loop", _name.c_str());
    if (_tf.is_finished()){
        Linfo("!!!!!!!!!!!!%s %s is finished!!!!!!!!!", _name.c_str(), _tf.name().c_str());
        if (_tf.is_stop_flow()){
            _ret = _tf.result();
            Linfo("!!!!!!!!!!!!%s %s force to stop!!!!!!!!!",
                  _name.c_str(), _tf.name().c_str());
        }
         _nxt_st = FIN;
    }
}

void Patrol::_clear(){
    Linfo("%s clear", _name.c_str());
    Scheme::_clear();
    _sch_op = {};
    _ret_pt = {};
    _paths.clear();
    _points.clear();
   // _tasks.clear();
    _valid = false;
    _enable = true;
    _chg_enable = true;
    _chg_pt_name = "charge";
}


}
