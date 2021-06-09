#include <fstream>
#include <unistd.h>
#include "locate_task.h"
#include "map_manager.h"
#include "scheme_manager.h"
#include "transferfile/transferfile.h"
#include "trans_notifier.h"
#include "scheduler/scheduler.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[sch_mgr]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[sch_mgr]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[sch_mgr]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[sch_mgr]"#format, ##args)

#define PARSE_JSON()    if(!reader.parse(msg.msg, req)){\
                            Lerror("parse json fail.");\
                            ret = ERR_INVALID_JSON;\
                            break;\
                        }

#define CHARGE_SCHEME_NAME    "charge"
namespace nav{
typedef std::function<void(sched::Event_Instance)> Fun;

bool SchemeManager::doGetPatrolStatus(atris_msgs::GetPatrolStatus::Request& req,
  atris_msgs::GetPatrolStatus::Response& res) {
    res.status = get_patrol_status();
    return true;
}

int SchemeManager::_start_auto_charge(){
       
    if (_chg_sch.is_charge_pt_valid()){
        Linfo("wait confirm timeout to auto charge");
        _pat_sch.stop();
        int sw = SW_ENABLE;
        int ret = _control_task(_chg_sch,sw);
        Linfo("%s : ret: %d", __FUNCTION__, ret);
    }else{
        Linfo("wait confirm timeout but auto charge condition is not ok");
    }
    return ERR_OK;
}

void SchemeManager::on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg)
{
    Linfo("SchemeManager on_receive_charge_state");
    if (msg.charge_msg_type == atris_msgs::PowerChargeCmd::CHASSIS_NOTIFY_CHARGE_STATUS) {
        if (msg.charge_status == 0x02) { 
            if (msg.charge_result == 0x00) {
                _is_leaving_pile = true;
                shm::ChargeState shm_state;
                shm_state.state = dockSDKState::RECEIVE_LEAVE_CMD;
                shm::iMemory_write_ChargeState(&shm_state);
            } else if(msg.charge_result == 0x02) {
                Linfo("leave pile fail ........");
            }
        }
    }
}

void SchemeManager::on_receive_network_state(const atris_msgs::NetworkState& msg)
{
    Linfo("%s network state: %d", __FUNCTION__, is_network_connected);
    is_network_connected = msg.network_state;
}

void SchemeManager::_handle_charge_msg(const atris_msgs::ChargeCmd &msg){
  Linfo("%s", __FUNCTION__);
 /*
  if (msg.cmd == PM_CMD_PC_LEAVE_PILE && msg.data[0] == 0){
      Linfo("PM_CMD_PC_LEAVE_PILE");
      if (!_pat_sch.have_task_to_run()){
          Linfo("none task to run after leave pile");
          return;
      }else{
      
      }

      Linfo("try run patrol after leave pile");

      if (Navigation::get_instance().is_locating()){
          Linfo("is locating");
          return; 
      }else if(!_pat_sch.enable()){
          Linfo("patrol scheme is disable");
          return;
      }else if(_chg_sch.is_active() || _ret_pt_tsk.is_active() || Navigation::get_instance().is_navigating()){
          Linfo("other task is running");
          return;
      }

      if(!_pat_sch.is_active()){
          int ret = GsApi::get_instance()->init();
          if(ret != ERR_OK){
              log_error("gaussian api init fail.");
              return;
          }else{
              log_error("gaussian api init success.");
          }

          string using_map = MapManager::get_instance().get_using_map();
          if (using_map.empty()){
              using_map = MapManager::get_instance().get_using_map_from_db();
              if (using_map.empty()){
                  Linfo("not using map");
                  return;
              }

              ret = MapManager::get_instance().load_map();
              if (ret != ERR_OK){
                  if (ret == ERR_MAP_LOADING){
                      int try_times = 120;
                      while ((ret = MapManager::get_instance().load_map()) == ERR_MAP_LOADING && --try_times){
                          this_thread::sleep_for(chrono::seconds(1));
                      }
                  }

                  if (ret != ERR_OK){
                      Linfo("try load map:%s err:%d", using_map.c_str());
                      return;
                  }
              }
          }

          if (GsApi::get_instance()->get_init_status(using_map) != ERR_OK){  
              Linfo("not inited by %s", using_map.c_str());
              return;
          }

          if (_load_scheme() != ERR_OK){
              Linfo("load scheme fail");
              return;
          }

          _pat_sch.set_times_from_db();
          _pat_sch.set_interval_from_db();
          _thd.push(&_pat_sch);
          Linfo("start %s", _pat_sch.name().c_str());
          ret = _pat_sch.start();
          Linfo("start %s ret:%d", _pat_sch.name().c_str(), ret);
      }
      _pat_sch.rm_from_db();
  }
  */
}

void SchemeManager::_handle_msg(const atris_msgs::SignalMessage &msg){
    Json::Reader reader;
    Json::Value req, resp;

    int ret = ERR_OK;
    std::string result("success");
    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;

    do{
        if(msg.title == "request_get_schedule_list"){
            PARSE_JSON();
            ret = _get_schedule_list(req, resp, result);
        }
        else if(msg.title == "request_get_schedule"){
            PARSE_JSON();
            ret = _get_schedule(req, resp, result);
        }
        else if(msg.title == "request_sync_all_schedule"){
            PARSE_JSON();
            ret = _sync_all_schedule(req, resp, result);
        }
        else if(msg.title == "request_sync_schedule"){
            PARSE_JSON();
            ret = _sync_schedule(req, resp, result);
        }
        else if(msg.title == "request_sync_patrol_scheme"){
            PARSE_JSON(); 
            if(req["content"]["url"].isNull() || req["content"]["mapname"].isNull()){
                Lerror("no patrol scheme url");
                result = "fail_invalid_data";
                ret = ERR_INVALID_FIELD;
                break;
            }

            string map_name = req["content"]["mapname"].asString();
            Linfo("sync scheme map:%s uisng:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
            if (map_name != MapManager::get_instance().get_using_map()){
                Lerror("%s is not using map:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
                ret = ERR_MAP_NOT_USING;
                break;
            }

            string url = req["content"]["url"].asString();
            ret = download(msg, map_name, url);
            if (ret == ERR_OK)return;
        }
        else if(msg.title == "request_set_scheme"){
            PARSE_JSON(); 
            ret = _set_scheme(req, resp, result);
        } 
        else if(msg.title == "request_get_scheme"){
            PARSE_JSON(); 
            if(req["content"]["mapname"].isNull()){
                result = "fail_invalid_data";
                ret = ERR_INVALID_FIELD;
                break;
            }
            string sch_name = req["content"]["mapname"].asString();
            ret = upload(msg, sch_name, resp, result);
        }
        else if(msg.title == "request_del_scheme") {//test case del in use scheme
            PARSE_JSON(); 
            if(req["content"]["mapname"].isNull()){
                result = "fail_invalid_data";
                ret = ERR_INVALID_FIELD;
                break;
            }

            string map_name = req["content"]["mapname"].asString();
            Linfo("del scheme map:%s uisng:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
            if (map_name != MapManager::get_instance().get_using_map()){
                Lerror("%s is not using map:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
                ret = ERR_MAP_NOT_USING;
                break;
            }

            ret = _del_scheme(req, resp, result);
        }
        else if(msg.title == "request_del_map") {
            PARSE_JSON(); 
            Value map_list;
            map_list = req["content"]["name"];
            Value del_map_list;
            int size = map_list.size();
            for(int i = 0; i < size; i++){
                string map_name = map_list[i].asString();
                if (MapManager::get_instance().get_using_map() != map_name){
                    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
                    Lwarn("rm scheme file:%s", map_name.c_str());
                    remove(file_path_name.c_str());
                }
            }
            return;
        }
        else if (msg.title == "request_recharge" || msg.title == "request_return_recharge") {
            PARSE_JSON();
            int sw = 0;
            if (req["content"]["switch"].isString()) {
                sw = atoi(req["content"]["switch"].asString().c_str());
            } else {
                sw = req["content"]["switch"].asInt();
            }
            resp["switch"] = Value(sw);
            if (sw == SW_ENABLE && !_chg_sch.is_active()){
                if (Navigation::get_instance().is_locating()){
                    ret = ERR_LOCATING; 
                    break;
                }
                else if (!_chg_sch.is_charge_pt_valid()){
                    ret = ERR_CHARGE_DISABLED;
                    break;
                }

                Navigation::get_instance().stop_navigate();
                _pat_sch.stop();
                _ret_pt_tsk.stop();
            }
            _timer.stop();
            ret = _control_task(_chg_sch, sw);
        }
        else if (msg.title == "request_leave_pile") {
            PARSE_JSON();
            int sw = req["content"]["switch"].asInt();
            resp["switch"] = Value(sw);
            if (sw == SW_ENABLE){
                if (Navigation::get_instance().is_locating()){
                    ret = ERR_LOCATING;
                    break;
                } else if (_ret_pt_tsk.is_paused()){
                    ret = _ret_pt_tsk.resume();
                    break;
                } else if (_ret_pt_tsk.is_running()){
                    ret = ERR_ON_RUNNING;
                    break;
                }

                if (_chg_sch.is_active()) {
                    _chg_sch.stop();
                }

                //Gaussian::get_instance()->set_charge_state_idel();
                Navigation::get_instance().stop_navigate();
                _pat_sch.stop();

                _ret_pt_tsk.nav_emergency_stop(0); //todo

                atris_msgs::PowerChargeCmd charge_cmd;
                charge_cmd.charge_msg_type = atris_msgs::PowerChargeCmd::START_LEAVE_PILE;
                _charge_request_pub_.publish(charge_cmd);
                Linfo("notify_start_leave pile.");
                int timeout = 0;
                while(!_is_leaving_pile) {
                    Linfo("wait to leave pile...........");
                    if(++timeout > 60) {
                        _is_leaving_pile = false;
                        Lerror("wait to leave pile. timeout........");
                        return;
                    }
                    this_thread::sleep_for(chrono::milliseconds(1000));
                }
                this_thread::sleep_for(chrono::milliseconds(30000));
                Linfo("ready to leave pile...........");
            }
            ret = _control_task(_ret_pt_tsk, sw);
            _is_leaving_pile = false;
        }
        else if (msg.title == "request_switch_patrol") {
            PARSE_JSON();
            if (req["content"]["switch"].isNull()){
                ret = ERR_INVALID_JSON;
                break;
            }
            Linfo("msg content: %s", msg.msg.c_str());
            ret = _hdl_patrol(req, resp, result);
        }
        else if(msg.title == "request_get_patrol_status"){
            PARSE_JSON(); 
            resp["switch"] = get_patrol_status();//todo charge?return task?
        }
        else if(msg.title == "request_return") {
            PARSE_JSON(); 
            int sw = req["content"]["switch"].asInt();
            resp["switch"] = Value(sw);
            if (sw == SW_ENABLE){
                if (Navigation::get_instance().is_locating()){
                    ret = ERR_LOCATING;
                    break;
                } else if (_chg_sch.is_active()){
                    ret = ERR_ON_RUNNING;
                    break;
                } else if (_ret_pt_tsk.is_paused()){
                    ret = _ret_pt_tsk.resume();
                    break;
                } else if (_ret_pt_tsk.is_running()){
                    ret = _ret_pt_tsk.stop();
                }

                Navigation::get_instance().stop_navigate();
                _pat_sch.stop();
            } 
            ret = _control_task(_ret_pt_tsk, sw);
        }
        else if(msg.title == "request_auto_patrol_enable"){
            PARSE_JSON(); 
        }
        else if(msg.title == "request_recharge_battery"){
            Linfo("recv battery");
            PARSE_JSON();
            int level = req["content"]["battery"].asInt();
            if(_chg_sch.check_battery(level)){
                _timer.timeout(Config::get_instance()->nav_auto_charge_confirm_timeout);
                _timer_thd.push(&_timer);
                _timer.start();
            }
            return;
        }
        else if (msg.title == "response_emergency_button_state"){
            Linfo("recv response_emergency_button_state");
            PARSE_JSON(); 
            int st = req["content"]["button"].asInt();
            if (st){
                if (_chg_sch.is_active()){
                    _chg_sch.pause();
                }
                if (_pat_sch.is_active()){
                    _pat_sch.pause(); 
                }
                if (_ret_pt_tsk.is_active()){
                    _ret_pt_tsk.pause(); 
                }
            }
            return;
        }else if(msg.title == "request_new_task") {
            PARSE_JSON();
            Linfo("reqeust new task msg: %s",  msg.msg.c_str());
            //todo running ? locating c? charging?
            if (_pat_sch.is_active()) {
                Navigation::get_instance().stop_navigate();
                _pat_sch.stop();
            }

            shm::ChargeState shm_state;
            shm::iMemory_read_ChargeState(&shm_state);
            Linfo("request_new_task: chargeState:%d", shm_state.state);
            if (shm_state.state == dockSDKState::DOCK_SUCESS)
            {
                Linfo("request_new_task : need leave pile, send request_leave_pile.");
                _ret_pt_tsk.save_task_data(msg.msg.c_str());

                Json::Value leave_pile_req;
                Json::FastWriter fw;
                leave_pile_req["title"] = "request_leave_pile";
                leave_pile_req["content"]["switch"] = 1;
                atris_msgs::SignalMessage req;
                std::string leave_pile_data = fw.write(leave_pile_req);
                req.title = "request_leave_pile";
                req.msg = leave_pile_data.c_str();
                _signal_msg_pub.publish(req);
                return;
            }

            _hdl_new_task(req, resp, result, msg);
            return;
        }else if(msg.title == "request_remove_task"){
            PARSE_JSON();
            //todo running ? locating c? charging?
            _hdl_remove_task(req, resp, result, msg);
            return;
        }else if(msg.title == "request_emergency_task") {
            PARSE_JSON();
            int sw = req["content"]["switch"].asInt();
            resp["switch"] = Value(sw);
            _hdl_emergency_task(req, resp, result, msg);
            return; 
        }else if(msg.title == "request_get_tsp_navigating") {
		}
        else return;
    } while (0);

    if (ret != ERR_OK && (result == "success" || result.empty()))
        result = get_err_msg(ret); 
    resp["result"] = result;
    resp["result_code"] = ret;
    Utils::get_instance()->responseResult(msg, resp, "response" + msg.title.substr(7));
}

int SchemeManager::_parser_cb(string param){
    int ret = ERR_OK;
    string scheme_content, result;
    string req_param = _req_param;
    Linfo("req parma: %s",req_param.c_str());
    Json::Reader reader;
    Json::Value req;
    if (!reader.parse(req_param, req)) {
        log_error("%s:fail parse req_param", __FUNCTION__);
        return ERR_INVALID_JSON;
    }
    string taskid = req["content"]["taskMouldId"].asString();
    string file_path =  Config::get_instance()->patrol_scheme_path + "/" + taskid;
    ret = parse_scheme_file(file_path, scheme_content);
    if (ret != ERR_OK) {
        return ret;
    }

    sched::Event event;
    sched::Job job;
    job.job_id = req["content"]["taskMouldId"].asString();
    job.job_name = req["content"]["schemeName"].asString();
    job.content = scheme_content;
    job.type = req["content"]["taskType"].asString();

    event.operation_type = req["content"]["operationType"].asString();
    event.map_name = req["content"]["mapName"].asString();
    event.name = req["content"]["schemeName"].asString();
    event.job_id = req["content"]["taskMouldId"].asString();

    if (req["content"]["operationType"].asString() == "loop"){ //todo enum ?
        if (req["content"]["loopFlag"].isNull()) {
            result = "no loopflag !";
            Lerror("%s",result.c_str());
            return ERR_INVALID_FIELD;
        }

        if (req["content"]["loopStartTime"].isNull()) {
            result = "no loopStartTime !";
            Lerror("%s",result.c_str());
            return ERR_INVALID_FIELD;
        }

        long loopStartTime = req["content"]["loopStartTime"].asLargestInt();
        sched::Date loop_time_dt;
        loop_time_dt.ConvertTimestampToDate(loopStartTime);
        event.dtstart = string(loop_time_dt);

        if (req["content"]["loopFlag"].asString() == "interval") {
            event.rrule.freq = sched::DAY;
            if (req["content"]["intervalCount"].isNull()) {
                result = "no intervalCount!";
                Lerror("%s",result.c_str());
                return ERR_INVALID_FIELD;

            }

            string intervalCount = req["content"]["intervalCount"].asString();
            event.rrule.count = atoi(intervalCount.c_str());;

            if (req["content"]["intervalDay"].isNull()) {
                result = "no intervalDay";
                Lerror("%s",result.c_str());
                return ERR_INVALID_FIELD;
            }

            string intervalDay = req["content"]["intervalDay"].asString();
            event.rrule.interval = atoi(intervalDay.c_str());
            event.is_recurring = true;

            scheduler.add_event(event);
        } else if (req["content"]["loopFlag"].asString() == "fixed") {
            if (req["content"]["fixedTime"].isNull()) {
                result = "no fixedTime";
                Lerror("%s",result.c_str());
               return ERR_INVALID_FIELD;
            
            }

            string fixedtime = req["content"]["fixedTime"].asString(); // "timestamp,timestamp,timestamp ..."
            std::string::size_type pos1, pos2;
            std::vector<string> v;
            std::string c = ",";
            pos2 = fixedtime.find(c);
            pos1 = 0;

            if (std::string::npos == pos2) {
                v.push_back(fixedtime);
            } else {
                while (std::string::npos != pos2) {
                    v.push_back(fixedtime.substr(pos1, pos2 - pos1));
                    pos1 = pos2 + c.size();
                    pos2 = fixedtime.find(c, pos1);
                }

                if(pos1 != fixedtime.length()) {
                    v.push_back(fixedtime.substr(pos1));
                }
            }

            for (auto &t : v) {
                sched::Date fixed_dt;
                fixed_dt.ConvertTimestampToDate(stol(t));
                event.dtstart = fixed_dt;
                event.is_recurring = false;
                ret = scheduler.add_event(event);
            }
            
        } else if (req["content"]["loopFlag"].asString() == "exec") {
            if (req["content"]["execTime"].isNull()) {
                result = "no execTime !";
                Lerror("%s",result.c_str());
                return ERR_INVALID_FIELD;
            }

            long exectime = req["content"]["execTime"].asLargestInt();
            sched::Date exec_dt;
            log_debug("exec_dt %ld", exectime);
            exec_dt.ConvertTimestampToDate(exectime);
            event.rrule.byhour = exec_dt.DateOfHour();
            event.rrule.byminute = exec_dt.DateOfMinute();
            event.rrule.bysecond = exec_dt.DateOfSecond();

            if (req["content"]["execType"].isNull()) {
                result = "no execType !";
                Lerror("%s",result.c_str());
                return ERR_INVALID_FIELD;
            }

            if (req["content"]["execType"].asString() == "day") {
                event.rrule.freq = sched::DAY;
            } else if (req["content"]["execType"].asString() == "week") {
                event.rrule.freq = sched::WEEK;
            } else if (req["content"]["execType"].asString() == "month") {
                event.rrule.freq = sched::MONTH;
            }
            event.is_recurring = true;
            ret = scheduler.add_event(event);
        }
     } else if (req["content"]["operationType"].asString() == "now") {
        if (req["content"]["nowStartTime"].isNull()) {
            result = "no nowStartTime !";
            Lerror("%s",result.c_str());
            return ERR_INVALID_FIELD;
        }
        long nowStartTime = req["content"]["nowStartTime"].asLargestInt();
        log_debug("nowStartTime %ld", nowStartTime);
        sched::Date now_dt;
        now_dt.ConvertTimestampToDate(nowStartTime);
        event.dtstart = now_dt;
        event.is_recurring = false;
        ret = scheduler.add_event(event);
        log_debug("now_dt: %s", now_dt.Format().c_str());
    } else if( req["content"]["operationType"].asString() == "timing") {
        if (req["content"]["timingStartTime"].isNull()) {
            Lerror("no timingStartTime !");
            result = "no timingStartTime !";
            return ERR_INVALID_FIELD;
        }

        long timingStartTime = req["content"]["timingStartTime"].asLargestInt();
        log_debug("timingStartTime %ld", timingStartTime);
        sched::Date time_dt;
        time_dt.ConvertTimestampToDate(timingStartTime);
        event.dtstart = time_dt;
        event.is_recurring = false;
        ret = scheduler.add_event(event);
    } else {
        result = "operationType value err !!";
        Lerror("%s",result.c_str());
        return ERR_INVALID_FIELD;
    }

    if (ret != ERR_OK) {
        return ret;
    }

    if (ERR_OK != (ret =scheduler.add_job(job))) {
        return ret;
    };
    _schedule_scheme();
    return ret;

}
int SchemeManager::_hdl_new_task(Value& req, Value &resp, string &result, const SignalMessage &msg) {
    Linfo("----------%s", __FUNCTION__);
    int ret = ERR_OK;

    if (req["content"]["schemes"].isNull() 
        //|| req["content"]["mapName"].isNull() 
        //|| req["content"]["mapId"].isNull() 
        || req["content"]["taskMouldId"].isNull() 
        || req["content"]["taskType"].isNull() || req["content"]["operationType"].isNull()
        || req["content"]["schemeName"].isNull()) {

        result = "fail_invalid_data";
        Lerror("%s",result.c_str());
        return ERR_INVALID_FIELD;
    }
    string sch_name = req["content"]["schemeName"].asString();
    string taskid = req["content"]["taskMouldId"].asString();
    string url = req["content"]["schemes"].asString();

    if (sch_name.empty() || taskid.empty() || url.empty()){
        Lerror("%s invalid param", __FUNCTION__);
        return ERR_INVALID_PARAM;
    }

    Json::FastWriter fw;
    _req_param = fw.write(req);
    Linfo("req: %s",_req_param.c_str());
    string file_path = Config::get_instance()->patrol_scheme_path + "/" + taskid;
    Linfo("download %s url:%s", file_path.c_str(), url.c_str());

    boost::shared_ptr<TransNotifyer> obj = boost::shared_ptr<TransNotifyer> (new TransNotifyer(true));
    
    obj->origin = msg;
    obj->remote_path = url;
    obj->local_path = file_path;
    //obj->param = &_req_param;
    obj->cb = boost::bind(&SchemeManager::_parser_cb, this, _1);

    TransferFile::download(obj);

    if (ret != ERR_OK) {
        return ret;
    }
    return ret;
}

int SchemeManager::_hdl_emergency_task(Value& req, Value &resp, string &result, const SignalMessage &msg) {
    Linfo("----------%s", __FUNCTION__);
    int ret = ERR_OK;
    if (req["content"]["PointInfo"].isNull()) {
        return ERR_INVALID_FIELD;
    }

    Value point_info = req["content"]["PointInfo"];
    if (point_info["locationX"].isNull() || point_info["locationY"].isNull() || 
        point_info["locationOrientation"].isNull() ||
        point_info["pointBaseId"].isNull() || point_info["mapPointId"].isNull() ||
        point_info["cameraFoucs"].isNull() || point_info["cameraHangle"].isNull() ||
        point_info["cameraVangle"].isNull() || point_info["cameraZoom"].isNull()  ||
        point_info["captureInfrared"].isNull() || point_info["captureVisibleLight"].isNull() ||
        point_info["heatType"].isNull() || point_info["meterType"].isNull() ||
        point_info["recognitionType"].isNull() ||  point_info["recordSound"].isNull() ||
        point_info["recordVideo"].isNull() || point_info["saveType"].isNull()) {
            return ERR_INVALID_FIELD;
    }
    //Todo
}

int SchemeManager::_hdl_remove_task(Value &req, Value &resp, string &result, const SignalMessage &origin) {
    Linfo("----------%s", __FUNCTION__);
    int ret = ERR_OK;
    if (req["content"]["mapName"].isNull() || req["content"]["taskMouldId"].isNull() ||
        req["content"]["schemeName"].isNull() ) {
        return ERR_INVALID_FIELD;
    }
    std::string job_id = req["content"]["taskMouldId"].asString();
    string sche_name = req["content"]["schemeName"].asString();

    //todo better : one interface remove task ?
    scheduler.del_job(job_id);
    scheduler.del_event(sche_name);
    scheduler.del_event_instance(sche_name);

    return ret;
}

int SchemeManager::_hdl_return_recharge(Value &req, Value &resp, string &result, const SignalMessage &origin) {
    Linfo("----------%s", __FUNCTION__);
    int ret = ERR_OK;
    return ret;
   //todo
}

int SchemeManager::_hdl_patrol(const Value& req, Value &resp, string &result){
    Linfo("%s", __FUNCTION__);
    int ret = ERR_OK;
    int sw = req["content"]["switch"].asInt();
    resp["switch"] = Value(sw);
    if (sw == SW_ENABLE){
        
        if (Navigation::get_instance().is_locating()){
            return ERR_LOCATING;
        }
        else if (!_pat_sch.enable()){
            return ERR_SCHEME_DISABLED;
        }
        else if (_chg_sch.is_active() || _ret_pt_tsk.is_active()){
            return ERR_ON_RUNNING;
        }
        
        if(!_pat_sch.is_active()){
        //
        }
    }
    ret = _control_task(_pat_sch, sw);

    return ret;
}


int SchemeManager::_load_scheme(const string &scheme_name, const string &scheme_file){
    Linfo("%s: %s: %s", __FUNCTION__,scheme_name.c_str(),  scheme_file.c_str());
    int ret = parse(scheme_name, scheme_file);
    if (ret == ERR_OK){
        Linfo("reload scheme file success");
    }
    else{
        Lerror("reload scheme file err:%d %s", ret, get_err_msg(ret));
    } 

    return ret;
}

void SchemeManager::_handle_scheme(sched::Event_Instance evi){ //todo thread safe ?.
    Linfo("%s", __FUNCTION__);
    //todo ? running ? locating ? recharging ?
    boost::lock_guard<boost::mutex> lock(scheme_mutex);
    int ret = ERR_OK;
    sched::Job job;
    scheduler.query_job(evi.job_id, job);
    Linfo("!!!!!!!!!!!!!_handle_scheme job id: %s,job_name: %s, job content :%s, evi name: %s",
                            job.job_id.c_str(), job.job_name.c_str(), job.content.c_str(), evi.name.c_str());
    if (get_patrol_status() != PATROL_IDLE ) {
        Lerror("!!!!!!!!!!!!!the other one patrol is running, set job:%s to overdue", job.job_id.c_str());
        ret = ERR_NONE_MAP;
		scheduler.update_event_instance(evi.id, "status", std::to_string(sched::OVERDUE));
		Notifier::publish_task_status(evi.map_name, evi.name, evi.operation_type, job.job_id,
                         std::to_string(evi.dtstart.ConvertDateToTimestamp()), "overdue");
		return;
    }

    do{
        ret = _load_scheme(job.job_name, job.content);
        if (ret != ERR_OK) {
            Lerror("_load_scheme failed !!.");
            break;
        }

        if (Config::get_instance()->nav_tsp_enable) {
            Ldebug("using tsp!");
            ret = _pat_sch.plan_route(evi.name, evi.operation_type, job.job_id,
                                std::to_string(evi.dtstart.ConvertDateToTimestamp()));
            if (ret != ERR_OK) {
            Lerror("plan_route failed !!.");
            break;
            }
        } else {
            Ldebug("Do not use tsp!");
        }

        int sw = SW_ENABLE;
        ret = _control_task(_pat_sch, sw);
        if (ret != ERR_OK) {
            Lerror("_control_task failed !!.");
            break;
        }
    } while(0);

    shm::TaskInfo shmtask;
    snprintf(shmtask.map_name, sizeof(shmtask.map_name), "%s", evi.map_name.c_str());
    snprintf(shmtask.scheme_name, sizeof(shmtask.scheme_name), "%s", evi.name.c_str());
    snprintf(shmtask.operation_type, sizeof(shmtask.operation_type), "%s", evi.operation_type.c_str());
    snprintf(shmtask.task_mould_id, sizeof(shmtask.task_mould_id), "%s", job.job_id.c_str());
    snprintf(shmtask.task_timestamp, sizeof(shmtask.task_timestamp), "%s", 
            std::to_string(evi.dtstart.ConvertDateToTimestamp()).c_str());
    shm::iMemory_write_TaskInfo(&shmtask);
    if (ret != ERR_OK) {
        scheduler.update_event_instance(evi.id, "status", std::to_string(sched::OVERDUE));
        Notifier::publish_task_status(evi.map_name, evi.name, evi.operation_type, job.job_id,
                             std::to_string(evi.dtstart.ConvertDateToTimestamp()), "overdue");
    } else {
        scheduler.update_event_instance(evi.id, "status", std::to_string(sched::RUNNING));
        Notifier::publish_task_status(evi.map_name, evi.name, evi.operation_type, job.job_id, 
                            std::to_string(evi.dtstart.ConvertDateToTimestamp()), "running");      
    }
}

int SchemeManager::_schedule_scheme() {
    Linfo("%s", __FUNCTION__);
   // scheduler.stop(); // todo if running ?
    int ret = ERR_OK;
    sched::Date now, final_date;
    now.SetToNow();
    final_date = now;
    final_date[sched::DAY] += 365; //todo better solution.
    std::list<sched::Event_Instance> levi;
    ret = scheduler.query_event_instance_before(final_date, levi);
    if (ret != ERR_OK) {
        Lerror("query_event_instance_before err!");
        return ret;
    }

    Fun func = boost::bind(&SchemeManager::_handle_scheme, this, _1);
    for (auto &evi : levi) {
        if (evi.status == sched::TODO) {
            if (evi.operation_type == "now") {
                Lerror("It's now time task!!");
                func(evi);
                continue;
            }

            if (!evi.dtstart.IsEmpty() && (evi.dtstart < now)) {
                Lerror("!! evi.dtstart : %s < now : %s", evi.dtstart.Format().c_str(), now.Format().c_str());
                scheduler.update_event_instance(evi.id, "status", std::to_string(sched::OVERDUE));
                sched::Job job;
                scheduler.query_job(evi.job_id, job);
                Notifier::publish_task_status(evi.map_name, evi.name, job.type, job.job_id,
                        std::to_string(evi.dtstart.ConvertDateToTimestamp()), "overdue");
                continue;
            }

            ret = scheduler.at(evi.dtstart.Format(), func, evi);
            if (ret != ERR_OK) {
                Lerror("scheduler at :%s  name : %s err!.", evi.dtstart.Format().c_str(), evi.name.c_str());
                return ret;
            }
        }
    }

    return ret;
}

int SchemeManager::_control_task(Task &task, int sw){
    Linfo("%s", __FUNCTION__);
    int ret = ERR_OK;
    switch(sw)
    {

    case SW_ENABLE:
        {
            
            if (task.is_paused()){
                Linfo("resume %s", task.name().c_str());
                if (task.type() == Task::TASK_PATROL) {
                    shm::TaskInfo shmtask;
                    shm::iMemory_read_TaskInfo(&shmtask);
                    sched::Date dtstart;
                    dtstart.ConvertTimestampToDate(atoi(shmtask.task_timestamp)); 
                    scheduler.update_event_instance(dtstart, "status", std::to_string(sched::RUNNING));
                    Notifier::publish_task_status(shmtask.map_name, shmtask.scheme_name, shmtask.operation_type, shmtask.task_mould_id,
                                    shmtask.task_timestamp, "running");
                }
                return task.resume();
            }

            
            string using_map = MapManager::get_instance().get_using_map();
            /*
            if (using_map.empty()) {
                Lerror("no using_map !!.");
                ret = ERR_MAP_NOT_USING;
                return ret;
            }
            */

            Linfo("start %s", task.name().c_str());
#if 0
            ret = Locator::get_instance().is_located();
            if (ret != ERR_OK){
                return ret;
            }
#endif
            GsNavPoint in_ret_point, out_ret_point, chg_point;
            vector<GsNavPoint> ret_point_list, chg_point_list;
            ret = GsApi::get_instance()->map_get_point_list(using_map, chg_point_list, StationInfo::StationType::CHARGE);
            if (ret != ERR_OK) {
                Lerror("get charge point list err.");
                ret = GsApi::get_instance()->map_get_point_list(using_map, chg_point_list, StationInfo::StationType::CHARGE); //try again
                if (ret != ERR_OK){
                    return ret;
                }
            }

            if (chg_point_list.size() >= 1){
                Ldebug("chg_point_list.size() >= 1***********************************************");
                chg_point = chg_point_list[0]; //todo check
                Ldebug("chg_point_list[0].id: %s", chg_point_list[0].id.c_str());
                Ldebug("chg_point_list[0].np.pos.name: %s", chg_point_list[0].np.name.c_str());
                Ldebug("chg_point_list[0].np.pos.x: %lf", chg_point_list[0].np.pos.x);
                Ldebug("chg_point_list[0].np.pos.y: %lf", chg_point_list[0].np.pos.y);
            }

            this_thread::sleep_for(chrono::milliseconds(1000));
           
            ret = GsApi::get_instance()->map_get_point_list(using_map, ret_point_list, StationInfo::StationType::STATION);
            if (ret != ERR_OK) {
                Lerror("get return point list err.");
                ret = GsApi::get_instance()->map_get_point_list(using_map, ret_point_list, StationInfo::StationType::STATION); //try again
                if (ret != ERR_OK) {
                    return ret;
                }
            }

            if (ret_point_list.size() >= 2){
                Ldebug("ret_point_list.size() >= 2***********************************************");
                Ldebug("ret_point_list[0].id: %s", ret_point_list[0].id.c_str());
                Ldebug("ret_point_list[0].np.pos.name: %s", ret_point_list[0].np.name.c_str());
                Ldebug("ret_point_list[0].np.pos.x: %lf", ret_point_list[0].np.pos.x);
                Ldebug("ret_point_list[0].np.pos.y: %lf", ret_point_list[0].np.pos.y);
                Ldebug("ret_point_list[1].id: %s", ret_point_list[1].id.c_str());
                Ldebug("ret_point_list[1].np.pos.name: %s", ret_point_list[1].np.name.c_str());
                Ldebug("ret_point_list[1].np.pos.x: %lf", ret_point_list[1].np.pos.x);
                Ldebug("ret_point_list[1].np.pos.y: %lf", ret_point_list[1].np.pos.y);

                double dist1 = (ret_point_list[0].np.pos.x - chg_point.np.pos.x) * (ret_point_list[0].np.pos.x - chg_point.np.pos.x)
                             + (ret_point_list[0].np.pos.y - chg_point.np.pos.y) * (ret_point_list[0].np.pos.y - chg_point.np.pos.y);

                double dist2 = (ret_point_list[1].np.pos.x - chg_point.np.pos.x) * (ret_point_list[1].np.pos.x - chg_point.np.pos.x)
                             + (ret_point_list[1].np.pos.y - chg_point.np.pos.y) * (ret_point_list[1].np.pos.y - chg_point.np.pos.y);

                if (dist1 <= dist2) {
                    Ldebug("dist1 <= dist2 *******************************************");
                    in_ret_point = ret_point_list[0];
                    out_ret_point = ret_point_list[1];
                } else {
                    Ldebug("dist1 > dist2 *******************************************");
                    in_ret_point = ret_point_list[1];
                    out_ret_point = ret_point_list[0];
                }

                Ldebug("in_ret_point.id: %s", in_ret_point.id.c_str());
                Ldebug("in_ret_point.np.pos.name: %s", in_ret_point.np.name.c_str());
                Ldebug("in_ret_point.np.pos.x: %lf", in_ret_point.np.pos.x);
                Ldebug("in_ret_point.np.pos.y: %lf", in_ret_point.np.pos.y);
                Ldebug("out_ret_point.id: %s", out_ret_point.id.c_str());
                Ldebug("out_ret_point.np.pos.name: %s", out_ret_point.np.name.c_str());
                Ldebug("out_ret_point.np.pos.x: %lf", out_ret_point.np.pos.x);
                Ldebug("out_ret_point.np.pos.y: %lf", out_ret_point.np.pos.y);
            }
            
            if (!_pat_sch.enable()){
                return ERR_SCHEME_DISABLED; 
            }

            if (task.type() == Task::TASK_RETURN){
                //_ret_pt_tsk.point(_pat_sch.return_point());
                _ret_pt_tsk.point(in_ret_point);
                if (_is_leaving_pile) {
                    Notifier::get_instance().publish_udock_status(begain_leave_pile);
                    _is_leaving_pile = false;
                }
            }
            else if (task.type() == Task::TASK_CHARGE) {

                _chg_sch.out_return_point(out_ret_point);
                _chg_sch.return_point(in_ret_point);

                _chg_sch.charge_point_name(chg_point.np.name);
                _chg_sch.charge_point(chg_point);
               // _chg_sch.point(chg_point);
                _chg_sch.charge_enable(true);
                Linfo("set chg in return point:%s %s [%lf:%lf:%lf] out return point:%s %s [%lf:%lf:%lf] charge point:%s",
                      _chg_sch.return_point().np.name.c_str(),
                      _chg_sch.return_point().id.c_str(),
                      _chg_sch.return_point().np.pos.x,
                      _chg_sch.return_point().np.pos.y,
                      _chg_sch.return_point().np.pos.angle,

                      _chg_sch.out_return_point().np.name.c_str(),
                      _chg_sch.out_return_point().id.c_str(),
                      _chg_sch.out_return_point().np.pos.x,
                      _chg_sch.out_return_point().np.pos.y,
                      _chg_sch.out_return_point().np.pos.angle,

                      _chg_sch.charge_point_name().c_str()
                      );

                shm::ChargeState shm_state;
                shm_state.state = dockSDKState::RECEIVE_CMD;
                shm::iMemory_write_ChargeState(&shm_state);

                Notifier::get_instance().publish_udock_status(begain_dock);
            }
            
            Linfo("start %s", task.name().c_str());
            _thd.push(&task);
            ret = task.start();
            Linfo("^_^ start %s ok ^_^", task.name().c_str());
        }
        break;
    case SW_DISABLE:
        Linfo("stop %s", task.name().c_str());
        ret = task.stop();
        break;
    case SW_PAUSE:
        if (task.is_running()){
            Linfo("pause %s", task.name().c_str());
            ret = task.pause();
            if (ret == ERR_OK) {
                if (task.type() == Task::TASK_PATROL) {
                    shm::TaskInfo shmtask;
                    shm::iMemory_read_TaskInfo(&shmtask);
                    sched::Date dtstart;
                    dtstart.ConvertTimestampToDate(atoi(shmtask.task_timestamp));
                    scheduler.update_event_instance(dtstart, "status", std::to_string(sched::PAUSE));
                    Notifier::publish_task_status(shmtask.map_name, shmtask.scheme_name, shmtask.operation_type, shmtask.task_mould_id,
                                    shmtask.task_timestamp, "pause");   
                }
            }
        }
        else
            Lerror("%s not running", task.name().c_str());

        break;
    default:
        Linfo("invalid switch value:%d", sw);
        ret = ERR_INVALID_VALUE;
        break;
    }
    return ret;
}

int SchemeManager::download(const SignalMessage &origin, const string &sch_name, const string &url){
    Linfo("%s", __FUNCTION__);
    if (sch_name.empty() || url.empty()){
        Lerror("%s invalid param", __FUNCTION__);
        return ERR_INVALID_PARAM;
    }

    string file_path = Config::get_instance()->patrol_scheme_path + "/" + sch_name;
    Linfo("download %s url:%s", file_path.c_str(), url.c_str());

    boost::shared_ptr<TransNotifyer> obj = boost::shared_ptr<TransNotifyer> (new TransNotifyer(true));

    obj->origin = origin;
    obj->remote_path = url;
    obj->local_path = file_path;
    obj->cb = nullptr;//boost::bind(&SchemeManager::parse, this, file_path);

    TransferFile::download(obj);
    _scheme_changed = true;
    return ERR_OK;
}

string SchemeManager::_gen_upload_url(const string &path_name){
    char md5buf[128] = {0};
    std::string url = "";
    Utils::get_instance()->gen_md5(path_name.c_str(), md5buf);
    string file_md5 = md5buf;
    if(Config::get_instance()->hfs_type == "qiniu"){
        url = QINIU_BUCKET_NAME + file_md5;
    }else{
        if(*(Config::get_instance()->hfs_url.end()-1) != '/') {
            url = Config::get_instance()->hfs_url + '/' + path_name.substr(path_name.rfind("/"));
        }else{
            url = Config::get_instance()->hfs_url + path_name.substr(path_name.rfind("/"));
        }
    }
    return url;

}

int SchemeManager::upload(const SignalMessage &origin, const string &sch_name, Value& resp, string& result){
    if (sch_name.empty()) return ERR_INVALID_PARAM;

    string file_path = Config::get_instance()->patrol_scheme_path + "/" + sch_name;
    if (access(file_path.c_str(), R_OK) != 0){
        Lerror("can't find the scheme:%s", file_path.c_str());
        result = "scheme_not_exist";
        //Utils::get_instance()->responseResult(origin, resp, "response_get_scheme");
        return ERR_NONE_SCHEME;
    }

    string url = _gen_upload_url(file_path);
    if(Config::get_instance()->hfs_type == "qiniu"){
        if(TransferFile::uploaded(url)){
            Linfo("already uploaded");
            resp["result"] = "success";
            resp["url"] = Json::Value(url);
            Utils::get_instance()->responseResult(origin, resp, "response_get_scheme");
            return ERR_OK;
        }
    }

    boost::shared_ptr<TransNotifyer> obj = boost::shared_ptr<TransNotifyer> (new TransNotifyer(false));
    
    obj->origin = origin;
    obj->remote_path = url;
    obj->local_path = file_path;

    TransferFile::upload(obj);
    
    return ERR_OK;
}

int SchemeManager::_get_scheme(Value &root, Value &scheme, int &idx, string &map_name, string &scheme_name){
    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    int ret = read_json_file(file_path_name, root);
    if(ret !=  ERR_OK) return ret;

    if(root["mapName"].isNull() || root["schemes"].isNull()) {
        Lerror("map name or schemes is null in patrol file");
        return ERR_INVALID_JSON;
    }

    Value js_schemes = root["schemes"];
    int sz = js_schemes.size();
    for (int i = 0; i < sz; i++){
        Value& js_scheme = js_schemes[i];
        string sch_name = js_scheme["schemeName"].asString();
        if(sch_name == scheme_name){
            Lerror("%s exist idx:%d", scheme_name.c_str(), i);
            scheme = js_scheme;
            idx = i;
            return ERR_OK;
        }
    }

    Lerror("%s not exist", scheme_name.c_str());
    return ERR_NONE_SCHEME;
}

int SchemeManager::_set_scheme(Value &req, Value &resp, string &result){
    if (req["content"]["mapname"].isNull()
    ||  req["content"]["schemename"].isNull()){
        Lerror("map name or schemes is null passed by client");
        result = "fail_invalid_data";
        return ERR_INVALID_VALUE;
    }

    if (Navigation::get_instance().is_running() || is_running()){
        return ERR_ON_RUNNING; 
    }

    string map_name = req["content"]["mapname"].asString();
#if 1//todo zy
    string using_map = MapManager::get_instance().get_using_map(); 
    if (using_map.empty()){
        Lerror("%s is not using map:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
        int ret = MapManager::get_instance().load_map(); 
        if (ret != ERR_OK)
            return ret;

        using_map = MapManager::get_instance().get_using_map(); 
    }

    if (map_name != using_map) 
        return ERR_MAP_MUST_EQ_USING;
#endif

    resp["schemename"] = req["content"]["schemename"];
    resp["mapname"]    = req["content"]["mapname"];
    string scheme_name = req["content"]["schemename"].asString();
    Value root, scheme;
    int idx = 0;
    int ret = _get_scheme(root, scheme, idx, map_name, scheme_name);
    if (ret == ERR_OK){
        root["schemeUse"] = scheme_name;
        string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
        ret = write_json_file(file_path_name, root);
        if (ret == ERR_OK){
            _scheme_changed = true;
        }
    }

    return ret;
}

int SchemeManager::_del_scheme(Value &req, Value &resp, string& result){
#if 0
    string map_name = req["content"]["mapname"].asString();
    if (map_name.empty()){
        Lerror("mapname is empty");
        return ERR_INVALID_VALUE;
    }

    Value root;
    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    int ret = read_json_file(file_path_name, root);
    if(ret !=  ERR_OK) return ret;

    if(root["mapName"].isNull() || root["schemes"].isNull()){
        Lerror("map name or schemes is null");
        return ERR_INVALID_JSON;
    }

    string scheme_name;
    if (!req["content"]["schemename"].isNull()){
        scheme_name = req["content"]["schemename"].asString();
        Lerror("del scheme:%s", scheme_name.c_str());
    }

    Value js_schemes = root["schemes"];
    int sz = js_schemes.size();
    bool exist = false;
    int i = 0;
    if (!scheme_name.empty()){
        for (; i < sz; i++){
            Value& js_scheme = js_schemes[i];
            string sch_name = js_scheme["schemeName"].asString();
            if(sch_name == scheme_name){
                Lerror("%s exist idx:%d", scheme_name.c_str(), i);
                exist = true;
                break;
            }
        }
        Lerror("%s not exist", scheme_name.c_str());
    }
    else{
        Value& js_del_scheme = js_schemes[0];
        scheme_name = js_del_scheme["schemeName"].asString();
        exist = true;
}
#endif

    if (is_running()){
        return ERR_ON_RUNNING;
    }

    if (req["content"]["mapname"].isNull()
    ||  req["content"]["schemename"].isNull()){
        Lerror("map name or schemes is null passed by client");
        return ERR_INVALID_VALUE;
    }

    resp["mapname"] = req["content"]["mapname"]; 
    resp["schemename"] = req["content"]["schemename"];  
    string map_name = req["content"]["mapname"].asString();
    string scheme_name = req["content"]["schemename"].asString();
    Value root, scheme;
    int idx = 0;
    int ret = _get_scheme(root, scheme, idx, map_name, scheme_name);
    if (ret == ERR_OK){
        string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
        Value js_schemes = root["schemes"];
        int sz = js_schemes.size();
        if (sz <= 1){
            Lerror("%s's schemes is empty to rm", file_path_name.c_str());
            int ret = remove(file_path_name.c_str());
            if(ret != 0){
                Lerror("remove %s fail", file_path_name.c_str());
                return ERR_REMOVE_FILE_FAIL;
            }
            _scheme_changed = true;
        }
        else{
            Lerror("%s rm idx:%d using scheme:%s schemeUse: %s del scheme:%s",
                   file_path_name.c_str(), idx, _pat_sch.scheme_name().c_str(),
                   root["schemeUse"].asString().c_str(), scheme_name.c_str());
            if (scheme_name == root["schemeUse"].asString()) {
                Value js_del_scheme;
                js_schemes.removeIndex(idx, &js_del_scheme);
                Value& js_scheme = js_schemes[0];
                root["schemeUse"] = js_scheme["schemeName"];
                root["schemes"] = js_schemes;
                write_json_file(file_path_name, root);
                _scheme_changed = true;
            }
            else{
                Value js_del_scheme;
                js_schemes.removeIndex(idx, &js_del_scheme);
                root["schemes"] = js_schemes;
                write_json_file(file_path_name, root);
            }
        }
        return ERR_OK;
    }

    return ERR_NONE_SCHEME;
}

int SchemeManager::_get_schedule(Value &req, Value &resp, string &result){
    if(req["content"]["mapname"].isNull() || req["content"]["schedule_name"].isNull()){
        result = "fail_invaild_data";
        return ERR_INVALID_FIELD;
    }

    resp["schedule_data"] = Value("");
    string map_name = req["content"]["mapname"].asString();
    string schedule_name = req["content"]["schedule_name"].asString();

    if (map_name.empty() || schedule_name.empty()) return ERR_INVALID_PARAM;

    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    Value root;
    int ret = read_json_file(file_path_name, root);
    if(ret !=  ERR_OK) return ret;

    if(root["mapName"].isNull() || root["schemes"].isNull()){
        Lerror("map name or schemes is null in patrol file");
        return ERR_INVALID_JSON;
    }

    Value js_schemes = root["schemes"];
    int sz = js_schemes.size();
    for (int i = 0; i < sz; i++){
        Value js_scheme = js_schemes[i];
        string sch_name = js_scheme["schemeName"].asString();
        if(sch_name == schedule_name){
            Ldebug("%s exist", schedule_name.c_str());
            resp["schedule_data"] = js_scheme;
            return ERR_OK;
        }
    }

    Lerror("%s not exist", schedule_name.c_str());
    return ERR_NONE_SCHEME;
}

int SchemeManager::_get_schedule_list(Value &req, Value &resp, string &result){
    if(req["content"]["mapname"].isNull()){
        result = "fail_invaild_data";
        return ERR_INVALID_FIELD;
    }

    string map_name = req["content"]["mapname"].asString();
    if (map_name.empty()) return ERR_INVALID_PARAM;

    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    Value root;
    int ret = read_json_file(file_path_name, root);
    if(ret !=  ERR_OK) return ret;

    if(root["mapName"].isNull() || root["schemes"].isNull() || root["schemeUse"].isNull()){
        Lerror("map name or schemes or schemeUse is null in patrol file");
        result = "fail_inner_error";
        return ERR_INVALID_JSON;
    }

    resp["schedule_using"] = Value(root["schemeUse"]);
    Value js_schemes = root["schemes"];
    int sz = js_schemes.size();
    for (int i = 0; i < sz; i++){
        Value& js_scheme = js_schemes[i];
        string sch_name = js_scheme["schemeName"].asString();
        if(!sch_name.empty()){
            resp["schedules"].append(Value(sch_name));
        }
    }
    return ERR_OK;
}

int SchemeManager::_sync_all_schedule(Value &req, Value &resp, string &result){
    Linfo("%s", __FUNCTION__);
    if(req["content"]["mapname"].isNull() || req["content"]["schedule_data"].isNull()){
        result = "fail_invaild_data";
        return ERR_INVALID_FIELD;
    }

    string map_name = req["content"]["mapname"].asString();
    Value sch_data = req["content"]["schedule_data"];
    if(map_name.empty() || sch_data.empty()){
        Lerror("%s invalid param", __FUNCTION__);
        result = "fail_invaild_data";
        return ERR_INVALID_PARAM;
    }

    Linfo("sync scheme map:%s uisng:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
    if (map_name != MapManager::get_instance().get_using_map()){
        Lerror("%s is not using map:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
        result = "fail_inner_error";
        return ERR_MAP_NOT_USING;
    }

    string file_path = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    write_json_file(file_path, sch_data);

    return ERR_OK;
}

int SchemeManager::_sync_schedule(Value &req, Value &resp, string &result){
    boost::lock_guard<boost::mutex> lock(schedule_mutex);
    Linfo("%s", __FUNCTION__);
    if(req["content"]["mapname"].isNull() || req["content"]["schedule_name"].isNull()
    || req["content"]["schedule_data"].isNull()){
        result = "fail_invaild_data";
        return ERR_INVALID_FIELD;
    }

    string map_name = req["content"]["mapname"].asString();
    string schedule_name = req["content"]["schedule_name"].asString();

    Reader reader;
    Value sch_data;
    if(req["content"]["schedule_data"].isString()){
        string schedule = req["content"]["schedule_data"].asString();
        if(!reader.parse(schedule, sch_data)){
            Lerror("parse json fail.");
            result = "fail_invaild_data";
            return ERR_INVALID_JSON;
        }
    }else{
        sch_data = req["content"]["schedule_data"];
    }

    if(map_name.empty() || sch_data.empty()){
        Lerror("%s invalid param", __FUNCTION__);
        result = "fail_invaild_data";
        return ERR_INVALID_PARAM;
    }

    Linfo("sync scheme map:%s uisng:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());

#if 0
    if (map_name != MapManager::get_instance().get_using_map()){
        Lerror("%s is not using map:%s", map_name.c_str(), MapManager::get_instance().get_using_map().c_str());
        result = "fail_inner_error";
        return ERR_MAP_NOT_USING;
    }
#endif

    vector<GsMap> maps;
    bool is_map_exist = false;
    int ret = GsApi::get_instance()->map_get_list(maps);
    if (ret != ERR_OK) {
        result = get_err_msg(ret);
        return ret;
    }

    if (maps.size() <= 0) {
        result = "fail_none_map";
        return ERR_NONE_MAP;
    }

    for (auto m:maps){
        if (m.name == map_name) {
            is_map_exist = true;
            break;
        }
    }

    if (!is_map_exist){
        Linfo("%s not exist.", map_name.c_str());
        result = "fail_map_not_found";
        return ERR_MAP_NOT_FOUND;
    }

    string file_path_name = Config::get_instance()->patrol_scheme_path + "/" + map_name;
    if (access(file_path_name.c_str(), R_OK) != 0) {
        Ldebug("scheme file not exsit, will create a new file.");
        Value params;
        params["mapName"] = map_name;
        params["schemeUse"] = schedule_name;
        params["schemes"].append(sch_data);
        write_json_file(file_path_name, params);
        return ERR_OK;
    }

    Ldebug("scheme file exsit, don't need create a new file.");
    Value root;
    ret = read_json_file(file_path_name, root);
    if(ret != ERR_OK) return ret;

    if(root["mapName"].isNull() || root["schemes"].isNull()){
        Lerror("map name or schemes is null in patrol file");
        return ERR_INVALID_JSON;
    }

    Value& js_schemes = root["schemes"];
    int sz = js_schemes.size();
    bool has_same_sch = false;
    for (int i = 0; i < sz; i++) {
        Value& js_scheme = js_schemes[i];
        string sch_name = js_scheme["schemeName"].asString();
        if (sch_name == schedule_name) {
            Ldebug("%s exist idx:%d", schedule_name.c_str(), i);
            js_scheme = sch_data;
            has_same_sch = true;
        }
    }

    if(!has_same_sch){
        root["schemes"].append(sch_data);
    }

    write_json_file(file_path_name, root);

    return ERR_OK;
}

int SchemeManager::write_json_file(const string &file_path_name, const Value &root){
    StyledWriter w;
    string js_data = w.write(root);
    Linfo("write js:%s", js_data.c_str());
    ofstream file_strm(file_path_name, ios::trunc);
    file_strm << js_data;
    file_strm.close();
    return ERR_OK;
}

int SchemeManager::read_json_file(const string &file_path_name, Value &root){
    Linfo("%s", __FUNCTION__);
    int   ret  = ERR_OK;
    FILE *file = nullptr;
    char *buf  = nullptr;
    Reader reader;

    do{
        if(access(file_path_name.c_str(), F_OK) != 0){
            Linfo("%s not exist", file_path_name.c_str());
            ret = ERR_FILE_NOT_EXIST;
            break;
        }

        Linfo("open %s", file_path_name.c_str());
        file = fopen(file_path_name.c_str(), "r");
        if(file == nullptr){
            Lerror("open %s fail", file_path_name.c_str());
            ret = ERR_OPEN_FILE_FAIL;
            break;
        }

        fseek(file, 0 , SEEK_END);
        int len = ftell(file);
        rewind(file);

        buf = new char[len];
        assert(buf);
        int ret = fread(buf, 1, len, file);
        if(ret != len){
            Lerror("read %s fail", file_path_name.c_str());
            ret = ERR_READ_FILE_FAIL;
            break;
        }

        Lerror("json data:%s", buf);
        if(!reader.parse(buf, root)){
            Lerror("parse json file %s fail", file_path_name.c_str());
            ret = ERR_INVALID_JSON;
            break;
        }

    }while (0);

    if (buf) delete buf;

    if (file) fclose(file);

    return ret;
}

int SchemeManager::parse(const string& scheme_name,const string& scheme_file){
    Linfo("%s", __FUNCTION__);
    int ret = ERR_OK;
    Value root;
    
    Linfo("load scheme:%s", scheme_file.c_str());
   //todo if scheme_name == scheme_name in using , dont't need load again.
    Reader reader;
    if (!reader.parse(scheme_file, root)) {
        log_error("%s:fail parse scheme_file", __FUNCTION__);
        return ERR_INVALID_JSON;
    }

    Linfo("load scheme:%s", scheme_name.c_str());
    _pat_sch.name("pat_" + scheme_name);
    ret = _pat_sch.parse(scheme_name, root);

    if (ret != ERR_OK) {
        Lerror("parse scheme_file fail");
        return ERR_INVALID_JSON;
    }
    
    return ret;
}

int SchemeManager::parse_scheme_file(const string& scheme_file, string& scheme_content) {
    Linfo("%s scheme file:%s", __FUNCTION__, scheme_file.c_str());
    int ret = ERR_NONE_SCHEME;
    Value root;
    Json::FastWriter fw;
    do{
        ret = read_json_file(scheme_file, root);
        if (ret != ERR_OK ) return ret;
        scheme_content = fw.write(root);
    }while(0);
    return ERR_OK;
}

}

