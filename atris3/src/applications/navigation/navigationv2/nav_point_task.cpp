#include "map_manager.h"
#include "gs_api.h"
#include "nav_point_task.h"
#include "locate_task.h"
#include "tts_strings/tts_strings.h"
#include "imemory/atris_imemory_api.h"
#include "platform/udock/UdockData.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_pnt]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_pnt]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_pnt]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_pnt]"#format, ##args)

#define TASK_NAME    "atris_task_queue"
#define JUDGE_TURNING_THRESHOLD (0.1)
#define BLOCKED_CNT_MAX (10)
#define FADE_CNT_MAX (5)

namespace nav{

void NavPointTask::on_status(string &data){
    TASK_LOCK();
    _st_ws_connected = true;
    _status_timeout = Config::get_instance()->nav_task_running_timeout;
    string::size_type pos;
    vector<int> codes;
    string key_str(R"({"statusCode")");
    pos = data.find(key_str);
    while (pos != data.npos){
        Linfo("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
        Linfo("str:%s", data.c_str() + pos);
        string str_code = data.substr(pos);
        on_status_code(str_code, lock);
        pos = data.find(key_str, pos + key_str.length());
    }
}

bool NavPointTask::_reparse_json(string &data, string &item, Json::Value &root){
    string::size_type pos1, pos2;
    string tmp;
    pos1 = data.find(item);
    Linfo("pos1:%s", data.c_str() + pos1);
    if (pos1 != data.npos){
        tmp = data.substr(0, pos1);
        Linfo("tmp1:%s", tmp.c_str());
        pos2 = data.find(",", pos1);
        if (pos2 != data.npos){
            Linfo("pos2:%s", data.c_str() + pos2);
            tmp += data.substr(pos2 + 1);
            Linfo("tmp2:%s", tmp.c_str());
        }
    }

    Json::Reader reader;
    return reader.parse(tmp, root);
}

void NavPointTask::on_status_code(string &data, task_lock &lock){
    Json::Reader reader;
    Json::Value root;

    if(!reader.parse(data, root)){
        //string item("indexPercent");
        //if (!_reparse_json(data, item, root)){
            Lerror("parse gs status js data fail js:%s", data.c_str());
            return;
        //}
    }

    int code = root["statusCode"].asInt();
    Linfo("status code:%d", code);
    switch(code){
    case 200:
        Lerror("ignore code 200 stopped");
        return;
    case 300:
        Lerror("ignore code 300 ");
        return;
    case 305:
        Lerror("ignore code 305 ");
        return;
    case 401:
    //case 402:
    //case 1006: //定位丢失不结束任务
    case 407:
        shm::ChargeState shm_state;
        shm::iMemory_read_ChargeState(&shm_state);
        if(shm_state.state == dockSDKState::RECEIVE_LEAVE_CMD) {
            Linfo("Notify leave pile success.");
            shm_state.state = dockSDKState::IDLE;
            shm::iMemory_write_ChargeState(&shm_state);
            Notifier::get_instance().publish_udock_status(leave_pile_success);
        }

        if (_type == Task::TASK_RETURN_CHARGER_DOOR) {
            Linfo("Arrived the out return point When exec return charge task");
            nav_set_light_color(color::PURPLE);
            nav_set_light_style(light_style::breathe_style);
            this_thread::sleep_for(chrono::milliseconds(8000));
            nav_release_light_ctrl();
        }
    case 412:
    //case 413: //目标点不可达
    case 414:
    case 415:
        _ret.ret = code;
        _show_status = true;
        Notifier::get_instance().set_status_cb(nullptr);
        Lerror("%s!!!!!!!!!!!stop all gs task by status code:%d js:%s", _name.c_str(), code, data.c_str());
        GsApi::get_instance()->nav_task_stop_all();
        _next_status(STP, lock);
        Linfo("%s stop ok", _name.c_str());
        break;
    default:
        break;
    }

    //todo if (_status_code == code)
    //    return;

    if (_show_status){
        Linfo("status code:%d ws js:%s", code, data.c_str());
        if(code != 300 && code != 305 && code != 701 && code != -1 && code != 200 && code != 407 && (code < 800 || code > 900)){          
            if(code == 306){
                if(_type != Task::TASK_PATROL || _type != Task::TASK_PATROL_AUTO){
                   // Linfo("report status code 111:%d", code);
                    Notifier::publish_pos_status(code, _type);
                }
            }
            else{
                //Linfo("report status code 222:%d", code);
                if(code == 404 || code == 408){
                    //do not report 
                } else {
                    Notifier::publish_pos_status(code, _type);
                }
            }
        }
        _status_data = "";
    }
    else
        _status_data = data;

    _status_code = code;
    _show_status = false;
   // Linfo("%s on_status ok", _name.c_str());
   // detect_front_obstacle();
}

void NavPointTask::on_loop(){
   Linfo("%s %s show status front:[%d|%d]", _name.c_str(), __FUNCTION__, _front_cnt, _fronts.size());
    if (!_st_ws_connected){
        if (--_status_timeout == 0){
            Linfo("%s connect ws timeout to reconnect", _name.c_str());
#if 0
            _ret.ret = ERR_NONE_NAV_STATUS;
            _nxt_st = FIN;
            Linfo("stop all gs task by ret:%d", _ret.ret);
            Notifier::get_instance().set_status_cb(nullptr);
            GsApi::get_instance()->nav_task_stop_all();
#endif 
            _status_timeout = Config::get_instance()->nav_task_start_timeout;
            Notifier::get_instance().connect_ws_status();
        }
    }
    else{
        _st_ws_connected = false;
        _show_status = true;
        if (!_status_data.empty()){
            _status_data.clear();
            if(_status_code != -1 && _status_code!= 200 && _status_code != 701 
            && _status_code!= 407 && (_status_code< 800 || _status_code> 900)){
                Linfo("report status code 333:%d", _status_code);
                Notifier::publish_pos_status(_status_code, _type);
            }
        }

        // added by jzx, 2019/12/11 , judge backward movement and obstacle in the front
        //tts_front_obstacle();
        // notify front obstacle status to web
        //notify_front_obstacle();
    }
    detect_front_obstacle();
}

void NavPointTask::notify_front_obstacle()
{
    Json::Value root;

    double now = ros::Time().now().toSec();

    if (now < front_obstacle_check_time_last_) 
    {
        front_obstacle_check_time_last_ = now;
    }

    if (now < front_obstacle_report_time_last_) 
    {
        front_obstacle_report_time_last_ = now;
    }

    if(1 == _front_obstacle_check_flag && 0 == _front_obstacle_check_flag_last)
    {
        front_obstacle_start_to_count_ = 1;
        // record the time that obstacle appeared and start to count for 1 min
        front_obstacle_report_time_last_ = now;
        //front_obstacle_check_time_last_ = now;
    }
    else if(0 == _front_obstacle_check_flag && 1 == _front_obstacle_check_flag_last)
    {
        front_obstacle_start_to_count_ = 0;
    }
    else
    {

    }

    //_front_obstacle_check_flag_last = _front_obstacle_check_flag;

    if ((1 == front_obstacle_start_to_count_) && (!front_obstacle_notified_) && (((now - front_obstacle_report_time_last_) >= 60.0) || ((now - front_obstacle_report_time_last_) < 0.0)))
    {
        char rand_str[16] = {0};
        Utils::get_instance()->getRandStr(rand_str);
        front_obstacle_serial_num_ = rand_str;
        reportFrontObstacleEvent(1, front_obstacle_serial_num_);
        //reportObstacleEventToPc(1);
        front_obstacle_notified_ = true;
    }
    else if((0 == front_obstacle_start_to_count_) && front_obstacle_notified_)
    {
        reportFrontObstacleEvent(0, front_obstacle_serial_num_);
        //reportObstacleEventToPc(0);
        front_obstacle_notified_ = false;
    }

    if(front_obstacle_start_to_count_ == 1 && !tts_sound_once_flg_)
    {
        sendNavTtsText(TTSStrings::TTS_KEY_ENCOUNTER_OBSTACLE);
        front_obstacle_check_time_last_ = now;
        tts_sound_once_flg_ = true;
    }
    else if(front_obstacle_start_to_count_ == 0)
    {
        tts_sound_once_flg_ = false;
    }
    else
    {

    }

    if ((tts_sound_once_flg_ == true) && (1 == front_obstacle_start_to_count_) && (((now - front_obstacle_check_time_last_) >= 30.0) || ((now - front_obstacle_check_time_last_) < 0.0)))
    {

       // Linfo("now time = %lf , front obstacle check last = %lf , diff = %lf",now,front_obstacle_check_time_last_, now - front_obstacle_check_time_last_);
        sendNavTtsText(TTSStrings::TTS_KEY_ENCOUNTER_OBSTACLE);
        front_obstacle_check_time_last_ = now;
    }

    _front_obstacle_check_flag_last = _front_obstacle_check_flag;

}

#if 0
void NavPointTask::reportObstacleEventToPc(int val)
{
    atris_msgs::SignalMessage diag_msg;
    Json::Value content;
    Json::Value evt_arr;
    Json::Value evt_arr_content;
    std::string event_content = "";
    Json::FastWriter writer;

    content["result"] = "success";
    content["result_code"] = 0;

    evt_arr_content["avoiding"] = val;

    evt_arr.append(evt_arr_content);
    
    content["events"] = evt_arr;
    event_content = writer.write(content);
    log_warn("%s event_content : %s",__FUNCTION__, event_content.c_str());

    Utils::get_instance()->NotifyRobotStatus("notify_event",content);
}

#endif


void NavPointTask::nav_emergency_stop(int state)
{
    Linfo("%s",__FUNCTION__);
    atris_msgs::SignalMessage msg;
    Json::Value root;
    Json::FastWriter fw;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    root["title"] = "request_emergency_stop";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["switch"] = state;
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;
    std::string req_data = fw.write(root);

    atris_msgs::SignalMessage req;
    req.account = shmrbt.robot.sn;
    req.msgID = uid.str();
    req.msg = req_data.c_str();
    req.title = "request_emergency_stop";
    _signal_request_pub.publish(req);
}

void NavPointTask::nav_set_light_color(int color)
{
    Linfo("%s",__FUNCTION__);
    atris_msgs::SignalMessage msg;
    Json::Value root;
    Json::FastWriter fw;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    root["title"] = "request_set_indicator_color";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["color"] = color;
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;
    std::string req_data = fw.write(root);

    atris_msgs::SignalMessage req;
    req.account = shmrbt.robot.sn;
    req.msgID = uid.str();
    req.msg = req_data.c_str();
    req.title = "request_set_indicator_color";
    _signal_request_pub.publish(req);
}

void NavPointTask::nav_set_light_style(int style)
{
    Linfo("%s",__FUNCTION__);
    atris_msgs::SignalMessage msg;
    Json::Value root;
    Json::FastWriter fw;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    root["title"] = "request_set_indicator_style";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["style"] = style;
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;
    std::string req_data = fw.write(root);

    atris_msgs::SignalMessage req;
    req.account = shmrbt.robot.sn;
    req.msgID = uid.str();
    req.msg = req_data.c_str();
    req.title = "request_set_indicator_style";
    _signal_request_pub.publish(req);
}

void NavPointTask::nav_release_light_ctrl()
{
    Linfo("%s",__FUNCTION__);
    atris_msgs::SignalMessage msg;
    Json::Value root;
    Json::FastWriter fw;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    root["title"] = "request_release_indicator_light_control";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;
    std::string req_data = fw.write(root);

    atris_msgs::SignalMessage req;
    req.account = shmrbt.robot.sn;
    req.msgID = uid.str();
    req.msg = req_data.c_str();
    req.title = "request_release_indicator_light_control";
    _signal_request_pub.publish(req);
}

void NavPointTask::on_recv_anti_drop(const atris_msgs::AntiDrop& msg) //detect cliff
{
    //Ldebug("%s",__FUNCTION__);
    if (Config::get_instance()->nav_anti_drop_enable){
        if (msg.trigged && !_is_trigged) {
            _is_trigged = true;
            Ldebug("anti drop sensor trigged.let's pause task.");
            pause();
        }

        if(!msg.trigged && _is_trigged) {
            Ldebug("anti drop sensor untrigged.let's resume task.");
            resume();
            _is_trigged = false;
        }
    }
}

void NavPointTask::detect_front_obstacle()
{
    Ldebug("_status_code : %d , _status_code_last_rnd: %d", _status_code, _status_code_last_rnd);
    if (_status_code == 403) {
        Ldebug("encounter_obstacle......");
        _after_obstacle_remove_count = 0;

        nav_set_light_color(RED);
        if(Config::get_instance()->obstacle_mode_ == Obstacle_Mode::EMERGENCY_STOP) {
            nav_emergency_stop(1);
            Ldebug("nav emergency_stop will start........");
        }

        _status_code_last_rnd = _status_code;
    }

    if (_status_code != 403 && _status_code_last_rnd == 403 ) {
        _after_obstacle_remove_count += 1;
        Ldebug("wait obstacle remove ?...%d", _after_obstacle_remove_count);
        if (_after_obstacle_remove_count > 3) {  //todo define timeout
            _after_obstacle_remove_count = 0;
            _status_code_last_rnd = -1;

            nav_release_light_ctrl();
            if(Config::get_instance()->obstacle_mode_ == Obstacle_Mode::EMERGENCY_STOP) {
                nav_emergency_stop(0);
                Ldebug("nav emergency_stop will free........");
            }
        }
    }

    _status_code = -1;
}

void NavPointTask::reportFrontObstacleEvent(int status, std::string serial_num)
{
    Json::Value root;
    Json::Value js_evt;

    js_evt["event_type"] = "avoiding";
    js_evt["avoiding"] = status;
    js_evt["serial_num"] = serial_num;

    root["result"] = "success";
    root["result_code"] = 0;
    root["events"].append(js_evt);
    
    Utils::get_instance()->NotifyRobotStatus("notify_event",root,"");
}

void NavPointTask::sendNavTtsText(int key)
{
    Linfo("%s nav sending tts msg",__FUNCTION__);
    atris_msgs::AisoundTTS msg;
    msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    msg.text = TTSStrings::text(key);
    _aisound_tts_pub.publish(msg);
}

int NavPointTask::_auto_nav_to_point(){
    string map_name = MapManager::get_instance().get_using_map(); 
    Linfo("%s map:%s mode:%d id:%s name:%s pos:[%lf,%lf,%lf]",
          __FUNCTION__, map_name.c_str(), _pt.mode, _pt.id.c_str(), _pt.np.name.c_str(), _pt.np.pos.x, _pt.np.pos.y, _pt.np.pos.angle);
    using namespace std::placeholders;
    NavStatusCB cb = std::bind(&NavPointTask::on_status, this, placeholders::_1);
    Notifier::get_instance().set_status_cb(cb);
    return GsApi::get_instance()->auto_nav_to(_pt);
}

int NavPointTask::_to_point(){
    Linfo("%s", __FUNCTION__);
    int ret = Locator::get_instance().is_located();

#if 0
    if (ret != ERR_OK){//todo to report to client?
        Lerror("not located:%d", ret);
        return ret;
    }
#endif 
    string task_name(TASK_NAME);

    using namespace std::placeholders;
    NavStatusCB cb = std::bind(&NavPointTask::on_status, this, placeholders::_1);
    Notifier::get_instance().set_status_cb(cb);
    return GsApi::get_instance()->auto_nav_to(_pt);
}


void NavChgPointTask::on_status_code(string &data, task_lock &lock){
    Json::Reader reader;
    Json::Value root;

    if (!reader.parse(data, root)){
        Lerror("parse gs status js data fail js:%s", data.c_str());
        return;
    }

    int code = root["statusCode"].asInt();
    if (code != _status_code){
        _show_status = false;
        _status_data = "";
        if (code != 200 || (code >= 800 && code <= 899))
            Notifier::publish_pos_status(code, _type);
    }
    else
        _status_data = data;
    _status_code = code;

    if (code == 407){
        Notifier::get_instance().set_status_cb(nullptr);
      //  Charger::get_instance()->start_charge();  //todo define xs udock & ubt udock
        _next_status(STP, lock);
        Linfo("%s stop ok2", _name.c_str());

        Notifier::publish_pos_status(code, _type);
    }
    else if (code == 411){
        _prev_code = 411;
    }
    else if(code == 408){
        Linfo("before code:%d", _prev_code);
        if (_prev_code == 411){
            Notifier::get_instance().set_status_cb(nullptr);
       //     Charger::get_instance()->start_charge();
            _prev_code = 0;
            _next_status(STP, lock);
            Linfo("%s stop ok1", _name.c_str());
            Notifier::publish_pos_status(code, _type);
        }
    }
/*    
    else if(code == 413){
        Notifier::get_instance().set_status_cb(nullptr);
        GsApi::get_instance()->nav_task_stop_all();
        _next_status(STP, lock);
        Linfo("%s charge stop err", _name.c_str());

        Notifier::publish_pos_status(code, _type);
    }
*/    
  //  Linfo("%s on_status ok", _name.c_str());

}

int NavChgPointTask::_to_point(){
    Linfo("%s", __FUNCTION__);
    int ret = Locator::get_instance().is_located();
#if 0
    if (ret != ERR_OK){//todo to report to client?
        Lerror("isn't located:%d", ret);
        return ret;
    }
#endif 
    using namespace std::placeholders;
    NavStatusCB cb = std::bind(&NavChgPointTask::on_status, this, placeholders::_1);
    Notifier::get_instance().set_status_cb(cb);
    //string map_name = MapManager::get_instance().get_using_map();
    //ret = GsApi::get_instance()->nav_auto_charge(map_name, _pt.np.name);//todo hard code should pass by scheme
    GsApi::get_instance()->auto_nav_to(_pt);
    if (ret != ERR_OK)
        Notifier::get_instance().set_status_cb(nullptr);

    return ret;
}


void NavChgPointTask::on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg)
{
    Linfo("NavChgPointTask on_receive_charge_state");
    if (msg.charge_msg_type == atris_msgs::PowerChargeCmd::START_LEAVE_PILE) {
        _is_leaving_pile = true;
        _is_charging = false;
        return;
    }

    if(msg.charge_msg_type == atris_msgs::PowerChargeCmd::CHASSIS_NOTIFY_CHARGE_STATUS
        && msg.charge_status == 0x01 && msg.charge_result == 0x00) {
            Linfo("111111111111111111111111111111111111111111111111111");
            if (_is_leaving_pile || _is_charging) {
                _is_leaving_pile = false;
                _is_charging = false;
                Linfo("222222222222222222222222222222222222222222");
                return;
            }
            Linfo("333333333333333333333333333333333333333333333333");
            nav_emergency_stop(0);
            TASK_LOCK();
            Notifier::get_instance().set_status_cb(nullptr);
            GsApi::get_instance()->nav_task_stop_all();
            _next_status(STP, lock);
            Linfo("%s robot stoped................", _name.c_str());
            atris_msgs::PowerChargeCmd charge_cmd;
            charge_cmd.charge_msg_type = atris_msgs::PowerChargeCmd::ROBOT_NOTIFY_IN_POSITION;
            charge_cmd.charge_status = 0x01;
            charge_request_pub_.publish(charge_cmd);
            Linfo("notify_reached_point................");
            shm::ChargeState shm_state;
            shm_state.state = dockSDKState::DOCK_SUCESS;
            shm::iMemory_write_ChargeState(&shm_state);
            Notifier::get_instance().publish_udock_status(dock_succuss);
            Notifier::get_instance().publish_udock_status(charge_success); //todo
            _is_charging = true;
            _is_leaving_pile = false;
    }

}

}
