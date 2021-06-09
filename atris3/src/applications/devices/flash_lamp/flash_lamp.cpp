#include <boost/thread/thread.hpp>
#include "flash_lamp.h"
#include "can/can.h"
#include <json/json.h>
#include "utils/utils.h"
#include "imemory/atris_imemory_api.h"

#define LAMP_CAN_ID 0x80
#define DEFULT_REB_BLUE_FLASH_SPEED   2.3  //unit:Hz

FlashLamp::FlashLamp() {
  log_info("%s", __FUNCTION__);
  signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &FlashLamp::on_recv_lamp_ctrl1, this);
  lamp_cmd_sub_ = nh_.subscribe(TOPIC_LAMP_CMD_MESSAGE, 100, &FlashLamp::on_recv_lamp_ctrl, this);
  get_lamp_status_srv_ = nh_.advertiseService(SRV_GET_LAMP_STATUS, &FlashLamp::do_get_lamp_status, this);
  get_gps_position_srv_client_ = nh_.serviceClient<atris_msgs::GetGpsPos>(SRV_GET_GPS_POSITION);
  new boost::thread(boost::bind(&FlashLamp::red_blue_flash_task, this));
  rb_serial_num_ = "";
  w_serial_num_ = "";
}

int FlashLamp::init()
{
    log_info("flash lamp init");

    return 0;
}

int FlashLamp::init(CanService *can_service)
{
    this->can_service = can_service;

    DevFilter filter;
    filter.filter = boost::bind(&FlashLamp::flash_lamp_filter, this, _1);
    can_service->add_filter(filter);

    return 0;
}

/**
 * @brief check_state 检查灯光设备是否工作正常
 *
 * @return
 */
bool FlashLamp::check_state()
{
    bool ret = false;
    CanPkg pkg = {0};

    pkg.head.id.channel = 0x80;
    pkg.data[0] = FL_CMD_STATUS;

    log_info("check lamp state.");
    if( can_service->send_for_ack(&pkg, 1) < 0){
        log_error("flash lamp can't work.");
        ret = false;
    }
    else{
        log_debug("flash lamp work fine.");
        ret = true;
    }

    lamp_state = ret;
    return ret;
}

bool FlashLamp::get_state()
{
    return lamp_state;
}

int FlashLamp::get_lamp_status(char& rb_status, char& w_status, char& alarm_status)
{
    CanPkg pkg = {0};

    pkg.head.id.channel = 0x80;
    pkg.data[0] = FL_CMD_STATUS;

    if( can_service->send_for_ack(&pkg, 1) < 0){
        log_error("can't get lamp state");
        return -1;
    }
    rb_status = pkg.data[1];
    w_status = pkg.data[2];
    alarm_status = pkg.data[3];

    return 0;
}

void FlashLamp::construct_light_evt_pack(int source, char status, int type)
{
    ros::Time now = ros::Time::now();
    Json::Value content;
    Json::Value evt_arr;
    Json::Value light_event_msg;
    std::string title;
    Json::FastWriter writer;
    std::string event_content = "";

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    content["category"] = "custom_event"; // custom event

    // eventId according to message type
    if(type == atris_msgs::LampCmd::LAMP_TYPE_FLASH)
    {
        content["eventId"] = "flash_light_event";
        title = "notify_flash_light_event";
    }
    else
    { 
        content["eventId"] = "white_light_event";
        title = "notify_white_light_event";
    }

    // accid
    content["userId"] = accid;

    // timestamp
    content["recordedAt"] = (uint64_t)(now.toSec());
    // segmentaition is NULL
    content["segmentation"] = "";

    content["customSegmentation"]["status"] = status;

    if(type == atris_msgs::LampCmd::LAMP_TYPE_FLASH)
    {
        content["customSegmentation"]["serialNum"] = rb_serial_num_;
    }
    else 
    {
        content["customSegmentation"]["serialNum"] = w_serial_num_;
    }

    // reason
    content["customSegmentation"]["reason"]["type"] = "default";
    content["customSegmentation"]["reason"]["value"] = "";

    //source
    content["customSegmentation"]["source"] = source;

    // gps position
    content["customSegmentation"]["lati"] = "";
    content["customSegmentation"]["longi"] = "";
    atris_msgs::GetGpsPos gpspos;
    if (get_gps_position_srv_client_.call(gpspos)) 
    {
        if(gpspos.response.status == 0)
        {
            content["customSegmentation"]["lati"] = gpspos.response.latitude;
            content["customSegmentation"]["longi"] = gpspos.response.longitude;
        }
    } 
    else 
    {
        log_warn("%s get_gps_position_srv_client_ call fail.", __FUNCTION__);
        //return;
    }

    evt_arr.append(content);
    event_content = writer.write(evt_arr);
    log_info("%s event_content is %s",__FUNCTION__, event_content.c_str());

    light_event_msg["event_message"] = evt_arr;

    // http post interface

    Utils::get_instance()->NotifyRobotStatus(title, light_event_msg);
}

void FlashLamp::control_red_blue_led(char status, float speed, int source)
{
    CanPkg pkg = {0};

    log_info("set rb :%c speed: %fHz", status, speed);
    uint16_t flash_speed = 217;
    if (speed > 0) {
        flash_speed = 1000.0/2/speed;
    }
    pkg.head.h32 = 0x80;
    pkg.data[0] = FL_CMD_RB;
    pkg.data[1] = status;
    pkg.data[2] = (unsigned char)(flash_speed & 0xFF);
    pkg.data[3] = (unsigned char)((flash_speed >> 8) & 0xFF);

    if( can_service->send_for_ack(&pkg, 4) < 0){
        log_error("set_red_blue_flash_status fail");
        rb_status = FL_ERR;
        return;
    }

    if(status == 1 && get_rb_lamp_status() == 0)
    {
        char rand_str[16] = {0};
        Utils::get_instance()->getRandStr(rand_str);
        rb_serial_num_ = rand_str;
        construct_light_evt_pack(source, status, atris_msgs::LampCmd::LAMP_TYPE_FLASH);
    }
    else if(status == 0 && get_rb_lamp_status() == 1)
    {
        construct_light_evt_pack(source, status, atris_msgs::LampCmd::LAMP_TYPE_FLASH);

    }
    else
    {

    }

    rb_status = (LampStatus)status;
    Utils::get_instance()->publish_event(EVT_FLASHING, status > 0 ? EVT_OPENED:EVT_CLOSED);

    return;
}

void FlashLamp::red_blue_flash_task(void)
{
    boost::unique_lock<boost::mutex> lock(FlashLamp::mutex, boost::defer_lock);
    RedBlueLedSetCmdExtra *cmd = NULL;
    while(1){
        cmd = cmd_list.get();
        log_info("%s status=%c speed=%f time=%d source=%d", __FUNCTION__, cmd->get_status(), cmd->get_speed(), cmd->get_delay_time(), cmd->get_command_source());
        control_red_blue_led(cmd->get_status(), cmd->get_speed() , cmd->get_command_source());
        if(cmd->get_status() == FL_ON && cmd->get_delay_time() > 0){   
            lock.lock();
            FlashLamp::cond.timed_wait(lock, boost::get_system_time() + boost::posix_time::seconds(cmd->get_delay_time()));
            control_red_blue_led(FL_OFF, 0, cmd->get_command_source());
            lock.unlock();
        }
        delete cmd;
    }
}

int FlashLamp::set_red_blue_flash_status(char status, int source)
{
    boost::unique_lock<boost::mutex> lock(mutex);
    if(status == FL_ON){
        cond.notify_one();
    }
    RedBlueLedSetCmdExtra *cmd = new RedBlueLedSetCmdExtra(status, DEFULT_REB_BLUE_FLASH_SPEED, 0, source);
    cmd_list.add(cmd);
    return 0;
}

int FlashLamp::set_red_blue_flash_status(char status, float speed, int source)
{
    boost::unique_lock<boost::mutex> lock(mutex);
    if(status == FL_ON){
        cond.notify_one();
    }
    RedBlueLedSetCmdExtra *cmd = new RedBlueLedSetCmdExtra(status, speed, 0, source);
    cmd_list.add(cmd);
    return 0;
}

int FlashLamp::set_red_blue_flash_status(char status, float speed, int time, int source)
{
    log_info("%s status=%d speed=%f time=%d list_size=%d", __FUNCTION__, (char)status, speed, time, cmd_list.size());
    boost::unique_lock<boost::mutex> lock(mutex);
    if(status == FL_ON){
        cond.notify_one();
    }
    RedBlueLedSetCmdExtra *cmd = new RedBlueLedSetCmdExtra(status, speed, time, source);
    cmd_list.add(cmd);
    return 0;
}

int FlashLamp::set_white_lamp_status(char status , int source)
{
    CanPkg pkg = {0};

    log_info("set white status:%d", status);

    pkg.head.id.channel = 0x80;
    pkg.data[0] = FL_CMD_W;
    pkg.data[1] = status;

    if( can_service->send_for_ack(&pkg, 2) < 0){
        log_error("set_white_lamp_status fail");
        w_status = FL_ERR;
        return -1;
    }

    if(status == 1 && get_white_lamp_status() == 0)
    {
        char rand_str[16] = {0};
        Utils::get_instance()->getRandStr(rand_str);
        w_serial_num_ = rand_str;
        construct_light_evt_pack(source, status, atris_msgs::LampCmd::LAMP_TYPE_WHITE);

    }
    else if(status == 0 && get_white_lamp_status() == 1)
    {

        construct_light_evt_pack(source, status, atris_msgs::LampCmd::LAMP_TYPE_WHITE);

    }
    else
    {

    }

    w_status = (LampStatus)status;
    Utils::get_instance()->publish_event(EVT_LAMP, status > 0 ? EVT_OPENED:EVT_CLOSED);
    return 0;
}

int FlashLamp::set_alarm_lamp_status(char status)
{
    CanPkg pkg = {0};

    log_info("set alarm status:%d", status);

    pkg.head.id.channel = 0x80;
    pkg.data[0] = FL_CMD_AL;
    pkg.data[1] = status;

    if( can_service->send_for_ack(&pkg, 2) < 0){
        log_error("set_alarm_lamp_status fail");
        return -1;
    }

    alarm_status = (LampStatus)status;

    return 0;
}

int FlashLamp::flash_lamp_filter(void *data)
{
    CanPkg *pkg = (CanPkg*)data;

    //如果是灯光控指令，则处理灯光控制指令，并且返回 0；
    if(pkg->head.id.channel == CH_LAMP){
        log_info("lamp cmd, do lamp filter");

        return 0;
    }

    return -1;
}

LampStatus FlashLamp::get_white_lamp_status()
{
    return w_status;
}

LampStatus FlashLamp::get_rb_lamp_status()
{
    return rb_status;
}

LampStatus FlashLamp::get_alarm_lamp_status()
{
    return alarm_status;
}

void FlashLamp::on_recv_lamp_ctrl(const atris_msgs::LampCmd &msg)
{
  if (msg.type == atris_msgs::LampCmd::LAMP_TYPE_WHITE) {
      set_white_lamp_status(msg.status , msg.source);
  } else if(msg.type == atris_msgs::LampCmd::LAMP_TYPE_FLASH) {
      set_red_blue_flash_status(msg.status , msg.source);
  }
}

void FlashLamp::on_recv_lamp_ctrl1(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value req, resp;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if (msg.title == "request_get_light_status") {
        char w_status, rb_status, alarm_status;
        int ret = get_lamp_status(rb_status, w_status, alarm_status);
        if(ret < 0){
            resp["result"] = "fail_inner_error";
            Utils::get_instance()->responseResult(msg, resp, "response_get_light_status");
            return ;
        }
        log_info("lamp status:%d %d %d", w_status, rb_status, alarm_status);
        {
            if(!reader.parse(msg.msg, req)){
                log_error("parse json fail.");
                resp["result"] = "fail_invalid_data";
                Utils::get_instance()->responseResult(msg, resp, "response_get_light_status");
                return;
            }

            resp["w_light"] = w_status;
            resp["rb_light"] = rb_status;
            resp["alarm_light"] = alarm_status;
            Utils::get_instance()->responseResult(msg, resp, "response_get_light_status");
        }
    }
    else if(msg.title == "request_switch_light"){
        reader.parse(msg.msg, req);
        int state = req["content"]["switch"].asInt();
        int ret = -1;
        int type = req["content"]["lamp"].asInt();
        if(state == 1)
            state = FL_ON;
        else
            state = FL_OFF;

        resp["switch"] = state;
        resp["lamp"] = type;
        if(type == 0){//白灯
            ret = set_white_lamp_status(state, atris_msgs::LampCmd::LAMP_SOURCE_SIGNAL);
        }
        else if(type == 1){//红蓝爆闪
            ret = set_red_blue_flash_status(state, atris_msgs::LampCmd::LAMP_SOURCE_SIGNAL);
        }
        else if(type == 2){//报警灯
            ret = set_alarm_lamp_status(state);
        }
        else{
            resp["result"] = "fail_invalid_data";
        }

        if(ret < 0)
            resp["result"] = "fail_inner_error";

        Utils::get_instance()->responseResult(msg, resp, "response_switch_light");
    }
}

int FlashLamp::close_all_lamp()
{
    set_white_lamp_status(FL_OFF, atris_msgs::LampCmd::LAMP_SOURCE_SIGNAL);
    set_red_blue_flash_status(FL_OFF, atris_msgs::LampCmd::LAMP_SOURCE_SIGNAL);
    return 0;
}

bool FlashLamp::do_get_lamp_status(atris_msgs::GetLampStatus::Request& req,
  atris_msgs::GetLampStatus::Response& res) {
    res.w_status = w_status;
    res.rb_status = rb_status;
    return true;
}

