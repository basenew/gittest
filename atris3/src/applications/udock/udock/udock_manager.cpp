/*
    Udock manager by xiangbin.huang
*/

#include "udock_manager.h"
#include "unistd.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <stdio.h>
#include <math.h>
#include "utils/utils.h"
#include "tts_strings/tts_strings.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"
#include "platform/pm/PMData.h"
#include "platform/chassis/ChassisDef.h"


UdockTankManager::UdockTankManager() {
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &UdockTankManager::on_pc_cmd, this);
    set_vel_to_udock_sub_ = nh_.subscribe(TOPIC_SET_VEL_TO_UDOCK, 100, &UdockTankManager::on_determine_safe_control, this);
    time_calibrated_sub_ = nh_.subscribe(TOPIC_TIME_CALIBRATED_MESSAGE, 100, &UdockTankManager::on_recv_time_calibrated, this);
    dock_sdk_cmd_sub_ = nh_.subscribe(TOPIC_DOCK_SDK_CMD, 100, &UdockTankManager::on_dock_sdk_cmd, this);
    charge_info_sub_ = nh_.subscribe(TOPIC_CHARGE_INFO_MESSAGE, 100, &UdockTankManager::on_charge_info, this);
    signal_resp_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    set_vel_cmd_pub_ = nh_.advertise<atris_msgs::VelCmd>(TOPIC_SET_VEL_TO_CHASSIS, 100);
    charge_cmd_pub_ = nh_.advertise<atris_msgs::ChargeCmd>(TOPIC_CHARGE_CMD_MESSAGE, 100);
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    get_nav_pose_srv_client_ = nh_.serviceClient<atris_msgs::GetNavPose>(SRV_GET_NAV_POSE);
    get_gs_laser_raw_data_srv_client_ = nh_.serviceClient<atris_msgs::GetGsLaserRawData>(SRV_GET_GS_LASER_RAW_DATA);
    
    udock = UdockTank::get_instance();
    gsudp = UdockData::get_instance();
    obstacle_avoid_under_given_control = ObstacleAvoidUnderGivenControl::get_instance();
    wheel_obstacle_detect = UdockWheelObstacleDetect::get_instance();

#ifdef _CHASSIS_MARSHELL_
    move_control = moveControl::get_instance();
#endif
    set_move_state(false);
    charge_state_ = IDLE;
    shm::ChargeState shm_state;
    shm_state.state = charge_state_;
    shm::iMemory_write_ChargeState(&shm_state);
    battery_charging_state_ = false;
    is_moving_state = false;
#if  (defined _POLICE_) && (defined _CHASSIS_JC_) //公安定制履带版
    log_info("[udock]Atris polic track udock!!!");
#elif (defined _POLICE_) && (defined _CHASSIS_MARSHELL_)//公安定制轮式版
    log_info("[udock]Atris polic wheel udock!!!");
#elif (defined _CHASSIS_JC_)    //履带版
    log_info("[udock]Atris standard track udock!!!");
#elif (defined _CHASSIS_MARSHELL_)
    log_info("[udock]Atris standard wheel udock!!!");
#else                           //制轮式版
    log_info("Atris no define udock!!!");
#endif

   shm::iMemory_read_TimeCalibrated(&tc_);
}

UdockTankManager::~UdockTankManager()
{
}

void UdockTankManager::on_recv_time_calibrated(const atris_msgs::TimeCalibrated &msg) {
    tc_.calibrated = msg.calibrated;
    tc_.interval = msg.interval;
}

void UdockTankManager::on_dock_sdk_cmd(const atris_msgs::DockSDKCmd &msg) {
  if (msg.cmd == atris_msgs::DockSDKCmd::BEGAIN_DOCK) {
    log_info("UdockTankManager::on_dock_sdk_cmd BEGAIN_DOCK");
    dock_sdk(BEGAIN_DOCK);
  } else if (msg.cmd == atris_msgs::DockSDKCmd::CANCEL_DOCK) {
    log_info("UdockTankManager::on_dock_sdk_cmd CANCEL_DOCK");
    dock_sdk(CANCEL_DOCK);
  } else {
    log_warn("Unknow dock command: %d)", msg.cmd);
  }
}

void UdockTankManager::on_determine_safe_control(const atris_msgs::VelCmd &msg) 
{
    atris_msgs::VelCmd vel_cmd;
    vel_cmd.priority = msg.priority;
    
#ifndef _CHASSIS_MARSHELL_
    ControlVariable ctrl_in;
    ControlVariable ctrl_out;
    ctrl_in.v_x = msg.x;
    ctrl_in.v_theta = msg.z;
    obstacle_avoid_under_given_control->determine_safe_control(ctrl_in, ctrl_out);

    vel_cmd.x = ctrl_out.v_x;
    vel_cmd.z = ctrl_out.v_theta;
#else
   float ms_x = msg.x;
   float ms_z = msg.z;
   if(ms_x >0){
     vel_cmd.x = fabs(wheel_obstacle_detect->get_safe_front_speed_stategy(ms_x, ms_z));
   } else if(ms_x <0) {
     vel_cmd.x = -fabs(wheel_obstacle_detect->get_safe_back_speed(ms_x, ms_z));
   }
   
   vel_cmd.z = msg.z;
#endif
   vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
   set_vel_cmd_pub_.publish(vel_cmd);
}

void UdockTankManager::udock_thread(void)
{
    static timespec time_begain;
    static timespec time_now;
	
    atris_msgs::AisoundTTS tts_msg;
    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
	
    sleep(1);
    while(1)
    {
        switch(get_charge_state())
        {
            case RECEIVE_CMD:
                log_info("[udock]RECEIVE_CMD!");
                udock->reset_wall_width();
                report_charge_state_to_pc(S_BEGAIN_DOCK);
                set_move_state(true);
                set_charge_state(MOVE_POINT);
                tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_BEGAIN);
                aisound_tts_pub_.publish(tts_msg);
                break;
            case MOVE_POINT:
                log_info("[udock]MOVE_POINT!");
                report_charge_state_to_pc(S_MOVING);
                if(move_to_dock()){
                    clock_gettime(CLOCK_REALTIME, &time_begain);
                    set_charge_state(BLUETOOTH_PARE);
                    report_charge_state_to_pc(S_BLUETOOTH_PARING);
                    blue_tooth_pare();
                }
                else{
                    log_info("[udock]DOCK_FAIL step12 move fail!");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                    report_charge_state_to_pc(S_FAIL_MOVE);
                    set_charge_state(IDLE);
                }
                break;
            case BLUETOOTH_PARE: //wait for mcu data
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >30){ //30s time out
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                    report_charge_state_to_pc(S_FAIL_BLUETOOTH_PARE);
                    set_charge_state(IDLE);
                    log_info("[udock]DOCK_FAIL bluetooth timeout!");
                }
                usleep(10 *1000);
                break;
            case MOVE_POINT_LAST_STEP:
                report_charge_state_to_pc(S_DOCKING);
                if(move_to_dock_last_step()){
                    set_charge_state(SWITCH_ON);
                    clock_gettime(CLOCK_REALTIME, &time_begain);
                }
                else{
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                    log_info("[udock]DOCK_FAIL last move fail!");
                    report_charge_state_to_pc(S_FAIL_MOVE);
                    set_charge_state(IDLE);
                }
                break;
            case SWITCH_ON:
                log_info("[udock]statemachine in SWITCH_ON!");
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >30){ //30s time out
                    report_charge_state_to_pc(S_FAIL_DOCK);
                    set_charge_state(IDLE);
                    log_info("[udock]DOCK_FAIL no switch data!");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                }
                usleep(10 *1000);    //wait for mcu data
                break;
            case WAITING_CHARGE:
                log_info("[udock]statemachine in WAITING_CHARGE!");
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >30){ //30s time out
                    report_charge_state_to_pc(S_FAIL_CHARGE);
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                    set_charge_state(IDLE);
                    log_info("[udock]DOCK_FAIL no charge data!");
                }
                usleep(10 *1000);    //wait for mcu data
                break;
            case DOCK_SUCESS:
                if (!battery_charging_state_){
                    battery_charging_state_ = true;
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_SUCCESS);
                    aisound_tts_pub_.publish(tts_msg);
                    log_info("[udock]DOCK_SUCESS!");
                    //set_charge_state(IDLE);
                    report_charge_state_to_pc(S_CHARGING);
                }
                break;
            case RECEIVE_LEAVE_CMD:
                tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_UDOCK);
                aisound_tts_pub_.publish(tts_msg);
                log_info("[udock][leave away from pile start!]");
                set_move_state(true);
    #ifndef _CHASSIS_MARSHELL_
                if(!move_away_from_dock(-160, 10, 80)){ //1.6m
                    log_info("[udock][leave away from pile success!]");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_SUCCESS);
                    aisound_tts_pub_.publish(tts_msg);
                }
                else{
                    log_info("[udock][leave away from pile fail!]");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                }
    #else
                if(move_away_from_dock_wheel(160, 20) == true){ //1.6m
                    log_info("[udock][w leave away from pile success!]");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_SUCCESS);
                    aisound_tts_pub_.publish(tts_msg);
                }
                else{
                    log_info("[udock][leave away from pile fail!]");
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_FAIL);
                    aisound_tts_pub_.publish(tts_msg);
                }
    #endif
                set_move_state(false);
                set_charge_state(IDLE);
                notify_leave_msg();
                break;
            case IDLE:
                set_move_state(false);
                break;
            default:
                break;
        }
        usleep(100 * 1000);//10HZ
    }
}

void UdockTankManager::play_tts_cycle_thread(void)
{
    static int time = 0;
    sleep(1);
    while(1)
    {
        if(get_charge_state() == IDLE || get_charge_state() == DOCK_SUCESS || get_charge_state() == RECEIVE_LEAVE_CMD){
            time = 0;
        }
        else{
            time ++;
            if(time >=30){// play tts 30 seconds
                atris_msgs::AisoundTTS tts_msg;
                if(get_charge_state() == RECEIVE_LEAVE_CMD){
                    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_LEAVE_UDOCK);
                    aisound_tts_pub_.publish(tts_msg);
                }
                else{
                    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_BEGAIN);
                    aisound_tts_pub_.publish(tts_msg);
                }
                time = 0;
            }
        }
        sleep(1);
    }
}

void UdockTankManager::udock_move_test_thread(void)
{
    const char *dir_path_m1 = "/home/atris/test_move_font";
    const char *dir_path_m2 = "/home/atris/test_move_back";
    const char *dir_path_l  = "/home/atris/test_move_left";
    const char *dir_path_r  = "/home/atris/test_move_right";

    float speed_front   = DOCK_LINEAR_SPEED     /100.0f;
    float speed_back    = -DOCK_LINEAR_SPEED    /100.0f;
    float speed_left    = DOCK_ANGULAR_SPEED    /180.0f * M_PI;
    float speed_right   = -DOCK_ANGULAR_SPEED   /180.0f * M_PI;
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;

    while(1)
    {
        if(access(dir_path_m1, F_OK) == 0)
        {
            ControlVariable in(speed_front, 0);
            ControlVariable out;
            obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
            vel_cmd.x = out.v_x;
            vel_cmd.z = 0.0;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }
        else if(access(dir_path_m2, F_OK) == 0){
            ControlVariable in(speed_back, 0);
            ControlVariable out;
            obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
            
            vel_cmd.x = out.v_x;
            vel_cmd.z = 0.0;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }

        if(access(dir_path_l, F_OK) == 0)
        {
            ControlVariable in(0, speed_left);
            ControlVariable out;
            obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
            
            vel_cmd.x = 0.0;
            vel_cmd.z = out.v_theta;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }
        else if(access(dir_path_r, F_OK) == 0){
            ControlVariable in(0, speed_right);
            ControlVariable out;
            obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
            
            vel_cmd.x = 0.0;
            vel_cmd.z = out.v_theta;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }
        usleep(200 *1000);//5HZ
    }
}

void UdockTankManager::udock_test_thread(void)
{
    const char *dir_path    = "/home/atris/testudock";
    const char *dir_path_c  = "/home/atris/testudockc";

    while(1)
    {
        if(access(dir_path, F_OK) == 0)
        {
            dock_sdk(BEGAIN_DOCK);
            remove(dir_path);
        }
        if(access(dir_path_c, F_OK) == 0)
        {
            dock_sdk(CANCEL_DOCK);
            remove(dir_path_c);
        }
        usleep(200 *1000);//5HZ
    }
}

void UdockTankManager::init(void)
{
    new boost::thread(boost::bind(&UdockTankManager::udock_thread, this));
    new boost::thread(boost::bind(&UdockTankManager::udock_test_thread, this));
    new boost::thread(boost::bind(&UdockTankManager::play_tts_cycle_thread, this));
//    new boost::thread(boost::bind(&UdockTankManager::udock_move_test_thread, this));
}
//回充控制状态
void UdockTankManager::set_charge_state(dockSDKState state)
{
    boost::lock_guard<boost::mutex> lock(charge_state_lock);
    charge_state_ = state;

    shm::ChargeState shm_state;
    shm_state.state = charge_state_;
    shm::iMemory_write_ChargeState(&shm_state);
}
dockSDKState UdockTankManager::get_charge_state(void)
{
    return charge_state_;
}
//运动控制状态
void UdockTankManager::set_move_state(bool state)
{
    boost::lock_guard<boost::mutex> lock(move_state_lock);
    move_state_ = state;
}
bool UdockTankManager::get_move_state(void)
{
    return move_state_;
}
//正在运动
bool UdockTankManager::is_moving(void)
{
    return is_moving_state;
}

void UdockTankManager::set_moving_state(bool state)
{
    is_moving_state = state;
}

void UdockTankManager::report_charge_state_to_pc(UdockMsg state_code)
{
    GsPos pos_cur;
    log_info("[udock]report charge state to pc!");
    //report state and position
    
    atris_msgs::GetNavPose navpos;
    bool ret = get_nav_pose_srv_client_.call(navpos);
    if(ret == false){
        log_info("[udock]fail get pos from gs!");
        return;
    }
    Json::Value root;
    Json::FastWriter fw;
    int type = 6;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["title"] = "response_nav_state";
    root["content"]["x"] = Json::Value(navpos.response.pos.position.x);
    root["content"]["y"] = Json::Value(navpos.response.pos.position.y);
    root["content"]["navtype"] = Json::Value(type);
    root["content"]["angle"] = Json::Value((float)(pos_cur.angle));
    root["content"]["state"] = Json::Value(state_code);
    root["content"]["id"] = uid.str();

    std::string req_data = fw.write(root);
    atris_msgs::SignalMessage resp;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    resp.account = shmrbt.robot.receiver;
    resp.msgID = uid.str();
    resp.msg = req_data;
    signal_resp_pub_.publish(resp);
}

void UdockTankManager::on_pc_cmd(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value root, resp;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if(msg.title =="request_test_udock_start"){

        if(dock_sdk(BEGAIN_DOCK) ==0){
            resp["result"] = "re_sucess!";
        }
        else{
            resp["result"] = "re_fail!";
        }
        Utils::get_instance()->responseResult(msg, resp, "response_test_udock_start");
        return;
    }
    else if(msg.title =="request_test_udock_stop"){
        if(dock_sdk(CANCEL_DOCK) ==0){
            resp["result"] = "re_sucess!";
        }
        else{
            resp["result"] = "re_fail!";
        }
        Utils::get_instance()->responseResult(msg, resp, "response_test_udock_stop");
        return;
    }
}

void UdockTankManager::on_charge_info(const atris_msgs::ChargeInfo &msg)
{
//    log_info("[----------data] = %x %x %x %x %x %x %x", msg.data[0], msg.data[1], msg.data[2],
//                msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    switch(msg.cmd){
        case PM_INFO_DOCK_LEAVE_PILE: //
            if(msg.data[0] == 0x01){
                log_info("[udock][leave away from pile (key cmd)]");
            }
            else if(msg.data[0] == 0x02){
                log_info("[udock][leave away from pile (pc cmd)]");
            }
            if(get_charge_state() == IDLE){
                log_info("[udock][leave away from pile dock on success]");
                set_charge_state(RECEIVE_LEAVE_CMD);
                respons_leave_msg();

            }
            else{
                log_info("[udock][leave away from pile (dock busy!)]");
            }
            break;
        case PM_INFO_BLUETOOTH:
            if(msg.data[0] == 0x01){
                if(get_charge_state() == BLUETOOTH_PARE){
                    set_charge_state(MOVE_POINT_LAST_STEP);
                    log_info("[udock][bluetooth pair success!]");
                }
                else{
                    log_info("[udock][ignore redundancy bluetooth info]");
                }
            }
            else{
                report_charge_state_to_pc(S_FAIL_BLUETOOTH_PARE);
                set_charge_state(IDLE);
                log_info("[udock]dock fail:pair bluetooth erro!");
            }
            break;
        case PM_INFO_CHARGE_STATE:
            //
            if(msg.data[1] & 0x01){
                if((get_charge_state() == MOVE_POINT_LAST_STEP) || (get_charge_state() == SWITCH_ON)){
                    report_charge_state_to_pc(S_SWITCH_ON);
                    set_charge_state(WAITING_CHARGE);
                    set_move_state(false);
                    log_info("[udock][switch on -----!]");
                }
                else{
                    log_info("[udock][switch on ----- but not in state machine!]");
                }
            }
            else if(msg.data[1] & 0x02){
                if((get_charge_state() == MOVE_POINT_LAST_STEP) || (get_charge_state() == SWITCH_ON)){
                    report_charge_state_to_pc(S_COLLISION_BAR_ON);
                    set_charge_state(WAITING_CHARGE);
                    set_move_state(false);
                    log_info("[udock][collision bar on -----!]");
                }
                else{
                    log_info("[power manager][collision bar  on----- but not in state machine!]");
                }
            }
            //charging state
            if(msg.data[4] & 0x01){
                if((get_charge_state() == WAITING_CHARGE)){
                    set_charge_state(DOCK_SUCESS);
                    Utils::get_instance()->publish_event(EVT_CHARGE, EVT_TRIGGERED);
                    log_info("[udock][charging now ----- dock success!]");
                }
                else{
                    if (get_charge_state() != DOCK_SUCESS){
                        battery_charging_state_ = false;
                    }
                    log_info("[udock][charging but not in state machine]");
                }
            }
            else{
                log_info("[power manager][chagre enable off]");
            }

            //log info
            if(msg.data[0] & 0x01){
                log_info("[power manager][bluetooth link ok]");
            }
            else{
                log_info("[power manager][bluetooth link fail]");
            }
            if(msg.data[2] & 0x01){
                log_info("[power manager][elevtodes link ok]");
            }
            else{
                log_info("[power manager][elevtodes link fail]");
            }
            if(msg.data[3] & 0x01){
                log_info("[power manager][brake on]");
            }
            else{
                log_info("[power manager][brake off]");
            }
            if(msg.data[5] & 0x01){
                log_info("[power manager][relay enable on]");
            }
            else{
                log_info("[power manager][relay enable off]");
            }
            break;
        case PM_INFO_DOCK_FAIL_STATE:
             if(msg.data[0] == 0x00){
                 log_info("[power manager dock fail] stage:pair code");
             }
             else if(msg.data[0] == 0x01){
                 log_info("[power manager dock fail] stage:on dock");
             }
             else if(msg.data[0] == 0x02){
                 log_info("[power manager dock fail] stage:charge");
             }
             else if(msg.data[0] == 0x03){
                 log_info("[power manager dock fail] stage:leave dock");
             }

             if(msg.data[1] == 0x01){
                 log_info("[power manager dock fail] reason:pair code erro");
             }
             else if(msg.data[1] == 0x02){
                 log_info("[power manager dock fail] reason:bluetooth send erro");
             }
             else if(msg.data[1] == 0x03){
                 log_info("[power manager dock fail] reason:bluetooth link erro");
             }
             else if(msg.data[1] == 0x04){
                 log_info("[power manager dock fail] reason:BMS CAN link erro");
             }
             else if(msg.data[1] == 0x05){
                 log_info("[power manager dock fail] reason:charge motor can erro");
             }
             else if(msg.data[1] == 0x06){
                 log_info("[power manager dock fail] reason:battery inernal relay erro");
             }
             else if(msg.data[1] == 0x07){
                 log_info("[power manager dock fail] reason:power 48v erro");
             }

             else if(msg.data[1] == 0x08){
                 log_info("[power manager dock fail] reason:charge 48v erro");
             }
             else if(msg.data[1] == 0x09){
                 log_info("[power manager dock fail] reason:switch detect erro");
             }
             else if(msg.data[1] == 0x0A){
                 log_info("[power manager dock fail] reason:relay enable connect erro");
             }
             else if(msg.data[1] == 0x0B){
                 log_info("[power manager dock fail] reason:start charge connect erro");
             }
             else if(msg.data[1] == 0x0C){
                 log_info("[power manager dock fail] reason:charging electric connect erro");
             }
             else if(msg.data[1] == 0x0D){
                 log_info("[power manager dock fail] reason:BMS cmd stop charge");
             }
             break;
        default:
            break;
    }
}

int UdockTankManager:: dock_sdk(dockSDKCMD cmd)
{
    boost::lock_guard<boost::mutex> lock(sdk_state_lock);
    int res = 0;
    switch(cmd)
    {
        case CANCEL_DOCK:
            set_charge_state(IDLE);
            set_move_state(false);
            log_info("[udock]cancel dock success!");
            break;
        case BEGAIN_DOCK:
            if(get_charge_state() == IDLE){
                set_charge_state(RECEIVE_CMD);
                log_info("[udock]begain dock!");
            }
            else{
                log_info("[udock]dock busy!");
                res = -1;
            }
            break;
        case PAUSE_DOCK:
            {//lock
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                if(get_charge_state() == MOVE_POINT && is_moving()){ //
                    set_charge_state(PAUSE);
                    log_info("[udock]pause success!");
                }
                else{
                    res = -1;
                    log_info("[udock]pause fail!");
                }
            }
            break;
        case CANCEL_PAUSE:
            {//lock
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                if(get_charge_state() == PAUSE){ //
                    set_charge_state(MOVE_POINT);
                    log_info("[udock]cancel pause success!");
                }
                else{
                    res = -1;
                    log_info("[udock]cancel pause fail!");
                }
            }
            break;
        default:
            break;
    }
    return res;
}

int UdockTankManager::move_dist(const float dist, const int speed) //cm cm/s
{
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    
    int res = 0;
    float spd = speed/100.0f; //--m/s
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = dist/100.0f + odo_start;
    timespec time_begain;
    timespec time_now;

    ControlVariable in(spd, 0.0);
    ControlVariable out;
    clock_gettime(CLOCK_REALTIME, &time_begain);
    //time out caculate
    int time_s = (int)fabs((dist /speed)) +10;
    int time_max_out = 0;
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    
    if(dist > 0){
        while(1)
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
                vel_cmd.x = out.v_x;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);

                shm::iMemory_read_OdomInfo( &odom_info );
                odo_cur = odom_info.dist_center;
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    break;
                }
                if(!(odo_cur < (dest -0.006f))){
                    set_moving_state(false);
                    break;
                }
            }
            if(fabs(0 - out.v_x) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                sleep(1);
            }
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
    else{
        in.v_x = -in.v_x;
        while(1)
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                obstacle_avoid_under_given_control->determine_safe_control(in, out, true);

                vel_cmd.x = out.v_x;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);

                shm::iMemory_read_OdomInfo( &odom_info );
                odo_cur = odom_info.dist_center;
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    break;
                }
                if(!(odo_cur > (dest +0.006f))){
                    set_moving_state(false);
                    break;
                }
            }
            if(fabs(0 - out.v_x) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                sleep(1);
            }
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
            
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    
    shm::iMemory_read_OdomInfo( &odom_info );
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >3.0f){
        res = -1;
    }
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    return res;
}

int UdockTankManager::move_dist_ob(const float dist, const int speed) //cm cm/s
{
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    
    int res = 0;
    float spd = speed/100.0f; //--m/s
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = dist/100.0f + odo_start;
    timespec time_begain;
    timespec time_now;

    ControlVariable in(spd, 0.0);
    clock_gettime(CLOCK_REALTIME, &time_begain);
    //time out caculate
    int time_s = (int)fabs((dist /speed)) +10;
    int time_max_out = 0;
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    
    if(dist > 0){
        while(1)
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                vel_cmd.x = in.v_x;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);

                shm::iMemory_read_OdomInfo( &odom_info );
                odo_cur = odom_info.dist_center;
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    break;
                }
                if(!(odo_cur < (dest -0.006f))){
                    set_moving_state(false);
                    break;
                }
            }
            if(fabs(0 - in.v_x) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                sleep(1);
            }
            //----
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
    else{
        in.v_x = -in.v_x;
        while(1)
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                vel_cmd.x = in.v_x;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);

                shm::iMemory_read_OdomInfo( &odom_info );
                odo_cur = odom_info.dist_center;
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    break;
                }
                if(!(odo_cur > (dest +0.006f))){
                    set_moving_state(false);
                    break;
                }
            }

            if(fabs(0 - in.v_x) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                sleep(1);
            }
            //----
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
    
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    
    shm::iMemory_read_OdomInfo( &odom_info );
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >3.0f){
        res = -1;
    }
    
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    return res;
}

float UdockTankManager::get_odom_yaw(void)
{
    float yaw = 0.0f;
#if USE_GSUDP_DATA | USE_GSROSERIAL_DATA
    if(!gsudp->get_yaw(yaw)){
        log_warn("[udock]get yaw erro!");
    }
#else
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    yaw = odom_info.dist_theta;
#endif
    return yaw;
}

int UdockTankManager::move_angle(const float dist, const int speed)
{
    int res = 0;
    float spd = speed / 180.0f * M_PI; //--rad/s
    double odo_start = get_odom_yaw();
    double odo_cur = 0.0f;

    ControlVariable in(0.0, spd);
    ControlVariable out;

#if USE_GSUDP_DATA | USE_GSROSERIAL_DATA
    double dest = (dist/ 180.0f * M_PI) + odo_start; //--rad
#else
    double dest = (dist/ 180.0f * M_PI) *1.30f + odo_start;
#endif
    if(fabs(dist) <1.0f){ //1 degree
        return res;
    }
    odo_cur = get_odom_yaw();
    timespec time_begain;
    timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_begain);
    //time out caculate
    int time_s = (int)fabs((dist /speed)) +10;
    int time_max_out = 0;
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    
    if(dist > 0){
        while(1) // 1 degree
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                set_moving_state(true);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
                vel_cmd.x = 0.0;
                vel_cmd.z = out.v_theta;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);
                odo_cur = get_odom_yaw();
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    set_moving_state(false);
                    break;
                }
                if(!(odo_cur < (dest - 0.01745f))){
                    set_moving_state(false);
                    break;
                }
            }
            if(fabs(0 - out.v_theta) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    set_moving_state(false);
                    res = -1;
                    break;
                }
                sleep(1);
            }
            //-------------
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
    else{
        in.v_theta = - in.v_theta;
        while(1) // 1 degree
        {
            {
                boost::lock_guard<boost::mutex> lock_pause(pause_state_lock);
                set_moving_state(true);
                clock_gettime(CLOCK_REALTIME, &time_now);
                if(time_now.tv_sec - time_begain.tv_sec >time_s){
                    log_info("[udock] move time out!");
                    res = -1;
                    set_moving_state(false);
                    break;
                }
                obstacle_avoid_under_given_control->determine_safe_control(in, out, true);
                vel_cmd.x = 0.0;
                vel_cmd.z = out.v_theta;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);
                odo_cur = get_odom_yaw();
                if(!get_move_state()){
                    log_info("[udock] move cancel!");
                    res = -1;
                    set_moving_state(false);
                    break;
                }
                if(!(odo_cur > (dest + 0.01745f))){
                    set_moving_state(false);
                    break;
                }

            }
            if(fabs(0 - out.v_theta) <0.001f){
                time_s ++;
                time_max_out ++;
                if(time_max_out >300){
                    log_info("[udock] move time out %d!", time_max_out);
                    res = -1;
                    set_moving_state(false);
                    break;
                }
                sleep(1);
            }
            //-------------
            usleep(30*1000);
            if(get_charge_state() == PAUSE){
                while(1)
                {
                    if(get_charge_state() != PAUSE){
                        break;
                    }
                    sleep(1);
                    time_s ++;
                }
            }
        }
    }
    //-------------
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    odo_cur = get_odom_yaw();
    float erro = fabs((odo_cur - dest)*57.3f);
    if(erro >5.0f){
        log_info("[udock] erro move preceise!");
        res = -1;
    }
    log_info("[udock][erro angle degreee] = dest: %f, cur: %f, erro: %f (degree)", dest*57.3f, odo_cur*57.3f, (odo_cur - dest)*57.3f);
    return res;
}

bool UdockTankManager::data_transform(void)
{
    for(int j=0; j<10; j++){
#if USE_GSUDP_DATA | USE_GSROSERIAL_DATA
        if(gsudp->get_lidar_data(lidar_data[j])){
            usleep(200*1000);
        }
#else
        atris_msgs::GetGsLaserRawData laserdata;
        if (get_gs_laser_raw_data_srv_client_.call(laserdata)) {
            lidar_data[j].frame_id = laserdata.response.frame_id;
            lidar_data[j].stamp = laserdata.response.stamp;
            lidar_data[j].angle_min = laserdata.response.angle_min;
            lidar_data[j].angle_max = laserdata.response.angle_max;
            lidar_data[j].range_min = laserdata.response.range_min;
            lidar_data[j].range_max = laserdata.response.range_max;
            lidar_data[j].range_size = laserdata.response.range_size;
            lidar_data[j].intens_size = laserdata.response.intens_size;
            lidar_data[j].angle_increment = laserdata.response.angle_increment;
            for (uint32_t i = 0; i < 2048; i++) {
                lidar_data[j].laser_range[i] = laserdata.response.laser_range[i];
                lidar_data[j].intensities[i] = laserdata.response.intensities[i];
            }
            usleep(100*1000);
        }
#endif
        else{
            log_error("[udock]get lidar data fail!\r\n");
            return false;
        }
    }
    lidar_data_filter.angle_min = lidar_data[0].angle_min;
    lidar_data_filter.angle_max = lidar_data[0].angle_max;
    lidar_data_filter.angle_increment = lidar_data[0].angle_increment;
    lidar_data_filter.range_size = lidar_data[0].range_size;

    for(unsigned int j=0; j<lidar_data[0].range_size; j++){
        lidar_data_filter.laser_range[j] =0;
        for(int i=0; i<10; i++){
            lidar_data_filter.laser_range[j] += lidar_data[i].laser_range[j];
        }
        lidar_data_filter.laser_range[j] = lidar_data_filter.laser_range[j]/10;
    }
    // filter inf
    for(unsigned int j=1; j<lidar_data_filter.range_size-1; j++){
        if(std::isinf(lidar_data_filter.laser_range[j])){
            if(std::isfinite(lidar_data_filter.laser_range[j-1]) && std::isfinite(lidar_data_filter.laser_range[j+1])){
                lidar_data_filter.laser_range[j] = (lidar_data_filter.laser_range[j-1] + lidar_data_filter.laser_range[j+1])/2;
            }
        }
    }
    log_info("[udock]transform ok!\r\n");
    return true;
}

bool UdockTankManager::track_location(float d, int point_num, int showd_num)
{
    float dis_lidar_wall;

    if(!data_transform()){
        log_info("[udock]transform erro!\r\n");
        return false;
    }

    if(!udock->udock_caculate(&lidar_data_filter, point_num, showd_num)){
        return false;
    }
    dis_lidar_wall = udock->get_lidar_wall_distance();
    log_info("[udock][point2 dis_lidar_wall] = %f", dis_lidar_wall);

    udock->point_distance(d);
    udock->lines_angle();
    //degree
    track_move_para.angle1 = udock->get_angle();
    log_info("[udock][point2 move ang] = %f", track_move_para.angle1);
    //cm
    track_move_para.distance = udock->get_distance() * 100.0f;
    log_info("[udock][point2 move dis] = %f", track_move_para.distance);
    //degree
    track_move_para.angle2 = fabs(udock->get_line_angle());
    log_info("[udock][point2 move ang] = %f", track_move_para.angle2);
    return true;
}

// lv dai shi
bool UdockTankManager::move_to_dock_last_point(float d)
{
    float dis, angle, line_angle, last_angle;
    int try_count = 6;
    int try_width_verify = 4;
    while(1)
    {
        if(try_width_verify <=0){
            return false;
        }
        if(!data_transform()){
            try_width_verify--;
            continue;
        }
        if(!udock->udock_caculate(&lidar_data_filter, 80, 15)){
            try_width_verify--;
            continue;
        }
        udock->point_distance(d);
        udock->lines_angle();
        last_angle = udock->caculate_last_angle();
        angle = udock->get_angle();
        dis = udock->get_distance() *100.0f;
        line_angle = fabs(udock->get_line_angle());
        // try count out
        if(try_count <= 0){
            float std_dis = (POINT3_DISTANCE - POINT4_DISTANCE) *100.0f;
            if((dis < std_dis - 5.0f) || (dis > std_dis + 5.0f)){
                log_info("[udock][step3 erro dis ---------]");
                return false;
            }
            if(fabs(angle) >5.0f || (fabs(udock->get_xposition()) >0.03f)){
                log_info("[udock][precise erro]!");
                return false;
            }
            break;
        }

        if(fabs(last_angle) < 2.0f){ //parallel the charging station
            float std_dis = (POINT3_DISTANCE - POINT4_DISTANCE) *100.0f;
            if((dis < std_dis - 5.0f) || (dis > std_dis + 5.0f)){
                log_info("[udock][step3 erro dis ---------]");
                return false;
            }
            if(fabs(angle) >4.8f || (fabs(udock->get_xposition()) >0.03f)){
                log_info("[udock][precise erro]!");
                return false;
            }
            break;
        }
        else{
            if(move_angle(last_angle, DOCK_ANGULAR_SPEED) !=0){
                return false;
            }
            try_count--;
        }
    }
    //----------------------------------------------
    //degree
    log_info("[udock][last point move ang] = %f", angle);
    //cm
    log_info("[udock][last point move dis] = %f", dis);
    //degree
    log_info("[udock][last point move ang] = %f", line_angle);
    if(move_dist_ob(dis, DOCK_LINEAR_SPEED/4) != 0){
        return true;
    }
    return true;
}

bool UdockTankManager::move_to_dock_point(void)
{
    float angle = track_move_para.angle1;
    float dis = track_move_para.distance;
    float line_angle = track_move_para.angle2;

    if((-80.0f < angle) && (angle < 80.0f))
    {
        log_info("[udock][point2 begain move ang]");
        if(move_angle(angle, DOCK_ANGULAR_SPEED) !=0){
            return false;
        }

        log_info("[udock][point2 begain move dis]");
        if(move_dist(dis, DOCK_LINEAR_SPEED) != 0){
            return false;
        }

        if(udock->get_xposition() >0){
            log_info("[udock][point2 end move angle] = %f", line_angle);
            if(move_angle(line_angle, DOCK_ANGULAR_SPEED) !=0){
                return false;
            }
        }
        else{
            log_info("[udock][point2 end move angle] = %f", -line_angle);
            if(move_angle(-line_angle, DOCK_ANGULAR_SPEED) !=0){
                return false;
            }
        }
        return true;
    }
    return false;
}

#ifdef _CHASSIS_MARSHELL_

bool UdockTankManager::localizition(robotPose *local, int point_num, int showd_num)
{
    int try_count =0;
    float dis_lidar_wall;
    while(1)
    {
        try_count ++;
        if(try_count >6){
            break;
        }
        if(!data_transform()){
            continue;
        }
        if(!udock->udock_caculate(&lidar_data_filter, point_num, showd_num)){
            log_info("[udock]udock_caculate fail!\r\n");
            continue;
        }
        dis_lidar_wall = udock->get_lidar_wall_distance();
        log_info("[udock][w dis_lidar_wall] = %f", dis_lidar_wall);
        
        float point3_distance = POINT3_DISTANCE;
        if (Config::get_instance()->dock_1_0 == 1) point3_distance = POINT3_DISTANCE - DOCK_OFFSET;

        udock->point_distance(point3_distance);
        udock->lines_angle();

        local->theta = -udock->caculate_last_angle(); //degree
        local->lidar_to_wall = dis_lidar_wall;
        udock->get_pose(&local->x, &local->y);
        log_info("[udock]location x= %f, y= %f, theta= %f", local->x, local->y, local->theta);
        return true;
    }
    atris_msgs::AisoundTTS tts_msg;
    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_DOCK_FAIL);
    aisound_tts_pub_.publish(tts_msg);
    return false;
}

bool UdockTankManager::move_to_dock_point(int step)
{
    if(step == 1){
        if(move_control->goto_point2()){
            return true;
        }
    }
    else{
        log_info("[udock][erro step!!!]");
        return false;
    }
    return false;
}
#endif
bool UdockTankManager::send_test_cmd(void)
{
    bool res = false;
    atris_msgs::ChargeCmd msg;
    msg.cmd = PM_CMD_CHARGE_STATE;
    for(int i=0; i<7; i++){
        msg.data[i] = 0;
    }
    charge_cmd_pub_.publish(msg);
    log_warn("[ udock ]------%d!",__FUNCTION__);
    return res;
}

bool UdockTankManager::blue_tooth_pare(void)
{
    bool res = true;
    atris_msgs::ChargeCmd msg;
    msg.cmd = PM_CMD_BLUETOOTH;
    for(int i=0; i<7; i++){
        msg.data[i] = 0;
    }
    charge_cmd_pub_.publish(msg);
    log_warn("[ udock ]------begain bluetooth pair!");
    return res;
}

bool UdockTankManager::respons_leave_msg(void)
{
    bool res = true;
    atris_msgs::ChargeCmd msg;
    msg.cmd = PM_CMD_DOCK_LEAVE_PILE;
    for(int i=0; i<7; i++){
        msg.data[i] = 0;
    }
    charge_cmd_pub_.publish(msg);
    log_warn("[ udock ]------response leave msg!");
    return res;
}

bool UdockTankManager::notify_leave_msg(void)
{
    bool res = true;
    atris_msgs::ChargeCmd msg;
    msg.cmd = PM_CMD_PC_LEAVE_PILE;
    for(int i=0; i<7; i++){
        msg.data[i] = 0;
    }
    charge_cmd_pub_.publish(msg);
    log_warn("[ udock ]------notify leave msg!");
    return res;
}

bool UdockTankManager::move_to_dock(void) //直接去充电
{
    // first step
#ifndef _CHASSIS_MARSHELL_
    bool res = false;
    int try_count = 6;
    while(1)
    {
        try_count--;
        if(try_count <=0){
            log_info("[udock]------step 1 try end erro!");
            return res;
        }
        if(!get_move_state()){
            log_info("[udock]------step 1 cancel erro!");
            return res;
        }

        if(!track_location(POINT2_DISTANCE, 38, 5)){
            sleep(3);
            continue;
        }
        if(!move_to_dock_point()){
            log_info("[udock]------step 1 move erro!");
            return res;
        }
        break;
    }
#else
    if(!move_to_dock_point(1)){
        log_info("[udock]------step 1 move erro!");
        return false;
    }
#endif
    log_info("[udock]------dock first step suceess!");


#ifndef _CHASSIS_MARSHELL_
    try_count = 6;
    // second step
    while(1)
    {
        try_count--;
        if(try_count <=0){
            log_info("[udock]------step 2 try end erro!");
            return res;
        }
        if(!get_move_state()){
            log_info("[udock]------step 2 cancel erro!");
            return res;
        }

        if(!track_location(POINT3_DISTANCE, 60, 10)){
            sleep(3);
            continue;
        }
        if(!move_to_dock_point()){
            log_info("[udock]------step 2 move erro!");
            return res;
        }
        break;
    }
#endif

    log_info("[udock]------dock second step suceess!");
    return true;

}

bool UdockTankManager::move_to_dock_last_step(void) //直接去充电
{
    bool res = false;
    // last step
#ifndef _CHASSIS_MARSHELL_
    if(!move_to_dock_last_point(POINT4_DISTANCE)){
        return res;
    }
#else
    move_control->goto_last_point();

#endif
    log_info("[udock]------dock third step suceess!");
    
    return true;
}


bool UdockTankManager::move_away_from_dock(const float dist, const int speed, const int time_s)
{
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    
    int res = 0;
    uint16_t ultro7, ultro8;

    float spd = speed/100.0f;
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = dist/100.0f + odo_start;
    timespec time_begain;
    timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_begain);
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    
    if(dist < 0){
        while(odo_cur > dest)
        {
            shm::UltraSound ultroso;
            memset(ultroso.data, 0, sizeof(ultroso.data));
            shm::iMemory_read_UltraSound(&ultroso);
            
            ultro7 = ultroso.data[6];
            ultro8 = ultroso.data[7];
            clock_gettime(CLOCK_REALTIME, &time_now);
            if(time_now.tv_sec - time_begain.tv_sec >time_s){
                return -1;
            }
            //add ultrasonic avoid
            if(ultro7 ==0xFFFF || ultro8 ==0xFFFF){
                log_info("[udock] valid ultrasonic data!");
                vel_cmd.x = 0.0;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);
                usleep(500*1000);
            }
            else if(ultro7 <300 || ultro8 <300){
                log_info("[udock] ultrasonic [%d] [%d]!", ultro7, ultro8);
                vel_cmd.x = 0.0;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);
                usleep(500*1000);
            }
            else if(ultro7 >300 && ultro8 >300){
                vel_cmd.x = -spd;
                vel_cmd.z = 0.0;
                vel_cmd.priority = *((int32_t*)&owner);
                vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                set_vel_cmd_pub_.publish(vel_cmd);
            }
            shm::iMemory_read_OdomInfo( &odom_info );
            odo_cur = odom_info.dist_center;
            if(!get_move_state()){
                res = -1;
                break;
            }
            usleep(30*1000);
        }
    }
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    shm::iMemory_read_OdomInfo( &odom_info );
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >60.0f){
        res = -1;
    }
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    //
    return res;
}

#ifdef _CHASSIS_MARSHELL_

bool UdockTankManager::move_away_from_dock_wheel(const float dist, const int speed)
{
    return move_control->leave_from_dock(dist, speed);
}

#endif

/*
    END file Udock manager by xiangbin.huang
*/
