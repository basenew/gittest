#include "chassis_manager.h"
#include <unistd.h>
#include "utils/utils.h"
#include "config/config.h"
#include "log/log.h"


ChassisManager::ChassisManager()
  : robot_cmd_vel_pub_(TOPIC_CMD_VEL, new tinyros::geometry_msgs::Twist()) {
  log_info("%s", __FUNCTION__);
  tinyros::nh()->advertise(robot_cmd_vel_pub_);
  signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &ChassisManager::on_recv_chassis_ctrl, this);
  time_calibrated_sub_ = nh_.subscribe(TOPIC_TIME_CALIBRATED_MESSAGE, 100, &ChassisManager::on_recv_time_calibrated, this);
  set_vel_to_chassis_sub_ = nh_.subscribe(TOPIC_SET_VEL_TO_CHASSIS, 100, &ChassisManager::set_vel_cmd, this);
  set_vel_to_udock_pub_ = nh_.advertise<atris_msgs::VelCmd>(TOPIC_SET_VEL_TO_UDOCK, 100);
  get_chassis_info_srv_ = nh_.advertiseService(SRV_GET_CHASSIS_INFO, &ChassisManager::doGetChassisInfo, this);
  
  shm::iMemory_read_TimeCalibrated(&tc_);
}

void ChassisManager::init()
{
    if(control_module ==NULL){
        log_info("[control module] ------------ControlModule init success!------------------");
        control_module = new ControlModule();
    }
    chassis_manager_proc_exit = false;
}

 ChassisManager::~ChassisManager()
{
    chassis_manager_proc_exit = true;
}

//todo
bool ChassisManager::doGetChassisInfo(atris_msgs::GetChassisInfo::Request& req,
    atris_msgs::GetChassisInfo::Response& res) {
    return true;
}

void ChassisManager::set_vel_cmd(const atris_msgs::VelCmd &msg)
{
    float x = msg.x, y = msg.y, z = msg.z;
    // limit speed
    speed_limit(x, y, z);

    tinyros::geometry_msgs::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = z;

    int64_t timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    Control_Owner owner = *((Control_Owner*)&msg.priority);

    if ((timestamp < msg.timestamp) || (timestamp - msg.timestamp) > 50) {
      log_once_warn("%s timeout: now_timestamp: %lld, msg_timestamp: %lld", __FUNCTION__, timestamp, msg.timestamp);
      return;
    }

    if(control_module->control_change(owner)){
        robot_cmd_vel_pub_.publish(&twist);
        if (owner == SHTTPD) {
          log_once_info("%s SHTTPD request_joystick_move vel.linear.x:%.2f,vel.linear.y:%.2f, vel.angular.z:%.2f", __FUNCTION__, twist.linear.x, twist.linear.y, twist.angular.z);
        } else if(owner == REMOTE) {
          log_once_info("%s REMOTE request_remote_move vel.linear.x:%.2f,vel.linear.y:%.2f, vel.angular.z:%.2f", __FUNCTION__, twist.linear.x, twist.linear.y, twist.angular.z);
        } else if(owner == PC_CTRL) {
          log_once_info("%s PC_CTRL request_remote_move vel.linear.x:%.2f,vel.linear.y:%.2f, vel.angular.z:%.2f", __FUNCTION__, twist.linear.x, twist.linear.y, twist.angular.z);
        } else if(owner == GS) {
          log_once_info("%s GS vel.linear.x:%.2f,vel.linear.y:%.2f, vel.angular.z:%.2f", __FUNCTION__,  twist.linear.x, twist.linear.y, twist.angular.z);
        } else if(owner == WEB_CTRL){
        }
    }
}

void ChassisManager::on_recv_time_calibrated(const atris_msgs::TimeCalibrated &msg) {
    tc_.calibrated = msg.calibrated;
    tc_.interval = msg.interval;
}

void ChassisManager::on_recv_chassis_ctrl(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value root, response;

    if (msg.title == "request_joystick_move" || msg.title == "request_robot_move") {
        reader.parse(msg.msg, root);
        if (!root["content"]["v_linear"].isNull() && !root["content"]["v_angular"].isNull()) {
            Control_Owner owner = PC_CTRL;
            atris_msgs::VelCmd vel;
            vel.x = root["content"]["v_linear"].asFloat();
            vel.z = root["content"]["v_angular"].asFloat();
            vel.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            if (msg.title == "request_joystick_move") {
                owner = SHTTPD;
            }
            vel.priority = *((int32_t*)&owner);
            set_vel_cmd(vel);
        }
    }else if (msg.title == "request_robot_move_control") {
      reader.parse(msg.msg, root);
      if (!root["content"]["operation"].isNull()) {
        int op = root["content"]["operation"].asInt();
        shm::Robot shmrbt;
        iMemory_read_Robot(&shmrbt);
        double speed = shmrbt.appdata.chassis_speed;
        Control_Owner owner = WEB_CTRL;
        atris_msgs::VelCmd vel;
        vel.y = 0;
        vel.priority = *((int32_t*)&owner);
        vel.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
        switch(op) { // 前进(w)/后退(s)/左转(a)/右转(d)
          case 0: vel.x = speed;       vel.z = 0;           break;
          case 1: vel.x = -1 * speed;  vel.z = 0;           break;
          case 2: vel.x = 0;           vel.z = speed;       break;
          case 3: vel.x = 0;           vel.z = -1 * speed;  break;
          default: break;
        }
        set_vel_cmd(vel);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        vel.x = 0; vel.y = 0; vel.z = 0;
        vel.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
        set_vel_cmd(vel);
        set_vel_cmd(vel);
      }
    }
}

void ChassisManager::speed_limit(float &x, float &y, float &z)
{
    if (fabs(x) > Config::get_instance()->forward_max) {
      if (x > 0) {
          x = Config::get_instance()->forward_max;
      }

      if (x < 0) {
        x = -1 * Config::get_instance()->forward_max;
      }
    }

    if(fabs(y) > Config::get_instance()->forward_max) {
      if (y > 0) {
        y = Config::get_instance()->forward_max;
      }

      if (y < 0) {
        y = -1 * Config::get_instance()->forward_max;
      }
    }

    if (fabs(z) > Config::get_instance()->angular_max) {
      if (z > 0) {
        z = Config::get_instance()->angular_max;
      }

      if (z < 0) {
        z = -1 * Config::get_instance()->angular_max;
      }
    }
}