/*

*/
#ifndef _CHASSIS_MOVE_INTERFACE_H_
#define _CHASSIS_MOVE_INTERFACE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/client/simple_action_client.h>
#include "u_msgs/MoveToAction.h"

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>  

#include "u_msgs/u_common.h"
#include "u_msgs/u_chassis_common.h"
#include "u_msgs/Diagnostics.h"
#include "u_msgs/Geomagnetism.h"
#include "u_msgs/u_diagnostics_info.h"
#include "ros_common.h"

#include <json_1_8_0/json.h>
#include <boost/bind.hpp>

#include <string>
#include <vector>
#include <thread>
#include <mutex>

typedef actionlib::SimpleActionClient<u_msgs::MoveToAction> Client;

class ChassisMoveInterface{

  public:
    ChassisMoveInterface(ros::NodeHandle *node):move_state_(MOVE_IDLE),chassis_emstop_state_(false), magn_trigger_state_(false){
        nodehandle_ = node;
        pub_notify_msg_ = nodehandle_->advertise<std_msgs::String>(TOPIC_NOTICES, 10);
        command_sub_ = nodehandle_->subscribe(TOPIC_CHASSIS_CTRL, 5, &ChassisMoveInterface::command_callback, this);
        sub_em_stop_ = nodehandle_->subscribe(TOPIC_DIAGNOSTICS, 10, &ChassisMoveInterface::emergency_callback, this);
        sub_geomagn_ = nodehandle_->subscribe(TOPIC_Geomagnetism, 10, &ChassisMoveInterface::geomagn_callback, this);
        move_to_client_ = new Client("/move_to_action", true);
        ROS_INFO("WAITING FOR ACTION SERVER TO START!");
        move_to_client_->waitForServer();
        ROS_INFO("ACTION SERVER START !");
    }
    ~ChassisMoveInterface()
    {
      delete move_to_client_;
    }

    void thread_run(void);

  private:
    bool move_to(const u_msgs::MoveToGoal &goal);
    void feedback_to_commander(int sessionid, int status, std::string msg);
    void feedback_to_commander_state(int sessionid, int status);
    void feedback_odom(int sessionid, float x, float y, float theta);
    void command_callback(const std_msgs::String &msgs);

    void active_call_back();
    void done_callback(const actionlib::SimpleClientGoalState& state, const u_msgs::MoveToResultConstPtr& result);
    void feedback_callback(const u_msgs::MoveToFeedbackConstPtr& feedback);

    void emergency_callback(const u_msgs::Diagnostics& msgIn);
    void geomagn_callback(const u_msgs::Geomagnetism &msgIn);
  private:
    ros::Subscriber command_sub_;
    ros::Publisher  pub_notify_msg_;

    ros::Subscriber sub_em_stop_;
    ros::Subscriber sub_geomagn_;

    int64_t sessionid_;
    int64_t sessionid_old_;
    int move_state_;
    ros::NodeHandle *nodehandle_;
    Client *move_to_client_;
    std::mutex move_state_mutex_;

    bool chassis_emstop_state_;
    bool magn_trigger_state_;
};

#endif
/*-----------------------------------------------------------------------------------------------------------*/

