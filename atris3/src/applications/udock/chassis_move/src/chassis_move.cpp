/*

*/
#include <ros/ros.h>
#include <ros/console.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>  
#include <tf/tf.h> 

#include "ros_common.h"
#include "u_msgs/u_common.h"
#include "u_msgs/u_chassis_common.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <string>
#include <vector>

#include "trapezoid.h"
#include "collision_detector.h"
#include <actionlib/server/simple_action_server.h>
#include "u_msgs/MoveToAction.h"

typedef actionlib::SimpleActionServer<u_msgs::MoveToAction> Server;

#define CONTROL_TIME_DELTA    0.1
#define MIN_LINEAR_VELOCITY   0.05  //--m/s
#define MAX_LINEAR_VELOCITY   0.8   //--m/s
#define LINEAR_ACCELERATOR    0.5   //--m/s^2

#define MIN_ANGULAR_VELOCITY  0.05   //--rad/s
#define MAX_ANGULAR_VELOCITY  3.14  //--rad/s
#define ANGULAR_ACCELERATOR   1.57  //--rad/s^2

#define     PUB_CMD_VEL       "/cmd_vel_chassis"

Rrapezoid linear_planning(LINEAR_ACCELERATOR, MIN_LINEAR_VELOCITY);
Rrapezoid angular_planning(ANGULAR_ACCELERATOR, MIN_ANGULAR_VELOCITY);

using namespace collision_detection;

enum
{
  MOVE_STOP,
  MOVE_ANGLE,
  MOVE_DISTANCE
};

enum
{
  R_FINISH,
  R_OBSTACLE,
  R_EM_STOP,
  R_CANCEL,
};

struct MovingState{
  bool    running;
  double  time;
  double  min_speed;
  int     move_type;
  double  move_end_value;
  double  move_time_out;
  double  odom_distance;
  double  odom_angle;
  double  odom_vx;
  double  odom_vz;
};

struct MovingState moving_state_={false, 0, 0, MOVE_STOP, 0, 0, 0, 0, 0};

ros::Subscriber sub_rel_odom_;
ros::Subscriber sub_lidar_scan_;

ros::Publisher  pub_twist_;
boost::thread   *run_thread_ptr_;

boost::mutex  lock_mutex_;
int64_t time_tick_ = 0;
int sessionid_;
sensor_msgs::LaserScanPtr scan_data_;
int64_t scan_time_tick_ = 0;
/*-----------------------------------------------------------------------------------------------------------*/
void move_stop(void);
bool move_angle_with_velocity(const double speed_now, double speed, const double angle_now, const double angle);
bool move_distance_with_velocity(const double speed_now, double speed, const double distance_now, const double distance);

void odom_rel_callback(const nav_msgs::Odometry &odom_msg)
{
  static double yaw_old = 0;
  tf::Quaternion q;
  
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double delta_yaw = yaw - yaw_old;
  yaw_old = yaw;
  if (delta_yaw > M_PI)
  {
    delta_yaw -= 2 * M_PI;
  }
  else if(delta_yaw < -M_PI){
    delta_yaw += 2 * M_PI;
  }

  moving_state_.odom_distance = odom_msg.pose.pose.position.x;
  moving_state_.odom_angle += delta_yaw;
  moving_state_.odom_vx = odom_msg.twist.twist.linear.x;
  moving_state_.odom_vz = odom_msg.twist.twist.angular.z;
}

void lidar_scan_callback(const sensor_msgs::LaserScanPtr& scan_ptr)
{
  scan_data_ = scan_ptr;
  scan_time_tick_ = time_tick_;
}

/*-----stop----------*/
void move_stop(void)
{
  lock_mutex_.lock();
  moving_state_.running = false;
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  pub_twist_.publish(msg);
  lock_mutex_.unlock();
}

/*-----角度旋转运动----------*/
bool move_angle_with_velocity(const double speed_now, double speed, const double angle_now, const double angle)
{
  
  // if (fabs(angle) <= 10 / DEGREE_TO_RAD)
  // {
  //   return false;
  // }

  if(speed > MAX_ANGULAR_VELOCITY){
    speed = MAX_ANGULAR_VELOCITY;
  }
  else if(speed < -MAX_ANGULAR_VELOCITY){
    speed = -MAX_ANGULAR_VELOCITY;
  }

  if(angular_planning.planning(speed_now, speed, angle)){
    lock_mutex_.lock();

    moving_state_.running = true;
    moving_state_.time = 0;
    moving_state_.move_type = MOVE_ANGLE;
    // try
    // {
    //  ROS_INFO_STREAM("size" << angular_planning.m_x.size()); /* code */
    //  ROS_INFO_STREAM("m_x" << angular_planning.m_x.back()); /* code */

    // }
    // catch(const std::exception& e)
    // {
    //   std::cerr << e.what() << '\n';
    // }
    
    moving_state_.move_time_out = angular_planning.m_x.back() * 2;
    double speed_end = 0;
    angular_planning(angular_planning.m_x.back() - CONTROL_TIME_DELTA, speed_end);
    if(speed_end > 0)
    {
      moving_state_.min_speed = MIN_ANGULAR_VELOCITY;
    }
    else{
      moving_state_.min_speed = -MIN_ANGULAR_VELOCITY;
    }
    moving_state_.move_end_value = angle + angle_now;
    ROS_INFO_STREAM(" [start angle]" << angle_now << " [end angle]" << moving_state_.move_end_value );

    lock_mutex_.unlock();
    return true;
  }
  return false;
}

/*-----距离直线运动----------*/
bool move_distance_with_velocity(const double speed_now, double speed, const double distance_now, const double distance)
{
  // if(fabs(distance) <= 0.1){
  //   return false;
  // }

  if(speed > MAX_LINEAR_VELOCITY){
    speed = MAX_LINEAR_VELOCITY;
  }
  else if(speed < -MAX_LINEAR_VELOCITY){
    speed = -MAX_LINEAR_VELOCITY;
  }

  if(linear_planning.planning(speed_now, speed, distance)){
    lock_mutex_.lock();

    moving_state_.running = true;
    moving_state_.time = 0;
    moving_state_.move_type = MOVE_DISTANCE;
    moving_state_.move_time_out = linear_planning.m_x.back() + 20; //20秒超时时间
    double speed_end = 0;
    linear_planning(linear_planning.m_x.back() - CONTROL_TIME_DELTA, speed_end);
    if(speed_end > 0)
    {
      moving_state_.min_speed = MIN_LINEAR_VELOCITY;
    }
    else{
      moving_state_.min_speed = -MIN_LINEAR_VELOCITY;
    }
    moving_state_.move_end_value = distance_now + distance;
    ROS_INFO_STREAM(" [start distance]" << distance_now << " [end distance]" << moving_state_.move_end_value );

    lock_mutex_.unlock();
    return true;
  }
  return false;
}

void stop_and_feedback(void)
{
  geometry_msgs::Twist msg;
  moving_state_.running = false;
  msg.linear.x = 0;
  msg.angular.z = 0;
  pub_twist_.publish(msg);
}

void stop_and_feedback(int status, std::string msg_feedback)
{
  geometry_msgs::Twist msg;
  moving_state_.running = false;
  msg.linear.x = 0;
  msg.angular.z = 0;
  pub_twist_.publish(msg);
}

void run(CollisionDetector *collision_detector_ptr)
{
  sleep(1);
  ROS_INFO("thread start!");
  ROS_INFO("------------------------1");


  while (1)
  {
    usleep(1000*100); time_tick_ += 100; //ms
    if(!moving_state_.running){
      continue;
    }

    moving_state_.time += CONTROL_TIME_DELTA;
    lock_mutex_.lock();
    double speed;
    geometry_msgs::Twist msg, msg_out;
    if(moving_state_.move_type == MOVE_ANGLE){ 
      switch(angular_planning.m_x.size()){
        case 2: //one line
          angular_planning(moving_state_.time, speed);
          if(moving_state_.time > angular_planning.m_x[0]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = 0;
          msg.angular.z = speed;
          pub_twist_.publish(msg);
          break;
        case 3: //two line
          angular_planning(moving_state_.time, speed);
          if(moving_state_.time > angular_planning.m_x[1]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = 0;
          msg.angular.z = speed;
          pub_twist_.publish(msg);
          break;
        case 4: //three line
          angular_planning(moving_state_.time, speed);
          if(moving_state_.time > angular_planning.m_x[2]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = 0;
          msg.angular.z = speed;
          pub_twist_.publish(msg);
          break;
        default:
          moving_state_.running = false;
          ROS_ERROR_STREAM("erro move angle!");
          break;
        }
        //arrive destination check
        ROS_INFO_STREAM(" [check angle]" << moving_state_.odom_angle);
        if(moving_state_.min_speed >0){
          if(moving_state_.odom_angle >= moving_state_.move_end_value){ 
            ROS_INFO_STREAM("arrive destination stop!");  
            stop_and_feedback();
          }
        }
        else{
          if(moving_state_.odom_angle <= moving_state_.move_end_value){
            ROS_INFO_STREAM("arrive destination stop!");  
            stop_and_feedback();
          }
        }

    }
    else if(moving_state_.move_type == MOVE_DISTANCE){
      switch(linear_planning.m_x.size()){
        case 2: //one line
          linear_planning(moving_state_.time, speed);
          if(moving_state_.time > linear_planning.m_x[0]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = speed;
          msg.angular.z = 0;
          collision_detector_ptr->collision_avoid(msg, msg_out);
          pub_twist_.publish(msg_out);
          break;
        case 3: //two line
          linear_planning(moving_state_.time, speed);
          if(moving_state_.time > linear_planning.m_x[1]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = speed;
          msg.angular.z = 0;
          collision_detector_ptr->collision_avoid(msg, msg_out);
          pub_twist_.publish(msg_out);
          break;
        case 4: //three line
          linear_planning(moving_state_.time, speed);
          if(moving_state_.time > linear_planning.m_x[2]){
            if(fabs(speed) < fabs(moving_state_.min_speed)){
              speed = moving_state_.min_speed;
            }
          }
          msg.linear.x = speed;
          msg.angular.z = 0;
          collision_detector_ptr->collision_avoid(msg, msg_out);
          pub_twist_.publish(msg_out);
          break;
        default:
          moving_state_.running = false;
          ROS_ERROR_STREAM("erro move angle!");
          break;
        }
      
      if(time_tick_ - scan_time_tick_ > 1000){ //ms
        ROS_INFO_STREAM("scan time out stop!");  
        stop_and_feedback(E_ACTION_FAILED, "abnormal scan data!");
        lock_mutex_.unlock();
        continue;
      }
      //arrive destination check
      ROS_INFO_STREAM(" [check distance]" << moving_state_.odom_distance);
      if(moving_state_.min_speed >0){
        if(moving_state_.odom_distance >= moving_state_.move_end_value){ 
          ROS_INFO_STREAM("arrive destination stop!");  
          stop_and_feedback();
        }
      }
      else{
        if(moving_state_.odom_distance <= moving_state_.move_end_value){
          ROS_INFO_STREAM("arrive destination stop!");
          stop_and_feedback();
        }
      }
    }
    //time out check
    if(moving_state_.time >= moving_state_.move_time_out){ //time out
      ROS_INFO_STREAM("time out stop!");
      stop_and_feedback();
    }

    lock_mutex_.unlock();
  }
}

void execute(const u_msgs::MoveToGoalConstPtr& goal, Server* as)
{
  ros::Rate rate_hz(3);
  u_msgs::MoveToFeedback feedback;
  u_msgs::MoveToResult result;
  bool success = true;
  bool preempted = false;

  feedback.complete_percent = 0;
  ROS_INFO_STREAM("move to goal: goal received ");
  if(goal->type == 1 ) ROS_INFO_STREAM("move linear: " << goal->distance);
  if(goal->type == 2 ) ROS_INFO_STREAM("move angular: " << goal->distance);  
  if(goal->type == 0 ) ROS_INFO_STREAM("stop " << goal->distance);

  switch (goal->type)
  {
    case 1: // 直线运动
    {
      double odom_start = moving_state_.odom_distance;
      if (move_distance_with_velocity(moving_state_.odom_vx, goal->velocity, moving_state_.odom_distance, goal->distance))
      {
        while(moving_state_.running)
        {
          rate_hz.sleep();
          if (as->isPreemptRequested() || !ros::ok())
          {
              ROS_INFO_STREAM("move to goal: Preempted");
              move_stop();
              as->setPreempted();
              preempted = true;
              success = false;
              break;
          }
          feedback.complete_percent = fabs((moving_state_.odom_distance - odom_start) / goal->distance) *100.;
          as->publishFeedback(feedback);
        }
        if(feedback.complete_percent < 98)
        {
          success = false;
        }
      }
      else
      {
        ROS_INFO_STREAM("move to goal: planning fail!");
        success = false;
      }
      break;
    }

    case 2: // 旋转
    {
      double odom_start = moving_state_.odom_angle;
      if (move_angle_with_velocity(moving_state_.odom_vz, goal->velocity, moving_state_.odom_angle, goal->distance))
      {
        while (moving_state_.running)
        {
          rate_hz.sleep();
          if (as->isPreemptRequested() || !ros::ok())
          {
            ROS_INFO_STREAM("move to goal: Preempted");
            as->setPreempted();
            move_stop();
            preempted = true;
            success = false;
            break;
          }
          feedback.complete_percent = fabs((moving_state_.odom_angle - odom_start) / goal->distance) *100.;
          as->publishFeedback(feedback);
        }
        if(feedback.complete_percent < 98)
        {
          success = false;
        }
      }
      else
      {
        ROS_INFO_STREAM("move to goal: planning fail!");
        success = false;
      }
      break;
    }

    case 0: // 停止
    {
      move_stop();
      break;
    }

    default:
      break;
  }

  if(success)
  {
    result.finish = true;
    result.fail_reason = "";
    ROS_INFO("move to goal: Succeeded");
    as->setSucceeded(result);
  }
  else{
    if(preempted == true)
    {
      result.finish = false;
      result.fail_reason = FAIL_CANCEL;
      ROS_INFO("move to goal: cancel");
      as->setAborted(result);
    }
    else{
      result.finish = false;
      result.fail_reason = FAIL_REASON_OBSTACLE;
      ROS_INFO("move to goal: obstacle");
      as->setAborted(result);
    }
  }
  //
}

//---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "chassis_move_to");
  ros::NodeHandle nh;
  ROS_INFO("chassis_move_to start!");

  pub_twist_ = nh.advertise<geometry_msgs::Twist>(PUB_CMD_VEL, 10);

  sub_rel_odom_ = nh.subscribe("/chassis_rel_pos", 50, &odom_rel_callback);
  sub_lidar_scan_ = nh.subscribe("/scan_bottom", 10, &lidar_scan_callback);

  collision_detection::CollisionDetector collision_detector(nh);
  run_thread_ptr_ = new boost::thread(&run, &collision_detector);

  Server server(nh, "/move_to_action", boost::bind(&execute, _1, &server), false);
  server.start();
  ROS_INFO("spin-------------------------------------");
  ros::MultiThreadedSpinner s(10);
  ros::spin(s);;

  if(run_thread_ptr_) delete run_thread_ptr_;

  ROS_INFO("exit");
  return 1;
}
