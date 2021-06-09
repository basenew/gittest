#ifndef _UDOCK_
#define _UDOCK_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include "udock_msgs/MoveToAction.h"

#include "u_dock/dock_recognizer.h"
#include <mutex>
#include <thread>
#include <deque>

#include "unistd.h"
#include "ros_common.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <json_1_8_0/json.h>

namespace dock{

    #define RAD2DEGREE  (180.0/M_PI)
    #define DEGREE2DRAD  (M_PI/180.0)

    typedef struct{
        double angle1;
        double dist;
        double angle2;
    }movePara_t;

    struct Vect
    {
        double x;
        double y;
    };

    enum
    {
        IDLE,
        RECEIVE_CMD,
        FIND_SATION,
        LOCALIZITION,
        PLANNING,
        MOVE_TO_POINT,
        TURN_BACK,
        DOCKING,
        CAHRGING,
        SUCCESS,
    };

    enum{
        ON_TARGET,
        X_ERROR,
        Y_ERROR,
        THETA_ERROR,
    };

    typedef actionlib::SimpleActionClient<u_msgs::MoveToAction> Client;

    class udockMagnager{
        
        public:
            udockMagnager(ros::NodeHandle &node):udock_move_state_(MOVE_IDLE),udock_state_(IDLE),charge_state_(0)
            {
                node_ = &node;
                move_point_pose_.x = 0.25;
                move_point_pose_.y = 0;
                move_point_pose_.theta =  -1.919;
                move_last_point_pose_.x = 0.25;
                move_last_point_pose_.y = 0;
                move_last_point_pose_.theta = 0;

                charge_state_pub_ = node_->advertise<std_msgs::Int32>("/dock_state", 10);
                pub_notify_msg_ = node_->advertise<std_msgs::String>(TOPIC_NOTICES, 10);

                scan_sub_ = node_->subscribe("/scan_bottom", 10, &udockMagnager::laser_scan_callback, this);
                charge_info_sub_ = node_->subscribe(TOPIC_CHARGE_SRC_FROM_DEV, 10, &udockMagnager::charge_info_callback, this);
                charge_cmd_sub_ = node_->subscribe(TOPIC_CHASSIS_CTRL, 5, &udockMagnager::command_callback, this);
                
                move_to_client_ = new Client("/move_to_action", true);
                ROS_INFO("WAITING FOR ACTION SERVER TO START!");
                move_to_client_->waitForServer();
                ROS_INFO("ACTION SERVER START !");

            }
            ~udockMagnager(){
                delete move_to_client_;
            }
            void cmd_receive_thread_run(void);
            void dock_thread_run(void);

        private:
            bool try_localizition(void);
            void average_filter(void);
            bool move_to_point(void);
            bool trun_back(void);
            bool move_to(const u_msgs::MoveToGoal &goal);
            bool planning_path(void);
            double planning_angle(void);
            void laser_scan_callback(const sensor_msgs::LaserScanConstPtr &msg_ptr);
            void charge_info_callback(const u_msgs::ChargeSource &msg);
            double v1_trans_v2_angle(Vect v1, Vect v2);
            void theta_trans_vect(const Pose &p, Vect &v);
            int is_on_target(void);
            bool find_charge_station(void);
            void publish_dock_state(int state);

            bool wait_for_charge_msg(void);
            //dock state
            void set_dock_state(const int state);
            int get_dock_state(void);
            //action
            void done_callback(const actionlib::SimpleClientGoalState &state, const u_msgs::MoveToResultConstPtr &result);
            void active_call_back();
            void feedback_callback(const u_msgs::MoveToFeedbackConstPtr &feedback);
            //jason interface
            bool check_command_title(const std::string &title);
            void command_callback(const std_msgs::String &msgs);
            void feedback_to_commander(int sessionid, int status, std::string msg);

        private:
            ros::NodeHandle *node_;
            ros::Subscriber scan_sub_;
            ros::Subscriber charge_info_sub_;
            ros::Subscriber charge_cmd_sub_;
            ros::Publisher pub_notify_msg_;
            ros::Publisher charge_state_pub_;
            volatile uint32_t udock_state_;
            volatile uint32_t udock_move_state_;
            volatile uint32_t charge_state_;
            
            movePara_t move_para_;
            DockRecognizer dock_recongnizer_;
            Pose robot_current_pose_;
            Pose move_point_pose_;
            Pose move_last_point_pose_;
            sensor_msgs::LaserScanConstPtr scan_;
            std::deque<sensor_msgs::LaserScanConstPtr> scan_ptr_deque_;
            Client *move_to_client_;
            sensor_msgs::LaserScan laser_scan_;
            std::mutex udock_state_mutex_;
            std::mutex scan_data_mutex_;
            std::mutex feedback_mutex_;

            int try_count_;
            int64_t session_id_;
    };

}







#endif