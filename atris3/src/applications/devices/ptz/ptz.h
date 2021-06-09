#ifndef __PTZ_H__
#define __PTZ_H__

#include <stdexcept>
#include <termios.h>
#include "config/config.h"
#include "utils/utils.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetPtzStatus.h"
#include "atris_msgs/PtzControl.h"

//#define PTZ_KEYBOARD_CONTROL

enum PtzRotateCaptureState
{
    ROTATE_CAP_IDLE = 0,
    ROTATE_CAP_RUNNING = 1
};


#define PTZ_POINT_DEFAULT   1

class Ptz
{
    private:
        enum PtzRotateStatus{
            PTZ_ROTATE_IDLE = 0,
            PTZ_ROTATE_PREPARE,
            PTZ_ROTATE_RUNING,
            PTZ_ROTATE_PAUSED,
            PTZ_ROTATE_STOP
        };
        bool ptz_rotate_pause_flag;
        Ptz();
        int interval;
        int vel_angle, hor_angle;
        bool is_rotate;
        enum PtzRotateStatus ptz_rotate_state_;
        bool is_rotate_async;
        int default_point;
        void ptz_login();
        bool ptz_state;
        bool is_login;
        void ptz_keyboard_input();
        bool ptz_move_control(int direction);
        bool ptz_up();
        bool ptz_down();
        bool ptz_left();
        bool ptz_right();
        bool ptz_up_ex(int angle);
        bool ptz_down_ex(int angle);
        bool ptz_left_ex(int angle);
        bool ptz_right_ex(int angle);
        bool ptz_zoom_control(int op);
        bool ptz_zoom_add();
        bool ptz_zoom_dec();
        bool ptz_focus_control(int op);
        bool ptz_focus_add();
        bool ptz_focus_dec();
        bool ptz_wiper_control(int on_off);
        bool wait_for_ptz_idle(int timeout);
        bool ptz_capture(const atris_msgs::SignalMessage &msg);
        bool setPtzRegisterInfo(std::string serverip, int port, std::string registerId, bool enable);
        bool getPtzRegisterInfo(std::string &serverip, int &port, std::string &registerId, bool &enable);

        boost::thread* capture_thread_;
        bool capture_uploading_;
        int h_angle_;
        int v_angle_;
        int zoom_;
        int def_point_;
        int wait_ms_;
        boost::condition_variable ptz_rotate_con_;
        boost::condition_variable ptz_pause_con_;
        bool ptz_rotate_run();
        
        Config *cfg;
        Utils *utils;
        boost::thread *login_thread;
        boost::thread *rotate_thread;
        int login_retry;
        boost::mutex ptz_mutex_;
        boost::mutex ptz_wait_mutex_;
        boost::mutex ptz_register_mutex_;
        boost::thread* ptz_thread_;
        boost::condition_variable ptz_cond_;
        atris_msgs::SignalMessage *ptz_rotate_data_;
        atris_msgs::SignalMessage *movePtzData_;
        ros::NodeHandle nh_;
        ros::Subscriber signal_req_sub_;
        ros::ServiceServer get_ptz_camera_srv_;
        ros::ServiceServer ptz_control_srv_;
        void on_recv_ptz_ctrl(const atris_msgs::SignalMessage &msg);
        void ptz_rotate_async();
        bool doGetPtzStatus(atris_msgs::GetPtzStatus::Request& req, atris_msgs::GetPtzStatus::Response& res);
        bool on_recv_ptz_ctrl(atris_msgs::PtzControl::Request& req, atris_msgs::PtzControl::Response& res);

    public:
        static Ptz* get_instance(){
            static Ptz singleton;
            return &singleton;
        }

        int init();
        bool check_state();
        bool get_state();
        bool is_reconnect;
        int set_horiz(int degree);
        int set_vertical(int degree);
        bool ptz_move(int h_angle, int v_angle, int zoom, int idleMicSec = 5000);
        bool ptz_move_point(int point);
        bool ptz_is_rotating(){return is_rotate;};
        bool ptz_is_rotate_finished(){return !is_rotate;};
        bool ptz_stop();

        bool ptz_rotate(int h_angle, int v_angle, int zoom, int def_point);
        bool ptz_rotate_stop();
        bool ptz_rotate_pause(int onoff); //pause:1 resume:0
        bool ptz_rotate_pause();
        bool ptz_is_idle();

#ifdef PTZ_KEYBOARD_CONTROL
        int kfd = 0;
        struct termios cooked, raw;

        void quit(int sig);
        void speed_keyboard_input(void);
        bool kb_run;
#endif
};
#endif
