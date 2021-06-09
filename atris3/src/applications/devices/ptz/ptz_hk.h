#ifndef __PTZ_HK_H__
#define __PTZ_HK_H__

#include <vector>
#include <string.h>
#include <array>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "config/config.h"
#include "utils/utils.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetPtzStatus.h"
#include "atris_msgs/PtzControl.h"
#include "atris_msgs/GetPtzTemperature.h"
#include "transferfile/transferfile.h"
#include "HCNetSDK.h"
using namespace std;
//#define PTZ_KEYBOARD_CONTROL

#define CALLBACK
#define ISAPI_STATUS_LEN 4096

class Ptz_hk
{
private:
    Ptz_hk();

public:
    static Ptz_hk *get_instance()
    {
        static Ptz_hk singleton;
        return &singleton;
    }

    int init();
    int init_hk_audio();
    int init_hk_compress_info();
    int init_hk_poweroff_mode();
    int set_horiz(int degree);
    int set_vertical(int degree);
    bool check_state();
    bool get_state();
    bool ptz_move(float h_angle, float v_angle, float zoom, int idleMicSec = 5000);
    bool ptz_move_point(unsigned int point);
    bool ptz_is_rotating() { return is_rotate; };
    bool ptz_is_rotate_finished() { return !is_rotate; };

    bool ptz_temperature();

    void get_dvr_Config();
    int set_focus_mode(int mode);
    bool get_ptz_status(NET_DVR_PTZABSOLUTEEX_CFG &m_struBuiltinPTZABSOLUTEEX);
    bool set_ptz_status(NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX);
    bool get_thermal_ability();
    bool set_thermal_ability();
    int get_preset_alarm(int check_boundary);
    int run_ptz_control(int controlType, int speed, int runFlag);

    bool wait_for_ptz_idle(int timeout);
    bool ptz_capture(const atris_msgs::SignalMessage &msg);
    bool ptz_capture2(std::string &visible_light_url, std::string &infrared_url, std::string &imagebuf);
    bool ptz_rotate(int h_angle, int v_angle, int zoom, int def_point);
    bool ptz_rotate_stop();
    bool ptz_rotate_pause(int onoff); //pause:1 resume:0
    bool ptz_rotate_pause();
    bool ptz_is_idle();
    int start_preview_v40(const atris_msgs::SignalMessage &msg);
    //video
    int start_preview_v40();
    int save_real_data();
    int stop_preview_record(LONG handle);
    int stop_save_realdata_record(std::string &visible_light_video_url, std::string &infrared_video_url);
    int cancel_save_realdata_record();
    //audio
    int start_record_audio();
    int save_audio_record();
    int stop_save_audio_record(std::string &audio_url);
    int cancel_save_audio();

    //replay
    int start_preview_rp();
    int stop_save_realdata_record_rp();

    int set_temperature_compatibility(int mode);
    void set_thermometry_mode();
    void set_preset_tempthermometry(float x1, float y1, float x2, float y2);
    bool start_remote_config();
    bool stop_disconnect_remote_config();
    int convert_to_mp4(std::string &source, std::string &target);
    int convert_to_mp3(std::string &source, std::string &target);

private:
    void init_title_list();
    int raw_login(const std::string &ip, const int port, const std::string &user, const std::string &passwd);
    void ptz_login();

    void ptz_keyboard_input();
    void ptz_move_control(int direction);
    bool ptz_stop();
    bool ptz_up();
    bool ptz_down();
    bool ptz_left();
    bool ptz_right();
    bool ptz_up_ex(float angle = 0);
    bool ptz_down_ex(float angle = 0);
    bool ptz_left_ex(float angle);
    bool ptz_right_ex(float angle);
    void ptz_zoom_control(int op);
    bool ptz_zoom_add();
    bool ptz_zoom_dec();
    void ptz_focus_control(int op);
    bool ptz_zoom(int value);
    bool ptz_focus_add();
    bool ptz_focus_dec();
    bool ptz_wiper_control(int on_off);
    bool ptz_light_control(int on_off);
    bool get_current_ptz_info();
    bool ptz_rotate_run();
    void ptz_rotate_async();
    bool ptz_get_range_temperature(int start_x, int start_y, int end_x, int end_y, float &min, float &max, float &avg, float &dif, DWORD &min_locate, DWORD &max_locate);

    bool ptz_realplay_run();
    bool upload_file(const atris_msgs::SignalMessage &msg, std::string filepath);
    int upload_file(const std::string &file_local_path, std::string &file_remote_url, int type); //0:no 1:visible_video,2:infrared_video,3:visible_picture,4:infrared_picture,5:audio
    void change_ptz_speed(float h_speed, float v_speed);
    void on_recv_ptz_ctrl(const atris_msgs::SignalMessage &msg);
    bool on_recv_ptz_ctrl(atris_msgs::PtzControl::Request &req, atris_msgs::PtzControl::Response &res);
    bool on_recv_get_ptz_temperature(atris_msgs::GetPtzTemperature::Request &req, atris_msgs::GetPtzTemperature::Response &res);
    bool doGetPtzStatus(atris_msgs::GetPtzStatus::Request &req, atris_msgs::GetPtzStatus::Response &res);
    bool onGetRotateStatus(atris_msgs::GetPtzStatus::Request &req, atris_msgs::GetPtzStatus::Response &res);
    bool onGetFocusStatus(atris_msgs::GetPtzStatus::Request &req, atris_msgs::GetPtzStatus::Response &res);
    boost::thread *capture_thread_;
    boost::thread *realplay_thread_;
    boost::thread *record_thread_;
    boost::thread *audio_thread_;

    boost::thread *thread_upload_visiable_video;
    boost::mutex ptz_register_mutex_;
    bool capture_uploading_;
    bool record_uploading_;
    bool file_uploading_;
    int uploading_status = 0;
    bool ptz_state;
    bool is_login;
    bool ptz_rotate_pause_flag;
    bool is_rotate;
    bool is_rotate_async;
    bool is_reconnect;
    bool m_save_video_data;
    bool m_is_recording;
    bool m_is_recording_real_play;
    bool m_is_uploading_record;
    bool m_is_uploading_realplay;
    int default_point;
    float h_angle_, v_angle_, zoom_;
    float h_speed_, v_speed_;

    DWORD focus_;
    //int login_handle;
    unsigned int def_point_;
    int wait_ms_;
    int login_retry;
    int _channel;
    //long _long_channel;

    LONG lHandle;

    LONG irealplayhandle_visible_rp;
    LONG irealplayhandle_ir_rp;

    LONG irealplayhandle_visible_v30;
    LONG irealplayhandle_ir_v30;

    LONG irealplayhandle_visible_rp_v30;
    LONG irealplayhandle_ir_rp_v30;
    int m_light_status, m_wiper_status;

    Config *cfg;
    Utils *utils;

    boost::condition_variable ptz_rotate_con_;
    boost::condition_variable ptz_pause_con_;
    boost::condition_variable ptz_cond_;
    boost::thread *login_thread;
    boost::thread *rotate_thread;
    boost::thread *realplay_thread;
    boost::thread *record_thread;
    boost::thread *record_upload_thread;

    boost::mutex ptz_mutex_;
    boost::mutex ptz_wait_mutex_;
    boost::thread *ptz_thread_;

    atris_msgs::SignalMessage *ptz_rotate_data_;
    atris_msgs::SignalMessage *movePtzData_;

    ros::NodeHandle nh_;
    ros::Subscriber signal_req_sub_;
    ros::ServiceServer get_ptz_camera_srv_;
    ros::ServiceServer ptz_control_srv_;
    ros::ServiceServer ptz_temperature_srv_;

    enum PtzRotateStatus
    {
        PTZ_ROTATE_IDLE = 0,
        PTZ_ROTATE_PREPARE,
        PTZ_ROTATE_RUNING,
        PTZ_ROTATE_PAUSED,
        PTZ_ROTATE_STOP
    };
    enum PtzRotateStatus ptz_rotate_state_;
};
#endif
