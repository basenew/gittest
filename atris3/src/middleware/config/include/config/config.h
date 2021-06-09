#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

class Config
{
private:
    Config();
    void checkWithVersion();

public:
    std::string default_psw, super_name, super_psw, server_type;
    std::string hfs_type, hfs_url;
    std::string appkey, appsecret;
    std::string yx_appkey, yx_appsecret;
    std::string agora_appkey, agora_appsecret;
    std::string yx_create_url, yx_update_url, yx_data_dir;
    std::string gs_ip, maps_dir;
    int gs_web_port, gs_ws_port,gs_srv_port;
    std::string local_ip;
    std::string patrol_scheme_path;

    std::string ptz_user, ptz_psw, ptz_ip;
    int ptz_login_timeout;//登录尝试次数,秒

    std::string dbcom_device_ip, dbcom_username, dbcom_passwd;
    int dbcom_login_timeout;

    std::string router_ip, router_type, router_user, router_psw;

    int timezone;
    std::vector<std::string> ntp_server_list;

    float forward_max;
    float backward_max;
    float angular_max;
    float forward_wheel_max;
    float backward_wheel_max;
    float angular_wheel_max;
    float battery_voltage_diff_;

    float pitch_max;
    float roll_max;
    float yaw_max;
    int filter_constant;

    // voice chatting
    int vchat_delay_time;
    bool vchat_low_energy, vchat_aec, vchat_ns, vchat_vad;

    int low_battery_warn;
    int low_battery_charge;

    // mqttengine
    std::string mqtt_host_;
    std::string mqtt_username_;
    std::string mqtt_password_;
    int mqtt_encrypt_;
    int mqtt_port_;
    int mqtt_keepalive_;

    // remote_mqtt
    std::string remote_mqtt_topic_url_;
    std::string remote_mqtt_host_;
    std::string remote_mqtt_username_;
    std::string remote_mqtt_password_;
    int remote_mqtt_port_;
    int remote_mqtt_encrypt_;
    int remote_mqtt_keepalive_;

    std::string runtime_data;

    bool abi_enable;
    std::string abi_id;
    std::string abi_key;
    std::string abi_version;
    std::string base_url;
    std::string timestamp_url;
    std::string robotinfo_url;
    int abi_get_user_list_period_sec;
    int abi_get_timestamp_period_sec;
    std::string abi_super_name;
    std::string abi_super_pwd;

    int dock_wall_width;
    int dock_1_0;
    int tcp_server_port;

    std::string ptz_box_ip;
    std::string power_board_ip;
    std::string gps_ip;
    std::string nvr_ip;
    std::string voip_ip;

    std::string ws_xml_ip;
    int ws_xml_port;
    std::string ws_xml_path;
    std::string ws_json_ip;
    int ws_json_port;
    std::string ws_json_path;

    bool sip_register_enable;
    std::string sip_uas_ip;
    std::string sip_uas_port;
    std::string sip_user_name;
    std::string sip_user_pwd;
    int sip_expires;
    std::string sip_send_account;
    std::string sip_send_port;

    bool voip_auto_setting;
    std::string voip_sip_server;

    bool open_red_blue_light_imsi_detection;

    std::string post_event_url;

    bool http_debug_enable;

	//nav
    bool nav_enable;
	bool nav_cycle_enable;
	bool nav_without_charge_path;
    bool nav_auto_charge_self;
    bool nav_without_merge_path;
	int  nav_obs_mode;
    int  nav_init_timeout;
	int  nav_task_start_timeout;
	int  nav_task_running_timeout;
    int  nav_auto_charge_confirm_timeout;
	std::string nav_cycle_dst;
	int nav_auto_charge_battery;
    bool nav_anti_drop_enable;
    bool nav_tsp_enable;

    //obstacle 
    int obstacle_mode_; // 遇障导航策略 0: 不制动停障 1：制动停障 2：绕障
    int obstacle_timeout_; // 停障超时时长(5s~1000s, 默认500s)
    int obstacle_timeout_strategy_; //停障超时机制  0: 停止当前任务,避障超时时停止机器人当前任务 1:重新规划导航路线

    // vision
    std::string vision_host_;
    int vision_port_;

    //sw platform
    bool swp_https;
    std::string swp_port;
    std::string swp_host;
    std::string swp_upload_map_api;
	
    static Config* get_instance(){
        static Config singleton;
        return &singleton;
    }

    bool setDevicesIP(std::string ptz_ip, std::string ptz_box_ip, std::string nvr_ip);
    bool setUdock(int dock_wall_width, int dock_1_0);
    bool setMQTT(std::string host, int port, std::string username,
                 std::string password, int encrypt, int keepalive);
    bool setRemoteMQTT(std::string topic_url, std::string host, int port,
                       std::string username, std::string password, int encrypt, int keepalive);
    bool setRouter(std::string type, std::string ip, std::string username, std::string password);
    bool setNTP(std::string ntp_server, int timezone);
    bool setHFS(std::string hfs_type, std::string hfs_url);
    bool setABI(std::string base_url, std::string timestamp_url, std::string robotinfo_url, int abi_enable);
    bool setVoipParams(int auto_setting, std::string voipSipServer);
    bool setOpenRedBlueLightImsiDetection(int open);
    bool setPostEventUrl(std::string url);
    bool setDefault();
    bool setNav(int obs_mode, std::string cycle_dst, int cycle_enable, \
                int without_charge_path,int task_start_timeout, int task_running_timeout,\
                int without_merge_path, int auto_charge_self, int auto_charge_battery, int anti_drop_enable, int tsp_enable); //TODO struct nav_param
    bool setObstacle(int mode, int timeout, int timeout_strategy);

    bool setVision(std::string host, int port);
};
#endif
