#include <json/json.h>
#include "config.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <libgen.h>
#include "log/log.h"

#define ATRIS_CONFIG_RAW_PATH   "/home/atris/atris_app/config/robot.cfg"
#define ATRIS_CONFIG_PATH   "/userdata/impdata/robot.cfg"
#define ATRIS_CONFIG_PART_PATH "/userdata/impdata/robot.cfg.part"
#define ATRIS_CONFIG_SET_BEGIN \
  std::ifstream _in_(ATRIS_CONFIG_PATH, std::ios::binary); \
  if (!_in_.is_open()) { \
      log_error("%s Open file failed: %s", __FUNCTION__, ATRIS_CONFIG_PATH); \
      return false; \
  } \
  Json::Reader _reader_; \
  Json::Value root; \
  if(!_reader_.parse(_in_, root)){ \
      log_error("%s Parse config file failed.", __FUNCTION__); \
      _in_.close(); \
      return false; \
  } \
  _in_.close();
  
#define ATRIS_CONFIG_SET_END \
  Json::StyledWriter _fw_; \
  std::string _json_ = _fw_.write(root); \
  int _fd_ = open(ATRIS_CONFIG_PART_PATH, O_WRONLY | O_TRUNC | O_CREAT,0777); \
  if(_fd_ < 0) { \
    log_error("%s open config part file failed.", __FUNCTION__); \
    return false; \
  } \
  std::size_t _ret_ = write(_fd_, _json_.c_str(), _json_.size()); \
  if(_ret_ != _json_.size()){ \
    log_error("%s save config part file error.", __FUNCTION__); \
  } else { \
    remove(ATRIS_CONFIG_PATH); \
    rename(ATRIS_CONFIG_PART_PATH, ATRIS_CONFIG_PATH); \
  } \
  close(_fd_); \
  return true;


static int createDir(std::string dirname) {
    if(access(dirname.c_str(), F_OK) == 0 ) {
      return 0;
    }
    
    char* dir = strdup(dirname.c_str());
    if (!dir) {
        log_error("%s no memory error", __FUNCTION__);
        return -1;
    }

    int i, ret = -1, len = strlen(dir);

    log_info("%s mkdir: %s", __FUNCTION__, dir);

    for(i=1; i < len; i++) {
        if(dir[i]=='/') {
            dir[i] = 0;
            if(access((const char*)dir, F_OK) != 0 ) {
                if(mkdir(dir, 0777) == -1) {
                    log_error("%s mkdir: %s", __FUNCTION__, dir);
                    goto beach;
                }
            }
            dir[i] = '/';
        }
    }

    if(access((const char*)dir, F_OK) != 0 ) {
        if(mkdir(dir, 0777) == -1) {
            log_error("%s mkdir: %s", __FUNCTION__, dir);
            goto beach;
        }
    }

    ret = 0;

beach:
    if (!dir) {
        free(dir);
    }
    return ret;
}

bool Config::setDevicesIP(std::string ptz_ip, std::string ptz_box_ip, std::string nvr_ip) {
  ATRIS_CONFIG_SET_BEGIN {
    root["devices_ip"]["ptz_ip"] = ptz_ip;
    root["devices_ip"]["ptz_box_ip"] = ptz_box_ip;
    root["devices_ip"]["nvr_ip"] = nvr_ip;
    this->ptz_ip = ptz_ip;
    this->ptz_box_ip =ptz_box_ip;
    this->nvr_ip = nvr_ip;
  } ATRIS_CONFIG_SET_END
}

bool Config::setUdock(int dock_wall_width, int dock_1_0) {
  ATRIS_CONFIG_SET_BEGIN {
    root["udock"]["dock_1_0"] = dock_1_0;
    root["udock"]["dock_wall_width"] = dock_wall_width;
    this->dock_1_0 = dock_1_0;
    this->dock_wall_width = dock_wall_width;
  } ATRIS_CONFIG_SET_END
}

bool Config::setMQTT(std::string host, int port, std::string username, std::string password, int encrypt, int keepalive) {
  ATRIS_CONFIG_SET_BEGIN {
    root["mqttengine"]["host"] = host;
    root["mqttengine"]["port"] = port;
    root["mqttengine"]["username"] = username;
    root["mqttengine"]["password"] = password;
    root["mqttengine"]["encrypt"] = encrypt;
    root["mqttengine"]["keepalive"] = keepalive;
    this->mqtt_host_ = host;
    this->mqtt_port_ = port;
    this->mqtt_username_ = username;
    this->mqtt_password_ = password;
    this->mqtt_encrypt_ = encrypt;
    this->mqtt_keepalive_ = keepalive;
  } ATRIS_CONFIG_SET_END
}

bool Config::setRemoteMQTT(std::string topic_url, std::string host, int port, std::string username,
                           std::string password, int encrypt, int keepalive)
{
  ATRIS_CONFIG_SET_BEGIN {
    root["remote_mqtt"]["topic_url"] = topic_url;
    root["remote_mqtt"]["host"] = host;
    root["remote_mqtt"]["port"] = port;
    root["remote_mqtt"]["username"] = username;
    root["remote_mqtt"]["password"] = password;
    root["remote_mqtt"]["encrypt"] = encrypt;
    root["remote_mqtt"]["keepalive"] = keepalive;
    this->remote_mqtt_topic_url_ = topic_url;
    this->remote_mqtt_host_ = host;
    this->remote_mqtt_port_ = port;
    this->remote_mqtt_username_ = username;
    this->remote_mqtt_password_ = password;
    this->remote_mqtt_encrypt_ = encrypt;
    this->remote_mqtt_keepalive_ = keepalive;
  } ATRIS_CONFIG_SET_END
}

bool Config::setRouter(std::string type, std::string ip, std::string username, std::string password) {
    ATRIS_CONFIG_SET_BEGIN {
    root["router"]["ip"] = ip;
    root["router"]["type"] = type;
    root["router"]["username"] = username;
    root["router"]["password"] = password;
    this->router_ip = ip;
    this->router_type = type;
    this->router_user = username;
    this->router_psw = password;
  } ATRIS_CONFIG_SET_END
}

bool Config::setNTP(std::string ntp_server, int timezone) {
  ATRIS_CONFIG_SET_BEGIN {
    Json::Value list;
    std::vector<std::string> elems;
    std::string elem = "";
    std::size_t pos = 0;
    std::size_t len = ntp_server.length();
    std::size_t delim_len = 1;
    list.resize(0);
    while (pos < len) {
      std::size_t find_pos = ntp_server.find(";", pos);
      if (find_pos == std::string::npos) {
        elem = ntp_server.substr(pos, len - pos);
        elem.erase(elem.find_last_not_of(" \t\f\v\n\r") + 1);
        elem.erase(0, elem.find_first_not_of(" \t\f\v\n\r"));
        elems.push_back(elem);
        list.append(Json::Value(elem));
        break;
      } else {
        elem = ntp_server.substr(pos, find_pos - pos);
        elem.erase(elem.find_last_not_of(" \t\f\v\n\r") + 1);
        elem.erase(0, elem.find_first_not_of(" \t\f\v\n\r"));
        elems.push_back(elem);
        list.append(Json::Value(elem));
      }
      pos = find_pos + delim_len;
    }
    root["ntp"]["server"] = list;
    root["ntp"]["timezone"] = timezone;
    this->timezone = timezone;
    this->ntp_server_list = elems;
  } ATRIS_CONFIG_SET_END
}

bool Config::setHFS(std::string hfs_type, std::string hfs_url) {
  ATRIS_CONFIG_SET_BEGIN {
    if (*(hfs_url.end()-1) != '/') {
      hfs_url += "/";
    }
    root["hfs_server"]["hfs_type"] = hfs_type;
    root["hfs_server"]["hfs_url"] = hfs_url;
    this->hfs_type = hfs_type;
    this->hfs_url = hfs_url;
  } ATRIS_CONFIG_SET_END
}

bool Config::setABI(std::string base_url, 
  std::string timestamp_url, std::string robotinfo_url, int abi_enable) {
  ATRIS_CONFIG_SET_BEGIN {
    root["abi"]["base_url"] = base_url;
    root["abi"]["timestamp_url"] = timestamp_url;
    root["abi"]["robotinfo_url"] = robotinfo_url;
    root["abi"]["enable"] = abi_enable;
    this->base_url = base_url;
    this->timestamp_url = timestamp_url;
    this->robotinfo_url = robotinfo_url;
    this->abi_enable = abi_enable ? true : false;
  } ATRIS_CONFIG_SET_END
}

bool Config::setVoipParams(int autoSetting, std::string voipSipServer) {
  ATRIS_CONFIG_SET_BEGIN {
    root["voip"]["auto_setting"] = autoSetting;
    root["voip"]["sip_server"] = voipSipServer;
    this->voip_auto_setting = autoSetting ? true : false;
    this->voip_sip_server = voipSipServer;
  } ATRIS_CONFIG_SET_END

  return true;
}

bool Config::setNav(int obs_mode, std::string cycle_dst, int cycle_enable, \
                  int without_charge_path,int task_start_timeout, int task_running_timeout,\
                  int without_merge_path,int auto_charge_self, int auto_charge_battery,\
                  int anti_drop_enable, int tsp_enable) {
  ATRIS_CONFIG_SET_BEGIN {
    root["nav"]["obs_mode"] = obs_mode;
    root["nav"]["cycle_dst"] = cycle_dst;
    root["nav"]["cycle_enable"] = cycle_enable;  
    root["nav"]["without_charge_path"] = without_charge_path;      
    root["nav"]["task_start_timeout"] = task_start_timeout;  
    root["nav"]["task_running_timeout"] = task_running_timeout;  
    root["nav"]["without_merge_path"] = without_merge_path;
    root["nav"]["auto_charge_self"] = auto_charge_self;
    root["nav"]["auto_charge_battery"] = auto_charge_battery;
    root["nav"]["anti_drop_enable"] = anti_drop_enable;
    root["nav"]["tsp_enable"] = tsp_enable;
    this->nav_cycle_enable = cycle_enable ? true : false;
    this->nav_cycle_dst = cycle_dst;
    this->nav_without_charge_path = without_charge_path ? true : false;;
    this->nav_task_start_timeout = task_start_timeout;
    this->nav_task_running_timeout = task_running_timeout;
    this->nav_without_merge_path = without_merge_path ? true : false;
    this->nav_auto_charge_self = auto_charge_self ? true: false;
    this->nav_auto_charge_battery = auto_charge_battery;
    this->nav_anti_drop_enable = anti_drop_enable ? true: false;
    this->nav_tsp_enable = tsp_enable ? true: false;
  }ATRIS_CONFIG_SET_END

  return true;
}

bool Config::setVision(std::string ip, int port) {
  ATRIS_CONFIG_SET_BEGIN {
    root["vision"]["host"] = ip;
    root["vision"]["port"] = port;
    this->vision_host_ = ip;
    this->vision_port_ = port;
  }ATRIS_CONFIG_SET_END

  return true;
}

bool Config::setObstacle(int mode, int timeout, int timeout_strategy) {
  ATRIS_CONFIG_SET_BEGIN {
    root["obstacle"]["mode"] = mode;
    root["obstacle"]["timeout"] = timeout;
    root["obstacle"]["timeout_strategy"] = timeout_strategy;
    this->obstacle_mode_ = mode;
    this->obstacle_timeout_ = timeout;
    this->obstacle_timeout_strategy_ = timeout_strategy;
  }ATRIS_CONFIG_SET_END

  return true;
}


bool Config::setOpenRedBlueLightImsiDetection(int open_red_blue_light)
{
    ATRIS_CONFIG_SET_BEGIN {
    root["imsi"]["open_red_blue_light"] = open_red_blue_light;
    this->open_red_blue_light_imsi_detection = open_red_blue_light ? true : false;
  } ATRIS_CONFIG_SET_END
}

bool Config::setPostEventUrl(std::string url) {
  ATRIS_CONFIG_SET_BEGIN {
    root["abi"]["post_event_url"] = url;
    this->post_event_url = url;
  } ATRIS_CONFIG_SET_END

  return true;
}

bool Config::setDefault() {
  remove(ATRIS_CONFIG_PATH);

  FILE *fp = NULL;
  if((fp = popen("cp " ATRIS_CONFIG_RAW_PATH " " ATRIS_CONFIG_PATH, "r"))) {
    pclose(fp);
  }
  return true;
}

Config::Config() {
  log_info("%s", __FUNCTION__);
  checkWithVersion();
          
  std::ifstream in(ATRIS_CONFIG_PATH, std::ios::binary);
  if (!in.is_open()) {
      log_error("%s Open file failed: %s", __FUNCTION__, ATRIS_CONFIG_PATH);
      return;
  }

  // parse config file
  Json::Reader reader;
  Json::Value root;
  if(!reader.parse(in, root)){
      log_error("%s Parse config file failed.", __FUNCTION__);
      in.close();
      return;
  }
  in.close();

  if (!root["robot"].isNull()) {
    if (!root["robot"]["default_pwd"].isNull()) {
      default_psw = root["robot"]["default_pwd"].asString();
    }
    if (!root["robot"]["super_name"].isNull()) {
      super_name = root["robot"]["super_name"].asString();
    }
    if (!root["robot"]["super_pwd"].isNull()) {
      super_psw = root["robot"]["super_pwd"].asString();
    }
    if (!root["robot"]["server_type"].isNull()) {
      server_type = root["robot"]["server_type"].asString();
    }
  }

  if (!root["udock"].isNull()) {
    if (!root["udock"]["dock_wall_width"].isNull()) {
      dock_wall_width = root["udock"]["dock_wall_width"].asInt();
    }
    if (!root["udock"]["dock_1_0"].isNull()) {
      dock_1_0 = root["udock"]["dock_1_0"].asInt();
    }
  }
 
  if (!root["hfs_server"].isNull()) {
    if (!root["hfs_server"]["hfs_type"].isNull()) {
      hfs_type = root["hfs_server"]["hfs_type"].asString();
    }
    if (!root["hfs_server"]["hfs_url"].isNull()) {
      hfs_url = root["hfs_server"]["hfs_url"].asString();
    }
  }

  //agora && yunxin
  const std::string yx = "yunxin";
  vchat_delay_time = vchat_low_energy = vchat_aec = vchat_ns = vchat_vad = 0;
  if (!root["yunxin"].isNull()) {
    if (!root["yunxin"]["appkey"].isNull()) {
      yx_appkey = root["yunxin"]["appkey"].asString();
    }
    if (!root["yunxin"]["appsecret"].isNull()) {
      yx_appsecret = root["yunxin"]["appsecret"].asString();
    }
    if (!root["yunxin"]["create_url"].isNull()) {
      yx_create_url = root["yunxin"]["create_url"].asString();
    }
    if (!root["yunxin"]["update_url"].isNull()) {
      yx_update_url = root["yunxin"]["update_url"].asString();
    }
    if (!root["yunxin"]["data_dir"].isNull()) {
      yx_data_dir = root["yunxin"]["data_dir"].asString();
    }
    if (!root["yunxin"]["vchat_delay_time"].isNull()) {
      vchat_delay_time = root["yunxin"]["vchat_delay_time"].asInt();
    }
    if (!root["yunxin"]["vchat_low_energy"].isNull()) {
      vchat_low_energy = root["yunxin"]["vchat_low_energy"].asInt() > 0;
    }
    if (!root["yunxin"]["vchat_aec"].isNull()) {
      vchat_aec = root["yunxin"]["vchat_aec"].asInt() > 0;
    }
    if (!root["yunxin"]["vchat_ns"].isNull()) {
      vchat_ns = root["yunxin"]["vchat_ns"].asInt() > 0;
    }
    if (!root["yunxin"]["vchat_vad"].isNull()) {
      vchat_vad = root["yunxin"]["vchat_vad"].asInt() > 0;
    }
  }
  
  if (!root["agora"].isNull()) {
    if (!root["agora"]["appkey"].isNull()) {
      agora_appkey = root["agora"]["appkey"].asString();
    }
    if (!root["agora"]["appsecret"].isNull()) {
      agora_appsecret = root["agora"]["appsecret"].asString();
    }
  }
  if(server_type == yx){
    appkey = yx_appkey;
    appsecret = yx_appsecret;
  } else {
    appkey = agora_appkey;
    appsecret = agora_appsecret;
  }

  if (!root["devices_ip"].isNull()) {
    if (!root["devices_ip"]["gs_ip"].isNull()) {
      gs_ip = root["devices_ip"]["gs_ip"].asString();
    }
    if (!root["devices_ip"]["imx_ip"].isNull()) {
      local_ip = root["devices_ip"]["imx_ip"].asString();
    }
    if (!root["devices_ip"]["ptz_ip"].isNull()) {
      ptz_ip = root["devices_ip"]["ptz_ip"].asString();
    }
    if (!root["devices_ip"]["ptz_box_ip"].isNull()) {
      ptz_box_ip = root["devices_ip"]["ptz_box_ip"].asString();
    }
    if (!root["devices_ip"]["power_board_ip"].isNull()) {
      power_board_ip = root["devices_ip"]["power_board_ip"].asString();
    }
    if (!root["devices_ip"]["gps_ip"].isNull()) {
      gps_ip = root["devices_ip"]["gps_ip"].asString();
    }
    if (!root["devices_ip"]["nvr_ip"].isNull()) {
      nvr_ip = root["devices_ip"]["nvr_ip"].asString();
    }
    if (!root["devices_ip"]["voip_ip"].isNull()) {
      voip_ip = root["devices_ip"]["voip_ip"].asString();
    }
  }

  //gs
  if (!root["gs"].isNull()) {
    if (!root["gs"]["gs_web_port"].isNull()) {
      gs_web_port = root["gs"]["gs_web_port"].asInt();
    }
    if (!root["gs"]["gs_ws_port"].isNull()) {
      gs_ws_port = root["gs"]["gs_ws_port"].asInt();
    }
    if (!root["gs"]["maps_dir"].isNull()) {
      maps_dir = root["gs"]["maps_dir"].asString();
    }
    if (!root["gs"]["gs_srv_port"].isNull()) {
      gs_srv_port = root["gs"]["gs_srv_port"].asInt();
    }
    if (!root["gs"]["patrol_scheme"].isNull()) {
      patrol_scheme_path = root["gs"]["patrol_scheme"].asString();
    }
    if (!root["gs"]["runtime_data"].isNull()) {
      runtime_data = root["gs"]["runtime_data"].asString();
    }
  }

  //ptz
  if (!root["ptz"].isNull()) {
    if (!root["ptz"]["ptz_user"].isNull()) {
      ptz_user = root["ptz"]["ptz_user"].asString();
    }
    if (!root["ptz"]["ptz_psw"].isNull()) {
      ptz_psw = root["ptz"]["ptz_psw"].asString();
    }
    if (!root["ptz"]["ptz_login_timeout"].isNull()) {
      ptz_login_timeout = root["ptz"]["ptz_login_timeout"].asInt();
    }
  }

  // dbcom
  if (!root["dbcom"].isNull()) {
    if (!root["dbcom"]["device_ip"].isNull()) {
      dbcom_device_ip = root["dbcom"]["device_ip"].asString();
    }
    if (!root["dbcom"]["username"].isNull()) {
      dbcom_username = root["dbcom"]["username"].asString();
    }
    if (!root["dbcom"]["passwd"].isNull()) {
      dbcom_passwd = root["dbcom"]["passwd"].asString();
    }
    if (!root["dbcom"]["login_timeout"].isNull()) {
      dbcom_login_timeout = root["dbcom"]["login_timeout"].asInt();
    }
  }

  // router
  if (!root["router"].isNull()) {
    if (!root["router"]["ip"].isNull()) {
      router_ip = root["router"]["ip"].asString();
    }
    if (!root["router"]["type"].isNull()) {
      router_type = root["router"]["type"].asString();
    }
    if (!root["router"]["username"].isNull()) {
      router_user = root["router"]["username"].asString();
    }
    if (!root["router"]["password"].isNull()) {
      router_psw = root["router"]["password"].asString();
    }
  }

  //mqttengine
  if (!root["mqttengine"].isNull()) {
    if (!root["mqttengine"]["host"].isNull()) {
      mqtt_host_ = root["mqttengine"]["host"].asString();
    }
    if (!root["mqttengine"]["username"].isNull()) {
      mqtt_username_ = root["mqttengine"]["username"].asString();
    }
    if (!root["mqttengine"]["password"].isNull()) {
      mqtt_password_ = root["mqttengine"]["password"].asString();
    }
    if (!root["mqttengine"]["encrypt"].isNull()) {
      mqtt_encrypt_ = root["mqttengine"]["encrypt"].asInt();
    }
    if (!root["mqttengine"]["port"].isNull()) {
      mqtt_port_ = root["mqttengine"]["port"].asInt();
    }
    if (!root["mqttengine"]["keepalive"].isNull()) {
      mqtt_keepalive_ = root["mqttengine"]["keepalive"].asInt();
    }
  }

  // remote_mqtt
  if (!root["remote_mqtt"].isNull()) {
    if (root["remote_mqtt"]["topic_url"].isString()) {
      remote_mqtt_topic_url_ = root["remote_mqtt"]["topic_url"].asString();
    }
    if (root["remote_mqtt"]["host"].isString()) {
      remote_mqtt_host_ = root["remote_mqtt"]["host"].asString();
    }
    if (root["remote_mqtt"]["username"].isString()) {
      remote_mqtt_username_ = root["remote_mqtt"]["username"].asString();
    }
    if (root["remote_mqtt"]["password"].isString()) {
      remote_mqtt_password_ = root["remote_mqtt"]["password"].asString();
    }
    if (root["remote_mqtt"]["encrypt"].isInt()) {
      remote_mqtt_encrypt_ = root["remote_mqtt"]["encrypt"].asInt();
    }
    if (root["remote_mqtt"]["port"].isInt()) {
      remote_mqtt_port_ = root["remote_mqtt"]["port"].asInt();
    }
    if (root["remote_mqtt"]["keepalive"].isInt()) {
      remote_mqtt_keepalive_ = root["remote_mqtt"]["keepalive"].asInt();
    }
  }
  
  if (!root["ntp"].isNull()) {
    if (!root["ntp"]["server"].isNull() && root["ntp"]["server"].isArray()) {
      Json::Value ntp_list = root["ntp"]["server"];
      int ntp_size = ntp_list.size();
      for(int i = 0; i < ntp_size; i ++){
        ntp_server_list.push_back(ntp_list[i].asString());
      }
    }
    if (!root["ntp"]["timezone"].isNull()) {
      timezone = root["ntp"]["timezone"].asInt();
    }
  }

  low_battery_warn = 30;
  low_battery_charge = 40;
  if (!root["battery"].isNull()) {
    if (!root["battery"]["low_battery_warn"].isNull()) {
      low_battery_warn = root["battery"]["low_battery_warn"].asInt();
    }
    if (!root["battery"]["low_battery_charge"].isNull()) {
      low_battery_charge = root["battery"]["low_battery_charge"].asInt();
    }
  }

  if (!root["body_euler"].isNull()) {
    if (!root["body_euler"]["pitch_max"].isNull()) {
      pitch_max = root["body_euler"]["pitch_max"].asFloat();
    }
    if (!root["body_euler"]["roll_max"].isNull()) {
      roll_max = root["body_euler"]["roll_max"].asFloat();
    }
    if (!root["body_euler"]["yaw_max"].isNull()) {
      yaw_max = root["body_euler"]["yaw_max"].asFloat();
    }
    if (!root["body_euler"]["filter_constant"].isNull()) {
      filter_constant = root["body_euler"]["filter_constant"].asFloat();
    }
  }

  if (!root["speed"].isNull()) {
    if (!root["speed"]["forward_max"].isNull()) {
      forward_max = root["speed"]["forward_max"].asFloat();
    }
    if (!root["speed"]["backward_max"].isNull()) {
      backward_max = root["speed"]["backward_max"].asFloat();
    }
    if (!root["speed"]["angular_max"].isNull()) {
      angular_max = root["speed"]["angular_max"].asFloat();
    }
    if (!root["speed"]["forward_wheel_max"].isNull()) {
      forward_wheel_max = root["speed"]["forward_wheel_max"].asFloat();
    }
    if (!root["speed"]["backward_wheel_max"].isNull()) {
      backward_wheel_max = root["speed"]["backward_wheel_max"].asFloat();
    }
    if (!root["speed"]["angular_wheel_max"].isNull()) {
      angular_wheel_max = root["speed"]["angular_wheel_max"].asFloat();
    }

    if (!root["speed"]["battery_voltage_diff"].isNull()) {
      battery_voltage_diff_ = root["speed"]["battery_voltage_diff"].asFloat();
    }
  }

  if (!root["abi"].isNull()) {
    if (!root["abi"]["enable"].isNull()) {
      abi_enable = root["abi"]["enable"].asInt() > 0;
    }
    if (!root["abi"]["abi_id"].isNull()) {
      abi_id = root["abi"]["abi_id"].asString();
    }
    if (!root["abi"]["abi_key"].isNull()) {
      abi_key = root["abi"]["abi_key"].asString();
    }
    if (!root["abi"]["abi_version"].isNull()) {
      abi_version = root["abi"]["abi_version"].asString();
    }
    if (!root["abi"]["base_url"].isNull()) {
      base_url = root["abi"]["base_url"].asString();
    }
    if (!root["abi"]["timestamp_url"].isNull()) {
      timestamp_url = root["abi"]["timestamp_url"].asString();
    }
    if (!root["abi"]["robotinfo_url"].isNull()) {
      robotinfo_url = root["abi"]["robotinfo_url"].asString();
    }
    if (!root["abi"]["get_user_list_period_sec"].isNull()) {
      abi_get_user_list_period_sec = root["abi"]["get_user_list_period_sec"].asInt();
    }
    if (!root["abi"]["get_timestamp_period_sec"].isNull()) {
      abi_get_timestamp_period_sec = root["abi"]["get_timestamp_period_sec"].asInt();
    }
    if (!root["abi"]["super_name"].isNull()) {
      abi_super_name = root["abi"]["super_name"].asString();
    }
    if (!root["abi"]["super_pwd"].isNull()) {
      abi_super_pwd = root["abi"]["super_pwd"].asString();
    }
    if (!root["abi"]["post_event_url"].isNull()) {
      post_event_url = root["abi"]["post_event_url"].asString();
    }
  }

  if (!root["id_card"].isNull()) {
    if (!root["id_card"]["tcp_server_port"].isNull()) {
      tcp_server_port = root["id_card"]["tcp_server_port"].asInt();
    }
  }
  
  if (!root["http_debug"].isNull()) {
    if (!root["http_debug"]["enable"].isNull()) {
      http_debug_enable = root["http_debug"]["enable"].asInt() > 0;
    }
  }

  if (!root["ws"].isNull()) {
    if (!root["ws"]["ws_xml_ip"].isNull()) {
      ws_xml_ip = root["ws"]["ws_xml_ip"].asString();
    }
    if (!root["ws"]["ws_xml_port"].isNull()) {
      ws_xml_port = root["ws"]["ws_xml_port"].asInt();
    }
    if (!root["ws"]["ws_xml_path"].isNull()) {
      ws_xml_path = root["ws"]["ws_xml_path"].asString();
    }
    if (!root["ws"]["ws_json_ip"].isNull()) {
      ws_json_ip = root["ws"]["ws_json_ip"].asString();
    }
    if (!root["ws"]["ws_json_port"].isNull()) {
      ws_json_port = root["ws"]["ws_json_port"].asInt();
    }
    if (!root["ws"]["ws_json_path"].isNull()) {
      ws_json_path = root["ws"]["ws_json_path"].asString();
    }
  }
  
  if (!root["sip"].isNull()) {
    if (!root["sip"]["register_enable"].isNull()) {
      sip_register_enable = root["sip"]["register_enable"].asInt() > 0;
    }
    if (!root["sip"]["uas_ip"].isNull()) {
      sip_uas_ip = root["sip"]["uas_ip"].asString();
    }
    if (!root["sip"]["uas_port"].isNull()) {
      sip_uas_port = root["sip"]["uas_port"].asString();
    }
    if (!root["sip"]["user_name"].isNull()) {
      sip_user_name = root["sip"]["user_name"].asString();
    }
    if (!root["sip"]["user_pwd"].isNull()) {
      sip_user_pwd = root["sip"]["user_pwd"].asString();
    }
    if (!root["sip"]["expires"].isNull()) {
      sip_expires = root["sip"]["expires"].asInt();
    }
    if (!root["sip"]["send_account"].isNull()) {
      sip_send_account = root["sip"]["send_account"].asString();
    }
    if (!root["sip"]["send_port"].isNull()) {
      sip_send_port = root["sip"]["send_port"].asString();
    }
  }

  if (!root["voip"].isNull()) {
    if (!root["voip"]["auto_setting"].isNull()) {
      voip_auto_setting = root["voip"]["auto_setting"].asInt() > 0;
    }
    if (!root["voip"]["sip_server"].isNull()) {
      voip_sip_server = root["voip"]["sip_server"].asString();
    }
  }

  if (!root["imsi"].isNull()) {
    if (!root["imsi"]["open_red_blue_light"].isNull()) {
      open_red_blue_light_imsi_detection = root["imsi"]["open_red_blue_light"].asInt() > 0;
    }
  }

  if (!root["vision"].isNull()) {
    if (!root["vision"]["host"].isNull()) {
      vision_host_ = root["vision"]["host"].asString();
    }

    if (!root["vision"]["port"].isNull()) {
      vision_port_ = root["vision"]["port"].asInt();
    }

  }

  if (!root["obstacle"].isNull()) {
    if (!root["obstacle"]["mode"].isNull()) {
      obstacle_mode_ = root["obstacle"]["mode"].asInt();
    }

    if (!root["obstacle"]["timeout"].isNull()) {
      obstacle_timeout_ = root["obstacle"]["timeout"].asInt();
    }

    if (!root["obstacle"]["timeout_strategy"].isNull()) {
      obstacle_timeout_strategy_ = root["obstacle"]["timeout_strategy"].asInt();
    }
  }

  //sw platform
  swp_https = true;
  swp_port = "80";
  swp_host = "local.ubtrobot.com";
  swp_upload_map_api = "/irms-service/irms/pc/uploadMap";
  if (!root["sw_platform"].isNull()) {
    if (!root["sw_platform"]["https"].isNull()) {
      swp_https = root["sw_platform"]["https"].asInt() > 0;
	}
    if (!root["sw_platform"]["port"].isNull()) {
      swp_port = root["sw_platform"]["port"].asString();
	}
    if (!root["sw_platform"]["host"].isNull()) {
      swp_host = root["sw_platform"]["host"].asString();
	}
    if (!root["sw_platform"]["upload_map_api"].isNull()) {
      swp_upload_map_api = root["sw_platform"]["upload_map_api"].asString();
	}
  }
  log_info("sw https:%d", swp_https);
  log_info("sw host:%s", swp_host.c_str());
  log_info("sw port:%s", swp_port.c_str());
  log_info("sw upload map api:%s", swp_upload_map_api.c_str());

  //nav
  nav_enable = true;
  nav_obs_mode = 0;
  nav_cycle_dst = "1.2";
  nav_cycle_enable = false;
  nav_without_charge_path = false;
  nav_init_timeout = 180;
  nav_task_start_timeout = 30;
  nav_task_running_timeout = 300;
  nav_without_merge_path = false;
  nav_auto_charge_self = false;
  nav_auto_charge_confirm_timeout = 1;
  nav_auto_charge_battery = 30;
  nav_anti_drop_enable = true;
  nav_tsp_enable = true;
#if 1 
  if (!root["nav"].isNull()) {
    if (!root["nav"]["enable"].isNull()) {
      nav_enable = root["nav"]["enable"].asInt() > 0;
    }

    if (!root["nav"]["init_timeout"].isNull()) {
      nav_init_timeout = root["nav"]["init_timeout"].asInt();
    }
    
    if (!root["nav"]["obs_mode"].isNull()) {
      nav_obs_mode = root["nav"]["obs_mode"].asInt();
    }

    if (!root["nav"]["auto_charge_battery"].isNull()) {
      nav_auto_charge_battery = root["nav"]["auto_charge_battery"].asInt();
    }

    if (!root["nav"]["cycle_enable"].isNull()) {
      nav_cycle_enable = root["nav"]["cycle_enable"].asInt() > 0;
    }

    if (!root["nav"]["cycle_dst"].isNull()) {
      nav_cycle_dst = root["nav"]["cycle_dst"].asString();
    }

    if (!root["nav"]["without_charge_path"].isNull()) {
      nav_without_charge_path = root["nav"]["without_charge_path"].asInt();
    }
    if (!root["nav"]["task_start_timeout"].isNull()) {
      nav_task_start_timeout = root["nav"]["task_start_timeout"].asInt();
    }
    if (!root["nav"]["task_running_timeout"].isNull()) {
      nav_task_running_timeout = root["nav"]["task_running_timeout"].asInt();
    }

    if (!root["nav"]["without_merge_path"].isNull()) {
      nav_without_merge_path = root["nav"]["without_merge_path"].asInt() > 0;
    }

    if (!root["nav"]["auto_charge_self"].isNull()) {
      nav_auto_charge_self = root["nav"]["auto_charge_self"].asInt() > 0;
    }

    if (!root["nav"]["auto_charge_confirm_timeout"].isNull()) {
      nav_auto_charge_confirm_timeout = root["nav"]["auto_charge_confirm_timeout"].asInt();
    }
    
    if (!root["nav"]["anti_drop_enable"].isNull()) {
      nav_anti_drop_enable = root["nav"]["anti_drop_enable"].asInt() > 0;
    }

    if (!root["nav"]["tsp_enable"].isNull()) {
      nav_tsp_enable = root["nav"]["tsp_enable"].asInt() > 0;
    }
    
  }
#endif

  log_info("nav enable:%d", nav_enable);
  log_info("nav init timeout:%d", nav_init_timeout);
  log_info("obs mode:%d", nav_obs_mode);
  log_info("cycle enable:%d", nav_cycle_enable);
  log_info("cycle dst:%s", nav_cycle_dst.c_str());
  log_info("without charge path:%d", nav_without_charge_path);
  log_info("task start timeout:%d", nav_task_start_timeout);
  log_info("task running timeout:%d", nav_task_running_timeout);
  log_info("without merge path:%d", nav_without_merge_path);
  log_info("auto charge self:%d", nav_auto_charge_self);
  log_info("auto charge confirm timeout:%d", nav_auto_charge_confirm_timeout);
  log_info("auto_charge_battery:%d", nav_auto_charge_battery);
  log_info("anti_drop_enable：%d", nav_anti_drop_enable);
  log_info("tsp_enable：%d", nav_tsp_enable);
}

void Config::checkWithVersion() {
  bool can_beach_file = true;
  bool can_beach_parse = true;

beach_file:
beach_parse:
  if(access(ATRIS_CONFIG_PATH, F_OK) !=0 ) {
    std::string temp = ATRIS_CONFIG_PATH;
    std::size_t idx = temp.rfind("/");
    if (idx != std::string::npos) {
      std::string strDir = temp.substr(0, idx);
      if(access(strDir.c_str(), F_OK) !=0 ) {
        createDir(strDir);
      }
    }

    FILE *fp = NULL;
    if((fp = popen("cp " ATRIS_CONFIG_RAW_PATH " " ATRIS_CONFIG_PATH, "r"))) {
      pclose(fp);
    } else {
      log_error("%s Copy config file failed.", __FUNCTION__);
      return;
    }
  }
          
  std::ifstream in_raw(ATRIS_CONFIG_RAW_PATH, std::ios::binary);
  if (!in_raw.is_open()) {
    log_error("%s Open raw failed: %s", __FUNCTION__, ATRIS_CONFIG_RAW_PATH);
    return;
  }
          
  std::ifstream in(ATRIS_CONFIG_PATH, std::ios::binary);
  if (!in.is_open()) {
    in_raw.close();
    if (can_beach_file) {
      log_error("%s Open file failed, try to restore with raw config.", __FUNCTION__);
      remove(ATRIS_CONFIG_PATH);
      can_beach_file = false;
      goto beach_file;
    } else {
      log_error("Open file failed.");
    }
    return;
  }

  // parse config file
  Json::Reader reader_raw, reader;
  Json::Value root_raw, root;
  if(!reader_raw.parse(in_raw, root_raw)){
    log_error("%s Parse raw file failed, can not check.", __FUNCTION__);
    in.close();
    in_raw.close();
    return;
  }
  if(!reader.parse(in, root)){
    in.close();
    in_raw.close();
    if (can_beach_parse) {
      log_error("%s Parse config file failed, try to restore with raw config.", __FUNCTION__);
      remove(ATRIS_CONFIG_PATH);
      can_beach_parse = false;
      goto beach_parse;
    } else {
      log_error("%s Parse config file failed.", __FUNCTION__);
    }
    return;
  }
  in.close();
  in_raw.close();

  std::string version_raw = root_raw["version"].isNull() ? "" : root_raw["version"].asString();
  std::string version = root["version"].isNull() ? "" : root["version"].asString();
  if (version_raw != version) {
    /* robot { */
    if (!root_raw["robot"].isNull() && !root_raw["robot"]["default_pwd"].isNull()) {
      if (!root["robot"].isNull() && !root["robot"]["default_pwd"].isNull()) {
        root_raw["robot"]["default_pwd"] = root["robot"]["default_pwd"];
      }
    }
    if (!root_raw["robot"].isNull() && !root_raw["robot"]["super_name"].isNull()) {
      if (!root["robot"].isNull() && !root["robot"]["super_name"].isNull()) {
        root_raw["robot"]["super_name"] = root["robot"]["super_name"];
      }
    }
    if (!root_raw["robot"].isNull() && !root_raw["robot"]["super_pwd"].isNull()) {
      if (!root["robot"].isNull() && !root["robot"]["super_pwd"].isNull()) {
        root_raw["robot"]["super_pwd"] = root["robot"]["super_pwd"];
      }
    }
    if (!root_raw["robot"].isNull() && !root_raw["robot"]["server_type"].isNull()) {
      if (!root["robot"].isNull() && !root["robot"]["server_type"].isNull()) {
        root_raw["robot"]["server_type"] = root["robot"]["server_type"];
      }
    }
    /* } robot */
    
    /* hfs_server { */
    if (!root_raw["hfs_server"].isNull() && !root_raw["hfs_server"]["hfs_url"].isNull()) {
      if (!root["hfs_server"].isNull() && !root["hfs_server"]["hfs_url"].isNull()) {
        root_raw["hfs_server"]["hfs_url"] = root["hfs_server"]["hfs_url"];
      }
    }
    if (!root_raw["hfs_server"].isNull() && !root_raw["hfs_server"]["hfs_type"].isNull()) {
      if (!root["hfs_server"].isNull() && !root["hfs_server"]["hfs_type"].isNull()) {
        root_raw["hfs_server"]["hfs_type"] = root["hfs_server"]["hfs_type"];
      }
    }
    /* } hfs_server */

    /* devices_ip { */
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["imx_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["imx_ip"].isNull()) {
        root_raw["devices_ip"]["imx_ip"] = root["devices_ip"]["imx_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["ptz_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["ptz_ip"].isNull()) {
        root_raw["devices_ip"]["ptz_ip"] = root["devices_ip"]["ptz_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["gs_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["gs_ip"].isNull()) {
        root_raw["devices_ip"]["gs_ip"] = root["devices_ip"]["gs_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["ptz_box_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["ptz_box_ip"].isNull()) {
        root_raw["devices_ip"]["ptz_box_ip"] = root["devices_ip"]["ptz_box_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["power_board_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["power_board_ip"].isNull()) {
        root_raw["devices_ip"]["power_board_ip"] = root["devices_ip"]["power_board_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["gps_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["gps_ip"].isNull()) {
        root_raw["devices_ip"]["gps_ip"] = root["devices_ip"]["gps_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["nvr_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["nvr_ip"].isNull()) {
        root_raw["devices_ip"]["nvr_ip"] = root["devices_ip"]["nvr_ip"];
      }
    }
    if (!root_raw["devices_ip"].isNull() && !root_raw["devices_ip"]["voip_ip"].isNull()) {
      if (!root["devices_ip"].isNull() && !root["devices_ip"]["voip_ip"].isNull()) {
        root_raw["devices_ip"]["voip_ip"] = root["devices_ip"]["voip_ip"];
      }
    }
    /* } devices_ip */

    /* mqttengine { */
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["host"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["host"].isNull()) {
        root_raw["mqttengine"]["host"] = root["mqttengine"]["host"];
      }
    }
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["port"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["port"].isNull()) {
        root_raw["mqttengine"]["port"] = root["mqttengine"]["port"];
      }
    }
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["keepalive"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["keepalive"].isNull()) {
        root_raw["mqttengine"]["keepalive"] = root["mqttengine"]["keepalive"];
      }
    }
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["username"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["username"].isNull()) {
        root_raw["mqttengine"]["username"] = root["mqttengine"]["username"];
      }
    }
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["password"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["password"].isNull()) {
        root_raw["mqttengine"]["password"] = root["mqttengine"]["password"];
      }
    }
    if (!root_raw["mqttengine"].isNull() && !root_raw["mqttengine"]["encrypt"].isNull()) {
      if (!root["mqttengine"].isNull() && !root["mqttengine"]["encrypt"].isNull()) {
        root_raw["mqttengine"]["encrypt"] = root["mqttengine"]["encrypt"];
      }
    }
    /* } mqttengine */

    /* remote_mqtt { */
     if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["topic_url"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["topic_url"].isNull()) {
        root_raw["remote_mqtt"]["topic_url"] = root["remote_mqtt"]["topic_url"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["host"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["host"].isNull()) {
        root_raw["remote_mqtt"]["host"] = root["remote_mqtt"]["host"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["port"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["port"].isNull()) {
        root_raw["remote_mqtt"]["port"] = root["remote_mqtt"]["port"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["keepalive"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["keepalive"].isNull()) {
        root_raw["remote_mqtt"]["keepalive"] = root["remote_mqtt"]["keepalive"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["username"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["username"].isNull()) {
        root_raw["remote_mqtt"]["username"] = root["remote_mqtt"]["username"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["password"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["password"].isNull()) {
        root_raw["remote_mqtt"]["password"] = root["remote_mqtt"]["password"];
      }
    }
    if (!root_raw["remote_mqtt"].isNull() && !root_raw["remote_mqtt"]["encrypt"].isNull()) {
      if (!root["remote_mqtt"].isNull() && !root["remote_mqtt"]["encrypt"].isNull()) {
        root_raw["remote_mqtt"]["encrypt"] = root["remote_mqtt"]["encrypt"];
      }
    }
    /* } remote_mqtt */
    
    /* ws { */
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_xml_ip"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_xml_ip"].isNull()) {
        root_raw["ws"]["ws_xml_ip"] = root["ws"]["ws_xml_ip"];
      }
    }
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_xml_port"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_xml_port"].isNull()) {
        root_raw["ws"]["ws_xml_port"] = root["ws"]["ws_xml_port"];
      }
    }
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_xml_path"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_xml_path"].isNull()) {
        root_raw["ws"]["ws_xml_path"] = root["ws"]["ws_xml_path"];
      }
    }
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_json_ip"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_json_ip"].isNull()) {
        root_raw["ws"]["ws_json_ip"] = root["ws"]["ws_json_ip"];
      }
    }
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_json_port"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_json_port"].isNull()) {
        root_raw["ws"]["ws_json_port"] = root["ws"]["ws_json_port"];
      }
    }
    if (!root_raw["ws"].isNull() && !root_raw["ws"]["ws_json_path"].isNull()) {
      if (!root["ws"].isNull() && !root["ws"]["ws_json_path"].isNull()) {
        root_raw["ws"]["ws_json_path"] = root["ws"]["ws_json_path"];
      }
    }
    /* } ws */
    
    /* sip { */
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["register_enable"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["register_enable"].isNull()) {
        root_raw["sip"]["register_enable"] = root["sip"]["register_enable"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["uas_ip"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["uas_ip"].isNull()) {
        root_raw["sip"]["uas_ip"] = root["sip"]["uas_ip"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["uas_port"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["uas_port"].isNull()) {
        root_raw["sip"]["uas_port"] = root["sip"]["uas_port"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["user_name"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["user_name"].isNull()) {
        root_raw["sip"]["user_name"] = root["sip"]["user_name"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["user_pwd"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["user_pwd"].isNull()) {
        root_raw["sip"]["user_pwd"] = root["sip"]["user_pwd"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["expires"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["expires"].isNull()) {
        root_raw["sip"]["expires"] = root["sip"]["expires"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["send_account"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["send_account"].isNull()) {
        root_raw["sip"]["send_account"] = root["sip"]["send_account"];
      }
    }
    if (!root_raw["sip"].isNull() && !root_raw["sip"]["send_port"].isNull()) {
      if (!root["sip"].isNull() && !root["sip"]["send_port"].isNull()) {
        root_raw["sip"]["send_port"] = root["sip"]["send_port"];
      }
    }
    /* } sip */

    /* voip { */
    if (!root_raw["voip"].isNull() && !root_raw["voip"]["auto_setting"].isNull()) {
      if (!root["voip"].isNull() && !root["voip"]["auto_setting"].isNull()) {
        root_raw["voip"]["auto_setting"] = root["voip"]["auto_setting"];
      }
    }
    if (!root_raw["voip"].isNull() && !root_raw["voip"]["sip_server"].isNull()) {
      if (!root["voip"].isNull() && !root["voip"]["sip_server"].isNull()) {
        root_raw["voip"]["sip_server"] = root["voip"]["sip_server"];
      }
    }
    /* } voip */
    
    /* agora { */
    if (!root_raw["agora"].isNull() && !root_raw["agora"]["appkey"].isNull()) {
      if (!root["agora"].isNull() && !root["agora"]["appkey"].isNull()) {
        root_raw["agora"]["appkey"] = root["agora"]["appkey"];
      }
    }
    if (!root_raw["agora"].isNull() && !root_raw["agora"]["appsecret"].isNull()) {
      if (!root["agora"].isNull() && !root["agora"]["appsecret"].isNull()) {
        root_raw["agora"]["appsecret"] = root["agora"]["appsecret"];
      }
    }
    /* } agora */
    
    /* yunxin { */
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["appkey"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["appkey"].isNull()) {
        root_raw["yunxin"]["appkey"] = root["yunxin"]["appkey"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["appsecret"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["appsecret"].isNull()) {
        root_raw["yunxin"]["appsecret"] = root["yunxin"]["appsecret"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["create_url"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["create_url"].isNull()) {
        root_raw["yunxin"]["create_url"] = root["yunxin"]["create_url"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["update_url"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["update_url"].isNull()) {
        root_raw["yunxin"]["update_url"] = root["yunxin"]["update_url"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["data_dir"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["data_dir"].isNull()) {
        root_raw["yunxin"]["data_dir"] = root["yunxin"]["data_dir"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["vchat_low_energy"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["vchat_low_energy"].isNull()) {
        root_raw["yunxin"]["vchat_low_energy"] = root["yunxin"]["vchat_low_energy"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["vchat_delay_time"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["vchat_delay_time"].isNull()) {
        root_raw["yunxin"]["vchat_delay_time"] = root["yunxin"]["vchat_delay_time"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["vchat_aec"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["vchat_aec"].isNull()) {
        root_raw["yunxin"]["vchat_aec"] = root["yunxin"]["vchat_aec"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["vchat_ns"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["vchat_ns"].isNull()) {
        root_raw["yunxin"]["vchat_ns"] = root["yunxin"]["vchat_ns"];
      }
    }
    if (!root_raw["yunxin"].isNull() && !root_raw["yunxin"]["vchat_vad"].isNull()) {
      if (!root["yunxin"].isNull() && !root["yunxin"]["vchat_vad"].isNull()) {
        root_raw["yunxin"]["vchat_vad"] = root["yunxin"]["vchat_vad"];
      }
    }
    /* } yunxin */
    
    /* gs { */
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["gs_web_port"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["gs_web_port"].isNull()) {
        root_raw["gs"]["gs_web_port"] = root["gs"]["gs_web_port"];
      }
    }
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["gs_ws_port"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["gs_ws_port"].isNull()) {
        root_raw["gs"]["gs_ws_port"] = root["gs"]["gs_ws_port"];
      }
    }
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["maps_dir"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["maps_dir"].isNull()) {
        root_raw["gs"]["maps_dir"] = root["gs"]["maps_dir"];
      }
    }
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["patrol_scheme"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["patrol_scheme"].isNull()) {
        root_raw["gs"]["patrol_scheme"] = root["gs"]["patrol_scheme"];
      }
    }
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["runtime_data"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["runtime_data"].isNull()) {
        root_raw["gs"]["runtime_data"] = root["gs"]["runtime_data"];
      }
    }
    if (!root_raw["gs"].isNull() && !root_raw["gs"]["gs_srv_port"].isNull()) {
      if (!root["gs"].isNull() && !root["gs"]["gs_srv_port"].isNull()) {
        root_raw["gs"]["gs_srv_port"] = root["gs"]["gs_srv_port"];
      }
    }
    /* } gs */
    
    /* ptz { */
    if (!root_raw["ptz"].isNull() && !root_raw["ptz"]["ptz_user"].isNull()) {
      if (!root["ptz"].isNull() && !root["ptz"]["ptz_user"].isNull()) {
        root_raw["ptz"]["ptz_user"] = root["ptz"]["ptz_user"];
      }
    }
    if (!root_raw["ptz"].isNull() && !root_raw["ptz"]["ptz_psw"].isNull()) {
      if (!root["ptz"].isNull() && !root["ptz"]["ptz_psw"].isNull()) {
        root_raw["ptz"]["ptz_psw"] = root["ptz"]["ptz_psw"];
      }
    }
    if (!root_raw["ptz"].isNull() && !root_raw["ptz"]["ptz_login_timeout"].isNull()) {
      if (!root["ptz"].isNull() && !root["ptz"]["ptz_login_timeout"].isNull()) {
        root_raw["ptz"]["ptz_login_timeout"] = root["ptz"]["ptz_login_timeout"];
      }
    }
    /* } ptz */

    /* dbcom { */
    if (!root_raw["dbcom"].isNull() && !root_raw["dbcom"]["device_ip"].isNull()) {
      if (!root["dbcom"].isNull() && !root["dbcom"]["device_ip"].isNull()) {
        root_raw["dbcom"]["device_ip"] = root["dbcom"]["device_ip"];
      }
    }
    if (!root_raw["dbcom"].isNull() && !root_raw["dbcom"]["username"].isNull()) {
      if (!root["dbcom"].isNull() && !root["dbcom"]["username"].isNull()) {
        root_raw["dbcom"]["username"] = root["dbcom"]["username"];
      }
    }
    if (!root_raw["dbcom"].isNull() && !root_raw["dbcom"]["passwd"].isNull()) {
      if (!root["dbcom"].isNull() && !root["dbcom"]["passwd"].isNull()) {
        root_raw["dbcom"]["passwd"] = root["dbcom"]["passwd"];
      }
    }
    if (!root_raw["dbcom"].isNull() && !root_raw["dbcom"]["login_timeout"].isNull()) {
      if (!root["dbcom"].isNull() && !root["dbcom"]["login_timeout"].isNull()) {
        root_raw["dbcom"]["login_timeout"] = root["dbcom"]["login_timeout"];
      }
    }
    /* } dbcom */


    /* body_euler { */
    if (!root_raw["body_euler"].isNull() && !root_raw["body_euler"]["pitch_max"].isNull()) {
      if (!root["body_euler"].isNull() && !root["body_euler"]["pitch_max"].isNull()) {
        root_raw["body_euler"]["pitch_max"] = root["body_euler"]["pitch_max"];
      }
    }
    if (!root_raw["body_euler"].isNull() && !root_raw["body_euler"]["roll_max"].isNull()) {
      if (!root["body_euler"].isNull() && !root["body_euler"]["roll_max"].isNull()) {
        root_raw["body_euler"]["roll_max"] = root["body_euler"]["roll_max"];
      }
    }
    if (!root_raw["body_euler"].isNull() && !root_raw["body_euler"]["yaw_max"].isNull()) {
      if (!root["body_euler"].isNull() && !root["body_euler"]["yaw_max"].isNull()) {
        root_raw["body_euler"]["yaw_max"] = root["body_euler"]["yaw_max"];
      }
    }
    if (!root_raw["body_euler"].isNull() && !root_raw["body_euler"]["filter_constant"].isNull()) {
      if (!root["body_euler"].isNull() && !root["body_euler"]["filter_constant"].isNull()) {
        root_raw["body_euler"]["filter_constant"] = root["body_euler"]["filter_constant"];
      }
    }
    
    /* speed { */
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["forward_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["forward_max"].isNull()) {
        root_raw["speed"]["forward_max"] = root["speed"]["forward_max"];
      }
    }
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["backward_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["backward_max"].isNull()) {
        root_raw["speed"]["backward_max"] = root["speed"]["backward_max"];
      }
    }
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["angular_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["angular_max"].isNull()) {
        root_raw["speed"]["angular_max"] = root["speed"]["angular_max"];
      }
    }
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["forward_wheel_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["forward_wheel_max"].isNull()) {
        root_raw["speed"]["forward_wheel_max"] = root["speed"]["forward_wheel_max"];
      }
    }
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["backward_wheel_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["backward_wheel_max"].isNull()) {
        root_raw["speed"]["backward_wheel_max"] = root["speed"]["backward_wheel_max"];
      }
    }
    if (!root_raw["speed"].isNull() && !root_raw["speed"]["angular_wheel_max"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["angular_wheel_max"].isNull()) {
        root_raw["speed"]["angular_wheel_max"] = root["speed"]["angular_wheel_max"];
      }
    }

    if (!root_raw["speed"].isNull() && !root_raw["speed"]["battery_voltage_diff"].isNull()) {
      if (!root["speed"].isNull() && !root["speed"]["battery_voltage_diff"].isNull()) {
        root_raw["speed"]["battery_voltage_diff"] = root["speed"]["battery_voltage_diff"];
      }
    }
    /* } speed */
    
    /* router { */
    if (!root_raw["router"].isNull() && !root_raw["router"]["ip"].isNull()) {
      if (!root["router"].isNull() && !root["router"]["ip"].isNull()) {
        root_raw["router"]["ip"] = root["router"]["ip"];
      }
    }
    if (!root_raw["router"].isNull() && !root_raw["router"]["type"].isNull()) {
      if (!root["router"].isNull() && !root["router"]["type"].isNull()) {
        root_raw["router"]["type"] = root["router"]["type"];
      }
    }
    if (!root_raw["router"].isNull() && !root_raw["router"]["username"].isNull()) {
      if (!root["router"].isNull() && !root["router"]["username"].isNull()) {
        root_raw["router"]["username"] = root["router"]["username"];
      }
    }
    if (!root_raw["router"].isNull() && !root_raw["router"]["password"].isNull()) {
      if (!root["router"].isNull() && !root["router"]["password"].isNull()) {
        root_raw["router"]["password"] = root["router"]["password"];
      }
    }
    /* } router */
    
    /* ntp_server { */
    if (!root_raw["ntp"].isNull() && !root_raw["ntp"]["server"].isNull()) {
      if (!root["ntp"].isNull() && !root["ntp"]["server"].isNull()) {
        root_raw["ntp"]["server"] = root["ntp"]["server"];
      }
    }
    if (!root_raw["ntp"].isNull() && !root_raw["ntp"]["timezone"].isNull()) {
      if (!root["ntp"].isNull() && !root["ntp"]["timezone"].isNull()) {
        root_raw["ntp"]["timezone"] = root["ntp"]["timezone"];
      }
    }
    /* } ntp_server */
    
    /* abi { */
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["enable"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["enable"].isNull()) {
        root_raw["abi"]["enable"] = root["abi"]["enable"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["abi_id"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["abi_id"].isNull()) {
        root_raw["abi"]["abi_id"] = root["abi"]["abi_id"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["abi_key"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["abi_key"].isNull()) {
        root_raw["abi"]["abi_key"] = root["abi"]["abi_key"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["abi_version"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["abi_version"].isNull()) {
        root_raw["abi"]["abi_version"] = root["abi"]["abi_version"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["base_url"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["base_url"].isNull()) {
        root_raw["abi"]["base_url"] = root["abi"]["base_url"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["timestamp_url"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["timestamp_url"].isNull()) {
        root_raw["abi"]["timestamp_url"] = root["abi"]["timestamp_url"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["robotinfo_url"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["robotinfo_url"].isNull()) {
        root_raw["abi"]["robotinfo_url"] = root["abi"]["robotinfo_url"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["get_user_list_period_sec"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["get_user_list_period_sec"].isNull()) {
        root_raw["abi"]["get_user_list_period_sec"] = root["abi"]["get_user_list_period_sec"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["get_timestamp_period_sec"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["get_timestamp_period_sec"].isNull()) {
        root_raw["abi"]["get_timestamp_period_sec"] = root["abi"]["get_timestamp_period_sec"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["super_name"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["super_name"].isNull()) {
        root_raw["abi"]["super_name"] = root["abi"]["super_name"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["super_pwd"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["super_pwd"].isNull()) {
        root_raw["abi"]["super_pwd"] = root["abi"]["super_pwd"];
      }
    }
    if (!root_raw["abi"].isNull() && !root_raw["abi"]["post_event_url"].isNull()) {
      if (!root["abi"].isNull() && !root["abi"]["post_event_url"].isNull()) {
        root_raw["abi"]["post_event_url"] = root["abi"]["post_event_url"];
      }
    }
    /* } abi */
    
    /* id_card { */
    if (!root_raw["id_card"].isNull() && !root_raw["id_card"]["tcp_server_port"].isNull()) {
      if (!root["id_card"].isNull() && !root["id_card"]["tcp_server_port"].isNull()) {
        root_raw["id_card"]["tcp_server_port"] = root["id_card"]["tcp_server_port"];
      }
    }
    /* } id_card */
    
    /* udock { */
    if (!root_raw["udock"].isNull() && !root_raw["udock"]["dock_wall_width"].isNull()) {
      if (!root["udock"].isNull() && !root["udock"]["dock_wall_width"].isNull()) {
        root_raw["udock"]["dock_wall_width"] = root["udock"]["dock_wall_width"];
      }
    }
    if (!root_raw["udock"].isNull() && !root_raw["udock"]["dock_1_0"].isNull()) {
      if (!root["udock"].isNull() && !root["udock"]["dock_1_0"].isNull()) {
        root_raw["udock"]["dock_1_0"] = root["udock"]["dock_1_0"];
      }
    }
    /* } udock */
    
    /* battery { */
    if (!root_raw["battery"].isNull() && !root["battery"].isNull()) {
      if (!root_raw["battery"]["low_battery_warn"].isNull() && !root["battery"]["low_battery_warn"].isNull()) {
        root_raw["battery"]["low_battery_warn"] = root["battery"]["low_battery_warn"].asInt();
      }
      if (!root_raw["battery"]["low_battery_charge"].isNull() && !root["battery"]["low_battery_charge"].isNull()) {
        root_raw["battery"]["low_battery_charge"] = root["battery"]["low_battery_charge"].asInt();
      }
    }
    /* } battery */

    if ((!root_raw["imsi"].isNull()) && (!root_raw["imsi"]["open_red_blue_light"].isNull())) {
      if ((!root["imsi"].isNull()) && (!root["imsi"]["open_red_blue_light"].isNull())) {
        root_raw["imsi"]["open_red_blue_light"] = root["imsi"]["open_red_blue_light"];
      }
    }
    
    /* http_debug { */
    if (!root_raw["http_debug"].isNull() && !root_raw["http_debug"]["enable"].isNull()) {
      if (!root["http_debug"].isNull() && !root["http_debug"]["enable"].isNull()) {
        root_raw["http_debug"]["enable"] = root["http_debug"]["enable"];
      }
    }
    /* } http_debug */
    
    /* vision { */
    if (!root_raw["vision"].isNull() && !root_raw["vision"]["host"].isNull()) {
      if (!root["vision"].isNull() && !root["vision"]["host"].isNull()) {
        root_raw["vision"]["host"] = root["vision"]["host"];
      }
    }

    if (!root_raw["vision"].isNull() && !root_raw["vision"]["port"].isNull()) {
      if (!root["vision"].isNull() && !root["vision"]["port"].isNull()) {
        root_raw["vision"]["port"] = root["vision"]["port"];
      }
    }
    /* } vision */

    /* obstacle { */
    if (!root_raw["obstacle"].isNull() && !root_raw["obstacle"]["mode"].isNull()) {
      if (!root["obstacle"].isNull() && !root["obstacle"]["mode"].isNull()) {
        root_raw["obstacle"]["mode"] = root["obstacle"]["mode"];
      }
    }

    if (!root_raw["obstacle"].isNull() && !root_raw["obstacle"]["timeout"].isNull()) {
      if (!root["obstacle"].isNull() && !root["obstacle"]["timeout"].isNull()) {
        root_raw["obstacle"]["timeout"] = root["obstacle"]["timeout"];
      }
    }

    if (!root_raw["obstacle"].isNull() && !root_raw["obstacle"]["timeout_strategy"].isNull()) {
      if (!root["obstacle"].isNull() && !root["obstacle"]["timeout_strategy"].isNull()) {
        root_raw["obstacle"]["timeout_strategy"] = root["obstacle"]["timeout_strategy"];
      }
    }
    /* } obstacle */

    /* nav { */
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["obs_mode"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["obs_mode"].isNull()) {
        root_raw["nav"]["obs_mode"] = root["nav"]["obs_mode"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["cycle_enable"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["cycle_enable"].isNull()) {
        root_raw["nav"]["cycle_enable"] = root["nav"]["cycle_enable"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["cycle_dst"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["cycle_dst"].isNull()) {
        root_raw["nav"]["cycle_dst"] = root["nav"]["cycle_dst"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["without_charge_path"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["without_charge_path"].isNull()) {
        root_raw["nav"]["without_charge_path"] = root["nav"]["without_charge_path"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["task_start_timeout"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["task_start_timeout"].isNull()) {
        root_raw["nav"]["task_start_timeout"] = root["nav"]["task_start_timeout"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["task_running_timeout"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["task_running_timeout"].isNull()) {
        root_raw["nav"]["task_running_timeout"] = root["nav"]["task_running_timeout"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["without_merge_path"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["without_merge_path"].isNull()) {
        root_raw["nav"]["without_merge_path"] = root["nav"]["without_merge_path"];
      }
    }
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["auto_charge_self"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["auto_charge_self"].isNull()) {
        root_raw["nav"]["auto_charge_self"] = root["nav"]["auto_charge_self"];
      }
    }

    if (!root_raw["nav"].isNull() && !root_raw["nav"]["auto_charge_confirm_timeout"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["auto_charge_confirm_timeout"].isNull()) {
        root_raw["nav"]["auto_charge_confirm_timeout"] = root["nav"]["auto_charge_confirm_timeout"];
      }
    }

    if (!root_raw["nav"].isNull() && !root_raw["nav"]["auto_charge_battery"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["auto_charge_battery"].isNull()) {
        root_raw["nav"]["auto_charge_battery"] = root["nav"]["auto_charge_battery"];
      }
    }
  
    if (!root_raw["nav"].isNull() && !root_raw["nav"]["anti_drop_enable"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["anti_drop_enable"].isNull()) {
        root_raw["nav"]["anti_drop_enable"] = root["nav"]["anti_drop_enable"];
      }
    }

    if (!root_raw["nav"].isNull() && !root_raw["nav"]["tsp_enable"].isNull()) {
      if (!root["nav"].isNull() && !root["nav"]["tsp_enable"].isNull()) {
        root_raw["nav"]["tsp_enable"] = root["nav"]["tsp_enable"];
      }
    }
    /* } nav */

    Json::StyledWriter fw;
    std::string json = fw.write(root_raw);
    int fd = open(ATRIS_CONFIG_PART_PATH, O_WRONLY | O_TRUNC | O_CREAT,0777);
    if(fd < 0) {
      log_error("%s open config part file failed.", __FUNCTION__);
      return;
    }
    
    std::size_t ret = write(fd, json.c_str(), json.size());
    if(ret != json.size()){
      log_error("%s save config part file error.", __FUNCTION__);
    } else {
      remove(ATRIS_CONFIG_PATH);
      rename(ATRIS_CONFIG_PART_PATH, ATRIS_CONFIG_PATH);
    }
    close(fd);
  }
}

