#include "router_hongdian.h"

#include "config/config.h"
#include "utils/utils.h"
#include "log/log.h"
#include "imemory/atris_imemory_api.h"
void HongDian::InitSetting()
{
    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string robot_sn = std::string(shmrbt.robot.sn);
    if (robot_sn.length() != 17) {
        log_error("robotsn[%s] is not right", robot_sn.c_str());
        return;
    }

    std::string robot_type(robot_sn, 2, 1);
    std::string robot_lang = "";
    if (robot_sn.compare(5, 1, "2") == 0) {
        robot_lang = "0";
    } else {
        robot_lang = "1";
    }
    std::string current_year = GetCurrentYear();
    std::string robot_tail (robot_sn, 13, 4);
    std::string ssid = "Atris-" + robot_type + robot_lang + current_year[3] + robot_tail;

    params["task_wlan_conf"] = "WLAN_CONF_REQ";
    params["wlan_mode"] = "ap";
    params["wlan_ssid"] = ssid;
    params["wlan_bandwidth"] = "20MHz";
    params["wlan_security"] = "wpa2";
    params["wlan_encryption"] = "aes";
    params["wlan_passwd"] = "ubtubt123";
    params["wlan_lifetime"] = 1200;
    params_string = writer.write(params);

    Json::Value result;
    HttpHandleRouter(params_string, result);
}

bool HongDian::IsMobileWebConnected()
{
    Json::Value info;
    bool ret = false;
    int res = GetModemInfo(info);
    if (res == 0) {
        if ((!info["modem_connect_status"].isNull()) &&
            (info["modem_connect_status"].isString())) {
            std::string status = info["modem_connect_status"].asString();
            if (status == "connected") {
                ret = true;
            } 
        }
    }

    return ret;
}

int HongDian::GetMobileWebSignal(int *signal)
{
    if (signal == nullptr) {
        return -1;
    }

    int ret = -1;
    Json::Value modem_info;
    int res = GetModemInfo(modem_info);
    if (res == 0) {
        if ((!modem_info["modem_csq"].isNull()) &&
            (modem_info["modem_csq"].isString())) {
            std::string csq_string = modem_info["modem_csq"].asString();
            if (!csq_string.empty()) {
                int csq = atoi(csq_string.c_str());
                *signal = -113 + (2*csq);
                ret = 0;
            }
        }
    }

    return ret;
}

int HongDian::GetMobileWebInfo(MobileWebInfo *info)
{
    Json::Value modem_info;
  
    int ret = GetModemInfo(modem_info);
    if (ret == 0) {
        if ((!modem_info["modem_csq"].isNull()) &&
            (modem_info["modem_csq"].isString())) {
            std::string csq_string = modem_info["modem_csq"].asString();
            if (!csq_string.empty()) {
                int csq = atoi(csq_string.c_str());
                info->signal = -113 + (2*csq);
            }
        }

        if ((!modem_info["modem_imei"].isNull()) &&
            (modem_info["modem_imei"].isString())) {
            info->imei = modem_info["modem_imei"].asString();
        }

        if (!modem_info["modem_sim1imsi"].isNull()) {
            if (modem_info["modem_sim1imsi"].isString()) {
                info->imsi = modem_info["modem_sim1imsi"].asString();
            }
        } else if (!modem_info["modem_sim2imsi"].isNull()) {
            if (modem_info["modem_sim2imsi"].isString()) {
                info->imsi = modem_info["modem_sim2imsi"].asString();
            }
        }

        if (!modem_info["modem_sim1iccid"].isNull()) {
            if (modem_info["modem_sim1iccid"].isString()) {
                info->iccid = modem_info["modem_sim1iccid"].asString();
            }
        } else if (!modem_info["modem_sim2iccid"].isNull()) {
            if (modem_info["modem_sim2iccid"].isString()) {
                info->iccid = modem_info["modem_sim2iccid"].asString();
            }
        }

        if (!modem_info["modem_sim1cellid"].isNull()) {
            if (modem_info["modem_sim1cellid"].isString()) {
                info->cellid = modem_info["modem_sim1cellid"].asString();
            }
        } else if (!modem_info["modem_sim2cellid"].isNull()) {
            if (modem_info["modem_sim2cellid"].isString()) {
                info->cellid = modem_info["modem_sim2cellid"].asString();
            }
        }
    }

    std::string mac;
    ret = GetMobileWebMac(mac);
    if (ret == 0) {
        info->mac = mac;
    }

    return ret;
}

int HongDian::GetMobileWebMac(std::string &mac)
{
    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;

    params["task_get_wlan_info"] = "WLAN_INFO_REQ";
    params["router_vendor"] = "hongdian";
    params_string = writer.write(params);

    int ret = -1;
    Json::Value result;
    int res = HttpHandleRouter(params_string, result);
    if (res == 0) {
        if (result["wlan_macid"].isString()) {
            mac = result["wlan_macid"].asString();
            ret = 0;
        } 
    }

    return ret;
}

int HongDian::GetModemInfo(Json::Value &info)
{
    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;

    params["task_get_modem_info"] = "MODEM_INFO_REQ";
    params["router_vendor"] = "hongdian";
    params_string = writer.write(params);

    Json::Value result;
    int ret = HttpHandleRouter(params_string, result);
    if (ret == 0) {
        info = result;
    }

    return ret;
}

int HongDian::HttpHandleRouter(const std::string &content, Json::Value &result)
{
    std::string url = Config::get_instance()->router_ip + ":9999";
    std::vector<std::string> http_header;
    std::string resp;
    Json::Reader reader;
    Json::Value root;

    int times = 3;
    int ret = -1;

    while (times > 0) {
        times --;
        http_header.clear();
        http_header.push_back("Content-Type:application/json");
        int res = Utils::get_instance()->http_post(url, content, resp, http_header, kHttpTimeoutSec);
        if (res == 0) {
            if (reader.parse(resp.c_str(), root)) {
                if (!root["error_resp"].isNull()) {
                    ret = -1;
                    continue;
                } else {
                    result = root;
                    ret = 0;
                }
            } else {
                ret = -2;
            }
        } 
    }

    return ret;
}

std::string  HongDian::GetCurrentYear()
{
    ros::Time now = ros::Time::now();
    time_t time = now.toSec();

    struct tm *t = localtime(&time);
    std::string  current_year = std::to_string(t->tm_year+1900);

    return current_year;
}