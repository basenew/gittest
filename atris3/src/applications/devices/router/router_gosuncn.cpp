#include "router_gosuncn.h"
#include "config/config.h"
#include "utils/utils.h"


bool Gosuncn::IsMobileWebConnected()
{
    bool ret = false;
    Json::Value result;

    int res = GetMobileWebStatus(result);
    if (res == 0) {
        if ((!result["status"].isNull()) &&
            (!result["status"]["card"].isNull()) &&
            (result["status"]["card"].isString())) {
            std::string card_status = result["status"]["card"].asString();
            if (card_status == "READY") {
                ret = true;
            } else {
                ret = false;
            }
        }
    }

    return ret;
}

int Gosuncn::GetMobileWebSignal(int *signal)
{
    if (signal == nullptr) {
        return -1;
    }

    MobileWebInfo info;
    if (GetMobileWebInfo(&info) == 0) {
        *signal = info.signal;
    }

    return 0;
}

int Gosuncn::GetMobileWebInfo(MobileWebInfo *info)
{
    if (info == nullptr) {
        return -1;
    }

    Json::Value result;
    int ret = GetMobileWebStatus(result);
    if (ret == 0) {
        if ((!result["cell_info"].isNull()) &&
            (!result["cell_info"]["id"].isNull())&&
            (result["cell_info"]["id"].isString())) {
            info->cellid = result["cell_info"]["id"].asString();
        }
        if (!result["status"].isNull()) {
            if ((!result["status"]["iccid"].isNull()) &&
                (result["status"]["iccid"].isString())) {
                info->iccid = result["status"]["iccid"].asString();
            }
            if ((!result["status"]["imei"].isNull()) &&
                (result["status"]["imei"].isString())) {
                info->imei = result["status"]["imei"].asString();
            }
            if ((!result["status"]["imsi"].isNull()) &&
                (result["status"]["imsi"].isString())) {
                info->imsi = result["status"]["imsi"].asString();
            }
            if ((!result["status"]["signal"].isNull())  &&
                (result["status"]["signal"].isString())) {
                std::string signal = result["status"]["signal"].asString();
                if ((!signal.empty()) || (signal != "unknow")) {
                    info->signal = atoi(signal.c_str());
                }
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

int Gosuncn::GetMobileWebMac(std::string &mac)
{
    std::string session = "";
    int ret = -1;

    ret = GetSessionLogin(session);
    if (ret != 0) {
        return -2;
    }

    std::string url = Config::get_instance()->router_ip + "/ubus/network.device.status";
    int id = 86;
    Json::Value result;

    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;
    params.append(session);
    params.append("network.device");
    params.append("status");

    params_string = writer.write(params);
    std::string status_string = "\"status\"";
    int len = status_string.length();
    std::string::size_type position = params_string.find(status_string);
    if (position != std::string::npos) {
        params_string.insert(position+len, ",{}");
    }

    ret = HttpHandleRouter(url, id, params_string, result);
    if (ret == 0) {
        if ((!result["usb0"].isNull()) &&
            (!result["usb0"]["macaddr"].isNull()) &&
            (result["usb0"]["macaddr"].isString())) {
            mac = result["usb0"]["macaddr"].asString();
        }
    }

    return ret;
}

int Gosuncn::GetMobileWebStatus(Json::Value &result)
{
    std::string session = "";
    int ret = -1;

    ret = GetSessionLogin(session);
    if (ret != 0) {
        return -2;
    }

    std::string url = Config::get_instance()->router_ip + "/ubus/network.interface.dump";
    int id = 193;
    Json::Value value;
    
    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;
    params.append(session);
    params.append("pgmm");
    params.append("status");

    params_string = writer.write(params);
    std::string status_string = "\"status\"";
    int len = status_string.length();
    std::string::size_type position = params_string.find(status_string);
    if (position != std::string::npos) {
        params_string.insert(position+len, ",{}");
    }

    ret = HttpHandleRouter(url, id, params_string, value);
    if (ret == 0) {
       result = value;
    }

    return ret;
}

int Gosuncn::GetSessionLogin(std::string &session_login)
{
    std::string url = Config::get_instance()->router_ip + "/ubus/session.login";
 
    Json::Value params;
    Json::FastWriter writer;
    std::string params_string;
    Json::Value login_value;

    login_value["username"] = Config::get_instance()->router_user;
    login_value["password"] = Config::get_instance()->router_psw;
    login_value["timeout"] = 300;

    params.append("00000000000000000000000000000000");
    params.append("session");
    params.append("login");
    params.append(login_value);
    params_string = writer.write(params);

    Json::Value result;
    int id = 23;
    int ret = HttpHandleRouter(url, id, params_string, result);
    if (ret == 0) {
        if ((!result["ubus_rpc_session"].isNull()) &&
            (result["ubus_rpc_session"].isString())) {
            session_login = result["ubus_rpc_session"].asString();
        } else {
            ret = -1;
        }
    } 
   
    return ret;
}

int Gosuncn::HttpHandleRouter(const std::string &url, int id, const std::string &params, Json::Value &result)
{
    Json::Value json_value;
    Json::Value login;
    Json::Value json_params;
    Json::FastWriter writer;
    std::string content;

    std::vector<std::string> http_header;
    std::string resp;
    Json::Reader reader;
    Json::Value root;


    json_value["jsonrpc"] = "2.0";
    json_value["id"] = id,
    json_value["method"] = "call";
    if (reader.parse(params.c_str(), json_params)) {
        json_value["params"] = json_params;
    }
    content = writer.write(json_value);

    int times = 3;
    int ret = -1;

    while (times > 0) {
        times --;
        http_header.clear();
        http_header.push_back("Content-Type:application/json");
        int res = Utils::get_instance()->http_post(url, content, resp, http_header, kHttpTimeoutSec);
        if (res == 0) {
            if (reader.parse(resp.c_str(), root)) {
                if (!root["result"].isNull()) {
                    int sz = root["result"].size();
                    for (int i = 0; i < sz; ++i) {
                        if (root["result"][i].isObject()) {
                            result = root["result"][i];
                        } 
                    }
                    ret = 0;
                    break;
                } else {
                    ret = -1;
                }
            } else {
                ret = -2;
            }
        } 
    }

    return ret;
}