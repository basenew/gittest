#ifndef ATRIS_DEVICE_ROUTER_GOSUNCN_H_
#define ATRIS_DEVICE_ROUTER_GOSUNCN_H_

#include <string>
#include <json/json.h>
#include "router_comm.h"

class Gosuncn : public IRourer {
public:
    ~Gosuncn() {}
    virtual void InitSetting() {}
    virtual bool IsMobileWebConnected();
    virtual int GetMobileWebSignal(int *signal);
    virtual int GetMobileWebInfo(MobileWebInfo *info);
    

private:
    int GetMobileWebMac(std::string &mac);
    int GetMobileWebStatus(Json::Value &result);
    int HttpHandleRouter(const std::string &url, int id, const std::string &params, Json::Value &result);
    int GetSessionLogin(std::string &session_login);

private:
    static const int kHttpTimeoutSec = 30;

};

#endif