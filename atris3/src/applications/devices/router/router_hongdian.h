#ifndef ATRIS_DEVICE_ROUTER_HONG_DIAN_H_
#define ATRIS_DEVICE_ROUTER_HONG_DIAN_H_

#include <string>
#include <json/json.h>
#include "router_comm.h"

class HongDian : public IRourer {
public:
    ~HongDian() {}
    virtual void InitSetting();
    virtual bool IsMobileWebConnected();
    virtual int GetMobileWebSignal(int *signal);
    virtual int GetMobileWebInfo(MobileWebInfo *info);

private:
    int GetMobileWebMac(std::string &mac);
    int GetModemInfo(Json::Value &result);
    int HttpHandleRouter(const std::string &content, Json::Value &result);
    std::string GetCurrentYear();

private:
    static const int kHttpTimeoutSec = 30;

};

#endif