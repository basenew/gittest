#ifndef ATRIS_DEVICE_ROUTER_COMM_H_
#define ATRIS_DEVICE_ROUTER_COMM_H_

#include <string>

struct  MobileWebInfo {
    int signal;
    std::string iccid;
    std::string imsi;
    std::string imei;
    std::string cellid;
    std::string mac;
};

class IRourer {
public:
    virtual ~IRourer() {}
    virtual void InitSetting() = 0;
    virtual bool IsMobileWebConnected() = 0;
    virtual int GetMobileWebInfo(MobileWebInfo *info) = 0;
    virtual int GetMobileWebSignal(int *signal) = 0;
};

#endif