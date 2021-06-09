#include "include/ChassisUpgradeErrorCode.h"
#include "log/log.h"
#include <unordered_map>
#include <map>

static std::map<int, const char*> code2msg = 
{
	{CHASSIS_UPGRADE_SUCCESS,                               "chassis upgrade success"},
    {CHASSIS_WEBPOST_FAILED,                                "chassis post download request failed"},
    {CHASSIS_UPGRADE_TIMEOUT,                               "chassis upgrade timeout reached"}
};

const char* get_chassis_upgrade_err_msg(int err)
{
    auto it = code2msg.find(err);
    if (it != code2msg.end()){
        log_info("err:%d msg:%s", err, it->second);
        return it->second;
    }
    else{
        log_error("err:%d is unknown err", err);
        return "unknow error";
    }
}