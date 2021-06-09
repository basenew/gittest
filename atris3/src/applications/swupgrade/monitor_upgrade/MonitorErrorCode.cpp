#include "include/MonitorUpgradeErrorCode.h"
#include "log/log.h"
#include <unordered_map>
#include <map>

static std::map<int, const char*> code2msg = 
{
	{MONITOR_UPGRADE_SUCCESS,                               "monitor upgrade success"},
    {MONITOR_WEBPOST_FAILED,                                "monitor post download request failed"},
    {MONITOR_UPGRADE_TIMEOUT,                               "monitor upgrade timeout reached"}
};

const char* get_monitor_upgrade_err_msg(int err)
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