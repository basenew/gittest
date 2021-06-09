#ifndef MONITOR_UPGRADE_ERROR_CODE_H__
#define MONITOR_UPGRADE_ERROR_CODE_H__

#include <iostream>

enum MONITOR_UPGRADE_ERR_CODE
{
	MONITOR_UPGRADE_SUCCESS       = 0,
    MONITOR_WEBPOST_FAILED        = 102,
    MONITOR_UPGRADE_TIMEOUT       = 256
};

const char* get_monitor_upgrade_err_msg(int err); // get verbose translate error message string

#endif