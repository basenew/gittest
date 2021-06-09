#ifndef CHASSIS_UPGRADE_ERROR_CODE_H__
#define CHASSIS_UPGRADE_ERROR_CODE_H__

#include <iostream>

enum CHASSIS_UPGRADE_ERR_CODE
{
	CHASSIS_UPGRADE_SUCCESS       = 0,
    CHASSIS_WEBPOST_FAILED        = 102,
    CHASSIS_UPGRADE_TIMEOUT       = 256
};

const char* get_chassis_upgrade_err_msg(int err); // get verbose translate error message string

#endif