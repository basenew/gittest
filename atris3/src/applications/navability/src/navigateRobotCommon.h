#ifndef NAVIGATE_ROBOT_COMMON_H_H_
#define NAVIGATE_ROBOT_COMMON_H_H_
/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
  File Name     : navigateRobotCommon.h
  Version       : Initial Draft
  Author        : marty.gong@ubtrobot.com
  Created       : 2020/3/25
  Last Modified :
  Description   : navigateRobotCommon.h

******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*

typedef enum 
{
    ERR_UNKNOW = -1,
    ERR_OK = 0,
    ERR_FAIL,

    ERR_ALLOC_MEM_FAIL,
    ERR_STAT_FILE,

    ERR_INVALID_LOCATE_TYPE,
    ERR_INVALID_PARAM          = 5,
    ERR_INVALID_JSON,
    ERR_INVALID_HOST_PORT,
    ERR_INVALID_OP,
    ERR_INVALID_URL,
    ERR_INVALID_VALUE          = 10,
    ERR_INVALID_FIELD,

	ERR_FILE_NOT_EXIST,
    ERR_MAP_ALREADY_LOADING,

    ERR_CREATE_THREAD_FAIL,
    ERR_OPEN_REPEAT            = 15,

    ERR_HTTP_GET_FAIL,
    ERR_HTTP_POST_FAIL,
    ERR_HTTP_GET_FILE_FAIL,
    ERR_HTTP_POST_FILE_FAIL,
    ERR_RSP_FAIL               = 20,
    ERR_OPEN_FILE_FAIL,
    ERR_READ_FILE_FAIL,
    ERR_PARSE_SCHEME_FAIL,
    ERR_TRANSFER_FILE_FAIL,

    ERR_NONE_MAP,
    ERR_NONE_PATROL            = 26,
    ERR_NONE_SCHEME,
    ERR_ON_LOCATING,
    ERR_ON_RUNNING,
    ERR_SCHEME_NOT_FOUND,
    ERR_SCHEME_OVERTIME        = 31,
    ERR_SCHEME_SAVE_FAIL,
    ERR_MAP_NOT_FOUND,
    ERR_REMOVE_FILE_FAIL,
    ERR_NONE_GPS,
	ERR_ALREADY_RUNNING        = 36,
    ERR_MAP_NOT_USING,
    ERR_MAP_MUST_EQ_USING,
	ERR_RENAME_USING_MAP,
	ERR_REMOVE_USING_MAP,
    ERR_DEVICE_GS_ERROR        = 41,
    ERR_NOT_LOCATED,
    ERR_NOT_RUNNING,
    ERR_LOCATE_FAIL,
    ERR_LOCATING,
    ERR_NAVIGATING             = 46,
    ERR_TASK_NOT_READY,
    ERR_NOT_LOCATED_BY_SPC_MAP,
	ERR_PATH_NOT_MERGED,
	ERR_NONE_CHARGE_SCHEME,
	ERR_CHARGE_POINT_NOT_SET   = 51,
	ERR_TASK_OP_CONFLICT,
	ERR_LOCATED_OTHER_MAP,
	ERR_SERVICE_NOT_READY,
	ERR_NONE_NAV_STATUS,
    ERR_SCHEME_DISABLED        = 56,
    ERR_CHARGE_DISABLED,
    ERR_MAP_NONE_PATH,
    ERR_TIMEOUT,


    ERR_RELOCATE_FINISH        = 1000,
    ERR_RELOCATING,
    ERR_RELOCATE_FAILED,

    ERR_NAVI_REACHED           = 3000,
    ERR_NAVI_PAUSING,
    ERR_NAVI_PLANNING,
    ERR_NAVI_PATH_NOT_SAFE,
    ERR_NAVI_PATH_INVALID,
    ERR_NAVI_PATH_PLANNING_FAILED,
    ERR_NAVI_OBSTACLE_IN_FRONT,
    ERR_NAVI_AVOIDING_OBSTACLE,
    ERR_NAVI_FOLLOWING_PATH,
    ERR_NAVI_FOLLOW_PATH_FINISHED,
    ERR_NAVI_NAVIGATING,
    ERR_NAVI_GOAL_POINT_UNREACHABLE,
    ERR_NAVI_GOAL_UNREACHED,
    ERR_NAVI_LOCALIZATION_LOST,

    ERR_ROTATING              = 4000,
    ERR_CANNOT_ROTATE,
    RER_ROTATE_FINISHED,
    ERR_ROTATE_FAILED,
}ROBOT_ERR_CODE;

*/

typedef enum 
{
    NAV_ERR_UNKNOW = -1,
    NAV_ERR_OK = 0,
    NAV_ERR_FAIL,

    NAV_ERR_ALLOC_MEM_FAIL,
    NAV_ERR_STAT_FILE,

    NAV_ERR_INVALID_LOCATE_TYPE,
    NAV_ERR_INVALID_PARAM          = 5,
    NAV_ERR_INVALID_JSON,
    NAV_ERR_INVALID_HOST_PORT,
    NAV_ERR_INVALID_OP,
    NAV_ERR_INVALID_URL,
    NAV_ERR_INVALID_VALUE          = 10,
    NAV_ERR_INVALID_FIELD,

	NAV_ERR_FILE_NOT_EXIST,
    NAV_ERR_MAP_ALREADY_LOADING,

    NAV_ERR_CREATE_THREAD_FAIL,
    NAV_ERR_OPEN_REPEAT            = 15,

    NAV_ERR_HTTP_GET_FAIL,
    NAV_ERR_HTTP_POST_FAIL,
    NAV_ERR_HTTP_GET_FILE_FAIL,
    NAV_ERR_HTTP_POST_FILE_FAIL,
    NAV_ERR_RSP_FAIL               = 20,
    NAV_ERR_OPEN_FILE_FAIL,
    NAV_ERR_READ_FILE_FAIL,
    NAV_ERR_PARSE_SCHEME_FAIL,
    NAV_ERR_TRANSFER_FILE_FAIL,

    NAV_ERR_NONE_MAP,
    NAV_ERR_NONE_PATROL            = 26,
    NAV_ERR_NONE_SCHEME,
    NAV_ERR_ON_LOCATING,
    NAV_ERR_ON_RUNNING,
    NAV_ERR_SCHEME_NOT_FOUND,
    NAV_ERR_SCHEME_OVERTIME        = 31,
    NAV_ERR_SCHEME_SAVE_FAIL,
    NAV_ERR_MAP_NOT_FOUND,
    NAV_ERR_REMOVE_FILE_FAIL,
    NAV_ERR_NONE_GPS,
	NAV_ERR_ALREADY_RUNNING        = 36,
    NAV_ERR_MAP_NOT_USING,
    NAV_ERR_MAP_MUST_EQ_USING,
	NAV_ERR_RENAME_USING_MAP,
	NAV_ERR_REMOVE_USING_MAP,
    NAV_ERR_DEVICE_GS_ERROR        = 41,
    NAV_ERR_NOT_LOCATED,
    NAV_ERR_NOT_RUNNING,
    NAV_ERR_LOCATE_FAIL,
    NAV_ERR_LOCATING,
    NAV_ERR_NAVIGATING             = 46,
    NAV_ERR_TASK_NOT_READY,
    NAV_ERR_NOT_LOCATED_BY_SPC_MAP,
	NAV_ERR_PATH_NOT_MERGED,
	NAV_ERR_NONE_CHARGE_SCHEME,
	NAV_ERR_CHARGE_POINT_NOT_SET   = 51,
	NAV_ERR_TASK_OP_CONFLICT,
	NAV_ERR_LOCATED_OTHER_MAP,
	NAV_ERR_SERVICE_NOT_READY,
	NAV_ERR_NONE_NAV_STATUS,
    NAV_ERR_SCHEME_DISABLED        = 56,
    NAV_ERR_CHARGE_DISABLED,
    NAV_ERR_MAP_NONE_PATH,
    NAV_ERR_TIMEOUT,


    NAV_ERR_RELOCATE_FINISH        = 1000,
    NAV_ERR_RELOCATING,
    NAV_ERR_RELOCATE_FAILED,

    NAV_ERR_NAVI_REACHED           = 3000,
    NAV_ERR_NAVI_PAUSING,
    NAV_ERR_NAVI_PLANNING,
    NAV_ERR_NAVI_PATH_NOT_SAFE,
    NAV_ERR_NAVI_PATH_INVALID,
    NAV_ERR_NAVI_PATH_PLANNING_FAILED,
    NAV_ERR_NAVI_OBSTACLE_IN_FRONT,
    NAV_ERR_NAVI_AVOIDING_OBSTACLE,
    NAV_ERR_NAVI_FOLLOWING_PATH,
    NAV_ERR_NAVI_FOLLOW_PATH_FINISHED,
    NAV_ERR_NAVI_NAVIGATING,
    NAV_ERR_NAVI_GOAL_POINT_UNREACHABLE,
    NAV_ERR_NAVI_GOAL_UNREACHED,
    NAV_ERR_NAVI_LOCALIZATION_LOST,

    NAV_ERR_ROTATING              = 4000,
    NAV_ERR_CANNOT_ROTATE,
    NAV_RER_ROTATE_FINISHED,
    NAV_ERR_ROTATE_FAILED,
}ROBOT_ERR_CODE;
/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/




#endif