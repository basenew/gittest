/*
* File      : gs_api_err_code.h
*
* Change Logs:
* Date           Author       Notes
* 2019-08-14     yangzou
*/
#include <map>
#include "nav_error_code.h"
#include "log/log.h"

static std::map<std::string, ERR_CODE> gs_err_code =
{
    {std::string("MAP_NOT_FOUND")                           ,ERR_GS_MAP_NOT_FOUND                       },
    {std::string("HTTP_REQUEST_ERROR")                      ,ERR_GS_HTTP_REQUEST_ERROR                  },
    {std::string("OPERATE_DATA_FAILED")                     ,ERR_GS_OPERATE_DATA_FAILED                 },
    {std::string("PATH_NOT_FOUND")                          ,ERR_GS_PATH_NOT_FOUND                      },
    {std::string("SYSTEM_NOT_HEALTHY")                      ,ERR_GS_SYSTEM_NOT_HEALTHY                  },
    {std::string("LOCALIZATION_NOT_READY")                  ,ERR_GS_LOCALIZATION_NOT_READY              },
    {std::string("MOVEBASE_NOT_READY")                      ,ERR_GS_MOVEBASE_NOT_READY                  },
    {std::string("LOCALIZATION_FILE_INVALID")               ,ERR_GS_LOCALIZATION_FILE_INVALID           },
    {std::string("LOCALIZATION_PARAM_INVALID")              ,ERR_GS_LOCALIZATION_PARAM_INVALID          },
    {std::string("MOVEBASE_USBKEY_INVALID")                 ,ERR_GS_MOVEBASE_USBKEY_INVALID             },
    {std::string("SERVICE_CALLING_FAILED")                  ,ERR_GS_SERVICE_CALLING_FAILED              },
    {std::string("INIT_POINT_NOT_FOUND")                    ,ERR_GS_INIT_POINT_NOT_FOUND                },
    {std::string("NOT_INITIALIZE")                          ,ERR_GS_NOT_INITIALIZE                      },
    {std::string("SCAN_MAP_PROGRAM_ALREADY_STOPPED")        ,ERR_GS_SCAN_MAP_PROGRAM_ALREADY_STOPPED    },
    {std::string("RECORD_PATH_PROGRAM_STATUS_ERROR")        ,ERR_GS_RECORD_PATH_PROGRAM_STATUS_ERROR    },
    {std::string("RECORD_PATH_PROGRAM_ALREADY_STOPPED")     ,ERR_GS_RECORD_PATH_PROGRAM_ALREADY_STOPPED },
    {std::string("SERVICE_CALLING_ERROR")                   ,ERR_GS_SERVICE_CALLING_ERROR               },
    {std::string("PATH_ALREADY_IN_USED")                    ,ERR_GS_PATH_ALREADY_IN_USED                },
    {std::string("JSON_FORMAT_ERROR")                       ,ERR_GS_JSON_FORMAT_ERROR                   },
    {std::string("PARSE_PATH_FILE_ERROR")                   ,ERR_GS_PARSE_PATH_FILE_ERROR               },
    {std::string("SAVE_PATH_ERROR")                         ,ERR_GS_SAVE_PATH_ERROR                     },
    {std::string("TM_TASK_QUEUE_IS_EMPTY")                  ,ERR_GS_TM_TASK_QUEUE_IS_EMPTY              },
    {std::string("TM_TASK_STATUS_IS_ERROR")                 ,ERR_GS_TM_TASK_STATUS_IS_ERROR             },
    {std::string("TM_TASK_QUEUE_PARAM_ERROR")               ,ERR_GS_TM_TASK_QUEUE_PARAM_ERROR           },
    {std::string("TM_TASK_MAP_NOT_SETTED")                  ,ERR_GS_TM_TASK_MAP_NOT_SETTED              },
    {std::string("DS_MAP_NOT_FOUND")                        ,ERR_GS_DS_MAP_NOT_FOUND                    },
    {std::string("DS_FILE_OPERATION_ERROR")                 ,ERR_GS_DS_FILE_OPERATION_ERROR             },
    {std::string("TM_SERVICE_ERROR")                        ,ERR_GS_TM_SERVICE_ERROR                    },
    {std::string("TM_SERVICE_RESPONSE_ERROR")               ,ERR_GS_TM_SERVICE_RESPONSE_ERROR           },
    {std::string("POSITION_NOT_EXIST")                      ,ERR_GS_POSITION_NOT_EXIST                  },
    {std::string("GEERTER_PROGRAM_ALREADY_STOP")            ,ERR_GS_GEERTER_PROGRAM_ALREADY_STOP        },
    {std::string("MULTI_TASK_NOT_RUNNING")                  ,ERR_GS_MULTI_TASK_NOT_RUNNING              },
    {std::string("MAP_NAME_ALREADY_IN_USED")                ,ERR_GS_MAP_NAME_ALREADY_IN_USED            },
    {std::string("COPY_FILE_FAILED")                        ,ERR_GS_COPY_FILE_FAILED                    },
    {std::string("MAP_LOADER_ERROR")                        ,ERR_GS_MAP_LOADER_ERROR                    },
    {std::string("TM_TASK_MAP_NOT_FOUND")                   ,ERR_GS_TM_TASK_MAP_NOT_FOUND               },
    {std::string("FILE_SYSTEM_ERROR")                       ,ERR_GS_FILE_SYSTEM_ERROR                   },
    {std::string("UPDATE_SYSTEM_ERROR")                     ,ERR_GS_UPDATE_SYSTEM_ERROR                 }
};

int get_gs_err_code(const std::string& err_msg)
{
    auto it = gs_err_code.find(err_msg);
    if (it != gs_err_code.end())
    {
        log_info("gs err msg:%s code:%d", err_msg.c_str(), it->second);
        return it->second;
    }
    else
    {
        log_info("gs err msg:%s unknow", err_msg.c_str());
        return ERR_GS_FAIL;
    }
}

