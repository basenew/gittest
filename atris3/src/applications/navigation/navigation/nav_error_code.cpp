#include "nav_error_code.h"
#include "log/log.h"
#include <unordered_map>
#include <map>

static std::map<int, const char*> code2msg = 
{
    {ERR_UNKNOW,                                "fail_unknow"                                  },    
    {ERR_OK,                                    "success"                                      },    
    {ERR_FAIL,                                  "fail"                                         },    
    {ERR_ALLOC_MEM_FAIL,                        "fail_alloc_mem_fail"                          },    
    {ERR_STAT_FILE,                             "fail_stat_file"                               },    
    {ERR_INVALID_LOCATE_TYPE,                   "fail_invalid_locate_type"                     },    
    {ERR_INVALID_PARAM,                         "fail_invalid_param"                           },    
    {ERR_INVALID_JSON,                          "fail_invalid_data"                            },    
    {ERR_INVALID_HOST_PORT,                     "fail_invalid_host_port"                       },    
    {ERR_INVALID_OP,                            "fail_invalid_op"                              },    
    {ERR_INVALID_URL,                           "fail_invalid_url"                             },    
    {ERR_INVALID_VALUE,                         "fail_invalid_value"                           },    
    {ERR_INVALID_FIELD,                         "fail_invalid_field"                           },    
    {ERR_FILE_NOT_EXIST,                        "fail_file_not_exist"                          },    
    {ERR_MAP_ALREADY_LOADING,                   "fail_map_already_loading"                     },    
    {ERR_CREATE_THREAD_FAIL,                    "fail_create_thread_fail"                      },    
    {ERR_OPEN_REPEAT,                           "fail_open_repeat"                             },    
    {ERR_HTTP_GET_FAIL,                         "fail_http_get_fail"                           },    
    {ERR_HTTP_POST_FAIL,                        "fail_http_post_fail"                          },    
    {ERR_HTTP_GET_FILE_FAIL,                    "fail_http_get_file_fail"                      },    
    {ERR_HTTP_POST_FILE_FAIL,                   "fail_http_post_file_fail"                     },    
    {ERR_RSP_FAIL,                              "fail_rsp_fail"                                },    
    {ERR_OPEN_FILE_FAIL,                        "fail_open_file_fail"                          },    
    {ERR_READ_FILE_FAIL,                        "fail_read_file_fail"                          },    
    {ERR_PARSE_SCHEME_FAIL,                     "fail_parse_scheme_fail"                       },    
    {ERR_TRANSFER_FILE_FAIL,                    "fail_transfer_file_fail"                      },   
    {ERR_NONE_MAP,                              "fail_none_map"                                },   
    {ERR_NONE_PATROL,                           "fail_none_patrol"                             },   
    {ERR_NONE_SCHEME,                           "fail_none_scheme"                             },   
    {ERR_ON_LOCATING,                           "fail_on_locating"                             },   
    {ERR_ON_RUNNING,                            "fail_on_running"                              },   
    {ERR_SCHEME_NOT_FOUND,                      "fail_scheme_not_found"                        },   
    {ERR_SCHEME_OVERTIME,                       "fail_scheme_overtime"                         },   
    {ERR_SCHEME_SAVE_FAIL,                      "fail_scheme_save_fail"                        },   
    {ERR_MAP_NOT_FOUND,                         "fail_map_not_found"                           },   
    {ERR_REMOVE_FILE_FAIL,                      "fail_remove_file_fail"                        },   
    {ERR_NONE_GPS,                              "fail_none_gps"                                },    
    {ERR_ALREADY_RUNNING,                       "fail_already_running"                         },    
    {ERR_MAP_NOT_USING,                         "fail_map_not_using"                           },    
    {ERR_MAP_MUST_EQ_USING,                     "fail_map_must_eq_using"                       },    
    {ERR_RENAME_USING_MAP,                      "fail_rename_using_map"                        },    
    {ERR_REMOVE_USING_MAP,                      "fail_remove_using_map"                        },    
    {ERR_DEVICE_GS_ERROR,                       "fail_device_gs_error"                         },    
    {ERR_NOT_LOCATED,                           "fail_not_located"                             },    
    {ERR_NOT_RUNNING,                           "fail_not_running"                             },    
    {ERR_LOCATE_FAIL,                           "fail_locate_fail"                             },    
    {ERR_LOCATING,                              "fail_locating"                                },    
    {ERR_NAVIGATING,                            "fail_navigating"                              },    
    {ERR_TASK_NOT_READY,                        "fail_task_not_ready"                          },    
    {ERR_NOT_LOCATED_BY_SPC_MAP,                "fail_not_located_by_spc_map"                  },    
    {ERR_PATH_NOT_MERGED,                       "fail_path_not_merged"                         },    
    {ERR_NONE_CHARGE_SCHEME,                    "fail_none_charge_scheme"                      },            
    {ERR_CHARGE_POINT_NOT_SET,                  "fail_charge_point_not_set"                    },    
    {ERR_TASK_OP_CONFLICT,                      "fail_task_op_conflict"                        },    
    {ERR_LOCATED_OTHER_MAP,                     "fail_located_other_map"                       },    
    {ERR_SERVICE_NOT_READY,                     "fail_service_not_ready"                       },    
    {ERR_NONE_NAV_STATUS,                       "fail_none_nav_status"                         },
    {ERR_SCHEME_DISABLED,                       "fail_scheme_disabled"                         },
    {ERR_CHARGE_DISABLED,                       "fail_charge_disabled"                         },
    {ERR_MAP_NONE_PATH,                         "fail_map_none_path"                           },
    {ERR_STOP_NAV_TIMEOUT,                      "fail_stop_nav_timeout"                        },
    {ERR_MAP_LOADING,                           "fail_map_loading"                             },

    {ERR_GS_FAIL,                               "fail_fail"                                 },
    {ERR_GS_MAP_NOT_FOUND,                      "fail_map_not_found"                        },
    {ERR_GS_HTTP_REQUEST_ERROR,                 "fail_http_request_error"                   },
    {ERR_GS_OPERATE_DATA_FAILED,                "fail_operate_data_failed"                  },
    {ERR_GS_PATH_NOT_FOUND,                     "fail_path_not_found"                       },
    {ERR_GS_SYSTEM_NOT_HEALTHY,                 "fail_system_not_healthy"                   },
    {ERR_GS_LOCALIZATION_NOT_READY,             "fail_localization_not_ready"               },
    {ERR_GS_MOVEBASE_NOT_READY,                 "fail_movebase_not_ready"                   },
    {ERR_GS_LOCALIZATION_FILE_INVALID,          "fail_localization_file_invalid"            },
    {ERR_GS_LOCALIZATION_PARAM_INVALID,         "fail_localization_param_invalid"           },
    {ERR_GS_MOVEBASE_USBKEY_INVALID,            "fail_movebase_usbkey_invalid"              },
    {ERR_GS_SERVICE_CALLING_FAILED,             "fail_service_calling_failed"               },
    {ERR_GS_INIT_POINT_NOT_FOUND,               "fail_init_point_not_found"                 },
    {ERR_GS_NOT_INITIALIZE,                     "fail_not_initialize"                       },
    {ERR_GS_SCAN_MAP_PROGRAM_ALREADY_STOPPED,   "fail_scan_map_program_already_stopped"     },
    {ERR_GS_RECORD_PATH_PROGRAM_STATUS_ERROR,   "fail_record_path_program_status_error"     },
    {ERR_GS_RECORD_PATH_PROGRAM_ALREADY_STOPPED,"fail_record_path_program_already_stopped"  },
    {ERR_GS_SERVICE_CALLING_ERROR,              "fail_service_calling_error"                },
    {ERR_GS_PATH_ALREADY_IN_USED,               "fail_path_already_in_used"                 },
    {ERR_GS_JSON_FORMAT_ERROR,                  "fail_json_format_error"                    },
    {ERR_GS_PARSE_PATH_FILE_ERROR,              "fail_parse_path_file_error"                },
    {ERR_GS_SAVE_PATH_ERROR,                    "fail_save_path_error"                      },
    {ERR_GS_TM_TASK_QUEUE_IS_EMPTY,             "fail_tm_task_queue_is_empty"               },
    {ERR_GS_TM_TASK_STATUS_IS_ERROR,            "fail_tm_task_status_is_error"              },
    {ERR_GS_TM_TASK_QUEUE_PARAM_ERROR,          "fail_tm_task_queue_param_error"            },
    {ERR_GS_TM_TASK_MAP_NOT_SETTED,             "fail_tm_task_map_not_setted"               },
    {ERR_GS_DS_MAP_NOT_FOUND,                   "fail_ds_map_not_found"                     },
    {ERR_GS_DS_FILE_OPERATION_ERROR,            "fail_ds_file_operation_error"              },
    {ERR_GS_TM_SERVICE_ERROR,                   "fail_tm_service_error"                     },
    {ERR_GS_TM_SERVICE_RESPONSE_ERROR,          "fail_tm_service_response_error"            },
    {ERR_GS_POSITION_NOT_EXIST,                 "fail_position_not_exist"                   },
    {ERR_GS_GEERTER_PROGRAM_ALREADY_STOP,       "fail_geerter_program_already_stop"         },
    {ERR_GS_MULTI_TASK_NOT_RUNNING,             "fail_multi_task_not_running"               },
    {ERR_GS_MAP_NAME_ALREADY_IN_USED,           "fail_map_name_already_in_used"             },
    {ERR_GS_COPY_FILE_FAILED,                   "fail_copy_file_failed"                     },
    {ERR_GS_MAP_LOADER_ERROR,                   "fail_map_loader_error"                     },
    {ERR_GS_TM_TASK_MAP_NOT_FOUND,              "fail_tm_task_map_not_found"                },
    {ERR_GS_FILE_SYSTEM_ERROR,                  "fail_file_system_error"                    },
    {ERR_GS_UPDATE_SYSTEM_ERROR,                "fail_update_system_error"                  },
    {ERR_GS_NT_SERVICE_ERROR,                   "fail_nt_service_error"                     },
    {ERR_GS_INVALID_JSON,                       "fail_invalid_json"                         },
    {ERR_GS_NO_POINT,                           "fail_gs_no_point"                          },
    {ERR_GS_NO_TYPE_POINT,                      "fail_gs_no_type_point"                     },
    {ERR_GS_LIDAR_ERR,                          "fail_gs_lidar_err"                         },
    {ERR_GS_CHASSIS_ERR,                        "fail_gs_chassis_err"                       }

};

static std::map<const char*, int> msg2code = 
{
    {"fail_unknow"                                 ,ERR_UNKNOW                                   },
    {"success"                                     ,ERR_OK                                       },
    {"fail"                                        ,ERR_FAIL                                     },
    {"fail_alloc_mem_fail"                         ,ERR_ALLOC_MEM_FAIL                           },
    {"fail_stat_file"                              ,ERR_STAT_FILE                                },
    {"fail_invalid_locate_type"                    ,ERR_INVALID_LOCATE_TYPE                      },
    {"fail_invalid_param"                          ,ERR_INVALID_PARAM                            },
    {"fail_invalid_data"                           ,ERR_INVALID_JSON                             },
    {"fail_invalid_host_port"                      ,ERR_INVALID_HOST_PORT                        },
    {"fail_invalid_op"                             ,ERR_INVALID_OP                               },
    {"fail_invalid_url"                            ,ERR_INVALID_URL                              },
    {"fail_invalid_value"                          ,ERR_INVALID_VALUE                            },
    {"fail_invalid_field"                          ,ERR_INVALID_FIELD                            },
    {"fail_file_not_exist"                         ,ERR_FILE_NOT_EXIST                           },
    {"fail_map_already_loading"                    ,ERR_MAP_ALREADY_LOADING                      },
    {"fail_create_thread_fail"                     ,ERR_CREATE_THREAD_FAIL                       },
    {"fail_open_repeat"                            ,ERR_OPEN_REPEAT                              },
    {"fail_http_get_fail"                          ,ERR_HTTP_GET_FAIL                            },
    {"fail_http_post_fail"                         ,ERR_HTTP_POST_FAIL                           },
    {"fail_http_get_file_fail"                     ,ERR_HTTP_GET_FILE_FAIL                       },
    {"fail_http_post_file_fail"                    ,ERR_HTTP_POST_FILE_FAIL                      },
    {"fail_rsp_fail"                               ,ERR_RSP_FAIL                                 },
    {"fail_open_file_fail"                         ,ERR_OPEN_FILE_FAIL                           },
    {"fail_read_file_fail"                         ,ERR_READ_FILE_FAIL                           },
    {"fail_parse_scheme_fail"                      ,ERR_PARSE_SCHEME_FAIL                        },
    {"fail_transfer_file_fail"                     ,ERR_TRANSFER_FILE_FAIL                       },
    {"fail_none_map"                               ,ERR_NONE_MAP                                 },
    {"fail_none_patrol"                            ,ERR_NONE_PATROL                              },
    {"fail_none_scheme"                            ,ERR_NONE_SCHEME                              },
    {"fail_on_locating"                            ,ERR_ON_LOCATING                              },
    {"fail_on_running"                             ,ERR_ON_RUNNING                               },
    {"fail_scheme_not_found"                       ,ERR_SCHEME_NOT_FOUND                         },
    {"fail_scheme_overtime"                        ,ERR_SCHEME_OVERTIME                          },
    {"fail_scheme_save_fail"                       ,ERR_SCHEME_SAVE_FAIL                         },
    {"fail_map_not_found"                          ,ERR_MAP_NOT_FOUND                            },
    {"fail_remove_file_fail"                       ,ERR_REMOVE_FILE_FAIL                         },
    {"fail_none_gps"                               ,ERR_NONE_GPS                                 },
    {"fail_already_running"                        ,ERR_ALREADY_RUNNING                          },
    {"fail_map_not_using"                          ,ERR_MAP_NOT_USING                            },
    {"fail_map_must_eq_using"                      ,ERR_MAP_MUST_EQ_USING                        },
    {"fail_rename_using_map"                       ,ERR_RENAME_USING_MAP                         },
    {"fail_remove_using_map"                       ,ERR_REMOVE_USING_MAP                         },
    {"fail_device_gs_error"                        ,ERR_DEVICE_GS_ERROR                          },
    {"fail_not_located"                            ,ERR_NOT_LOCATED                              },
    {"fail_not_running"                            ,ERR_NOT_RUNNING                              },
    {"fail_locate_fail"                            ,ERR_LOCATE_FAIL                              },
    {"fail_locating"                               ,ERR_LOCATING                                 },
    {"fail_navigating"                             ,ERR_NAVIGATING                               },
    {"fail_task_not_ready"                         ,ERR_TASK_NOT_READY                           },
    {"fail_not_located_by_spc_map"                 ,ERR_NOT_LOCATED_BY_SPC_MAP                   },
    {"fail_path_not_merged"                        ,ERR_PATH_NOT_MERGED                          },
    {"fail_none_charge_scheme"                     ,ERR_NONE_CHARGE_SCHEME                       },
    {"fail_charge_point_not_set"                   ,ERR_CHARGE_POINT_NOT_SET                     },
    {"fail_task_op_conflict"                       ,ERR_TASK_OP_CONFLICT                         },
    {"fail_located_other_map"                      ,ERR_LOCATED_OTHER_MAP                        },
    {"fail_service_not_ready"                      ,ERR_SERVICE_NOT_READY                        },
    {"fail_none_nav_status"                        ,ERR_NONE_NAV_STATUS                          },
    {"fail_scheme_disabled"                        ,ERR_SCHEME_DISABLED                          },
    {"fail_charge_disabled"                        ,ERR_CHARGE_DISABLED                          },
    {"fail_map_none_path"                          ,ERR_MAP_NONE_PATH                            },
    {"fail_stop_nav_timeout"                       ,ERR_STOP_NAV_TIMEOUT                         },
    {"fail_map_loading"                            ,ERR_MAP_LOADING                              },

    {"fail_gs_fail"                                ,ERR_GS_FAIL                                  },
    {"fail_gs_map_not_found"                       ,ERR_GS_MAP_NOT_FOUND                         },
    {"fail_gs_http_request_error"                  ,ERR_GS_HTTP_REQUEST_ERROR                    },
    {"fail_gs_operate_data_failed"                 ,ERR_GS_OPERATE_DATA_FAILED                   },
    {"fail_gs_path_not_found"                      ,ERR_GS_PATH_NOT_FOUND                        },
    {"fail_gs_system_not_healthy"                  ,ERR_GS_SYSTEM_NOT_HEALTHY                    },
    {"fail_gs_localization_not_ready"              ,ERR_GS_LOCALIZATION_NOT_READY                },
    {"fail_gs_movebase_not_ready"                  ,ERR_GS_MOVEBASE_NOT_READY                    },
    {"fail_gs_localization_file_invalid"           ,ERR_GS_LOCALIZATION_FILE_INVALID             },
    {"fail_gs_localization_param_invalid"          ,ERR_GS_LOCALIZATION_PARAM_INVALID            },
    {"fail_gs_movebase_usbkey_invalid"             ,ERR_GS_MOVEBASE_USBKEY_INVALID               },
    {"fail_gs_service_calling_failed"              ,ERR_GS_SERVICE_CALLING_FAILED                },
    {"fail_gs_init_point_not_found"                ,ERR_GS_INIT_POINT_NOT_FOUND                  },
    {"fail_gs_not_initialize"                      ,ERR_GS_NOT_INITIALIZE                        },
    {"fail_gs_scan_map_program_already_stopped"    ,ERR_GS_SCAN_MAP_PROGRAM_ALREADY_STOPPED      },
    {"fail_gs_record_path_program_status_error"    ,ERR_GS_RECORD_PATH_PROGRAM_STATUS_ERROR      },
    {"fail_gs_record_path_program_already_stopped" ,ERR_GS_RECORD_PATH_PROGRAM_ALREADY_STOPPED   },
    {"fail_gs_service_calling_error"               ,ERR_GS_SERVICE_CALLING_ERROR                 },
    {"fail_gs_path_already_in_used"                ,ERR_GS_PATH_ALREADY_IN_USED                  },
    {"fail_gs_json_format_error"                   ,ERR_GS_JSON_FORMAT_ERROR                     },
    {"fail_gs_parse_path_file_error"               ,ERR_GS_PARSE_PATH_FILE_ERROR                 },
    {"fail_gs_save_path_error"                     ,ERR_GS_SAVE_PATH_ERROR                       },
    {"fail_gs_tm_task_queue_is_empty"              ,ERR_GS_TM_TASK_QUEUE_IS_EMPTY                },
    {"fail_gs_tm_task_status_is_error"             ,ERR_GS_TM_TASK_STATUS_IS_ERROR               },
    {"fail_gs_tm_task_queue_param_error"           ,ERR_GS_TM_TASK_QUEUE_PARAM_ERROR             },
    {"fail_gs_tm_task_map_not_setted"              ,ERR_GS_TM_TASK_MAP_NOT_SETTED                },
    {"fail_gs_ds_map_not_found"                    ,ERR_GS_DS_MAP_NOT_FOUND                      },
    {"fail_gs_ds_file_operation_error"             ,ERR_GS_DS_FILE_OPERATION_ERROR               },
    {"fail_gs_tm_service_error"                    ,ERR_GS_TM_SERVICE_ERROR                      },
    {"fail_gs_tm_service_response_error"           ,ERR_GS_TM_SERVICE_RESPONSE_ERROR             },
    {"fail_gs_position_not_exist"                  ,ERR_GS_POSITION_NOT_EXIST                    },
    {"fail_gs_geerter_program_already_stop"        ,ERR_GS_GEERTER_PROGRAM_ALREADY_STOP          },
    {"fail_gs_multi_task_not_running"              ,ERR_GS_MULTI_TASK_NOT_RUNNING                },
    {"fail_gs_map_name_already_in_used"            ,ERR_GS_MAP_NAME_ALREADY_IN_USED              },
    {"fail_gs_copy_file_failed"                    ,ERR_GS_COPY_FILE_FAILED                      },
    {"fail_gs_map_loader_error"                    ,ERR_GS_MAP_LOADER_ERROR                      },
    {"fail_gs_tm_task_map_not_found"               ,ERR_GS_TM_TASK_MAP_NOT_FOUND                 },
    {"fail_gs_file_system_error"                   ,ERR_GS_FILE_SYSTEM_ERROR                     },
    {"fail_gs_update_system_error"                 ,ERR_GS_UPDATE_SYSTEM_ERROR                   },
    {"fail_gs_nt_service_error"                    ,ERR_GS_NT_SERVICE_ERROR                      },
    {"fail_gs_invalid_json"                        ,ERR_GS_INVALID_JSON                          },
    {"fail_gs_no_point"                            ,ERR_GS_NO_POINT                              },
    {"fail_gs_no_type_point"                       ,ERR_GS_NO_TYPE_POINT                         },
    {"fail_gs_lidar_err"                           ,ERR_GS_LIDAR_ERR                             },
    {"fail_gs_chassis_err"                         ,ERR_GS_CHASSIS_ERR                           }
};


int get_err_code(const char* err_msg)
{
    auto it = msg2code.find(err_msg);
    if (it != msg2code.end()){
        log_info("err:%d msg:%s", it->second, err_msg);
        return it->second;
    }
    else{
        log_error("err msg:%s is unknow err", err_msg);
        return ERR_UNKNOW;
    }
}

const char* get_err_msg(int err)
{
    auto it = code2msg.find(err);
    if (it != code2msg.end()){
        log_info("err:%d msg:%s", err, it->second);
        return it->second;
    }
    else{
        log_error("err:%d is unknow err", err);
        return "unknow error";
    }
}
