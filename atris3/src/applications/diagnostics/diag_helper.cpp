#include "diag_helper.h"
#include "tiny_ros/ros.h"
#include <unordered_map>
#include <map>

// for database event use
static std::map<std::string, int> event_content_to_level = 
{
    {"localization_lost",               1},    
    {"signal_bad",                0},
    {"ptz_error",              1},    
    {"lidar_error",              1},    
    {"ultra_error",            1},    
    {"anti_drop_error",   1},
    {"imu_error",        1},
    {"battery_error",     1},
    {"chassis_error",         1},
    {"speaker_error",              1},
    {"ccb_error",     1},
    {"sys_error",     1},
    {"mic_error",            1},    
    {"voip_error",   1},
    {"flood_warnning",  2},
    {"overtemp_warnning",  2},
    {"emerge_stop_warnning",  0},
    {"collision_warnning",  2},
    {"abnormal_shutdown_warnning", 2},
    {"battery_level_low_normal", 0},
    {"battery_level_low_urgent", 1},

    {"ultra_error_0", 1},
    {"ultra_error_1", 1},
    {"ultra_error_2", 1},
    {"ultra_error_3", 1},

    {"battery_cell_pressure_large",     1},
    {"battery_hight_temp",     1},
    {"battery_low_temp",     1},
    {"battery_low_power",     1},
    {"battery_severe_low_power",     1},
    {"battery_under_vol",     1},
    {"battery_over_discharge",     1},

    {"driver_left_front_motor_overload",     1},
    {"driver_left_front_motor_fault",     1},
    {"driver_left_front_elect_fault",     1},
    {"driver_left_front_encoder_fault",     1},

    {"driver_right_front_motor_overload",     1},
    {"driver_right_front_motor_fault",     1},
    {"driver_right_front_elect_fault",     1},
    {"driver_right_front_encoder_fault",     1},

    {"driver_left_rear_motor_overload",     1},
    {"driver_left_rear_motor_fault",     1},
    {"driver_left_rear_elect_fault",     1},
    {"driver_left_rear_encoder_fault",     1},

    {"driver_right_rear_motor_overload",     1},
    {"driver_right_rear_motor_fault",     1},
    {"driver_right_rear_elect_fault",     1},
    {"driver_right_rear_encoder_fault",     1},

};
// for database event use
static std::map<std::string, int> event_content_to_event_type = 
{
    {"localization_lost",                         NAVIGATION_WARNING},
    {"signal_bad",                        COMMUNICATION_WARNING},
    {"ptz_error",                                 DEVICE_WARNING},
    {"lidar_error",                    DEVICE_WARNING},
    {"ultra_error",               DEVICE_WARNING},
    {"anti_drop_error",               DEVICE_WARNING},
    {"imu_error",               DEVICE_WARNING},
    {"chassis_error",               DEVICE_WARNING},
    {"ccb_error",                            DEVICE_WARNING},
    {"sys_error",                   DEVICE_WARNING},
    {"battery_error",               DEVICE_WARNING},
    {"speaker_error",               DEVICE_WARNING},
    {"mic_error",               DEVICE_WARNING},
    {"voip_error",               DEVICE_WARNING},
    {"flood_warnning",               ABNORMAL_WARNING},
    {"overtemp_warnning",               ABNORMAL_WARNING},
    {"emerge_stop_warnning",               ABNORMAL_WARNING},
    {"collision_warnning",               ABNORMAL_WARNING},
    {"abnormal_shutdown_warnning",               ABNORMAL_WARNING},

    {"ultra_error_0",               DEVICE_WARNING},
    {"ultra_error_1",               DEVICE_WARNING},
    {"ultra_error_2",               DEVICE_WARNING},
    {"ultra_error_3",               DEVICE_WARNING},

    {"battery_cell_pressure_large",               DEVICE_WARNING},
    {"battery_hight_temp",               DEVICE_WARNING},
    {"battery_low_temp",               DEVICE_WARNING},
    {"battery_low_power",               DEVICE_WARNING},
    {"battery_severe_low_power",               DEVICE_WARNING},
    {"battery_under_vol",               DEVICE_WARNING},
    {"battery_over_discharge",               DEVICE_WARNING},

    {"driver_left_front_motor_overload",               DEVICE_WARNING},
    {"driver_left_front_motor_fault",               DEVICE_WARNING},
    {"driver_left_front_elect_fault",               DEVICE_WARNING},
    {"driver_left_front_encoder_fault",               DEVICE_WARNING},

    {"driver_right_front_motor_overload",               DEVICE_WARNING},
    {"driver_right_front_motor_fault",               DEVICE_WARNING},
    {"driver_right_front_elect_fault",               DEVICE_WARNING},
    {"driver_right_front_encoder_fault",               DEVICE_WARNING},

    {"driver_left_rear_motor_overload",               DEVICE_WARNING},
    {"driver_left_rear_motor_fault",               DEVICE_WARNING},
    {"driver_left_rear_elect_fault",               DEVICE_WARNING},
    {"driver_left_rear_encoder_fault",               DEVICE_WARNING},

    {"driver_right_rear_motor_overload",               DEVICE_WARNING},
    {"driver_right_rear_motor_fault",               DEVICE_WARNING},
    {"driver_right_rear_elect_fault",               DEVICE_WARNING},
    {"driver_right_rear_encoder_fault",               DEVICE_WARNING},

};


static std::map<int, std::string> event_var_index_to_event_content = 
{
    {LOCALIZATION_LOST_INDEX,                         "localization_lost"},
    {SIGNAL_BAD_INDEX,                        "signal_bad"},
    {PTZ_ERROR_INDEX,                                 "ptz_error"},
    {LIDAR_ERROR_INDEX,                    "lidar_error"},
    {ULTRA_ERROR_INDEX,               "ultra_error"},
    {ANTI_DROP_ERROR_INDEX,               "anti_drop_error"},
    {IMU_ERROR_INDEX,               "imu_error"},
    {CHASSIS_ERROR_INDEX,               "chassis_error"},
    {CCB_ERROR_INDEX,               "ccb_error"},
    {SYS_ERROR_INDEX,               "sys_error"},
    {BATTERY_ERROR_INDEX,               "battery_error"},
    {SPEAKER_ERROR_INDEX,               "speaker_error"},
    {MIC_ERROR_INDEX,               "mic_error"},
    {VOIP_ERROR_INDEX,               "voip_error"},
    {FLOOD_WARNING_INDEX,               "flood_warnning"},
    {OVERTEMP_WARNING_INDEX,               "overtemp_warnning"},
    {EMERGE_STOP_WARNING_INDEX,               "emerge_stop_warnning"},
    {COLLISION_WARNING_INDEX,               "collision_warnning"},
    {ABNORMAL_SHUTDOWN_WARNING_INDEX,               "abnormal_shutdown_warnning"},

    {ULTRA_ERROR_INDEX_0,               "ultra_error_0"},
    {ULTRA_ERROR_INDEX_1,               "ultra_error_1"},
    {ULTRA_ERROR_INDEX_2,               "ultra_error_2"},
    {ULTRA_ERROR_INDEX_3,               "ultra_error_3"},

    {BATTERY_ERROR_INDEX_0,               "battery_cell_pressure_large"},
    {BATTERY_ERROR_INDEX_1,               "battery_hight_temp"},
    {BATTERY_ERROR_INDEX_2,               "battery_low_temp"},
    {BATTERY_ERROR_INDEX_3,               "battery_low_power"},
    {BATTERY_ERROR_INDEX_4,               "battery_severe_low_power"},
    {BATTERY_ERROR_INDEX_5,               "battery_under_vol"},
    {BATTERY_ERROR_INDEX_6,               "battery_over_discharge"},

    {DRIVER_LEFT_FRONT_INDEX_0,               "driver_left_front_motor_overload"},
    {DRIVER_LEFT_FRONT_INDEX_1,               "driver_left_front_motor_fault"},
    {DRIVER_LEFT_FRONT_INDEX_2,               "driver_left_front_elect_fault"},
    {DRIVER_LEFT_FRONT_INDEX_3,               "driver_left_front_encoder_fault"},

    {DRIVER_RIGHT_FRONT_INDEX_0,               "driver_right_front_motor_overload"},
    {DRIVER_RIGHT_FRONT_INDEX_1,               "driver_right_front_motor_fault"},
    {DRIVER_RIGHT_FRONT_INDEX_2,               "driver_right_front_elect_fault"},
    {DRIVER_RIGHT_FRONT_INDEX_3,               "driver_right_front_encoder_fault"},

    {DRIVER_LEFT_REAR_INDEX_0,               "driver_left_rear_motor_overload"},
    {DRIVER_LEFT_REAR_INDEX_1,               "driver_left_rear_motor_fault"},
    {DRIVER_LEFT_REAR_INDEX_2,               "driver_left_rear_elect_fault"},
    {DRIVER_LEFT_REAR_INDEX_3,               "driver_left_rear_encoder_fault"},

    {DRIVER_RIGHT_REAR_INDEX_0,               "driver_right_rear_motor_overload"},
    {DRIVER_RIGHT_REAR_INDEX_1,               "driver_right_rear_motor_fault"},
    {DRIVER_RIGHT_REAR_INDEX_2,               "driver_right_rear_elect_fault"},
    {DRIVER_RIGHT_REAR_INDEX_3,               "driver_right_rear_encoder_fault"}

};

// for normal event use 
// look up table
static std::map<std::string, std::string> event_content_to_event_id = 
{
    {"planning_fail",                         "notify_nav_fail_event"},
    {"navigate_to_path_fail",                         "notify_nav_fail_event"},
    {"bypass_obstacle_planning_fail",                        "notify_nav_fail_event"},
    {"no_localization_info",                                 "notify_nav_fail_event"},
    {"obstacle_blocking_timeout_fail",                    "notify_nav_fail_event"},
    {"map_not_match",               "notify_nav_fail_event"},
    {"navigation_fail",               "notify_nav_fail_event"},
    {"localization_fail",               "notify_locate_fail_event"},
    {"localization_lost",               "notify_location_lost_event"},
    {"ptz_error",               "notify_ptz_error_event"},
    {"lidar_error",               "notify_lidar_error_event"},
    //{"ultra_error",               "notify_ultra_error_event"},
    {"anti_drop_error",               "notify_anti_drop_error_event"},
    {"imu_error",               "notify_imu_error_event"},
    {"chassis_error",               "notify_chassis_error_event"},
    {"ccb_error",               "notify_ccb_error_event"},
    {"sys_error",               "notify_sys_error_event"},
    {"battery_error",               "notify_battery_error_event"},
    {"speaker_error",               "notify_speaker_error_event"},
    {"mic_error",               "notify_mic_error_event"},
    {"voip_error",               "notify_voip_error_event"},
    {"battery_level_low_normal",         "notify_low_battery_event"},
    {"battery_level_low_urgent",         "notify_low_battery_event"},
    {"no_charge_point",                "notify_recharge_fail_event"},
    {"up_pile_fail",                 "notify_recharge_fail_event"},
    {"leave_pile_fail",               "notify_recharge_fail_event"},
    {"flood_warnning",          "notify_flood_warning_event"},
    {"overtemp_warnning",           "notify_overtemp_warning_event"},
    {"emerge_stop_warnning",             "notify_emergency_stop_warning_event"},
    {"collision_warnning",            "notify_collision_warning_event"},
    {"abnormal_shutdown_warnning",         "notify_abnormal_shutdown_warning_event"},
    {"emerge_button_task_pause",        "notify_task_pause_event"},
    {"collision_task_pause",        "notify_task_pause_event"},
    {"remote_control_task_pause",        "notify_task_pause_event"},
    {"navigation_lead_task_fail",      "notify_task_fail_event"},
    {"abnormal_lead_task_fail",      "notify_task_fail_event"},

    {"ultra_error_0",               "notify_ultra_error_event"},
    {"ultra_error_1",               "notify_ultra_error_event"},
    {"ultra_error_2",               "notify_ultra_error_event"},
    {"ultra_error_3",               "notify_ultra_error_event"},

    {"battery_cell_pressure_large",               "notify_battery_fail_event"},
    {"battery_hight_temp",               "notify_battery_fail_event"},
    {"battery_low_temp",               "notify_battery_fail_event"},
    {"battery_low_power",               "notify_battery_fail_event"},
    {"battery_severe_low_power",               "notify_battery_fail_event"},
    {"battery_under_vol",               "notify_battery_fail_event"},
    {"battery_over_discharge",               "notify_battery_fail_event"},

    {"driver_left_front_motor_overload",               "notify_chassis_fail_event"},
    {"driver_left_front_motor_fault",               "notify_chassis_fail_event"},
    {"driver_left_front_elect_fault",               "notify_chassis_fail_event"},
    {"driver_left_front_encoder_fault",               "notify_chassis_fail_event"},

    {"driver_right_front_motor_overload",               "notify_chassis_fail_event"},
    {"driver_right_front_motor_fault",               "notify_chassis_fail_event"},
    {"driver_right_front_elect_fault",               "notify_chassis_fail_event"},
    {"driver_right_front_encoder_fault",               "notify_chassis_fail_event"},

    {"driver_left_rear_motor_overload",               "notify_chassis_fail_event"},
    {"driver_left_rear_motor_fault",               "notify_chassis_fail_event"},
    {"driver_left_rear_elect_fault",               "notify_chassis_fail_event"},
    {"driver_left_rear_encoder_fault",               "notify_chassis_fail_event"},

    {"driver_right_rear_motor_overload",               "notify_chassis_fail_event"},
    {"driver_right_rear_motor_fault",               "notify_chassis_fail_event"},
    {"driver_right_rear_elect_fault",               "notify_chassis_fail_event"},
    {"driver_right_rear_encoder_fault",               "notify_chassis_fail_event"}
};

static std::map<std::string, int> event_content_to_alarm_type = 
{
    {"ultra_error_0",               0},
    {"ultra_error_1",               1},
    {"ultra_error_2",                2},
    {"ultra_error_3",               3},

    {"battery_cell_pressure_large",           0},
    {"battery_hight_temp",                    1},
    {"battery_low_temp",                      2},
    {"battery_low_power",                     3},
    {"battery_severe_low_power",              4},
    {"battery_under_vol",                     5},
    {"battery_over_discharge",                6},

    {"driver_left_front_motor_overload",            0},
    {"driver_left_front_motor_fault",               1},
    {"driver_left_front_elect_fault",               2},
    {"driver_left_front_encoder_fault",             3},

    {"driver_right_front_motor_overload",           4},
    {"driver_right_front_motor_fault",              5},
    {"driver_right_front_elect_fault",              6},
    {"driver_right_front_encoder_fault",            7},

    {"driver_left_rear_motor_overload",             8},
    {"driver_left_rear_motor_fault",                9},
    {"driver_left_rear_elect_fault",                10},
    {"driver_left_rear_encoder_fault",              11},

    {"driver_right_rear_motor_overload",            12},
    {"driver_right_rear_motor_fault",               13},
    {"driver_right_rear_elect_fault",               14},
    {"driver_right_rear_encoder_fault",             15}
};

int get_alarm_type_by_content(std::string event_id)
{
    auto it = event_content_to_alarm_type.find(event_id);
    if (it != event_content_to_alarm_type.end()){
        log_info("%s event id %s,  alarm_type :%d", __FUNCTION__, event_id.c_str(), it->second);
        return it->second;
    }
    else{
        log_error("%s event id :%s alarm_type is unknow ", __FUNCTION__, event_id.c_str());
        return -1;
    }
}

int get_warn_level_by_content(std::string event_id)
{
    auto it = event_content_to_level.find(event_id);
    if (it != event_content_to_level.end()){
        log_info("%s event id %s,  level :%d", __FUNCTION__, event_id.c_str(), it->second);
        return it->second;
    }
    else{
        log_error("%s event id :%s level is unknow ", __FUNCTION__, event_id.c_str());
        return -1;
    }
}

int get_warn_type_by_content(std::string event_id)
{
    auto it = event_content_to_event_type.find(event_id);
    if (it != event_content_to_event_type.end()){
        log_info("%s event id %s,  event_type :%d", __FUNCTION__, event_id.c_str(), it->second);
        return it->second;
    }
    else{
        log_error("%s event id :%s type is unknow ", __FUNCTION__, event_id.c_str());
        return -1;
    }
}

std::string get_event_id_by_content(std::string event_content)
{
    auto it = event_content_to_event_id.find(event_content);
    if (it != event_content_to_event_id.end()){
        log_info("%s event content %s,  event id :%s", __FUNCTION__, event_content.c_str(), it->second.c_str());
        return it->second;
    }
    else{
        log_error("%s event content :%s event id is unknow ", __FUNCTION__, event_content.c_str());
        return "unknow";
    }
}

std::string get_event_content_by_var_index(int var_index)
{
    auto it = event_var_index_to_event_content.find(var_index);
    if (it != event_var_index_to_event_content.end()){
        log_info("%s event var index %d,  event content :%s", __FUNCTION__, var_index, it->second.c_str());
        return it->second;
    }
    else{
        log_error("%s event var index :%d event content is unknow ", __FUNCTION__, var_index);
        return "unknow";
    }
}
