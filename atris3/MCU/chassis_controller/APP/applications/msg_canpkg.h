/*
 * File      : tinyros_entries.h
 * This file is part of tinyros
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-22     Pinkie.Fu    initial version
 */

#ifndef _TINYROS_MSG_CANPKG_H_
#define _TINYROS_MSG_CANPKG_H_
#include <stdint.h>
#include <stdlib.h>
#include "tiny_ros/atris_msgs/CanPkg.h"
#include "tiny_ros/atris_msgs/RobotPose.h"
#include "tiny_ros/atris_msgs/chassis_abnor_info.h"
#include "tiny_ros/geometry_msgs/Twist.h"
#include "complex.h"

#define   DF_CHASSIS_CONTROL_TOPIC    "cmd_vel"
#define   DF_CHASSIS_ODOM_TOPIC    "odom"
#define   DF_CHASSIS_DIAGNOSIS_TOPIC    "chassis_diagnosis"

#define CANPKG_CMD_BATTERY_INFO_REPORT          100
#define CANPKG_CMD_BATTERY_LOG_ASK              101
#define CANPKG_CMD_BATTERY_LOG_REPORT       	102
#define CANPKG_CMD_BMS_HVER_ASK                 103
#define CANPKG_CMD_BMS_HVER_ACK                 104
#define CANPKG_CMD_BMS_SVER_ASK                 105
#define CANPKG_CMD_BMS_SVER_ACK                 106

#define CANPKG_CMD_POWER_ONOFF_REQUEST          112
#define CANPKG_CMD_POWER_ONOFF_REQUEST_ACK      111
#define CANPKG_CMD_POWER_SHUTDOWN_REQUEST       113
#define CANPKG_CMD_POWER_SHUTDOWN_REQUEST_ACK   114
#define CANPKG_CMD_UDOCK_POINT_DOWN             115
#define CANPKG_CMD_UDOCK_POINT_DOWN_ACK         116
#define CANPKG_CMD_UDOCK_STATUS_REPORT          118
#define CANPKG_CMD_UDOCK_STATUS_REPORT_ACK      117
#define CANPKG_CMD_UDOCK_POINT_UP               119
#define CANPKG_CMD_UDOCK_POINT_UP_ACK           120
#define CANPKG_CMD_LIGHT_MODE_REQUEST           122  	//info the system contrl board
#define CANPKG_CMD_ROBOT_MODE_REQUEST           124  	//info the system contrl board

#define CANPKG_CMD_POWER_STATUS_ASK_SET         131
#define CANPKG_CMD_POWER_STATUS_REPORT          132
#define CANPKG_CMD_VOLTAGE_DET_DATA_REPORT      134
#define CANPKG_CMD_CURRENT_DET_DATA_REPORT      136

#define CANPKG_CMD_REMOTE_INFO_REPORT           142
#define CANPKG_CMD_ANTI_INFO_REPORT             144
#define CANPKG_CMD_HSU_CHM_INFO_REPORT          146
//#define CANPKG_CMD_FAN_STATUS_ASK_SET           147
//#define CANPKG_CMD_FAN_STATUS_REPORT            148

#define CANPKG_CMD_BRAKE_STATUS_SET             161
#define CANPKG_CMD_BRAKE_STATUS_ACK             162
#define CANPKG_CMD_BRAKE_STATUS_REPORT          164
#define CANPKG_CMD_TRANSPORT_STATUS_SET         165
#define CANPKG_CMD_TRANSPORT_STATUS_ACK         166

#define CANPKG_CMD_MCU_VER_ASK                  191
#define CANPKG_CMD_MCU_VER_ACK                  192

#define CANPKG_CMD_LOG_UPLOAD_ASK               193
#define CANPKG_CMD_LOG_UPLOAD_ACK               194



#define CANPKG_CMD_DROP_PREVENT_INFO_REPORT     228

#ifdef __cplusplus
 extern "C" {
#endif
     
typedef int32_t (*canpkg_sub_cb_t)(const tinyros::atris_msgs::CanPkg &msg);




int32_t canpkg_publisher_init(void);
int32_t canpkg_subscriber_init(void);
int32_t canpkg_publish_msg(tinyros::atris_msgs::CanPkg& txmsg);
int32_t diagnosis_publish_msg(tinyros::atris_msgs::chassis_abnor_info& txmsg);
int32_t canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb);
int32_t canpkg_subscribe_cb_detach(int32_t cmd);
int32_t canpkg_pulisher_negotiated_get(void);
int32_t tinyros_nh_ok_get(void);

int32_t send_odom(COMPLEX_def post,COMPLEX_def speed,double pTheta,double sTheta,double absOdom);
//int32_t sbus_data_post(uint16_t* _channels, uint8_t _flag);
int32_t monitor_canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb);
int32_t monitor_canpkg_subscribe_cb_detach(int32_t cmd);

#ifdef __cplusplus
}
#endif

#endif


