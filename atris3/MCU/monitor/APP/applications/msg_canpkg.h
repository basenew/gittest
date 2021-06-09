/*
 * File      : tinyros_entries.h
 * This file is part of tinyros
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-22     Pinkie.Fu    initial version
 */

#ifndef _TINYROS_ENTRIES_H_
#define _TINYROS_ENTRIES_H_
#include <stdint.h>
#include <stdlib.h>
#include "tiny_ros/atris_msgs/CanPkg.h"

//chassis
#define CANPKG_CMD_BATTERY_INFO_REPORT          100
#define CANPKG_CMD_LIGHT_MODE_REQUEST           122  	//info the system contrl board
#define CHASSIS_CANPKG_CMD_ANTI_INFO_REPORT     144
#define CANPKG_CMD_BRAKE_STATUS_REPORT          164


//monitor
#define CANPKG_CMD_LIGHT_ASK_SET                201
#define CANPKG_CMD_LIGHT_ACK                    202
//#define CANPKG_CMD_LIGHT_FILL_ASK_SET           203
//#define CANPKG_CMD_LIGHT_FILL_ACK               204

#define CANPKG_CMD_POWER_STATUS_ASK_SET         211
#define CANPKG_CMD_POWER_STATUS_REPORT          212
#define CANPKG_CMD_VOLTAGE_DET_DATA_REPORT      214
#define CANPKG_CMD_CURRENT_DET_DATA_REPORT      216

#define CANPKG_CMD_SONAR_VER_ASK                221
#define CANPKG_CMD_SONAR_VER_ACK                222
#define CANPKG_CMD_SONAR_DATA_REPORT            224
#define CANPKG_CMD_HSU_CHM_INFO_REPORT          226
#define CANPKG_CMD_DROP_PREVENT_INFO_REPORT     228

#define CANPKG_CMD_FAN_STATUS_ASK_SET           241
#define CANPKG_CMD_FAN_STATUS_REPORT            242

#define CANPKG_CMD_MCU_VER_ASK                  291
#define CANPKG_CMD_MCU_VER_ACK                  292

#ifdef __cplusplus
 extern "C" {
#endif
     
typedef int32_t (*canpkg_sub_cb_t)(const tinyros::atris_msgs::CanPkg &msg);




int32_t canpkg_publisher_init(void);
int32_t canpkg_subscriber_init(void);
int32_t canpkg_publish_msg(tinyros::atris_msgs::CanPkg& txmsg);
int32_t canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb);
int32_t canpkg_subscribe_cb_detach(int32_t cmd);
int32_t canpkg_pulisher_negotiated_get(void);
int32_t tinyros_nh_ok_get(void);

int32_t chassis_canpkg_subscriber_init(void);
int32_t chassis_canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb);
int32_t chassis_canpkg_subscribe_cb_detach(int32_t cmd);


#ifdef __cplusplus
}
#endif

#endif

