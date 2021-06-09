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

typedef int32_t (*canpkg_sub_cb_t)(const tinyros::atris_msgs::CanPkg &msg);




int32_t canpkg_publisher_init(void);
int32_t canpkg_subscriber_init(void);
int32_t canpkg_publish_msg(tinyros::atris_msgs::CanPkg& txmsg);
int32_t canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb);
int32_t canpkg_subscribe_cb_detach(int32_t cmd);
int32_t canpkg_pulisher_negotiated_get(void);
int32_t tinyros_nh_ok_get(void);


#endif


