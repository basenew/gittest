/*
 * File      : tinyros_entries.cpp
 * This file is part of tinyros
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-22     Pinkie.Fu    initial version
 * 2020-07-22     xiaofeng.wu  modify for msg canpkg
 */

#include "msg_canpkg.h"
#include "tiny_ros/ros.h"

//#define USING_UDP
#define USING_TCP
#define TINYROS_NODE_NAME "monitor"
#define CANPKG_PUB_TOPIC "/topic/MCU/Motor_Controller/response"
#define CANPKG_SUB_TOPIC "/topic/MCU/Motor_Controller/request"
#define MASTER_IP "10.20.18.2"
#define MSG_SUB_TABLE_NUM_MAX 50

typedef struct
{
    int32_t cmd;
    canpkg_sub_cb_t sub_cb;
} canpkg_sub_tb_t;

static canpkg_sub_tb_t canpkg_sub_table[MSG_SUB_TABLE_NUM_MAX] = {0};



static tinyros::Publisher canpkg_pub(CANPKG_PUB_TOPIC, new tinyros::atris_msgs::CanPkg());
int32_t canpkg_publisher_init(void)
{
    tinyros::init(TINYROS_NODE_NAME, MASTER_IP);
    
#ifdef USING_TCP
    tinyros::nh()->advertise(canpkg_pub);
#else
    tinyros::udp()->advertise(canpkg_pub);
#endif
    return 0;
}

static void canpkg_subscriber_cb(const tinyros::atris_msgs::CanPkg& rxmsg) 
{
    for (int32_t i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (canpkg_sub_table[i].cmd == rxmsg.cmd) {
            canpkg_sub_table[i].sub_cb(rxmsg);
            break;
        }
    }
}

static tinyros::Subscriber<tinyros::atris_msgs::CanPkg> canpkg_sub(CANPKG_SUB_TOPIC, canpkg_subscriber_cb);
int32_t canpkg_subscriber_init(void)
{
    tinyros::init(TINYROS_NODE_NAME, MASTER_IP);
#ifdef USING_TCP
  tinyros::nh()->subscribe(canpkg_sub);
#else
  tinyros::udp()->subscribe(canpkg_sub);
#endif
    return 0;
}

int32_t canpkg_pulisher_negotiated_get(void)
{
    return (canpkg_pub.negotiated() == true ? 0 : -1);
}

int32_t canpkg_publish_msg(tinyros::atris_msgs::CanPkg& txmsg)
{
    return (int32_t)canpkg_pub.publish(&txmsg);
}

int32_t canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb)
{
    int32_t i = 0;
    if (cb == NULL) return -1;

    rt_enter_critical();
    for (i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (canpkg_sub_table[i].cmd != 0) {
            if (canpkg_sub_table[i].cmd == cmd) {
                canpkg_sub_table[i].sub_cb = cb;
                 break;
            }
        }
        else {
            canpkg_sub_table[i].cmd = cmd;
            canpkg_sub_table[i].sub_cb = cb;
            break;
        }
    }
    rt_exit_critical();
    if (i == MSG_SUB_TABLE_NUM_MAX) 
        return -2;
    
    return 0;
}

int32_t canpkg_subscribe_cb_detach(int32_t cmd)
{
    int32_t i = 0;
    rt_enter_critical();
    for (i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (canpkg_sub_table[i].cmd == cmd) { 
            canpkg_sub_table[i].cmd = 0;
            canpkg_sub_table[i].sub_cb = NULL;
        }
    }
    rt_exit_critical();
    return 0;
}

int32_t tinyros_nh_ok_get(void)
{
    return (tinyros::nh()->ok() == true ? 0 : -1);
}
















