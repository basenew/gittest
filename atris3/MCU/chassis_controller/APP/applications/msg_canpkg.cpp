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
#include "controller.h"
//#define USING_UDP
#define USING_TCP
#define TINYROS_NODE_NAME "chassis_controller"
#define CANPKG_PUB_TOPIC  "/topic/MCU/chassis_controller/response"
#define CANPKG_SUB_TOPIC  "/topic/MCU/chassis_controller/request"
#define MONITOR_CANPKG_PUB_TOPIC "/topic/MCU/Monitor/response"
#define MASTER_IP "10.20.18.2"
#define MSG_SUB_TABLE_NUM_MAX 50
#define DF_MEMORY_MAX_USER  45000

typedef struct
{
    int32_t cmd;
    canpkg_sub_cb_t sub_cb;
} canpkg_sub_tb_t;

static canpkg_sub_tb_t canpkg_sub_table[MSG_SUB_TABLE_NUM_MAX] = {0};
static canpkg_sub_tb_t monitor_canpkg_sub_table[MSG_SUB_TABLE_NUM_MAX] = {0};



static tinyros::Publisher canpkg_pub(CANPKG_PUB_TOPIC, new tinyros::atris_msgs::CanPkg());

static tinyros::Publisher odom_pub(DF_CHASSIS_ODOM_TOPIC, new tinyros::atris_msgs::RobotPose());
/*monitor com*/
static void monitor_canpkg_subscriber_cb(const tinyros::atris_msgs::CanPkg& rxmsg) 
{
    for (int32_t i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (monitor_canpkg_sub_table[i].cmd == rxmsg.cmd) {
            monitor_canpkg_sub_table[i].sub_cb(rxmsg);
            break;
        }
    }
}

static tinyros::Subscriber<tinyros::atris_msgs::CanPkg> monitor_canpkg_sub(MONITOR_CANPKG_PUB_TOPIC, monitor_canpkg_subscriber_cb);

static tinyros::Publisher diagnosis_pub(DF_CHASSIS_DIAGNOSIS_TOPIC, new tinyros::atris_msgs::chassis_abnor_info());
int32_t canpkg_publisher_init(void)
{
    tinyros::init(TINYROS_NODE_NAME, MASTER_IP);
    
#ifdef USING_TCP
    tinyros::nh()->advertise(canpkg_pub);
    tinyros::nh()->advertise(odom_pub);
    tinyros::nh()->advertise(diagnosis_pub);
	  tinyros::nh()->subscribe(monitor_canpkg_sub);
#else
    tinyros::udp()->advertise(canpkg_pub);
    tinyros::udp()->advertise(odom_pub);
    tinyros::udp()->advertise(diagnosis_pub);
#endif
    return 0;
}

int32_t diagnosis_publish_msg(tinyros::atris_msgs::chassis_abnor_info& txmsg)
{
   rt_uint32_t total;
   rt_uint32_t used;
   rt_uint32_t max_used; 
   rt_memory_info(&total,&used,&max_used);
    if(used < DF_MEMORY_MAX_USER)
        return (int32_t)diagnosis_pub.publish(&txmsg);
    else return 0;  
    
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

static void cmd_vel_subscriber_cb(const tinyros::geometry_msgs::Twist& rxmsg) 
{
    COMPLEX_def speed;
    double theta;
    speed.real = rxmsg.linear.x;
    speed.imag = rxmsg.linear.y;
    theta = rxmsg.angular.z;
    fromMasterControl(speed,theta);
//	rt_kprintf("test123\n");
}
static tinyros::Subscriber<tinyros::atris_msgs::CanPkg> canpkg_sub(CANPKG_SUB_TOPIC, canpkg_subscriber_cb);
static tinyros::Subscriber<tinyros::geometry_msgs::Twist> cmd_vel_sub(DF_CHASSIS_CONTROL_TOPIC, cmd_vel_subscriber_cb);
//static tinyros::Subscriber<tinyros::atris_msgs::CanPkg> monitor_canpkg_sub(MONITOR_CANPKG_PUB_TOPIC, monitor_canpkg_subscriber_cb);
int32_t canpkg_subscriber_init(void)
{
    tinyros::init(TINYROS_NODE_NAME, MASTER_IP);
#ifdef USING_TCP
  tinyros::nh()->subscribe(canpkg_sub);
  tinyros::nh()->subscribe(cmd_vel_sub);
//  tinyros::nh()->subscribe(monitor_canpkg_sub);
#else
  tinyros::udp()->subscribe(canpkg_sub);
  tinyros::udp()->subscribe(cmd_vel_sub);
  tinyros::udp()->subscribe(monitor_canpkg_sub);
#endif
    return 0;
}

int32_t canpkg_pulisher_negotiated_get(void)
{
    return (canpkg_pub.negotiated() == true ? 0 : -1);
}

int32_t canpkg_publish_msg(tinyros::atris_msgs::CanPkg& txmsg)
{
   rt_uint32_t total;
   rt_uint32_t used;
   rt_uint32_t max_used; 
   rt_memory_info(&total,&used,&max_used);
    if(used < DF_MEMORY_MAX_USER)
        return (int32_t)canpkg_pub.publish(&txmsg);
    else return 0;
    
    
}

 int32_t send_odom(COMPLEX_def post,COMPLEX_def speed,double pTheta,double sTheta,double absOdom)
{   
   rt_uint32_t total;
   rt_uint32_t used;
   rt_uint32_t max_used; 
    tinyros::atris_msgs::RobotPose txmsg;
    txmsg.p_x = post.real;
    txmsg.p_y = post.imag;
    txmsg.p_z = pTheta;
    txmsg.v_x = speed.real;
    txmsg.v_y = speed.imag;
    txmsg.v_z = sTheta;
    txmsg.odom = absOdom;
    rt_memory_info(&total,
                    &used,
                    &max_used);
    if(used < DF_MEMORY_MAX_USER)
        return (int32_t)odom_pub.publish(&txmsg);
    else return 0;
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

int32_t monitor_canpkg_subscribe_cb_add(int32_t cmd, canpkg_sub_cb_t cb)
{
    int32_t i = 0;
    if (cb == NULL) return -1;

    rt_enter_critical();
    for (i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (monitor_canpkg_sub_table[i].cmd != 0) {
            if (monitor_canpkg_sub_table[i].cmd == cmd) {
                monitor_canpkg_sub_table[i].sub_cb = cb;
                 break;
            }
        }
        else {
            monitor_canpkg_sub_table[i].cmd = cmd;
            monitor_canpkg_sub_table[i].sub_cb = cb;
            break;
        }
    }
    rt_exit_critical();
    if (i == MSG_SUB_TABLE_NUM_MAX) 
        return -2;
    
    return 0;
}

int32_t monitor_canpkg_subscribe_cb_detach(int32_t cmd)
{
    int32_t i = 0;
    rt_enter_critical();
    for (i = 0; i < MSG_SUB_TABLE_NUM_MAX; i++) {
        if (monitor_canpkg_sub_table[i].cmd == cmd) { 
            monitor_canpkg_sub_table[i].cmd = 0;
            monitor_canpkg_sub_table[i].sub_cb = NULL;
        }
    }
    rt_exit_critical();
    return 0;
}













