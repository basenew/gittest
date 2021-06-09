/*
 * COPYRIGHT (C) Copyright 2012-2017; UBT TECH; SHENZHEN, CHINA
 *
 * File       : power_manager.c
 * Brief      : 电源控制管理

 * Change Logs
 * Date           Author        Version       Notes
 * 2018-12-18     wuxiaofeng    v1.0          
 *
 */
#include "common.h"
#include "power_ctrl.h"
#include "utility.h"


#define LOG_TAG              "power"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>



void spk_ctrl_set(uint8_t status)
{
	if(status == DF_ENABLE)
	{
		SPK_CTRL_ENABLE;		//打开功放喇叭
		LOG_W("enable the speaker.");
	}
	else
	{
		SPK_CTRL_DISABLE;		//关闭功放喇叭
		LOG_W("disable the speaker.");
	}
}

static void power_ctrl_gpio_init(void)
{
    rt_pin_mode(PIN_POWER_3V3_IMU_PA6,             PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_5V9_REMOTE_PC3,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DT35_G10_PD0,        PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_GATEWAY_PE6,         PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_MAST_PE7,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_3V3_PHY_PE15,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_5V_RFID_PF0,             PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_X86_PF3,             PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_SWITCH_PF5,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_RADAR_PF10,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_VOIP_PF12,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_3V3_SYS_PF13,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_3V3_GNSS_PG12,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_CAM_PPH9,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_RES1_PF2,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_RES2_PE2,            PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_RES3_PE10,           PIN_MODE_OUTPUT);
	
    rt_pin_mode(PIN_SPK_CTRL_PF7,           PIN_MODE_OUTPUT);
	
	POWER_3V3_IMU_ENABLE;
	POWER_3V3_SYS_ENABLE;
	POWER_3V3_GNSS_ENABLE;
	POWER_3V3_PHY_ENABLE;
//	POWER_5V_RFID_ENABLE;
//	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_X86_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_GATEWAY_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_CAM_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_RADAR_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DT35_G10_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_VOIP_ENABLE;
    rt_thread_mdelay(200);
	
	POWER_12V_SWITCH_ENABLE;
    rt_thread_mdelay(200);
//	POWER_24V_MAST_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES1_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES2_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_RES3_ENABLE;
//    rt_thread_mdelay(200);
	
	spk_open_start();
	
	LOG_W("power init ok.");
}

void power_ctrl_all_turn_on(void)
{   
	POWER_3V3_IMU_ENABLE;
	POWER_3V3_SYS_ENABLE;
	POWER_3V3_GNSS_ENABLE;
	POWER_3V3_PHY_ENABLE;
	POWER_5V_RFID_ENABLE;
	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_X86_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_GATEWAY_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_CAM_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_RADAR_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DT35_G10_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_VOIP_ENABLE;
    rt_thread_mdelay(200);
	
	POWER_12V_SWITCH_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_MAST_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES1_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES2_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_RES3_ENABLE;
    rt_thread_mdelay(200);

	LOG_W("turn on all power.\n");
}

void power_ctrl_all_turn_off(void)
{
	POWER_3V3_IMU_DISABLE;
	POWER_3V3_SYS_DISABLE;
	POWER_3V3_GNSS_DISABLE;
	POWER_3V3_PHY_DISABLE;
	POWER_5V_RFID_DISABLE;
	POWER_5V9_REMOTE_DISABLE;
	POWER_12V_X86_DISABLE;
	POWER_12V_SWITCH_DISABLE;
	POWER_12V_VOIP_DISABLE;
	POWER_24V_DT35_G10_DISABLE;
	POWER_24V_GATEWAY_DISABLE;
	POWER_24V_MAST_DISABLE;
	POWER_24V_RADAR_DISABLE;
	POWER_24V_CAM_DISABLE;
	POWER_12V_RES1_DISABLE;
	POWER_12V_RES2_DISABLE;
	POWER_24V_RES3_DISABLE;
}

void power_work_status(void)
{   
	POWER_3V3_IMU_ENABLE;
	POWER_3V3_SYS_ENABLE;
	POWER_3V3_GNSS_ENABLE;
	POWER_3V3_PHY_ENABLE;
//	POWER_5V_RFID_ENABLE;
//	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_X86_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_GATEWAY_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_CAM_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_RADAR_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DT35_G10_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_VOIP_ENABLE;
    rt_thread_mdelay(200);
	
	
	
	POWER_12V_SWITCH_ENABLE;
    rt_thread_mdelay(200);
//	POWER_24V_MAST_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES1_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES2_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_RES3_ENABLE;
//    rt_thread_mdelay(200);
	spk_open_start();

	LOG_W("set power work mode.\n");
}

void power_standby_status(void)
{
	POWER_3V3_IMU_DISABLE;
//	POWER_3V3_SYS_DISABLE;
	POWER_3V3_GNSS_DISABLE;
//	POWER_3V3_PHY_DISABLE;
	POWER_5V_RFID_DISABLE;
	POWER_5V9_REMOTE_DISABLE;
//	POWER_12V_X86_DISABLE;
//	POWER_12V_SWITCH_DISABLE;
	POWER_12V_VOIP_DISABLE;
	POWER_24V_DT35_G10_DISABLE;
//	POWER_24V_GATEWAY_DISABLE;
	POWER_24V_MAST_DISABLE;
	POWER_24V_RADAR_DISABLE;
	POWER_24V_CAM_DISABLE;
	POWER_12V_RES1_DISABLE;
	POWER_12V_RES2_DISABLE;
	POWER_24V_RES3_DISABLE;
	spk_ctrl_set(DF_DISABLE);
	
	LOG_W("set power standby mode.\n");
}

void power_set_status(uint32_t status)
{
    if(READBIT(status,0)) {if(!POWER_3V3_IMU_STATUS)          {POWER_3V3_IMU_ENABLE;      rt_thread_delay(200);}}else{POWER_3V3_IMU_DISABLE;}
    if(READBIT(status,1)) {if(!POWER_3V3_SYS_STATUS)          {POWER_3V3_SYS_ENABLE;      rt_thread_delay(200);}}else{POWER_3V3_SYS_DISABLE;}
    if(READBIT(status,2)) {if(!POWER_3V3_GNSS_STATUS)         {POWER_3V3_GNSS_ENABLE;     rt_thread_delay(200);}}else{POWER_3V3_GNSS_DISABLE;}
    if(READBIT(status,3)) {if(!POWER_3V3_PHY_STATUS)          {POWER_3V3_PHY_ENABLE;      rt_thread_delay(200);}}else{POWER_3V3_PHY_DISABLE;}
    if(READBIT(status,4)) {if(!POWER_5V_RFID_STATUS)          {POWER_5V_RFID_ENABLE;      rt_thread_delay(200);}}else{POWER_5V_RFID_DISABLE;}
    if(READBIT(status,5)) {if(!POWER_5V9_REMOTE_STATUS)       {POWER_5V9_REMOTE_ENABLE;   rt_thread_delay(200);}}else{POWER_5V9_REMOTE_DISABLE;}
    if(READBIT(status,6)) {if(!POWER_12V_X86_STATUS)          {POWER_12V_X86_ENABLE;      rt_thread_delay(200);}}else{POWER_12V_X86_DISABLE;}
    if(READBIT(status,7)) {if(!POWER_12V_SWITCH_STATUS)       {POWER_12V_SWITCH_ENABLE;   rt_thread_delay(200);}}else{POWER_12V_SWITCH_DISABLE;}
    if(READBIT(status,8)) {if(!POWER_12V_VOIP_STATUS)         {POWER_12V_VOIP_ENABLE;     rt_thread_delay(200);}}else{POWER_12V_VOIP_DISABLE;}
    if(READBIT(status,9)) {if(!POWER_24V_DT35_G10_STATUS)     {POWER_24V_DT35_G10_ENABLE; rt_thread_delay(200);}}else{POWER_24V_DT35_G10_DISABLE;}
    if(READBIT(status,10)) {if(!POWER_24V_GATEWAY_STATUS)     {POWER_24V_GATEWAY_ENABLE;  rt_thread_delay(200);}}else{POWER_24V_GATEWAY_DISABLE;}
    if(READBIT(status,11)) {if(!POWER_24V_MAST_STATUS)        {POWER_24V_MAST_ENABLE;     rt_thread_delay(200);}}else{POWER_24V_MAST_DISABLE;}
    if(READBIT(status,12)) {if(!POWER_24V_RADAR_STATUS)       {POWER_24V_RADAR_ENABLE;    rt_thread_delay(200);}}else{POWER_24V_RADAR_DISABLE;}
    if(READBIT(status,13)) {if(!POWER_24V_CAM_STATUS)         {POWER_24V_CAM_ENABLE;      rt_thread_delay(200);}}else{POWER_24V_CAM_DISABLE;}
    if(READBIT(status,14)) {if(!POWER_12V_RES1_STATUS)        {POWER_12V_RES1_ENABLE;     rt_thread_delay(200);}}else{POWER_12V_RES1_DISABLE;}
    if(READBIT(status,15)) {if(!POWER_12V_RES2_STATUS)        {POWER_12V_RES2_ENABLE;     rt_thread_delay(200);}}else{POWER_12V_RES2_DISABLE;}
    if(READBIT(status,16)) {if(!POWER_24V_RES3_STATUS)        {POWER_24V_RES3_ENABLE;     rt_thread_delay(200);}}else{POWER_24V_RES3_DISABLE;}
}

uint32_t power_get_status(void)
{
	uint32_t status = 0;
    
    (void)(POWER_3V3_IMU_STATUS       ? SETBIT(status,0) : CLEARBIT(status,0));
    (void)(POWER_3V3_SYS_STATUS       ? SETBIT(status,1) : CLEARBIT(status,1));
    (void)(POWER_3V3_GNSS_STATUS      ? SETBIT(status,2) : CLEARBIT(status,2));
    (void)(POWER_3V3_PHY_STATUS       ? SETBIT(status,3) : CLEARBIT(status,3));
    (void)(POWER_5V_RFID_STATUS       ? SETBIT(status,4) : CLEARBIT(status,4));
    (void)(POWER_5V9_REMOTE_STATUS    ? SETBIT(status,5) : CLEARBIT(status,5));
    (void)(POWER_12V_X86_STATUS       ? SETBIT(status,6) : CLEARBIT(status,6));
    (void)(POWER_12V_SWITCH_STATUS    ? SETBIT(status,7) : CLEARBIT(status,7));
    (void)(POWER_12V_VOIP_STATUS      ? SETBIT(status,8) : CLEARBIT(status,8));
    (void)(POWER_24V_DT35_G10_STATUS  ? SETBIT(status,9) : CLEARBIT(status,9));
    (void)(POWER_24V_GATEWAY_STATUS   ? SETBIT(status,10) : CLEARBIT(status,10));
    (void)(POWER_24V_MAST_STATUS      ? SETBIT(status,11) : CLEARBIT(status,11));
    (void)(POWER_24V_RADAR_STATUS     ? SETBIT(status,12) : CLEARBIT(status,12));
    (void)(POWER_24V_CAM_STATUS       ? SETBIT(status,13) : CLEARBIT(status,13));
    (void)(POWER_12V_RES1_STATUS      ? SETBIT(status,14) : CLEARBIT(status,14));
    (void)(POWER_12V_RES2_STATUS      ? SETBIT(status,15) : CLEARBIT(status,15));
    (void)(POWER_24V_RES3_STATUS      ? SETBIT(status,16) : CLEARBIT(status,16));
	
    return status;
}


int32_t power_ctrl_init(void)
{
    
    power_ctrl_gpio_init();


    
//    rt_thread_t thread = rt_thread_create("task_power", \
//                                       power_task_main, \
//                                               RT_NULL, \
//                                 TASK_STACK_SIZE_POWER, \
//                                   TASK_PRIORITY_POWER, \
//                                       20);
//    if (thread != RT_NULL) {
//        rt_thread_startup(thread);
//    }
//    else {
//        LOG_E("task_power init fail!");
//        return -1;
//    }



//    can_add_msg_service(CANCMD_POWER_STATE_ASK, power_ctrl_status_query, NO, 1,NO);
//    can_add_msg_service(CANCMD_POWER_STATE_SET, power_ctrl_switch_set,   NO, 4,YES);
//    can_add_msg_service(CANCMD_MONITOR_HEARTBEAT_ACK, power_ctrl_batcomm_status, NO, 2,NO);
    
    return 0;
}




//--------------------------------------------------------------------------------------------------------------
#include "finsh.h"
static void power_all(uint8_t argc, char **argv)
{
    if(argc != 2)
    {
        rt_kprintf("Please input: power_all <1/0>\n");
    }
    else
    {
        if(atoi(argv[1]) != 0) {
            power_ctrl_all_turn_on();
        }
        else {
            power_ctrl_all_turn_off();
        }
        
    }
}
MSH_CMD_EXPORT(power_all, contrl all power that under contrl.);


static void power_status(uint8_t argc, char **argv)
{
    if(argc != 1)
    {
        rt_kprintf("Please input: power_status\n");
    }
    else
    {
        uint32_t cur_status = power_get_status();
        rt_kprintf("cur_status: 0x%04X\n", cur_status);
    }
}
MSH_CMD_EXPORT(power_status, get all power status.);


static void power_status_set(uint8_t argc, char **argv)
{
    if(argc != 2)
    {
        rt_kprintf("Please input: power_status_set <0~>\n");
    }
    else
    {
		
        uint32_t set_status = atoi(argv[1]);
		power_set_status(set_status);
    }
}
MSH_CMD_EXPORT(power_status_set, set power status.);


