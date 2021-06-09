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
#include "power_management.h"
//#include "flash_map.h"
//#include "iap.h"
//#include "easyflash.h"


#define LOG_TAG              "power"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>




static void power_ctrl_gpio_init(void)
{
    rt_pin_mode(PIN_POWER_3V3_SYS_PF13,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_RES1_PF14,         PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_RES2_PG0,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER0_PA0,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER1_PH2,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER2_PH3,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER3_PA6,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER4_PF15,      PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER5_PG1,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER6_PE8,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_DRIVER7_PE10,      PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_3V3_SYS_PF13,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_TDK_PF1,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_24V_TDK_PI11,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_5V9_REMOTE_PC3,        PIN_MODE_OUTPUT);
	
    rt_pin_mode(PIN_POWER_12V_KEY_LED_PI9,   PIN_MODE_OUTPUT);
	
	//检测是否升级后重启，保证升级时各模块电源不掉电
	
/*	POWER_12V_KEY_LED_ENABLE;
	
	

    rt_thread_mdelay(10);
	POWER_3V3_SYS_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER0_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER1_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER2_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER3_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER4_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER5_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER6_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER7_ENABLE;
    rt_thread_mdelay(200);
//	POWER_12V_RES1_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES2_ENABLE;
//    rt_thread_mdelay(200);
	POWER_12V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);*/
	
	
}
void power_ctrl_all_turn_on(void)
{
    rt_thread_mdelay(10);
	POWER_3V3_SYS_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER0_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER1_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER2_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER3_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER4_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER5_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER6_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER7_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES1_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES2_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
}

void power_ctrl_all_turn_off(void)
{
	POWER_3V3_SYS_DISABLE;
	POWER_24V_DRIVER0_DISABLE;
	POWER_24V_DRIVER1_DISABLE;
	POWER_24V_DRIVER2_DISABLE;
	POWER_24V_DRIVER3_DISABLE;
	POWER_24V_DRIVER4_DISABLE;
	POWER_24V_DRIVER5_DISABLE;
	POWER_24V_DRIVER6_DISABLE;
	POWER_24V_DRIVER7_DISABLE; 
	POWER_12V_RES1_DISABLE;
	POWER_12V_RES2_DISABLE;
	POWER_12V_TDK_DISABLE;
	POWER_24V_TDK_DISABLE;
	POWER_5V9_REMOTE_DISABLE;
}

void power_into_wrok(void)
{
	POWER_12V_KEY_LED_ENABLE;
    rt_thread_mdelay(10);
	POWER_3V3_SYS_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER0_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER1_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER2_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER3_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER4_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER5_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER6_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_24V_DRIVER7_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES1_ENABLE;
//    rt_thread_mdelay(200);
//	POWER_12V_RES2_ENABLE;
//    rt_thread_mdelay(200);
	POWER_12V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
}

void power_into_standby(void)
{

//	POWER_3V3_SYS_DISABLE;
//	POWER_24V_DRIVER0_DISABLE;
//	POWER_24V_DRIVER1_DISABLE;
//	POWER_24V_DRIVER2_DISABLE;
//	POWER_24V_DRIVER3_DISABLE;
//	POWER_24V_DRIVER4_DISABLE;
//	POWER_24V_DRIVER5_DISABLE;
//	POWER_24V_DRIVER6_DISABLE;
//	POWER_24V_DRIVER7_DISABLE; 
	POWER_12V_RES1_DISABLE;
	POWER_12V_RES2_DISABLE;
//	POWER_12V_TDK_DISABLE;
//	POWER_24V_TDK_DISABLE;
	POWER_5V9_REMOTE_DISABLE;
}
void power_driver_ctrl(uint8_t driver_num, uint8_t power_state)
{
	switch(driver_num)
	{
		case POWER_DRIVER_LEFT_FRONT_STEER:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER0_ENABLE;}
			else{
				POWER_24V_DRIVER0_DISABLE;}
			break;
				
		case POWER_DRIVER_LEFT_FRONT_TRAVEL:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER1_ENABLE;}
			else{
				POWER_24V_DRIVER1_DISABLE;}
			break;
				
		case POWER_DRIVER_RIGHT_FRONT_STEER:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER2_ENABLE;}
			else{
				POWER_24V_DRIVER2_DISABLE;}
			break;
				
		case POWER_DRIVER_RIGHT_FRONT_TRAVEL:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER3_ENABLE;}
			else{
				POWER_24V_DRIVER3_DISABLE;}
			break;
				
		case POWER_DRIVER_LEFT_BACK_STEER:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER4_ENABLE;}
			else{
				POWER_24V_DRIVER4_DISABLE;}
			break;
				
		case POWER_DRIVER_LEFT_BACK_TRAVEL:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER5_ENABLE;}
			else{
				POWER_24V_DRIVER5_DISABLE;}
			break;
				
		case POWER_DRIVER_RIGHT_BACK_STEER:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER6_ENABLE;}
			else{
				POWER_24V_DRIVER6_DISABLE;}
			break;
				
		case POWER_DRIVER_RIGHT_BACK_TRAVEL:
			if(power_state == POWER_ON){
				POWER_24V_DRIVER7_ENABLE;}
			else{
				POWER_24V_DRIVER7_DISABLE;}
			break;
	}
}	

void power_set_status(uint32_t status)
{
    if(READBIT(status,0)) {if(!POWER_3V3_SYS_STATUS)          {POWER_3V3_SYS_ENABLE;           rt_thread_delay(200);}}else{POWER_3V3_SYS_DISABLE;}
    if(READBIT(status,1)) {if(!POWER_24V_DRIVER0_STATUS)      {POWER_24V_DRIVER0_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER0_DISABLE;}
    if(READBIT(status,2)) {if(!POWER_24V_DRIVER1_STATUS)      {POWER_24V_DRIVER1_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER1_DISABLE;}
    if(READBIT(status,3)) {if(!POWER_24V_DRIVER2_STATUS)      {POWER_24V_DRIVER2_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER2_DISABLE;}
    if(READBIT(status,4)) {if(!POWER_24V_DRIVER3_STATUS)      {POWER_24V_DRIVER3_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER3_DISABLE;}
    if(READBIT(status,5)) {if(!POWER_24V_DRIVER4_STATUS)      {POWER_24V_DRIVER4_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER4_DISABLE;}
    if(READBIT(status,6)) {if(!POWER_24V_DRIVER5_STATUS)      {POWER_24V_DRIVER5_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER5_DISABLE;}
    if(READBIT(status,7)) {if(!POWER_24V_DRIVER6_STATUS)      {POWER_24V_DRIVER6_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER6_DISABLE;}
    if(READBIT(status,8)) {if(!POWER_24V_DRIVER7_STATUS)      {POWER_24V_DRIVER7_ENABLE;       rt_thread_delay(200);}}else{POWER_24V_DRIVER7_DISABLE;}
    if(READBIT(status,9)) {if(!POWER_12V_RES1_STATUS)         {POWER_12V_RES1_ENABLE;          rt_thread_delay(200);}}else{POWER_12V_RES1_DISABLE;}
    if(READBIT(status,10)) {if(!POWER_12V_RES2_STATUS)         {POWER_12V_RES2_ENABLE;          rt_thread_delay(200);}}else{POWER_12V_RES2_DISABLE;}
    if(READBIT(status,11)) {if(!POWER_12V_TDK_STATUS)          {POWER_12V_TDK_ENABLE;           rt_thread_delay(200);}}else{POWER_12V_TDK_DISABLE;}
    if(READBIT(status,12)) {if(!POWER_24V_TDK_STATUS)          {POWER_24V_TDK_ENABLE;           rt_thread_delay(200);}}else{POWER_24V_TDK_DISABLE;}
    if(READBIT(status,13)) {if(!POWER_5V9_REMOTE_STATUS)       {POWER_5V9_REMOTE_ENABLE;        rt_thread_delay(200);}}else{POWER_5V9_REMOTE_DISABLE;}
}

void power_up(void)
{
	POWER_12V_KEY_LED_ENABLE;
    rt_thread_mdelay(10);
	POWER_3V3_SYS_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER0_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER1_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER2_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER3_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER4_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER5_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER6_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER7_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES1_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_RES2_ENABLE;
    rt_thread_mdelay(200);
	POWER_12V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_TDK_ENABLE;
    rt_thread_mdelay(200);
	POWER_5V9_REMOTE_ENABLE;
    rt_thread_mdelay(200);
}

void power_up_driver(void)
{
    rt_thread_mdelay(200);
	POWER_24V_DRIVER0_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER1_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER2_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER3_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER4_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER5_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER6_ENABLE;
    rt_thread_mdelay(200);
	POWER_24V_DRIVER7_ENABLE;
    rt_thread_mdelay(200);
}

void power_set(uint32_t status)
{
    if(READBIT(status,0)) {if(!POWER_3V3_SYS_STATUS)          {POWER_3V3_SYS_ENABLE;           }}else{POWER_3V3_SYS_DISABLE;}
    if(READBIT(status,1)) {if(!POWER_24V_DRIVER0_STATUS)      {POWER_24V_DRIVER0_ENABLE;       }}else{POWER_24V_DRIVER0_DISABLE;}
    if(READBIT(status,2)) {if(!POWER_24V_DRIVER1_STATUS)      {POWER_24V_DRIVER1_ENABLE;       }}else{POWER_24V_DRIVER1_DISABLE;}
    if(READBIT(status,3)) {if(!POWER_24V_DRIVER2_STATUS)      {POWER_24V_DRIVER2_ENABLE;       }}else{POWER_24V_DRIVER2_DISABLE;}
    if(READBIT(status,4)) {if(!POWER_24V_DRIVER3_STATUS)      {POWER_24V_DRIVER3_ENABLE;       }}else{POWER_24V_DRIVER3_DISABLE;}
    if(READBIT(status,5)) {if(!POWER_24V_DRIVER4_STATUS)      {POWER_24V_DRIVER4_ENABLE;       }}else{POWER_24V_DRIVER4_DISABLE;}
    if(READBIT(status,6)) {if(!POWER_24V_DRIVER5_STATUS)      {POWER_24V_DRIVER5_ENABLE;       }}else{POWER_24V_DRIVER5_DISABLE;}
    if(READBIT(status,7)) {if(!POWER_24V_DRIVER6_STATUS)      {POWER_24V_DRIVER6_ENABLE;       }}else{POWER_24V_DRIVER6_DISABLE;}
    if(READBIT(status,8)) {if(!POWER_24V_DRIVER7_STATUS)      {POWER_24V_DRIVER7_ENABLE;       }}else{POWER_24V_DRIVER7_DISABLE;}
    if(READBIT(status,9)) {if(!POWER_12V_RES1_STATUS)         {POWER_12V_RES1_ENABLE;          }}else{POWER_12V_RES1_DISABLE;}
    if(READBIT(status,10)) {if(!POWER_12V_RES2_STATUS)         {POWER_12V_RES2_ENABLE;          }}else{POWER_12V_RES2_DISABLE;}
    if(READBIT(status,11)) {if(!POWER_12V_TDK_STATUS)          {POWER_12V_TDK_ENABLE;           }}else{POWER_12V_TDK_DISABLE;}
    if(READBIT(status,12)) {if(!POWER_24V_TDK_STATUS)          {POWER_24V_TDK_ENABLE;           }}else{POWER_24V_TDK_DISABLE;}
    if(READBIT(status,13)) {if(!POWER_5V9_REMOTE_STATUS)       {POWER_5V9_REMOTE_ENABLE;        }}else{POWER_5V9_REMOTE_DISABLE;}
}

uint32_t power_get_status(void)
{
	uint32_t status = 0;
    
    (void)(POWER_3V3_SYS_STATUS          ? SETBIT(status,0) : CLEARBIT(status,0));
    (void)(POWER_24V_DRIVER0_STATUS      ? SETBIT(status,1) : CLEARBIT(status,1));
    (void)(POWER_24V_DRIVER1_STATUS      ? SETBIT(status,2) : CLEARBIT(status,2));
    (void)(POWER_24V_DRIVER2_STATUS      ? SETBIT(status,3) : CLEARBIT(status,3));
    (void)(POWER_24V_DRIVER3_STATUS      ? SETBIT(status,4) : CLEARBIT(status,4));
    (void)(POWER_24V_DRIVER4_STATUS      ? SETBIT(status,5) : CLEARBIT(status,5));
    (void)(POWER_24V_DRIVER5_STATUS      ? SETBIT(status,6) : CLEARBIT(status,6));
    (void)(POWER_24V_DRIVER6_STATUS      ? SETBIT(status,7) : CLEARBIT(status,7));
    (void)(POWER_24V_DRIVER7_STATUS      ? SETBIT(status,8) : CLEARBIT(status,8));
    (void)(POWER_12V_RES1_STATUS         ? SETBIT(status,9) : CLEARBIT(status,9));
    (void)(POWER_12V_RES2_STATUS         ? SETBIT(status,10) : CLEARBIT(status,10));
    (void)(POWER_12V_TDK_STATUS          ? SETBIT(status,11) : CLEARBIT(status,11));
    (void)(POWER_24V_TDK_STATUS          ? SETBIT(status,12) : CLEARBIT(status,12));
    (void)(POWER_5V9_REMOTE_STATUS       ? SETBIT(status,13) : CLEARBIT(status,13));
	
    return status;
}
#include <stdio.h>
void power_set_single_power(int sw,int onoff)
{
    int set = (onoff==POWER_ON)?sw:0;
    int pow_status = (power_get_status() & (~sw)) | set;
    
    power_set(pow_status);
    int sw_status = power_get_status();
    printf("%s(%d)(%.8x):%.8x %.8x\r\n",__FUNCTION__,onoff,sw,pow_status,sw_status);
}
void motor_power_on(void)
{

    int pow_status = power_get_status()  | POWER_24V_DRIVER_ALL;
    LOG_I("motor_power_on");
    power_set_status(pow_status);
    
}

void motor_power_off(void)
{

    int pow_status = power_get_status() & (~POWER_24V_DRIVER_ALL);
    LOG_I("motor_power_off");
    power_set_status(pow_status);
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


