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
#include "version.h"


#define LOG_TAG              "version"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>


#define   PIN_CORE_HW_VER1_PD8                     GET_PIN(D, 8)
#define   PIN_CORE_HW_VER2_PD9                     GET_PIN(D, 9)
#define   PIN_CORE_HW_VER3_PD10                    GET_PIN(D, 10)

#define   PIN_BOARD_HW_VER1_PI5                    GET_PIN(I, 5)
#define   PIN_BOARD_HW_VER2_PI6                    GET_PIN(I, 6)
#define   PIN_BOARD_HW_VER3_PI7                    GET_PIN(I, 7)

static uint8_t core_hw_version = 0;
static uint8_t board_hw_vesion = 0;

uint8_t core_hw_version_get(void)
{
	uint8_t temp1=0, temp2=0, temp3=0;
	
    rt_pin_mode(PIN_CORE_HW_VER1_PD8,          PIN_MODE_INPUT);
    rt_pin_mode(PIN_CORE_HW_VER2_PD9,          PIN_MODE_INPUT);
    rt_pin_mode(PIN_CORE_HW_VER3_PD10,          PIN_MODE_INPUT);
	
	temp1 = rt_pin_read(PIN_CORE_HW_VER1_PD8);
	temp2 = rt_pin_read(PIN_CORE_HW_VER2_PD9);
	temp3 = rt_pin_read(PIN_CORE_HW_VER3_PD10);
	core_hw_version =  temp3 | (temp2<<1) | (temp1<<2);
	
	return core_hw_version;
}

uint8_t board_hw_version_get(void)
{
	uint8_t temp1=0, temp2=0, temp3=0;
	
    rt_pin_mode(PIN_BOARD_HW_VER1_PI5,          PIN_MODE_INPUT);
    rt_pin_mode(PIN_BOARD_HW_VER2_PI6,          PIN_MODE_INPUT);
    rt_pin_mode(PIN_BOARD_HW_VER3_PI7,          PIN_MODE_INPUT);
	
	temp1 = rt_pin_read(PIN_BOARD_HW_VER1_PI5);
	temp2 = rt_pin_read(PIN_BOARD_HW_VER1_PI5);
	temp3 = rt_pin_read(PIN_BOARD_HW_VER1_PI5);
	board_hw_vesion =  temp1 | (temp2<<1) | (temp3<<2);
	
	return board_hw_vesion;
}


//--------------------------------------------------------------------------------------------------------------
#include "finsh.h"




