#ifndef _VOLTAGE_DETECT_H_
#define _VOLTAGE_DETECT_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"
	
#define VOLDET_CH_SYS_24V		0
#define VOLDET_CH_MCHARGE_24V	1
#define VOLDET_CH_ACHARGE_24V	2
#define VOLDET_CH_CHARGER_POLE	3
#define VOLDET_CH_SYS_12V		4
#define VOLDET_CH_SYS_5V		5
	

int32_t voltage_detect_init(void);		//电压检测初始化
uint16_t vol_get_data(uint8_t ch);		//获取当前电压检测值，mv
uint16_t get_voltage(uint8_t ch);		//WEB页：获取当前电压检测值, mv


#ifdef __cplusplus
}
#endif


#endif
