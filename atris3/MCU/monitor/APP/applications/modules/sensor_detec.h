#ifndef __APP_MODULES_SENSOR_DETEC_H__
#define __APP_MODULES_SENSOR_DETEC_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "board.h"

#define   PIN_DT35_DET1_Q1_IN_PD1                  GET_PIN(D, 1)
#define   PIN_DT35_DET1_Q2_IN_PD4                  GET_PIN(D, 4)
#define   PIN_DT35_DET2_Q1_IN_PD7                  GET_PIN(D, 7)
#define   PIN_DT35_DET2_Q2_IN_PG9                  GET_PIN(G, 9)

int32_t  sensor_detec_init(void);           //传感器检测初始化
int32_t sensor_get_hsu_temp(void);          //获取温湿度传感器的温度信息
uint16_t sensor_get_hsu_hum(void);          //获取温湿度传感器的湿度信息
uint8_t sensor_get_dt35(void);              //获取防跌落传感器状态


#ifdef __cplusplus
}
#endif

#endif




