#ifndef __APP_MODULES_SENSOR_DETEC_H__
#define __APP_MODULES_SENSOR_DETEC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

int32_t  sensor_detec_init(void);           //传感器检测初始化
int32_t sensor_get_hsu_temp(void);          //获取温湿度传感器的温度信息
uint16_t sensor_get_hsu_hum(void);          //获取温湿度传感器的湿度信息


#ifdef __cplusplus
}
#endif

#endif




