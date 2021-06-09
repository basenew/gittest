#ifndef __APP_MODULES_SENSOR_DETEC_H__
#define __APP_MODULES_SENSOR_DETEC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

int32_t  sensor_detec_init(void);           //����������ʼ��
int32_t sensor_get_hsu_temp(void);          //��ȡ��ʪ�ȴ��������¶���Ϣ
uint16_t sensor_get_hsu_hum(void);          //��ȡ��ʪ�ȴ�������ʪ����Ϣ


#ifdef __cplusplus
}
#endif

#endif




