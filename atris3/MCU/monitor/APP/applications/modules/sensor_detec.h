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

int32_t  sensor_detec_init(void);           //����������ʼ��
int32_t sensor_get_hsu_temp(void);          //��ȡ��ʪ�ȴ��������¶���Ϣ
uint16_t sensor_get_hsu_hum(void);          //��ȡ��ʪ�ȴ�������ʪ����Ϣ
uint8_t sensor_get_dt35(void);              //��ȡ�����䴫����״̬


#ifdef __cplusplus
}
#endif

#endif




