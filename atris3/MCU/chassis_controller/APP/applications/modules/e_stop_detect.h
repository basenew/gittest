#ifndef __APP_MODULES_E_STOP_DETECT_H__
#define __APP_MODULES_E_STOP_DETECT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"


#define ESTOP_RELEASE	0
#define ESTOP_TRIGE		1
	

void e_stop_detect(void);	//��ͣ�źż��
int32_t e_stop_init(void);		//��ͣ�ź�GPIO��ʼ��
uint8_t e_stop_get_status(void);	//��ͣ�źŵ�ǰ״̬��ȡ


#ifdef __cplusplus
}
#endif


#endif



