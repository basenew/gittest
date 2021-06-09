#ifndef __APP_MODULES_VERSION_H__
#define __APP_MODULES_VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"


	

uint8_t core_hw_version_get(void);         //��ȡ���İ�Ӳ���汾��
uint8_t board_hw_version_get(void);        //��ȡ�װ�Ӳ���汾��


#ifdef __cplusplus
}
#endif


#endif



