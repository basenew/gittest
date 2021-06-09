#ifndef __APP_MODULES_VERSION_H__
#define __APP_MODULES_VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"


	

uint8_t core_hw_version_get(void);         //获取核心板硬件版本号
uint8_t board_hw_version_get(void);        //获取底板硬件版本号


#ifdef __cplusplus
}
#endif


#endif



