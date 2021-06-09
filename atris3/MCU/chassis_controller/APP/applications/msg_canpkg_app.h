/*
 * File      : tinyros_entries.h
 * This file is part of tinyros
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-22     Pinkie.Fu    initial version
 */

#ifndef _TINYROS_MSG_CANPKG_APP_H_
#define _TINYROS_MSG_CANPKG_APP_H_
#include <stdint.h>
#include <stdlib.h>


#ifdef __cplusplus
 extern "C" {
#endif
     


int32_t canpkg_app_init(void);		//canpkg数据发送接收进程初始化

#ifdef __cplusplus
}
#endif

#endif


