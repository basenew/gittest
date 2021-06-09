/*
 * File      : rt_fota.h
 * COPYRIGHT (C) 2012-2018, Shanghai Real-Thread Technology Co., Ltd
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-09-22     warfalcon    the first version
 */

#ifndef _RT_FOTA_CFG_H_
#define _RT_FOTA_CFG_H_



/* RT-FOTA Version */
#define RT_FOTA_SW_VERSION      "1.0.0"


/* Enable Ymodem OTA */
//#define PKG_USING_YMODEM_OTA

/* FOTA application partition name */
#define RT_FOTA_APP_PART_NAME   "app"

/* FOTA bootloader partition name */
#define RT_FOTA_BL_PART_NAME   "bl"

/* FOTA download partition name */
#define RT_FOTA_DL_PART_NAME    "download"

/* FOTA backup partition name */
#define RT_FOTA_BK_PART_NAME    "backup"

#define RT_FOTA_HEADER_TYPE     "RBL"


/*the max cnt of jumping to bl continuously becauce of errors*/
#define RT_FOTA_TOBL_CNT_MAX    (10)

#define RT_FOTA_DEBUG



#endif /* _RT_FOTA_H_ */

