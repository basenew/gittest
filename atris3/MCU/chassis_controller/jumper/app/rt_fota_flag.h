#ifndef _RT_FOTA_FLAG_H_
#define _RT_FOTA_FLAG_H_

#include "stdint.h"

#define FLAG_OK 0
#define FLAG_ERROR 1

#define FLAG_YES 0
#define FLAG_NO 1

#define FLAG_FROM_DL 0
#define FLAG_FROM_BK 1

#define FLAG_TO_APP 0
#define FLAG_TO_BL 1

#define FLAG_INIT 0xA5


typedef struct 
{
    uint8_t jumper_version; /*jumper版本号 固定在结构体第一字节*/

    union 
    {
        uint8_t flags;
        struct
        {
            uint8_t jump_to_where     : 1; /*跳转目标分区*/
            uint8_t update_from       : 1; /*固件来源分区*/
            uint8_t force_up          : 1; /*强制升级*/
            uint8_t : 1;/*预留*/
            uint8_t : 1;
            uint8_t : 1;
            uint8_t : 1;
            uint8_t : 1;
        } sflag;
    } uflag;
    
    uint8_t initialized; /*flags是否已经初始化*/
    uint8_t errcode; /*错误码*/
    uint8_t tobl_cnt; /*连续跳转到bootloader自动升级的次数*/
    char app_version[24]; /*app当前版本号*/
    char bl_version[24];  /*bootloader当前版本号*/
    uint8_t reserved[11]; /*保留 使整个flag的字节数凑够4页，16字节/页*/
} fota_flags_t;


#endif

