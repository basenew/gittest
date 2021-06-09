#ifndef ____LCOALPACKAGES_SW_DEF_H__
#define ____LCOALPACKAGES_SW_DEF_H__

#include "sw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sw_unit sw_unit_t;
struct sw_unit
{
    sw_idx_t        Idx;

    int32_t         Pin;

    uint32_t        Ms;             //采样周期 最小为 SW_POLL_SLICE

    uint32_t        Debonce;        //消抖时间 一般设置为 2倍 采样周期

    uint8_t         LastStatus;     //上一次状态
    
    uint8_t         CurStatus;      //当前状态

    uint8_t         En;             //是否使能

    int32_t         (*Init)(sw_unit_t*);

    uint8_t         (*Read)(sw_unit_t*);

    sw_callback_t    CallBack;

    uint32_t        Tmr;           // 计时器
    uint32_t        DbCnt;         // 消抖计次

};

sw_unit_t* sw_port_table(void);
uint8_t sw_port_table_size(void);

#ifdef __cplusplus
}
#endif

#endif

