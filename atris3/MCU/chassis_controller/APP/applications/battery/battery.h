#ifndef __BATTERY_H__
#define __BATTERY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define BAT_SOC_LEVEL1 5         // 低于此电量自动关机且不能开机
#define BAT_CELLVOLT_LEVEL1 3000 //最小电芯电压-低于此电压则不允许开机,且要自动关机

#define BAT_SAMPLE_PERIOD (1000)


enum
{
    BAT_MSG_IDX_IDLE = 0,
    BAT_MSG_IDX_CTRL,
    BAT_MSG_IDX_SYNC,
    BAT_MSG_IDX_GETLOG,
    BAT_MSG_IDX_GETVER,
    BAT_MSG_IDX_BMS_NOTIFY,
    BAT_MSG_INDEX_KEY,
    BAT_MSG_IDX_MAX,
};

typedef struct
{
    int32_t idx;
    int32_t buff[2];
} bat_msg_t;

enum
{//battery status
    BAT_STATUS_IDLE = 0,   // 电池空闲
    BAT_STATUS_CHARGING,   // 电池充电
    BAT_STATUS_DISCHARING, // 电池放电
    BAT_STATUS_COMERR,     // 电池通信异常
    BAT_STATUS_FULL,       // 充电充满
};

enum
{//charger status
    BAT_CHARGER_STATUS_IDLE = 1, //没有充电
    BAT_CHARGER_STATUS_ING = 2,  //正在充电
    BAT_CHARGER_STATUS_FULL = 3, //满充
    BAT_CHARGER_STATUS_ERR = 4,  //充电异常
};

enum
{
    SOURCE_IDLE = 0,
    SOURCE_ADAPTER,
    SOURCE_BASE,
};

typedef struct
{
    uint8_t status;  //状态
    uint32_t moment; //状态变化的时刻
} charger_gun_t;

typedef struct
{
    uint8_t status;
    charger_gun_t adapter;
    charger_gun_t base;
    uint8_t charge_source;
} charger_t;


typedef struct
{
    
    uint16_t volt_01V;     //电压
    int16_t curr_01A;      //电流
    uint8_t soc;       //电量
    uint8_t status;    //状态
    uint8_t temp_high;      //最高温度
    uint16_t cellvolt_low_mV; //最小电芯电压
    uint16_t chg_dischg_cnts; //充放电总次数
	uint8_t ex_io_status;
	uint8_t sleep_flag;

} bat_data_t;


typedef struct
{
    rt_mq_t mq;               //消息队列
    charger_t charger;        //充电器
    //bms_data_t *pbms;         //bms数据指针
    bat_data_t data;          //电池综合数据
} bat_t;


int32_t battery_init(void);
int32_t bat_get_data(bat_data_t *pdata);
int32_t bat_get_charger(charger_t *pcharger);
int32_t bat_is_refuse_to_startup(void);

bat_data_t* bat_get_data_ptr(void);
void bat_set_status(uint8_t sta);

int32_t battery_init(void);




#ifdef __cplusplus
}
#endif

#endif



