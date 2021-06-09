#ifndef __APP_MODULES_BRAKE_H__
#define __APP_MODULES_BRAKE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"


enum
{
    BRAKE_STATUS_UNLOCK  = 0,       //刹车解锁
    BRAKE_STATUS_LOCK,              //刹车锁定
};

enum
{
    BRAKE_CAUSE_REBOOT = 0,         //开机自动刹车
	BRAKE_CAUSE_E_STOP,             //急停刹车
    BRAKE_CAUSE_ANTI,               //防撞条刹车
    BRAKE_CAUSE_CHARGEGUN,          //直充刹车
    BRAKE_CAUSE_HOST,               //主控端刹车命令刹车
	  BRAKE_CAUSE_CLIFF,
    //最多8种原因
};


typedef struct
{
    int8_t     status;    // 刹车状态
    uint8_t    cause;     // 刹车原因
    rt_mutex_t lock;
} brake_t;
	
	
void brake_get_status(brake_t* _brake);            //获取当前刹车信息
void brake_set(uint8_t cause, uint8_t _status);    //刹车设置
void brake_set_byhost(uint8_t status);             //主控设置刹车状态
void anti_brake_process(void);                     //防撞条刹车处理函数
void brake_unlock_byestop(void);                   //解除急停刹车
uint8_t get_brake_status(void);                    //获取当前刹车状态
uint8_t get_brake_cause(void);                     //获取当前刹车原因

void chassis_transport_mode_set(uint8_t _staus);   //运输模式设置			运输模式：机器运输时不能将轮子调整到八字状态。
uint8_t chassis_transport_mode_get(void);          //获取运输模式状态

#ifdef __cplusplus
}
#endif


#endif



