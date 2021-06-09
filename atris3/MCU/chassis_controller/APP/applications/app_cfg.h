/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : app_cfg.h
 * Brief      : app config

 * Change Logs
 * Date           Author        Version       Notes
 * 2020-06-18     wuxiaofeng    v1.0          
 *
 */

#ifndef __APP_CFG_H__
#define __APP_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif


/*是否使用硬件看门狗*/
#define  APP_USING_HW_WDG               (0)






/*---线程栈空间大小---*/
#define  TASK_STACK_SIZE_KEY            (512)
#define  TASK_STACK_SIZE_SW             (512)
#define  TASK_STACK_SIZE_REMOTECTRL     (1024)
#define  TASK_STACK_SIZE_LOG            (1024*3)
#define  TASK_STACK_SIZE_LOGUPLOAD      (1024*2)
#define  TASK_STACK_SIZE_OTA            (2048)
#define  TASK_STACK_SIZE_CONTROLLER     (2048)   
#define  TASK_STACK_SIZE_PRIORITY       (1024)   
#define  TASK_STACK_SIZE_ODOM           (2048)
#define  TASK_STACK_SIZE_VOLTAGEDET     (512)
#define  TASK_STACK_SIZE_UTILITY     	(768)
#define  TASK_STACK_SIZE_SENSOR         (1024)
#define  TASK_STACK_SIZE_CANPKG_APP     (2048)
#define  TASK_STACK_SIZE_POWER_MANAGEMENT (1024)

#define TASK_STACK_SIZE_BAT 			(1024)
#define TASK_STACK_SIZE_CAN2 			(1024)
#define TASK_STACK_SIZE_BMSCAN 			1024
#define TASK_STACK_SIZE_BMSLOG 			512
/*---线程优先级---*/
// reserved 0, 1, 2, 3
// soft_timer  4
// main 10
// etx 11
// erx 11
// tcpip 12
// reserved 13 14
// ros_msg 15, 16, 17 18
// resvered 19~49
#define TASK_PRIORITY_OTA 		 50
#define TASK_PRIORITY_SW         51
#define TASK_PRIORITY_KEY        52
#define TASK_PRIORITY_CONTROLLER 53
#define TASK_PRIORITY_ODOM       54
#define TASK_PRIORITY_REMOTECTRL 55
#define TASK_PRIORITY_PRIORITY   56
#define TASK_PRIORITY_VOLTAGEDET 57
#define TASK_PRIORITY_UTILITY    58
#define TASK_PRIORITY_SENSOR_ADC 59
#define TASK_PRIORITY_CANPKG_APP 60
#define TASK_PRIORITY_BAT        61
#define TASK_PRIORITY_BMSCAN     62
#define TASK_PRIORITY_CAN2       63
#define TASK_PRIORITY_BMSLOG     64
#define TASK_PRIORITYBMSLOG      65
#define TASK_PRIORITY_POWER_MANAGEMENT      66
#define TASK_PRIORITY_LOG        245
// resvered 246~249
// finsh 250
// resvered 251~254
// idle 255





#ifdef __cplusplus
}
#endif

#endif /* #define __APP_CFG_H__ */


