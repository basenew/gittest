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
#define  TASK_STACK_SIZE_KEY            (1024)
#define  TASK_STACK_SIZE_SW             (1024)
#define  TASK_STACK_SIZE_LOG            (4096)
#define  TASK_STACK_SIZE_OTA            (2048)

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
#define TASK_PRIORITY_OTA 50
#define TASK_PRIORITY_SW 51
#define TASK_PRIORITY_KEY 52


#define TASK_PRIORITY_LOG 245
// resvered 246~249
// finsh 250
// resvered 251~254
// idle 255





#ifdef __cplusplus
}
#endif

#endif /* #define __APP_CFG_H__ */


