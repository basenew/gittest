#ifndef __APP_MODULES_DISPLAY_H__
#define __APP_MODULES_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif




typedef enum
{
    DISPALY_NORMAL     = 0,   // 正常状态
    DISPALY_WARNNIG,          // 告警状态
    DISPALY_TESTING,          // 测试状态
} display_state_t;



int32_t display_init(void);                         //电量显示屏初始化函数

void display_state_write(display_state_t _state);   //设置显示屏状态
display_state_t display_state_read(void);           //读取显示屏状态

void display_loop(void);                            //电量显示屏显示信息设置


#ifdef __cplusplus
}
#endif


#endif




