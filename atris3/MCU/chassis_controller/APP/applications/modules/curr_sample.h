#ifndef __CURR_SAMPLE_H__
#define __CURR_SAMPLE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
#define CURR_NUM   4
    
int32_t curr_sample_init(void);       //电流检测初始化
void curr_read(void);                 //读取电流检测IC数据
uint16_t curr_get_data(uint8_t ch);   //获取当前各路电流值
void curr_print(void);                //当前电流信息打印函数
    
    
#ifdef __cplusplus
}
#endif


#endif

