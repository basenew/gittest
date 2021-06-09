#ifndef __CURR_SAMPLE_H__
#define __CURR_SAMPLE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
#define CURR_NUM   4
    
int32_t curr_sample_init(void);       //��������ʼ��
void curr_read(void);                 //��ȡ�������IC����
uint16_t curr_get_data(uint8_t ch);   //��ȡ��ǰ��·����ֵ
void curr_print(void);                //��ǰ������Ϣ��ӡ����
    
    
#ifdef __cplusplus
}
#endif


#endif

