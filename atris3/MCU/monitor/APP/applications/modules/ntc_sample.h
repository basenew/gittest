#ifndef _APP_NTC_SAMPLE_H__
#define _APP_NTC_SAMPLE_H__

#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stdint.h"

enum
{
    NTC_IDX_CH1 = 0,		//NTC1
    NTC_IDX_CH2 = 0, 		//NTC2 
    ANTI_IDX_CH3 = 0,   	//��ײ��
};


typedef struct 
{
    uint8_t idx;
    
    uint8_t ch;
    
    uint32_t adc;
    
    double volt;
    
    double r;
    
    double temp;
    
} adc_ch_t;


int32_t ntc_init(void);	   				//adc��ʼ��
void ntc_sample(void);   				//ntc��������
adc_ch_t* ntc_get_obj(uint8_t idx);   	//��ȡNTC����ֵ
void ntc_get_temp_data(int8_t* temp);   //��ȡNTC�¶�ֵ
int getTempture(int ch);   				//��ȡ��·NTC�¶�ֵ
#ifdef __cplusplus
}
#endif

#endif


