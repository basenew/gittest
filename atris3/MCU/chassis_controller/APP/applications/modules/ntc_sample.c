
#include "common.h"
#include "app_cfg.h"
#include "board.h"
#include "ntc_sample.h"
#include "brake.h"

#define LOG_TAG              "ntc"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

#define ADC_DEV_NAME        "adc3"      
#define REFER_VOLTAGE        (3.3)        
#define CONVERT_BITS        (1 << 12)

#define NTC_TEMP_MAX  (105.0)
#define NTC_TEMP_MIN (-40.0)

#define ANTI_CLOSED_TYPE_TRIGED_ADC				248 // 0.2V   		//0.5V -- 620  

#define ANTI_CLOSED_TYPE_NOT_TRIGED_ADC_L		868		//0.7V
#define ANTI_CLOSED_TYPE_NOT_TRIGED_ADC_H		2110	//1.7V

#define ANTI_CLOSED_TYPE_NOT_CONNECTED_ADC		3723	//3.0V


static rt_adc_device_t adc_dev;

static adc_ch_t ntc_table[5] = 
{
    {NTC_IDX_CH1, 9, 0, 0, 0, 0},
    {NTC_IDX_CH2, 14, 0, 0, 0, 0},
    {NTC_IDX_CH3, 15, 0, 0, 0, 0},
    {NTC_IDX_CH4, 6, 0, 0, 0, 0},
    {ANTI_IDX_CH5, 4, 0, 0, 0, 0},
};
#define NTC_TABLE_SIZE (sizeof(ntc_table) / sizeof(adc_ch_t))

static uint8_t anti_value = 0;
static int8_t ntc_temp[4] = {0};


int32_t  ntc_init(void)
{
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        LOG_E("adc %s device can't find!\n", ADC_DEV_NAME);
        return -DF_ERR;
    }

    for (uint8_t i = 0; i < NTC_TABLE_SIZE; i++) 
    {
        if (rt_adc_enable(adc_dev, ntc_table[i].ch) != RT_EOK)
        {
            LOG_W("adc device %s enable channle [%d] failed!\n", ADC_DEV_NAME, ntc_table[i].ch);
        }
    }

    return DF_OK;
}


const float Rp = 10000.0; //10K
const float T2 = (273.15+25.0);//T2
const float Bx = 3435.0;//B
const float Ka = 273.15; 
    
#include "math.h"
float Get_Temp(float NTC_Res)
{
	float Rt;
	float temp;
	Rt = NTC_Res;
	
	//like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)),  
	temp = Rt/Rp;
	temp = log(temp); //ln(Rt/Rp)
	temp /= Bx;       //ln(Rt/Rp)/B
	temp += (1/T2);
	temp = 1/(temp);
	temp -= Ka;
	return temp;
}

void ntc_sample(void)
{
    adc_ch_t* pobj = RT_NULL;
    double   value_temp = 0;
	uint8_t anti_value_new = 0;
	static uint8_t anti_det_cnt = 0;
    
    for (uint8_t i = 0; i < NTC_TABLE_SIZE; i++) 
    {
        pobj = &ntc_table[i];
        pobj->adc = rt_adc_read(adc_dev, pobj->ch);
		if(i == 4)
		{
//			LOG_D("the adc is :[ch]%d,  %d\n",i+1, pobj->adc );
			if(pobj->adc < ANTI_CLOSED_TYPE_TRIGED_ADC)
			{
				anti_value_new |= (1<<0);
			}
			else if(pobj->adc  > ANTI_CLOSED_TYPE_NOT_CONNECTED_ADC)
			{
				anti_value_new |= (2<<0);
			}
			
			if(anti_value_new != anti_value)
			{
				anti_det_cnt ++;
				if(anti_det_cnt >= 10)
				{
					anti_det_cnt = 0;
					anti_value = anti_value_new;
					
					brake_set(BRAKE_CAUSE_ANTI, BRAKE_STATUS_LOCK);
					LOG_W("the anti_value: %d\n",anti_value );
					/*keep the error value 200ms for hysteresis keep the error value,so that it can be send to ROS
					fix issue :the anti_value refresh too offen to send the value to ros  
					 send to ROS every 1000ms*/
					if(anti_value !=0)
					{
						rt_thread_mdelay(1000);					
					}
				}
			}
			else
			{
				anti_det_cnt = 0;
			}
	//			pobj->volt = pobj->adc * REFER_VOLTAGE / CONVERT_BITS;
	//			LOG_D("the voltage is :%d\n", (uint8_t)pobj->volt);
		}
		else
		{
			/*pobj->volt = pobj->adc * REFER_VOLTAGE / CONVERT_BITS; // 100倍
			//pobj->temp = 
			//LOG_D("the voltage is :%d.%02d \n", pobj->volt / 100, pobj->volt % 100);
			
			if (pobj->volt >= 3.3) {
			   pobj->volt = 3.299;  
			}
			pobj->r = 10033 * pobj->volt / (3.3 - pobj->volt);
			
			pobj->temp = Get_Temp(pobj->r);*/
			
		value_temp = 1185146.25 / ((298.15 * log(((float)pobj->adc) / (4096 - pobj->adc))) + 3975);
		value_temp = value_temp - 273.15 + 0.5;
			pobj->temp = value_temp;
			ntc_temp[i] = (int8_t) value_temp;
			
//			LOG_D("[ch]%d, [temp]%d \n", i+1, (uint32_t)pobj->temp);
			
			if (pobj->temp < NTC_TEMP_MIN) pobj->temp = NTC_TEMP_MIN;
			if (pobj->temp > NTC_TEMP_MAX) pobj->temp = NTC_TEMP_MAX;
			}
    }
}


adc_ch_t* ntc_get_obj(uint8_t idx)
{
    return &ntc_table[idx];
}

int32_t ntc_get_temp(uint8_t idx)
{
    return ntc_table[idx].temp;
}

uint8_t anti_get_data(void)
{
	return anti_value;
}

void ntc_get_temp_data(int8_t* temp)
{
	rt_memcpy(temp, ntc_temp, sizeof(ntc_temp));
}

int getTempture(int ch)
{
    if((ch >= ANTI_IDX_CH5) || (ch < 0))
        return 0;
    return ntc_temp[ch];
}


