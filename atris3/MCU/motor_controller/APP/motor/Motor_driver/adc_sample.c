/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/

#include "adc_sample.h"
#define ADC3_DR_ADDRESS    ((uint32_t)&DF_ADC_DR)			//((uint32_t)0x4001224C)

volatile  unsigned short ADC_ConvertedValue[DF_ADC_SIZE];
volatile  float Vatage_Offset[6]={0};
/**********************************************************/
void set_votage_offset(void)
{
	uint16_t i, j;
	float votage_sum[6] ={0};
	for( i=0; i<2000; i++){
		for(j=0; j<15000; j++){
			//delay
		}
		votage_sum[0] += (ADC_ConvertedValue[0])*3.3f/4096;
		votage_sum[1] += (ADC_ConvertedValue[1])*3.3f/4096;
		votage_sum[2] += (ADC_ConvertedValue[2])*3.3f/4096;
		votage_sum[3] += (ADC_ConvertedValue[3])*3.3f/4096;
		votage_sum[4] += (ADC_ConvertedValue[4])*3.3f/4096;
		votage_sum[5] += (ADC_ConvertedValue[5])*3.3f/4096;
	}
	votage_sum[0] = votage_sum[0]/2000;
	votage_sum[1] = votage_sum[1]/2000;
	votage_sum[2] = votage_sum[2]/2000;
	votage_sum[3] = votage_sum[3]/2000;
	votage_sum[4] = votage_sum[4]/2000;
	votage_sum[5] = votage_sum[5]/2000;
	Vatage_Offset[0] = votage_sum[0];
	Vatage_Offset[1] = votage_sum[1];
	Vatage_Offset[2] = votage_sum[2];
	Vatage_Offset[3] = votage_sum[3];
	Vatage_Offset[4] = votage_sum[4];
	Vatage_Offset[5] = votage_sum[5];
}

/**********************************************************/
void get_motor1_current(float *curent_a, float *curent_b, float *curent_c)
{
//	*curent_a = -(Vatage_Offset[0] - (ADC_ConvertedValue[0])*3.3f/4096)/(0.02f*5);
//	*curent_b = -(Vatage_Offset[1] - (ADC_ConvertedValue[1])*3.3f/4096)/(0.02f*5);
//	*curent_c = -(Vatage_Offset[2] - (ADC_ConvertedValue[2])*3.3f/4096)/(0.02f*5);
	
	*curent_a = -(16.5f - (ADC_ConvertedValue[0])*0.00806f);
	*curent_b = -(16.5f - (ADC_ConvertedValue[1])*0.00806f);
	*curent_c = -(16.5f - (ADC_ConvertedValue[2])*0.00806f);
}

/**********************************************************/
void get_motor2_current(float *curent_a, float *curent_b, float *curent_c)
{
//	*curent_a = -(Vatage_Offset[3] - (ADC_ConvertedValue[3])*3.3f/4096)/(0.02f*5);
//	*curent_b = -(Vatage_Offset[4] - (ADC_ConvertedValue[4])*3.3f/4096)/(0.02f*5);
//	*curent_c = -(Vatage_Offset[5] - (ADC_ConvertedValue[5])*3.3f/4096)/(0.02f*5);
	
	*curent_a = -(16.5f  - (ADC_ConvertedValue[3])*0.00806f);
	*curent_b = -(16.5f  - (ADC_ConvertedValue[4])*0.00806f);
	*curent_c = -(16.5f  - (ADC_ConvertedValue[5])*0.00806f);
}

/**********************************************************/
void adc_current_init(void)
{
	ADC_STD_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_STD_InitTypeDef       DMA_InitStructure;
	GPIO_STD_InitTypeDef      GPIO_InitStructure;

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd( 	DF_MOTOR1_IU_PERIPH_GPIO |
							DF_MOTOR1_IV_PERIPH_GPIO |
							DF_MOTOR1_IW_PERIPH_GPIO |
							DF_MOTOR2_IU_PERIPH_GPIO |
							DF_MOTOR2_IV_PERIPH_GPIO |
							DF_MOTOR2_IW_PERIPH_GPIO |
							DF_BUS_U_PERIPH_GPIO |
							DF_ADC_DMA_PERIPH
							, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	/* DMA2 Stream0 channel2 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = DF_ADC_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Configure ADC3 Channel7 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	/* Configure ADC3 Channel7 pin as analog input ******************************/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	GPIO_InitStructure.GPIO_Pin = DF_MOTOR1_IU_PIN;	   
	GPIO_Init(DF_MOTOR1_IU_GPIO, &GPIO_InitStructure); 
													 
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR1_IV_PIN;	   
	GPIO_Init(DF_MOTOR1_IV_GPIO, &GPIO_InitStructure); 
													 
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR1_IW_PIN;	   
	GPIO_Init(DF_MOTOR1_IW_GPIO, &GPIO_InitStructure); 
													 
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR2_IU_PIN;	   
	GPIO_Init(DF_MOTOR2_IU_GPIO, &GPIO_InitStructure); 
													 
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR2_IV_PIN;	   
	GPIO_Init(DF_MOTOR2_IV_GPIO, &GPIO_InitStructure); 
													 
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR2_IW_PIN;	   
	GPIO_Init(DF_MOTOR2_IW_GPIO, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = DF_BUS_U_PIN;	   
	GPIO_Init(DF_BUS_U_GPIO, &GPIO_InitStructure); 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init((GPIO_STD_TypeDef*)GPIOD, &GPIO_InitStructure); 


	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC3 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = DF_ADC_SIZE;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 regular channel7 configuration *************************************/
	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR1_IU_CHANNEL, DF_MOTOR1_IU_LOCAL + 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR1_IV_CHANNEL, DF_MOTOR1_IV_LOCAL + 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR1_IW_CHANNEL, DF_MOTOR1_IW_LOCAL + 1, ADC_SampleTime_480Cycles);

	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR2_IU_CHANNEL, DF_MOTOR2_IU_LOCAL + 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR2_IV_CHANNEL, DF_MOTOR2_IV_LOCAL + 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(DF_ADC, DF_MOTOR2_IW_CHANNEL, DF_MOTOR2_IW_LOCAL + 1, ADC_SampleTime_480Cycles);

	ADC_RegularChannelConfig(DF_ADC, DF_BUS_U_CHANNEL, DF_BUS_U_LOCAL + 1, ADC_SampleTime_480Cycles);
	
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC3);
}

/*===========================end of file====================*/
