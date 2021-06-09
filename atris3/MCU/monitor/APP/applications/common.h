#ifndef __COMMON_H__
#define __COMMON_H__

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif


#define  RTN_OK                            (0)  
#define  RTN_ERR                           (1)
#define  ING (2) 

#define  YES                               (1)  
#define  NO                                (0) 

#define  HIGH_LEVEL                        (1)   
#define  LOW_LEVEL                         (0)
	
#define OUT (0)
#define IN (1)
	
#define OFF (0)
#define ON (1)

#define  DF_YES                               (1)  
#define  DF_NO                                (0) 

#define  DF_OK                            (0)  
#define  DF_ERR                           (1)
#define  DF_ING (2) 
	
#define DF_RELEASE (0)
#define DF_TRIGE (1)

#define DF_DISABLE (0)
#define DF_ENABLE (1)

//λ����
#define SETBIT(Data, Bit)                   ((Data)|=(1<<Bit))     //����
#define CLEARBIT(Data, Bit)                 ((Data)&=~(1<<Bit))    //����
#define READBIT(Data, Bit)                  ((Data>>Bit)%2)        //��ȡ 
#define REVERBIT(Data,Bit)                  ((Data)^=(1<<Bit))     //��ת 

//�ֲ���
#define GETHHBYTE(Data)                     ((uint8_t)(Data>>24))       //��ȡ����ֽ�
#define GETHLBYTE(Data)                     ((uint8_t)(Data>>16)&0xff)  //��ȡ�θ��ֽ�
#define GETLHBYTE(Data)                     ((uint8_t)(Data>>8)&0xff)   //��ȡ�ε��ֽ�
#define GETLLBYTE(Data)                     ((uint8_t)(Data&0xff))      //��ȡ����ֽ�

//���ֲ���
#define GETHBYTE(Data)                      ((uint8_t)(Data>>8)&0xff)   //��ȡ���ֽ�
#define GETLBYTE(Data)                      ((uint8_t)(Data&0xff))      //��ȡ���ֽ�

#define BYTETOINT(Byte0,Byte1,Byte2,Byte3)  (((uint32_t)(Byte0)<<24)|((uint32_t)(Byte1)<<16)|((uint32_t)(Byte2)<<8)|Byte3)                          
#define BYTETOSHORT(Byte0,Byte1)            ((((uint16_t)(Byte0))<<8)|Byte1) 

#define BIG_LITTLE_SWITCH_U16(U16)          ((uint16_t)BYTETOSHORT(GETLBYTE(U16),GETHBYTE(U16)))                                       /* 16bit ��С��ת��*/
#define BIG_LITTLE_SWITCH_U32(U32)          ((uint32_t)BYTETOINT((GETLLBYTE(U32)),(GETLHBYTE(U32)),(GETHLBYTE(U32)),GETHHBYTE(U32)))   /* 32bit ��С��ת��*/


//ms ת��Ϊ ticks
#define MS_TO_TICKS(_MS) (rt_tick_from_millisecond(_MS))


int32_t os_gettime_ms(void);		//��ȡ��ǰϵͳʱ�� ms
uint8_t find_first_1bit_uint32(uint32_t _param, uint8_t _n);





#ifdef __cplusplus
}
#endif

#endif
