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

//位操作
#define SETBIT(Data, Bit)                   ((Data)|=(1<<Bit))     //设置
#define CLEARBIT(Data, Bit)                 ((Data)&=~(1<<Bit))    //清零
#define READBIT(Data, Bit)                  ((Data>>Bit)%2)        //读取 
#define REVERBIT(Data,Bit)                  ((Data)^=(1<<Bit))     //翻转 

//字操作
#define GETHHBYTE(Data)                     ((uint8_t)(Data>>24))       //获取最高字节
#define GETHLBYTE(Data)                     ((uint8_t)(Data>>16)&0xff)  //获取次高字节
#define GETLHBYTE(Data)                     ((uint8_t)(Data>>8)&0xff)   //获取次低字节
#define GETLLBYTE(Data)                     ((uint8_t)(Data&0xff))      //获取最低字节

//半字操作
#define GETHBYTE(Data)                      ((uint8_t)(Data>>8)&0xff)   //获取高字节
#define GETLBYTE(Data)                      ((uint8_t)(Data&0xff))      //获取低字节

#define BYTETOINT(Byte0,Byte1,Byte2,Byte3)  (((uint32_t)(Byte0)<<24)|((uint32_t)(Byte1)<<16)|((uint32_t)(Byte2)<<8)|Byte3)                          
#define BYTETOSHORT(Byte0,Byte1)            ((((uint16_t)(Byte0))<<8)|Byte1) 

#define BIG_LITTLE_SWITCH_U16(U16)          ((uint16_t)BYTETOSHORT(GETLBYTE(U16),GETHBYTE(U16)))                                       /* 16bit 大小端转换*/
#define BIG_LITTLE_SWITCH_U32(U32)          ((uint32_t)BYTETOINT((GETLLBYTE(U32)),(GETLHBYTE(U32)),(GETHLBYTE(U32)),GETHHBYTE(U32)))   /* 32bit 大小端转换*/


//ms 转换为 ticks
#define MS_TO_TICKS(_MS) (rt_tick_from_millisecond(_MS))


int32_t os_gettime_ms(void);		//获取当前系统时钟 ms
uint8_t find_first_1bit_uint32(uint32_t _param, uint8_t _n);





#ifdef __cplusplus
}
#endif

#endif
