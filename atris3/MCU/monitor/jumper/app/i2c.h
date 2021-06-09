#ifndef __I2C_H__
#define __I2C_H__

#include "bitmap.h"
#define IIC_DELAY_TIME_  2

//PH5输入 
#define SDA_IN()                         \
    {                                    \
        GPIOH->MODER &= ~(3 << (5 * 2)); \
        GPIOH->MODER |= 0 << 5 * 2;      \
    } 

//PH5输出
#define SDA_OUT()                        \
    {                                    \
        GPIOH->MODER &= ~(3 << (5 * 2)); \
        GPIOH->MODER |= 1 << 5 * 2;      \
    }


#define IIC_SCL PHout(4) 
#define IIC_SDA PHout(5) 
#define READ_SDA PHin(5)



void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

#endif
