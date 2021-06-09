
#include "i2c.h"
#include "delay.h"

void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    IIC_SCL = 1;
    delay_us(IIC_DELAY_TIME_);
    IIC_SDA = 1;
    delay_us(IIC_DELAY_TIME_);
}


void IIC_Start(void)
{
    SDA_OUT();     
    IIC_SDA = 1;
    delay_us(IIC_DELAY_TIME_);
    IIC_SCL = 1;
    delay_us(IIC_DELAY_TIME_ * 2);
    IIC_SDA = 0; //START:when CLK is high, DATA change form high to low
    delay_us(IIC_DELAY_TIME_ * 2);
    IIC_SCL = 0; 
    delay_us(IIC_DELAY_TIME_);
}

void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL = 0;
    delay_us(IIC_DELAY_TIME_);
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    delay_us(IIC_DELAY_TIME_ * 2);
    IIC_SCL = 1;
    delay_us(IIC_DELAY_TIME_);
    IIC_SDA = 1;
    delay_us(IIC_DELAY_TIME_ * 2);
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();    
    IIC_SDA = 1; delay_us(IIC_DELAY_TIME_);
    IIC_SCL = 1; delay_us(IIC_DELAY_TIME_);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL = 0;
    delay_us(IIC_DELAY_TIME_);
    return 0;
}

void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(IIC_DELAY_TIME_);
    IIC_SCL = 1;
    delay_us(IIC_DELAY_TIME_ );
    IIC_SCL = 0;
    delay_us(IIC_DELAY_TIME_);
}

void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(IIC_DELAY_TIME_ );
    IIC_SCL = 1;
    delay_us(IIC_DELAY_TIME_);
    IIC_SCL = 0;
    delay_us(IIC_DELAY_TIME_);
}

void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL = 0; 
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(IIC_DELAY_TIME_); 
        IIC_SCL = 1;
        delay_us(IIC_DELAY_TIME_);
        IIC_SCL = 0;
        delay_us(IIC_DELAY_TIME_);
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++ )
    {
        IIC_SCL = 0;
        delay_us(IIC_DELAY_TIME_);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)receive++;
        delay_us(IIC_DELAY_TIME_);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack(); 
    return receive;
}

