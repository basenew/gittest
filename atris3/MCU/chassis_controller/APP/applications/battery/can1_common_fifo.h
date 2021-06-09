/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.02.21    The first version template
***************************************************************/
#ifndef _COMMON_FIFO_
#define	_COMMON_FIFO_

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#define GET_PACKAGE 1
#define NO_PACKAGE  0


struct can1_Common_Fifo_Stru
{
	int16_t Head;
	int16_t Tail;
	int16_t Total;
	int16_t Fifo_Size;
	int16_t Data_Size;
	uint8_t *Data_Ptr;
};


void can1_common_fifo_init(struct can1_Common_Fifo_Stru *fifo_stru, int16_t fifo_size, int16_t data_size, void *ptr);
void can1_common_fifo_insert_package(struct can1_Common_Fifo_Stru *fifo_stru, void *data);
uint8_t can1_common_fifo_get_package(struct can1_Common_Fifo_Stru *fifo_stru, void *data);



#endif

