/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.02.21    The first version template
***************************************************************/
#ifndef _CAN2_FIFO_RX_H_
#define	_CAN2_FIFO_RX_H_

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

void can2_fifo_rx_init(void);
void can2_fifo_rx_insert(CanRxMsg *msg);
uint8_t can2_fifo_rx_get(CanRxMsg *msg);
void can2_fifo_rx_clear(void);

#endif

