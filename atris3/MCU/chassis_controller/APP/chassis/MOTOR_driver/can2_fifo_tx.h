/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.02.21    The first version template
***************************************************************/
#ifndef _CAN1_FIFO_TX_
#define	_CAN1_FIFO_TX_

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
void can_fifo_tx_init(void);
uint8_t can_fifo_tx_get(CanTxMsg *msg);
uint8_t can_send_package(CanTxMsg *msg);
void can_fifo_tx_insert(CanTxMsg *msg);
void send_data_to_can(uint8_t *txdata, uint8_t id, uint8_t mode, uint8_t size);
void set_tx_flag(uint8_t flag);

#endif

