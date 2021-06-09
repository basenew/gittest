/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.04.11    The first version template
***************************************************************/
#include "can2_fifo_rx.h"
#include <string.h>
#include <stdio.h>
#include "common_fifo.h"

#define         CAN_FIFO_PAKAGE_SIZE    10
struct          Common_Fifo_Stru        can2_fifo_rx;
CanRxMsg        CanPackage[CAN_FIFO_PAKAGE_SIZE];

volatile uint8_t motor_can_init_state_ = 0;

void can2_fifo_rx_init(void)
{
   common_fifo_init(&can2_fifo_rx, CAN_FIFO_PAKAGE_SIZE, sizeof(CanRxMsg), CanPackage);
}

void can2_fifo_rx_insert(CanRxMsg *msg)
{
   common_fifo_insert_package(&can2_fifo_rx, msg);
}

uint8_t can2_fifo_rx_get(CanRxMsg *msg)
{
  return common_fifo_get_package(&can2_fifo_rx, msg);
}

void can2_fifo_rx_clear(void)
{
	common_fifo_clear_data(&can2_fifo_rx);
}

/******************* (C) COPYRIGHT 2016*****END OF FILE****/

