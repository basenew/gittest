/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.04.11    The first version template
xiangbin.huang   2017.06.20    add can1 send state, if no mailbox
xiangbin.huang   2017.07.06    fix send_tx failure bug
***************************************************************/
#include "can1_fifo_tx.h"
#include <string.h>
#include <stdio.h>
#include "can1_common_fifo.h"

#define                CAN1_FIFO_TX_PAKAGE_SIZE         50
static struct          can1_Common_Fifo_Stru           Can1_Fifo_Tx;
static CanTxMsg        Can1_Tx_Package[CAN1_FIFO_TX_PAKAGE_SIZE];
static volatile        uint8_t         Can1_Tx_Flag = 1;
/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
void can1_fifo_tx_init(void)
{
   can1_common_fifo_init(&Can1_Fifo_Tx, CAN1_FIFO_TX_PAKAGE_SIZE, sizeof(CanTxMsg), Can1_Tx_Package);
}
/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
uint8_t can1_fifo_tx_get(CanTxMsg *msg)
{
    uint8_t re = can1_common_fifo_get_package(&Can1_Fifo_Tx, msg);
     if(!re)
         Can1_Tx_Flag =1; //发送失败标志位1
  return re;
}

/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
void can1_fifo_tx_insert(CanTxMsg *msg)
{
    CanTxMsg can_tx_msg;
    can1_common_fifo_insert_package(&Can1_Fifo_Tx, msg);
    if(Can1_Tx_Flag == 1) //表示需要启动发送
    {
      if(can1_fifo_tx_get(&can_tx_msg))
      {
        Can1_Tx_Flag =0; //发送成功标志位0
        can1_send_package(&can_tx_msg);
      }
      return;
    }
}

void set_can1_tx_flag(uint8_t flag){
    Can1_Tx_Flag =flag;
}


/******************* (C) COPYRIGHT 2016*****END OF FILE****/

