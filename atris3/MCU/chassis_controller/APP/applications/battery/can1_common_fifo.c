/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.04.11    The first version template
***************************************************************/
#include "can1_common_fifo.h"
#include <string.h>


/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
void can1_common_fifo_init(struct can1_Common_Fifo_Stru *fifo_stru, int16_t fifo_size, int16_t data_size, void *ptr)
{
	fifo_stru->Head = 0;
	fifo_stru->Tail = 0;
	fifo_stru->Total = 0;
	fifo_stru->Fifo_Size = fifo_size;
	fifo_stru->Data_Size = data_size;
	fifo_stru->Data_Ptr = ptr;
}
/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
void can1_common_fifo_insert_package(struct can1_Common_Fifo_Stru *fifo_stru, void *data)
{
    __disable_irq();
	if(fifo_stru->Total >= fifo_stru->Fifo_Size)
	{
        __enable_irq();
		return;
	}
	memcpy(&fifo_stru->Data_Ptr[fifo_stru->Tail * fifo_stru->Data_Size], data, fifo_stru->Data_Size);
	fifo_stru->Tail++;
        fifo_stru->Total++;
	if(fifo_stru->Tail >= fifo_stru->Fifo_Size)
	{
		fifo_stru->Tail = 0;
	}
   __enable_irq();
}
/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
uint8_t can1_common_fifo_get_package(struct can1_Common_Fifo_Stru *fifo_stru, void *data)
{
    __disable_irq();
	if(0 >= fifo_stru->Total)
	{
        __enable_irq();
		return NO_PACKAGE;
	}
	memcpy(data, &fifo_stru->Data_Ptr[fifo_stru->Head * fifo_stru->Data_Size], fifo_stru->Data_Size);
	fifo_stru->Total--;
	fifo_stru->Head++;
	if(fifo_stru->Head >= fifo_stru->Fifo_Size)
	{
		fifo_stru->Head = 0;
	}
    __enable_irq();
	return GET_PACKAGE;
}




/******************* (C) COPYRIGHT 2016*****END OF FILE****/

