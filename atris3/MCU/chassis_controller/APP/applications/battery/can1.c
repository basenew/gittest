/********************************************************************************
  
	* @file    can.c 
  * @author  luys 
  * @version V1.0.0
  * @date    06-20-2017
  * @brief   can program body

*********************************************************************************/ 
#include "can1.h"
#include "can1_fifo_tx.h"
//#include "can1_fifo_rx.h"
#include "string.h"
//#include "chassis_common.h"
#include "bms.h"

/*************************************************************
  Function   :
  Description:
  Input      :
  return     :
*************************************************************/
uint8_t can1_send_package(CanTxMsg *msg)
{
   uint8_t TransmitMailbox = 0;
        if(0)
    rt_kprintf("send id:%4x(%d):%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \r\n",msg->StdId,msg->DLC,
                                    msg->Data[0],
                                    msg->Data[1],
                                    msg->Data[2],
                                    msg->Data[3],
                                    msg->Data[4],
                                    msg->Data[5],
                                    msg->Data[6],
                                    msg->Data[7]
                                    ); 
   TransmitMailbox = CAN_Transmit(CAN1, msg);;
   if(CAN_NO_MB == TransmitMailbox)
   {
       set_can1_tx_flag(1);
       //Tx_Flag =1; //����ʧ�ܱ�־λ1
      return  0;
   }
   else
   {
      
      return 1;
   }
}

/////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Function   :
  Description:
  Input      : none
  return     : none
*************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg Can_Rx_Message;
	memset((void *)&Can_Rx_Message,0,sizeof(Can_Rx_Message));
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET){
		CAN_Receive(CAN1, CAN_FIFO0, &Can_Rx_Message);
        copy_datum(Can_Rx_Message);
		//can1_fifo_rx_insert(&Can1_Rx_Message);
	}
	if(SET == CAN_GetITStatus(CAN1,CAN_IT_FF0)){
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);
	}
	if(SET == CAN_GetITStatus(CAN1,CAN_IT_FOV0)){
		CAN_ClearITPendingBit(CAN1,CAN_IT_FOV0);
	}
}

/*************************************************************
  Function   :
  Description:
  Input      : none
  return     : none
*************************************************************/
void CAN1_TX_IRQHandler(void)
{
  CanTxMsg can_tx_msg;
  if (CAN_GetITStatus(CAN1,CAN_IT_TME) != RESET)
  {
    CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    if(!can1_fifo_tx_get(&can_tx_msg))
    {    
      return;
    }
    if(can1_send_package(&can_tx_msg) == 0)
    {
      
    }
  }
}



/*************************************************************
  Function   :
  Description:
  Input      : none
  return     : none
*************************************************************/
static void can1_nvic_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
}
/*************************************************************
  Function   :
  Description:
  Input      : none
  return     : none
*************************************************************/
static void can1_gpio_init(void)
{
	GPIO_STD_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	//CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init((GPIO_STD_TypeDef*)GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12

	//���Ÿ���ӳ������
	GPIO_PinAFConfig((GPIO_STD_TypeDef*)GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig((GPIO_STD_TypeDef*)GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=ENABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 

	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 	
	CAN_InitStructure.CAN_SJW   =CAN_SJW_1tq;
//	CAN_InitStructure.CAN_BS1   =CAN_BS1_7tq;
//	CAN_InitStructure.CAN_BS2   =CAN_BS2_6tq;
//	CAN_InitStructure.CAN_Prescaler     = 6;		
	CAN_InitStructure.CAN_BS1   =CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2   =CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler     = 6;		
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
}
/*************************************************************
  Function   :
  Description:
  Input      : none
  return     : none
*************************************************************/
static void can1_filter_init(void)
{
    

  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		//���ù�����
		CAN_FilterInitStructure.CAN_FilterNumber=0;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh=0;
		CAN_FilterInitStructure.CAN_FilterIdLow=0;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);
		
		CAN_FilterInitStructure.CAN_FilterNumber=1;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh=0;
		CAN_FilterInitStructure.CAN_FilterIdLow=0;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);
}


void CAN1_init(void){

 can1_fifo_tx_init();
 can1_gpio_init();
 can1_filter_init();
 can1_nvic_init();
}

int32_t CAN1_Send_bms(u32 Id,u8 *buf,u8 CAN1_DLC)
{	
//	u8 mail;
    CanTxMsg TxMes;
	u16 i=0;
	TxMes.StdId = Id;	         // ��׼��ʶ��ID
	TxMes.ExtId = Id;	         // ��չ��ʾ��ID��29λ��
	TxMes.IDE = CAN_Id_Extended; //
	TxMes.RTR = CAN_RTR_Data;		 // ��Ϣ����Ϊ����֡��/ң��֡
	TxMes.DLC = CAN1_DLC;		         // 	1�����ĳ���	
    for(i=0;i<CAN1_DLC;i++)
	  TxMes.Data[i]=buf[i];				 // ��һ֡��Ϣ    
if(0)

    rt_kprintf("send id:%4x(%d):%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \r\n",TxMes.StdId,TxMes.DLC,
                                    TxMes.Data[0],
                                    TxMes.Data[1],
                                    TxMes.Data[2],
                                    TxMes.Data[3],
                                    TxMes.Data[4],
                                    TxMes.Data[5],
                                    TxMes.Data[6],
                                    TxMes.Data[7]
                                    );    
    can1_fifo_tx_insert(&TxMes);
	return 0;		
}
		



#include "finsh.h"
/****************************************************************************
 setting the CAN in science mode so that it can't send the data,
 only listenning. the purpose is for motor driver to upgrade.
**************************************/
static void SetCANSilentMode(uint8_t argc, char **argv)
{
  	if (argc < 2 || argc > 2) 
		{
		rt_kprintf("Please input: parameter <1/0> 1:enable silence mode \n 0:disable silence mode\n ");
    }
    else
    {
			if(atoi(argv[1])==0)
			{
								 /* Request initialisation */
         CAN1->MCR |= CAN_MCR_INRQ ;
				 CAN2->MCR |= CAN_MCR_INRQ ;
				 CAN1->BTR = CAN1->BTR & (~CAN_BTR_SILM);
         CAN2->BTR = CAN1->BTR & (~CAN_BTR_SILM);
								    /* Request leave initialisation */
        CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;
				CAN2->MCR &= ~(uint32_t)CAN_MCR_INRQ;
				 rt_kprintf(" CAN is in normal mode SILM bit is 0\n");
			}
			else
			{
				 /* Request initialisation */
         CAN1->MCR |= CAN_MCR_INRQ ;
				 CAN2->MCR |= CAN_MCR_INRQ ;
				 CAN1->BTR = CAN1->BTR | CAN_BTR_SILM;
         CAN2->BTR = CAN1->BTR | CAN_BTR_SILM;
				    /* Request leave initialisation */
        CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;
				CAN2->MCR &= ~(uint32_t)CAN_MCR_INRQ;
			   rt_kprintf(" CAN is in silence mode SILM bit is 1\n");
			}
		}			

}
MSH_CMD_EXPORT(SetCANSilentMode, ******************Set CAN silence mode);








