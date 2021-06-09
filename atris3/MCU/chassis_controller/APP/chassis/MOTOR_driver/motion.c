/********************************************************************************
  
	* @file     
  * @author  luys 
  * @version V3.0.0
  * @date    06-20-2017
  * @brief   
	
*********************************************************************************/ 
/********************************************************************************
  
	* @    
  * @ 
  * @ 
  * @ 
  * @  
 
*********************************************************************************/ 

#include "can2.h"
#include "string.h"
#include "can2_fifo_rx.h"
#define LOG_TAG              "mo"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
//速度模式PV：
u32 PV_spd;
u32 PP_spd;
u32 PT_spd;
u8 setId(u8 default_ID,u8 CANopen_ID);
u8 Motor_enable(int motorId);
u8 SDO_Write_OD_NODELAY(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA);
/*模式设置*/
u8 Contol_Mode_SET(u8 CANopen_ID,u8 CANopen_mode){
     u8 Sedbuf[8];	 
	 Sedbuf[0]=SDO_W1;
	 Sedbuf[1]=0x60;
     Sedbuf[2]=0x60;
	 Sedbuf[3]=0x00;
	 Sedbuf[4]=CANopen_mode;
	 Sedbuf[5]=0x00;
	 Sedbuf[6]=0x00;
	 Sedbuf[7]=0x00;	
	 CAN_Send_motor(0x600 + CANopen_ID,Sedbuf,8);
	 rt_thread_mdelay(5);
	 return(1);
} 
/*激活节点*/
u8 CANopen_Activate(u8 CANopen_ID){
     u8 Sedbuf[8];	 
	 Sedbuf[0]=0x01;
	 Sedbuf[1]=CANopen_ID;	
	 CAN_Send_motor(0x000,Sedbuf,2);
	 rt_thread_mdelay(5);
	 return(1);
} 

/*激活节点*/
u8 CANopen_learn(void){
     u8 Sedbuf[8];	 
	 Sedbuf[0]=0xc8;
	 Sedbuf[1]= 0;	
	 CAN_Send_motor(0x000,Sedbuf,2);
	 rt_thread_mdelay(5);

	 return(1);
} 
u8 CANopen_Motor_Self_learn(u8 CANopen_ID){
     u8 Sedbuf[8];	 
     Sedbuf[0]=0xc8;
	 Sedbuf[1]=CANopen_ID;	
	 CAN_Send_motor(0x000,Sedbuf,2);
	 rt_thread_mdelay(5);
	 return(1);
} 



u8 SDO_Write_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA){
    int delay = 20;
    u8 Sedbuf[8];
	  CanRxMsg rx_msg;
    Sedbuf[0] = CMD;
    Sedbuf[1] = (u8)(Index    & 0xFF);
    Sedbuf[2] = (u8)(Index >> 8 & 0xFF);
    Sedbuf[3] = SubIndex;
    Sedbuf[4] = (u8)(DATA     & 0xFF);
    Sedbuf[5] = (u8)(DATA >> 8  & 0xFF);
    Sedbuf[6] = (u8)(DATA >> 16 & 0xFF);
    Sedbuf[7] = (u8)(DATA >> 24 & 0xFF);
		can2_fifo_rx_clear();
		while(delay--)
		{
			CAN_Send_motor(0x600 + CANopen_ID, Sedbuf, 8);
			rt_thread_mdelay(5);
			while(can2_fifo_rx_get(&rx_msg))
			{
				if(rx_msg.StdId == (0x580 + CANopen_ID)){
					if(rx_msg.Data[0] == 0x60){
						return 1;
					}
				}
			}
		}
     return 0;
}

int CHECK_UPGRADE_STATUS(u8 CANopen_ID){
    int delay = 20;
    u8 Sedbuf[8];
	  CanRxMsg rx_msg;

		can2_fifo_rx_clear();
		while(delay--)
		{//rt_kprintf("send id %s:0x700 +%x\r\n",__FUNCTION__,CANopen_ID);
			CAN_Send_motor(0x700 + CANopen_ID, Sedbuf, 0);
			rt_thread_mdelay(5);
			while(can2_fifo_rx_get(&rx_msg))
			{
                    rt_kprintf("send id:%4x(%d):%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \r\n",rx_msg.StdId,rx_msg.DLC,
                                    rx_msg.Data[0],
                                    rx_msg.Data[1],
                                    rx_msg.Data[2],
                                    rx_msg.Data[3],
                                    rx_msg.Data[4],
                                    rx_msg.Data[5],
                                    rx_msg.Data[6],
                                    rx_msg.Data[7]
                                    );  
                
				if(rx_msg.StdId == (0x700 + CANopen_ID)){
					
						return rx_msg.Data[0];
					
				}
			}
		}
     return -1;
}


u8 setId(u8 default_ID,u8 CANopen_ID){
    int error = 1;
    
    if(default_ID == CANopen_ID){

            LOG_I("%s(%.2x): default_ID CANopen_ID equal\n",__FUNCTION__, CANopen_ID);
        return 0;        
    }
    
    error&=CANopen_Activate(0);
    error&=SDO_Write_OD( default_ID, SDO_W4, 0x450a,0x00,1);
    rt_thread_mdelay(5);
    error&=SDO_Write_OD( default_ID, SDO_W4, 0x4506,0x00,CANopen_ID);
    rt_thread_mdelay(5);
    error&=SDO_Write_OD( default_ID, SDO_W4, 0x450a,0x00,1);   
	rt_thread_mdelay(5);
    if(error == 0)
        LOG_I("%s(%.2x): error\n",__FUNCTION__, CANopen_ID);
    return error ^ 1;
}

u8 setRestoreFactory(u8 id){
    int error = 1;
    
    error&=CANopen_Activate(0);
    error&=SDO_Write_OD( id, SDO_W4, 0x450a,0x00,1);
    rt_thread_mdelay(5);
    error&=SDO_Write_OD( id, SDO_W4, 0x4601,0x00,1);
    rt_thread_mdelay(5);
    error&=SDO_Write_OD( id, SDO_W4, 0x450a,0x00,0);   
	rt_thread_mdelay(500);
    error&=SDO_Write_OD( id, SDO_W4, 0x4603,0x00,1);   
	rt_thread_mdelay(5);
    
    if(error == 0)
        LOG_I("%s(%.2x): error\n",__FUNCTION__, id);
    return error ^ 1;
}


u32 SDO_Read_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex){
    int delay = 20;
    u8 Sedbuf[8];
	CanRxMsg rx_msg;
    u32 *rep;
    Sedbuf[0] = SDO_RD;
    Sedbuf[1] = (u8)(Index    & 0xFF);
    Sedbuf[2] = (u8)(Index >> 8 & 0xFF);
    Sedbuf[3] = SubIndex;
    Sedbuf[4] = 0;
    Sedbuf[5] = 0;
    Sedbuf[6] = 0;
    Sedbuf[7] = 0;
		can2_fifo_rx_clear();
		while(delay--)
		{
			CAN_Send_motor(0x600 + CANopen_ID, Sedbuf, 8);
			rt_thread_mdelay(5);
			while(can2_fifo_rx_get(&rx_msg))
			{
				if(rx_msg.StdId == (0x580 + CANopen_ID)){
					if(rx_msg.Data[0] == 0x4b){
                        rep = (u32 *)&rx_msg.Data[4];
						return *rep;
					}
				}
			}
		}
     return 0;
}
//PV canopen设置

	
int CANopen_PV_Init(int motorId){
	int error = 1;
	LOG_I("CANopen_PV_Init: %.2x\n", motorId);
//STEP1:激活节点1、节点2
  error&=CANopen_Activate( motorId  );
	//CANopen_Activate( Right_Wheel_ID );
	 
//STEP2:设置速度模式	6060H写为3 
	error&=Contol_Mode_SET( motorId,  PV_Mode );
//	Contol_Mode_SET( Right_Wheel_ID, PV_Mode );
	 
//STEP3:设置加减速	写6083H和6084H	 
	 error&=SDO_Write_OD( motorId, SDO_W4, 0x6083,0x00,0x000053e8);
   error&=SDO_Write_OD( motorId, SDO_W4, 0x6084,0x00,0x000053e8);    
	
	 error&=SDO_Write_OD( motorId,SDO_W4, 0x6083,0x00,0x000003e8);
	error&=SDO_Write_OD( motorId,SDO_W4, 0x6084,0x00,0x000003e8);
    error&=SDO_Write_OD( motorId, SDO_W4, 0x60FF,0x00,0);
    error&=Motor_enable(motorId);	
//STEP4:设置目标转速为0	写60FFH	 
    if(error == 0)
        LOG_I("CANopen_PV_Init(%.2x): error\n", motorId);
    return error ^ 1;
}

void CANopen_PV_SET(int motorId,u32 Acc,u32 Dec,s32 TargetVelocity){
	
//STEP1:设置加减速	写6083H和6084H	 
	 SDO_Write_OD_NODELAY( motorId, SDO_W4,0x6083,0x00,Acc);
   SDO_Write_OD_NODELAY( motorId, SDO_W4,0x6084,0x00,Dec);
	 
 
//STEP2:设置目标转速 	写60FFH	 
	 SDO_Write_OD_NODELAY( motorId, SDO_W4,0x60FF,0x00,TargetVelocity);
	 
}


void Motor_PV_Zero(int motorId){
//写0x60FF速度值		
   SDO_Write_OD( motorId, SDO_W4,0x60FF,0x00,0);
	
}

void Motor_PV_Go(int motorId){
//写0x60FF速度值		
   SDO_Write_OD(motorId,SDO_W4,0x60FF,0x00,PV_spd);
}


void Motor_PV_Left(int motorId){ 
//写0x60FF速度值	
	 SDO_Write_OD(motorId,SDO_W4,0x60FF,0x00,~PV_spd+1);
}


void Motor_PV_Right(int motorId){
//写0x60FF速度值		
	SDO_Write_OD(motorId,SDO_W4,0x60FF,0x00,PV_spd);
	
}
int Motor_Read_Speed(int motorId){
    int error = 1;
    error&=SDO_Write_OD(motorId,SDO_W1,0x1a00,0x00,0);
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a00,0x01,0x60630020);//编码器位置 32位
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a00,0x02,0x60690020);//当前速度值

	error&=SDO_Write_OD(motorId,SDO_W1,0x1a00,0x00,2);

	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x05,10);	

	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x03,10);	
	
	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x02,0xfe);	
    
    
	error&=SDO_Write_OD(motorId,SDO_W1,0x1a01,0x00,0);
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x01,0x603f0010);//异常码
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x02,0x501b0010);//温度值
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x03,0x50020020);//力矩数据

	error&=SDO_Write_OD(motorId,SDO_W1,0x1a01,0x00,4);

	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x05,10);	

	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x03,10);	
	
	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x02,0xfe);  
    if(error == 0)
        LOG_I("%S(%.2x): error\n",__FUNCTION__, motorId);    
    return error ^ 1;
}
int Motor_Read_Position_Speed(int motorId){
    int error = 1;	
    
	error&=SDO_Write_OD(motorId,SDO_W1,0x1a00,0x00,0);
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a00,0x01,0x60630020);//位置值
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a00,0x02,0x50020020);//力矩值 千分比

	error&=SDO_Write_OD(motorId,SDO_W1,0x1a00,0x00,2);

	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x05,10);	

	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x03,10);	
	
	error&=SDO_Write_OD(motorId,SDO_W1,0x1800,0x02,0xfe);	

    //return;
	error&=SDO_Write_OD(motorId,SDO_W1,0x1a01,0x00,0);
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x01,0x603f0010);//异常码
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x02,0x50120010);//异常码
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x03,0x501b0010);//温度值
	error&=SDO_Write_OD(motorId,SDO_W4,0x1a01,0x04,0x60780010);//485绝对编码器数据

	error&=SDO_Write_OD(motorId,SDO_W1,0x1a01,0x00,4);

	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x05,10);	

	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x03,10);	
	
	error&=SDO_Write_OD(motorId,SDO_W1,0x1801,0x02,0xfe); 
    if(error == 0)
        LOG_I("%S(%.2x): error\n",__FUNCTION__, motorId);    
    return error ^ 1;
    
}
void set_deit(int motorId){
    //Motor_Read_Speed();
   Motor_Read_Position_Speed(motorId); 
}

void Motor_PV_Back(int motorId){
//写0x60FF速度值	
	SDO_Write_OD(motorId,SDO_W4,0x60FF,0x00,~PV_spd+1);

}	

int CANopen_get_version_year(int motorId)
{
   u32 year = SDO_Read_OD(motorId,SDO_RD,0x5018,0);
   LOG_I("motor version (%d):%d", motorId,year); 
   return year;
}

int CANopen_get_version_date(int motorId)
{
  u32 date = SDO_Read_OD(motorId,SDO_RD,0x5019,0);
   LOG_I("motor version (%d):%d", motorId,date); 
   return date; 
}

//PP canopen设置
int CANopen_PP_Init(int motorId){
    int error = 1;
	LOG_I("CANopen_PP_Init: %.2x\n", motorId);
//STEP1:激活节点1、节点2
	error&= CANopen_Activate(motorId);

//STEP2:设置位置模式	6060H写为1 
	 error&= Contol_Mode_SET(motorId,PP_Mode);


	 
//STEP6:设置电子齿轮比 分子分母 	写6093H的sub1和sub2	 
     error&= SDO_Write_OD(motorId,SDO_W4,0x6093,0x01,1);
 
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6093,0x02,1);
     
     error&= Motor_enable(motorId);	
     error&= SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x1F);
 
	  //STEP3:6040 bits4清0 
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x0F);
     
     //STEP1:设置目标脉冲	写607AH	TargetPosition 	 
	 error&= SDO_Write_OD(motorId,SDO_W4,0x607a,0x00,0);
//STEP2:设置目标转速为	写6081H	 ProfileVelocity
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6081,0x00,0);
  
    //STEP5:设置加减速	写6083H和6084H	 
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6083,0x00,20);

	 
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6084,0x00,20);
    
    
	 error&= SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x1F);
 
	  //STEP3:6040 bits4清0 
	 //SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x0F);
    error&= SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x07);
    if(error == 0)
        LOG_E("CANopen_PP_Init(%.2x):ERROR\n", motorId);
    return error ^ 1;
}


u8 SDO_Write_OD_NODELAY(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA){
     u8 CAN1Sedbuf[8];
     CAN1Sedbuf[0]=CMD;
	 CAN1Sedbuf[1]=(u8)(Index    & 0xFF);
     CAN1Sedbuf[2]=(u8)(Index>>8 & 0xFF);
	 CAN1Sedbuf[3]=SubIndex;
	 CAN1Sedbuf[4]=(u8)(DATA     & 0xFF);
	 CAN1Sedbuf[5]=(u8)(DATA>>8  & 0xFF);
	 CAN1Sedbuf[6]=(u8)(DATA>>16 & 0xFF);
	 CAN1Sedbuf[7]=(u8)(DATA>>24 & 0xFF);	
	 CAN_Send_motor(0x600 + CANopen_ID,CAN1Sedbuf,8);
	 
   return(1);
}

void CANopen_PP_Set_RUN(int motorId,s32 TargetPosition,u32 ProfileVelocity){
	
//STEP1:设置目标脉冲	写607AH	TargetPosition 	 
	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x607a,0x00,TargetPosition);
//STEP2:设置目标转速为	写6081H	 ProfileVelocity
	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6081,0x00,ProfileVelocity);
  
    //STEP5:设置加减速	写6083H和6084H	 
	// SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6083,0x00,20);

	 
	// SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6084,0x00,20);
    
    
	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6040,0x00,0x1F);
 
	  //STEP3:6040 bits4清0 
	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6040,0x00,0x0F);
}

//void CANopen_PP_Set(int motorId,s32 TargetPosition,u32 ProfileVelocity){
//	
////STEP1:设置目标脉冲	写607AH	TargetPosition 	 
//	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x607a,0x00,TargetPosition);
////STEP2:设置目标转速为	写6081H	 ProfileVelocity
//	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6081,0x00,ProfileVelocity);
//  
//    //STEP5:设置加减速	写6083H和6084H	 
////	 SDO_Write_OD(motorId,SDO_W4,0x6083,0x00,20);

////	 
////	 SDO_Write_OD(motorId,SDO_W4,0x6084,0x00,20);
//    
//    
//	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6040,0x00,0x1F);
// 
//	  //STEP3:6040 bits4清0 
//	 SDO_Write_OD_NODELAY(motorId,SDO_W4,0x6040,0x00,0x0F);
//}

void Motor_PP_Trigger(int motorId){
	
   //STEP1:6040 bits4清0 
	 SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x0F);
	 
	  //STEP2:置bits4为1，电机运动 
	 SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x5F);
 
	  //STEP3:6040 bits4清0 
	 SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x4F);

}

void Motor_PP_SetEncodeValue(int motorId,int value){
    SDO_Write_OD(motorId,SDO_W4,Position_actual_value,0x00,value);
    
}

//PT canopen设置
void CANopen_PT_Init(int motorId){
	
//STEP1:激活节点1、节点2
	 CANopen_Activate(motorId);

	 
//STEP2:设置转矩模式	6060H写为4 
   Contol_Mode_SET(motorId,PT_Mode);

//STEP3:设置加减速	写6087H	 
	 SDO_Write_OD(motorId,SDO_W4,0x6087,0x00,0x03e8);
 
//STEP4:设置目标转矩为0	写6071H	 
	 SDO_Write_OD(motorId,SDO_W4,0x6071,0x00,0);

}

void CANopen_PT_Set(int motorId,s16 TargetTorque){
	//STEP1:设置目标转矩为0	写6071H	 
	 SDO_Write_OD(motorId,SDO_W4,0x6071,0x00,TargetTorque);

}



u8 Motor_enable(int motorId){
    int re = 1;
//使能写0x6040分别为6、7、F
	
	 re&=SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x06);

	 re&=SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x07);
	
	 re&=SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x0F);
    return re;

}

void Motor_Disable(int motorId){
	
 //失能写0x6040分别为7   	
	 SDO_Write_OD(motorId,SDO_W4,0x6040,0x00,0x07);

}

void Motor_ERRtest(void){


}	

void Motor_read(void)//主循环函数
{	   

     Motor_ERRtest();	
}

/* set the motor timeout*/
u8 MotorOfflineSet(int motorId,int t)
{
	int re = 1;
   re&=SDO_Write_OD(motorId,SDO_W2,0x450B,0x00,t);
   rt_thread_mdelay(10);
	/*0:report warning   1:disable motor  2: 0 speed*/
   re&=SDO_Write_OD(motorId,SDO_W2,0x450C,0x00,2);
   rt_thread_mdelay(10);
	return re;
}



/* set the motor timeout  0: clear the error*/
u8 MotorErrorClear(int motorId)
{
	int re = 1;
   re&=SDO_Write_OD(motorId,SDO_W2,0x4602,0x00,0x00);
	return re;
}

#include "finsh.h"
/****************************************************************************
 setting the the motor watchdog to avoid the motor offline.if the motor offline, set the motor speed 0
**************************************/
static void SetMotorWatchdog(uint8_t argc, char **argv)
{
  	if (argc < 2 || argc > 2) 
		{
		rt_kprintf("Please input: parameter <1/0> 1/x :set the timeout time \n 0:no time out time \n ");
    }
    else
    {
		   MotorOfflineSet(0x11,atoi(argv[1]));
	     MotorOfflineSet(0x12,atoi(argv[1]));
	     MotorOfflineSet(0x13,atoi(argv[1]));
	     MotorOfflineSet(0x14,atoi(argv[1]));
		}			

}
		
MSH_CMD_EXPORT(SetMotorWatchdog, ******************Set CAN silence mode);
