/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#include "motor_driver.h"
#include "hall_driver.h"


extern volatile uint8_t motor1HallState, motor2HallState;

/**************************************************************************/

void motorOn(void)
{
		TIM_CtrlPWMOutputs(MOTOR1_TIM, ENABLE);
		TIM_CtrlPWMOutputs(MOTOR2_TIM, ENABLE);
}

void motorOff(void)
{
		TIM_CtrlPWMOutputs(MOTOR1_TIM, DISABLE);
		TIM_CtrlPWMOutputs(MOTOR2_TIM, DISABLE);
}

void motor1_gpio_init(void)
{
	GPIO_STD_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOA |
							RCC_AHB1Periph_GPIOB |
							RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,
							ENABLE);	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;	

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_ENABLE_PIN;			// CH1N
	GPIO_Init(MOTOR1_ENABLE_PORT, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;	

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_UN_PIN;		// CH1N
	GPIO_Init(MOTOR1_PWM_UN_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_VN_PIN;        // CH2N
	GPIO_Init(MOTOR1_PWM_VN_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_WN_PIN;         // CH3N
	GPIO_Init(MOTOR1_PWM_WN_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_UP_PIN;			// CH1P
	GPIO_Init(MOTOR1_PWM_UP_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_VP_PIN;           // CH2P
	GPIO_Init(MOTOR1_PWM_VP_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_PWM_WP_PIN;           // CH3P
	GPIO_Init(MOTOR1_PWM_WP_PORT, &GPIO_InitStructure); 

	GPIO_PinAFConfig(MOTOR1_PWM_UN_PORT, MOTOR1_PWM_UN_PinSource, MOTOR1_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR1_PWM_VN_PORT, MOTOR1_PWM_VN_PinSource, MOTOR1_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR1_PWM_WN_PORT, MOTOR1_PWM_WN_PinSource, MOTOR1_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR1_PWM_UP_PORT, MOTOR1_PWM_UP_PinSource, MOTOR1_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR1_PWM_VP_PORT, MOTOR1_PWM_VP_PinSource, MOTOR1_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR1_PWM_WP_PORT, MOTOR1_PWM_WP_PinSource, MOTOR1_GPIO_AF_TIM);
}
/**************************************************************************/
void motor1_tim_pwm_init(void)
{			 
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
//	NVIC_InitTypeDef 		NVIC_InitStructure;	
	GPIO_STD_InitTypeDef 		GPIO_InitStructure;
	TIM_BDTRInitTypeDef 	TIM_BDTRInitStructure;
	TIM_OCInitTypeDef  		TIM_OCInitStructure;  

	RCC_APB2PeriphClockCmd(MOTOR1_RCC_APB_PERIPH, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Period = TS;	  
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTOR1_TIM, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(MOTOR1_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(MOTOR1_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(MOTOR1_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(MOTOR1_TIM,TIM_OCPreload_Enable); //随时写入PWM并立即更新
	TIM_OC2PreloadConfig(MOTOR1_TIM,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(MOTOR1_TIM,TIM_OCPreload_Enable); 

	RCC_AHB1PeriphClockCmd(	MOTOR1_ERROR_RCC_AHB_PERIPH, ENABLE);	
												
	/* GPIOE Configuration: BKIN pin */   
	GPIO_InitStructure.GPIO_Pin = MOTOR1_ERROR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(MOTOR1_ERROR_PORT, &GPIO_InitStructure);   
	GPIO_PinAFConfig(MOTOR1_ERROR_PORT, MOTOR1_ERROR_PinSource, MOTOR1_GPIO_AF_TIM);

	TIM_BDTRInitStructure.TIM_OSSRState 		= TIM_OSSRState_Enable;	   //定时器不工作时输出无效电平
	TIM_BDTRInitStructure.TIM_OSSIState 		= TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel 		= TIM_LOCKLevel_OFF;  
	TIM_BDTRInitStructure.TIM_DeadTime 			= 50;
	TIM_BDTRInitStructure.TIM_Break 			= TIM_Break_Disable;//TIM_Break_Enable;//Enable;  //制动使能
	TIM_BDTRInitStructure.TIM_BreakPolarity 	= TIM_BreakPolarity_Low; //低电平有效
	TIM_BDTRInitStructure.TIM_AutomaticOutput 	= TIM_AutomaticOutput_Disable; //自动输出不使能
	TIM_BDTRConfig(MOTOR1_TIM, &TIM_BDTRInitStructure);	  

	TIM_ClearITPendingBit(MOTOR1_TIM, TIM_IT_Break);
	TIM_Cmd(MOTOR1_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR1_TIM, DISABLE);

//	NVIC_InitStructure.NVIC_IRQChannel = MOTOR1_TIM_UP_IRQN;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//TIM1_UP_PRIORITY;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure); 
//	TIM_ITConfig(MOTOR1_TIM, TIM_IT_Update, ENABLE);
}

/**************************************************************************/
void motor1_set_pwm(int pwm_u,int pwm_v, int pwm_w)
{
	MOTOR1_TIM->CCR1 = pwm_u;	
	MOTOR1_TIM->CCR2 = pwm_v;
	MOTOR1_TIM->CCR3 = pwm_w;
}
/**************************************************************************/
void motor2_gpio_init(void)
{
	GPIO_STD_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOA |
							RCC_AHB1Periph_GPIOB |
							RCC_AHB1Periph_GPIOC |RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,
							ENABLE);	


	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  	= GPIO_PuPd_UP ;	

	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_ENABLE_PIN;		// CH1N
	GPIO_Init(MOTOR2_ENABLE_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  	= GPIO_PuPd_UP ;	
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_UN_PIN;			// CH1N
	GPIO_Init(MOTOR2_PWM_UN_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_VN_PIN;           // CH2N
	GPIO_Init(MOTOR2_PWM_VN_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_WN_PIN;           // CH3N
	GPIO_Init(MOTOR2_PWM_WN_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_UP_PIN;			// CH1P
	GPIO_Init(MOTOR2_PWM_UP_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_VP_PIN;           // CH2P
	GPIO_Init(MOTOR2_PWM_VP_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_PWM_WP_PIN;           // CH3P
	GPIO_Init(MOTOR2_PWM_WP_PORT, &GPIO_InitStructure); 

	GPIO_PinAFConfig(MOTOR2_PWM_UN_PORT, MOTOR2_PWM_UN_PinSource, MOTOR2_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR2_PWM_VN_PORT, MOTOR2_PWM_VN_PinSource, MOTOR2_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR2_PWM_WN_PORT, MOTOR2_PWM_WN_PinSource, MOTOR2_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR2_PWM_UP_PORT, MOTOR2_PWM_UP_PinSource, MOTOR2_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR2_PWM_VP_PORT, MOTOR2_PWM_VP_PinSource, MOTOR2_GPIO_AF_TIM);
	GPIO_PinAFConfig(MOTOR2_PWM_WP_PORT, MOTOR2_PWM_WP_PinSource, MOTOR2_GPIO_AF_TIM);
}
/**************************************************************************/
void motor2_tim_pwm_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	
	GPIO_STD_InitTypeDef GPIO_InitStructure;
	TIM_BDTRInitTypeDef 			TIM_BDTRInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;  
	RCC_APB2PeriphClockCmd(MOTOR2_RCC_APB_PERIPH, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Period = TS;	  
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
	TIM_TimeBaseInit(MOTOR2_TIM, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(MOTOR2_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(MOTOR2_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(MOTOR2_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(MOTOR2_TIM,TIM_OCPreload_Enable); //随时写入PWM并立即更新
	TIM_OC2PreloadConfig(MOTOR2_TIM,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(MOTOR2_TIM,TIM_OCPreload_Enable); 

	RCC_AHB1PeriphClockCmd(	MOTOR2_ERROR_RCC_AHB_PERIPH,ENABLE);	
								
	/* GPIOE Configuration: BKIN pin */   
	GPIO_InitStructure.GPIO_Pin = MOTOR2_ERROR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(MOTOR2_ERROR_PORT, &GPIO_InitStructure);   
	GPIO_PinAFConfig(MOTOR2_ERROR_PORT, MOTOR2_ERROR_PinSource, MOTOR2_GPIO_AF_TIM);

	TIM_BDTRInitStructure.TIM_OSSRState 			= TIM_OSSRState_Enable;	   			//定时器不工作时输出无效电平
	TIM_BDTRInitStructure.TIM_OSSIState 			= TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel 			= TIM_LOCKLevel_OFF;   					//死区时间可随时设置 
	TIM_BDTRInitStructure.TIM_DeadTime 				= 50;
	TIM_BDTRInitStructure.TIM_Break 					= TIM_Break_Disable;						//Enable;  //制动使能
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low; 					//低电平有效
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable; 	//自动输出不使能
	TIM_BDTRConfig(MOTOR2_TIM, &TIM_BDTRInitStructure);	  

	TIM_ClearITPendingBit(MOTOR2_TIM, TIM_IT_Break);
	TIM_ITConfig(MOTOR2_TIM, TIM_IT_Break,ENABLE);
	TIM_ITConfig(MOTOR2_TIM, TIM_IT_Update,DISABLE);
	TIM_Cmd(MOTOR2_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR2_TIM, DISABLE);
	
//	/* Enable the TIM1 BRK Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = MOTOR2_BRK_IRQN;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure); 	
//	
//	/* Enable the TIM1 BRK Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = MOTOR2_TIM_UP_IRQN;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//TIM1_UP_PRIORITY;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure); 
	
}
/**********************************************************/
//void TIM1_UP_TIM10_IRQHandler(void)	//40kHZ
//{	
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
//		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
//		
//	}
//}
/**************************************************************************/
void motor2_set_pwm(int pwm_u,int pwm_v, int pwm_w)
{
	MOTOR2_TIM->CCR1 = pwm_u;	
	MOTOR2_TIM->CCR2 = pwm_v;
	MOTOR2_TIM->CCR3 = pwm_w;
}

/******************* (C) COPYRIGHT 2016*****END OF FILE****/

