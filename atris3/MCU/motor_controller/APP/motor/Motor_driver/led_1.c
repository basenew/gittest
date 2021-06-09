

#include "led_1.h"


/*************************************************************
  Function   :
  Description:
*************************************************************/
void led_init(void)
{
	
	GPIO_STD_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init((GPIO_STD_TypeDef*)GPIOE, &GPIO_InitStructure);
	GPIO_SetBits((GPIO_STD_TypeDef*)GPIOE,GPIO_Pin_1);
}

/*************************************************************
  Function   :
  Description:
*************************************************************/
void red_led_er_init(void)
{
	
	GPIO_STD_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init((GPIO_STD_TypeDef*)GPIOA, &GPIO_InitStructure);
	GPIO_SetBits((GPIO_STD_TypeDef*)GPIOA,GPIO_Pin_6);
}
