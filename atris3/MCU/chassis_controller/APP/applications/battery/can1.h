/********************************************************************************
  
	* @file    can.c 
  * @author  luys 
  * @version V1.0.0
  * @date    06-20-2017
  * @brief   

*********************************************************************************/ 

#ifndef __CAN_H
#define __CAN_H
#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#ifdef __cplusplus
 extern "C" {
#endif

     
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

     
     
     
     
//     
// //canopenÇøÓò
// 
// #define PP_Mode  1
// #define PV_Mode  3 
// #define PT_Mode  4 
// 
// 
// //SDO CMD
// #define  SDO_W1   0x2F
// #define  SDO_W2   0x2B
// #define  SDO_W4   0x23
// #define  SDO_RD   0x40
// 
// 

// #define  Control_word                  0x6040
// #define  Status_word                   0x6041
// #define  Modes_of_operation            0x6060
// #define  Modes_0f_operation_display    0x6061
// #define  Position_actual_value         0x6063
// #define  Velocity_sensor_actual_value  0x6069
// #define  Velocity_actual_value         0x606C
// #define  Target_torque                 0x6071
// #define  Target_position               0x607A
// #define  Profile_velocity              0x6081
// #define  Profile_accleration           0x6083
// #define  Profile_deceleration          0x6084
// #define  Torque_slope                  0x6087
// #define  Position_factor               0x6093
// #define  Target_velocity               0x60FF
      
     
     
     


void CAN1_init(void);
int32_t CAN1_Send_bms(u32 Id,u8 *buf,u8 CAN1_DLC);	



#ifdef __cplusplus
}
#endif



#endif


