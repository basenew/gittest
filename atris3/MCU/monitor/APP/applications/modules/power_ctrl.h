#ifndef __APP_MODULES_POWER_CTRL_H__
#define __APP_MODULES_POWER_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"
	
#define   PIN_POWER_3V3_IMU_PA6                      GET_PIN(A, 6)			//IMU电源
#define   PIN_POWER_5V9_REMOTE_PC3                   GET_PIN(C, 3)			//遥控接收机电源
#define   PIN_POWER_24V_DT35_G10_PD0                 GET_PIN(D, 0)			//防跌落传感器电源
#define   PIN_POWER_24V_GATEWAY_PE6                  GET_PIN(E, 6)			//路由器电源
#define   PIN_POWER_24V_MAST_PE7                     GET_PIN(E, 7)			//升降杆
#define   PIN_POWER_3V3_PHY_PE15                     GET_PIN(E, 15)			//PHY芯片电源
#define   PIN_POWER_5V_RFID_PF0                      GET_PIN(F, 0)			//RFID模块电源
#define   PIN_POWER_12V_X86_PF3                      GET_PIN(F, 3)			//X86
#define   PIN_POWER_12V_SWITCH_PF5                   GET_PIN(F, 5)			//交换机
#define   PIN_POWER_24V_RADAR_PF10                   GET_PIN(F, 10)			//雷达
#define   PIN_POWER_12V_VOIP_PF12                    GET_PIN(F, 12)			//VOIP
#define   PIN_POWER_3V3_SYS_PF13                     GET_PIN(F, 13)			//232/485、温湿度传感器、电压检测、电流检测等芯片电源
#define   PIN_POWER_3V3_GNSS_PG12                    GET_PIN(G, 12)			//千寻GPS模块电源
#define   PIN_POWER_24V_CAM_PPH9                     GET_PIN(H, 9)			//云台电源
#define   PIN_POWER_12V_RES1_PF2                     GET_PIN(F, 2)			//预留12V电源
#define   PIN_POWER_12V_RES2_PE2                     GET_PIN(E, 2)			//预留12V电源
#define   PIN_POWER_24V_RES3_PE10                    GET_PIN(E, 10)			//预留24V电源




//-----------------------------------------------------------------------------------------------------------------------------
#define  POWER_3V3_IMU_ENABLE                     rt_pin_write(PIN_POWER_3V3_IMU_PA6, PIN_HIGH)
#define  POWER_3V3_IMU_DISABLE                    rt_pin_write(PIN_POWER_3V3_IMU_PA6, PIN_LOW)
#define  POWER_3V3_IMU_STATUS                     rt_pin_read(PIN_POWER_3V3_IMU_PA6)

#define  POWER_5V9_REMOTE_ENABLE                  rt_pin_write(PIN_POWER_5V9_REMOTE_PC3, PIN_HIGH)
#define  POWER_5V9_REMOTE_DISABLE                 rt_pin_write(PIN_POWER_5V9_REMOTE_PC3, PIN_LOW)
#define  POWER_5V9_REMOTE_STATUS                  rt_pin_read(PIN_POWER_5V9_REMOTE_PC3)

#define  POWER_24V_DT35_G10_ENABLE                rt_pin_write(PIN_POWER_24V_DT35_G10_PD0, PIN_HIGH)
#define  POWER_24V_DT35_G10_DISABLE               rt_pin_write(PIN_POWER_24V_DT35_G10_PD0, PIN_LOW)
#define  POWER_24V_DT35_G10_STATUS                rt_pin_read(PIN_POWER_24V_DT35_G10_PD0)

#define  POWER_24V_GATEWAY_ENABLE                 rt_pin_write(PIN_POWER_24V_GATEWAY_PE6, PIN_HIGH)
#define  POWER_24V_GATEWAY_DISABLE                rt_pin_write(PIN_POWER_24V_GATEWAY_PE6, PIN_LOW)
#define  POWER_24V_GATEWAY_STATUS                 rt_pin_read(PIN_POWER_24V_GATEWAY_PE6)

#define  POWER_24V_MAST_ENABLE                    rt_pin_write(PIN_POWER_24V_MAST_PE7, PIN_HIGH)
#define  POWER_24V_MAST_DISABLE                   rt_pin_write(PIN_POWER_24V_MAST_PE7, PIN_LOW)
#define  POWER_24V_MAST_STATUS                    rt_pin_read(PIN_POWER_24V_MAST_PE7)

#define  POWER_3V3_PHY_ENABLE                     rt_pin_write(PIN_POWER_3V3_PHY_PE15, PIN_HIGH)
#define  POWER_3V3_PHY_DISABLE                    rt_pin_write(PIN_POWER_3V3_PHY_PE15, PIN_LOW)
#define  POWER_3V3_PHY_STATUS                     rt_pin_read(PIN_POWER_3V3_PHY_PE15)

#define  POWER_5V_RFID_ENABLE                     rt_pin_write(PIN_POWER_5V_RFID_PF0, PIN_HIGH)
#define  POWER_5V_RFID_DISABLE                    rt_pin_write(PIN_POWER_5V_RFID_PF0, PIN_LOW)
#define  POWER_5V_RFID_STATUS                     rt_pin_read(PIN_POWER_5V_RFID_PF0)

#define  POWER_12V_X86_ENABLE                     rt_pin_write(PIN_POWER_12V_X86_PF3, PIN_HIGH)
#define  POWER_12V_X86_DISABLE                    rt_pin_write(PIN_POWER_12V_X86_PF3, PIN_LOW)
#define  POWER_12V_X86_STATUS                     rt_pin_read(PIN_POWER_12V_X86_PF3)

#define  POWER_12V_SWITCH_ENABLE                  rt_pin_write(PIN_POWER_12V_SWITCH_PF5, PIN_HIGH)
#define  POWER_12V_SWITCH_DISABLE                 rt_pin_write(PIN_POWER_12V_SWITCH_PF5, PIN_LOW)
#define  POWER_12V_SWITCH_STATUS                  rt_pin_read(PIN_POWER_12V_SWITCH_PF5)

#define  POWER_24V_RADAR_ENABLE                   rt_pin_write(PIN_POWER_24V_RADAR_PF10, PIN_HIGH)
#define  POWER_24V_RADAR_DISABLE                  rt_pin_write(PIN_POWER_24V_RADAR_PF10, PIN_LOW)
#define  POWER_24V_RADAR_STATUS                   rt_pin_read(PIN_POWER_24V_RADAR_PF10)

#define  POWER_12V_VOIP_ENABLE                    rt_pin_write(PIN_POWER_12V_VOIP_PF12, PIN_HIGH)
#define  POWER_12V_VOIP_DISABLE                   rt_pin_write(PIN_POWER_12V_VOIP_PF12, PIN_LOW)
#define  POWER_12V_VOIP_STATUS                    rt_pin_read(PIN_POWER_12V_VOIP_PF12)

#define  POWER_3V3_SYS_ENABLE                     rt_pin_write(PIN_POWER_3V3_SYS_PF13, PIN_HIGH)
#define  POWER_3V3_SYS_DISABLE                    rt_pin_write(PIN_POWER_3V3_SYS_PF13, PIN_LOW)
#define  POWER_3V3_SYS_STATUS                     rt_pin_read(PIN_POWER_3V3_SYS_PF13)

#define  POWER_3V3_GNSS_ENABLE                    rt_pin_write(PIN_POWER_3V3_GNSS_PG12, PIN_HIGH)
#define  POWER_3V3_GNSS_DISABLE                   rt_pin_write(PIN_POWER_3V3_GNSS_PG12, PIN_LOW)
#define  POWER_3V3_GNSS_STATUS                    rt_pin_read(PIN_POWER_3V3_GNSS_PG12)

#define  POWER_24V_CAM_ENABLE                     rt_pin_write(PIN_POWER_24V_CAM_PPH9, PIN_HIGH)
#define  POWER_24V_CAM_DISABLE                    rt_pin_write(PIN_POWER_24V_CAM_PPH9, PIN_LOW)
#define  POWER_24V_CAM_STATUS                     rt_pin_read(PIN_POWER_24V_CAM_PPH9)

#define  POWER_12V_RES1_ENABLE                    rt_pin_write(PIN_POWER_12V_RES1_PF2, PIN_HIGH)
#define  POWER_12V_RES1_DISABLE                   rt_pin_write(PIN_POWER_12V_RES1_PF2, PIN_LOW)
#define  POWER_12V_RES1_STATUS                    rt_pin_read(PIN_POWER_12V_RES1_PF2)

#define  POWER_12V_RES2_ENABLE                    rt_pin_write(PIN_POWER_12V_RES2_PE2, PIN_HIGH)
#define  POWER_12V_RES2_DISABLE                   rt_pin_write(PIN_POWER_12V_RES2_PE2, PIN_LOW)
#define  POWER_12V_RES2_STATUS                    rt_pin_read(PIN_POWER_12V_RES2_PE2)

#define  POWER_24V_RES3_ENABLE                    rt_pin_write(PIN_POWER_24V_RES3_PE10, PIN_HIGH)
#define  POWER_24V_RES3_DISABLE                   rt_pin_write(PIN_POWER_24V_RES3_PE10, PIN_LOW)
#define  POWER_24V_RES3_STATUS                    rt_pin_read(PIN_POWER_24V_RES3_PE10)


#define   PIN_SPK_CTRL_PF7                           GET_PIN(F, 7)			//VOIP功放

#define  SPK_CTRL_ENABLE                     rt_pin_write(PIN_SPK_CTRL_PF7, PIN_HIGH)
#define  SPK_CTRL_DISABLE                    rt_pin_write(PIN_SPK_CTRL_PF7, PIN_LOW)
#define  SPK_CTRL_STATUS                     rt_pin_read(PIN_SPK_CTRL_PF7)



typedef enum
{
    POWRE_WORK = 0,
	POWER_STANDBY,
}power_mode_t;


int32_t power_ctrl_init(void);				//电源控制GPIO初始化
void power_standby_status(void);		    //电源进入待机状态
void power_set_status(uint32_t status);		//所有受控电源控制接口
uint32_t power_get_status(void);			//获取当前所有电源状态
void spk_ctrl_set(uint8_t status);			//功放电源状态设置
void power_work_status(void);		   		//电源进入工作状态


#ifdef __cplusplus
}
#endif


#endif



