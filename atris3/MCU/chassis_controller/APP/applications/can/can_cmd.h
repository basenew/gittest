#ifndef __APP_CAN_CMD_H__
#define __APP_CAN_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif


#define  BROADCAST_ID                       0x00
#define  NODE_ID                            0x69

#define  CAN_CONTROL_MODE                   0x00  //notity 
#define  CAN_UP_SERIAL_NUM                  0x01
#define  CAN_RETURN_MODE                    0x02  //call-response
#define  CAN_SET_NODE_ID                    0x03
#define  CAN_BULK_CONTROL_MODE              0x04
#define  CAN_BULK_DATA_OUT_MODE             0x05
#define  CAN_BULK_RETURN_MODE               0x06
#define  CAN_BULK_DATA_IN_MODE              0x07

//---------------------------------------------------------------------------------------------------------------
//节点私用的can命令号
#define  CANCMD_GET_POWER_STATUS           0x01  // 查询电源状态                      
  
#define  CANCMD_SET_POWER_STATUS           0x03  // 设置电源开关                      

#define  CANCMD_SET_RTC                    0x05  // 同步RTC       

#define  CANCMD_GET_RTC                    0x07  // 查询RTC     

#define CANCMD_SET_TIMINGBOOT              0x09 // 设置定时开机时间

#define CANCMD_GET_TIMINGBOOT              0x0B // 查询定时开机时间

#define  CANCMD_EANBLE_TIMINGBOOT          0x0D  // 定时自动开机使能  

#define  CANCMD_ENABLE_CHARGEFULL_BOOT     0x0F  //充满电自动开机使能

#define  CANCMD_GET_BOOT_ENABLE_FLAG       0x11  //查询充满电使能,定时开机使能的值

#define CANCMD_FAN_SPEED                   0x13  //风速


#define  CANCMD_GET_BUMPER_KEY             0x15   /* 查询急停开关状态                    */


#define  CANCMD_GET_URGENT_KEY             0x17   /* 查询急停开关状态                    */


#define CANCMD_REPORT_UV_STATUS            0x20   


//---------------------------------------------------------------------------------------------------------------
#define CANCMD_REPORT_BAT_INFO1            0x22

#define CANCMD_REPORT_BAT_INFO2            0x24

#define CANCMD_BAT_HALT                    0x25  // 电池休眠

#define CANCMD_BAT_SHUTDOWN                0x27  // 电池关机

#define CANCMD_BAT_VERSION                 0x29  // 查询电池版本号

//---------------------------------------------------------------------------------------------------------------
#define  CANCMD_POWER_OFF                       0x31   /* 关机命令                      */

#define  CANCMD_GET_CHARGE_SOURCE               0x33   /* 查询充电来源                      */

#define  CANCMD_GET_BOOT_TYPE                   0x35   /* 查询开机启动类型                    */  

#define  CANCMD_REPORT_SOFT_KEY                 0x38  //上报软开关状态 






#define  CANCMD_LOG_GET_LEN                     0x41  /* 读取log长度,单位字节                */
#define  CANCMD_LOG_UPLOAD_START                0x43  /* 要求上传LOG                     */
#define  CANCMD_LOG_CLEAN                       0x45  /* 清除LOG                       */
#define  CANCMD_LOG_CONTENT                     0x48  /* 日志内容                        */
#define  CANCMD_LOG_UPLOAD_FINISH               0x4A  /* 上传日志结束                      */
#define  CANCMD_LOG_UPLOAD_BREAK                0x4B  /* 中断上传日志                      */




#define  CANCMD_MCU_RESET                       0xE1  /* 软复位机器                        */

#define  CANCMD_LOG_EARESE_WHOLE_FLASH          0xE3  /* 擦除外部Flash                 */ 

#define  CANCMD_SET_LOWPOWER                    0xE5  /* 低功耗功能使能                   */ 

#define  CANCMD_RESTORE_ENV                     0xE6  /* 重载KV数据库                   */

#define  CANCMD_GETENVVERSION                   0xE8  /* 获取env版本                     */


//---------------------------------------------------------------------------------------------------------------
//节点公用的can命令号
#define CANCMD_SET_CANTXSWICH               0x19   /* 设置can发送开关                   */
#define CANCMD_QUERY_DEVICE_ID              0xCE   /* 获取设备码                       */
#define CANCMD_RESPONE_DEVICE_ID            0xCF   /* 获取设备码应答                     */
#define CANCMD_QUERY_VERSION                0xCC   /* 获取设备版本                      */
#define CANCMD_RESPONE_VERSION              0xCD   /* 获取设备版本应答                    */
#define CANCMD_REQUIRE_LOCKNUM              0xCA   /* 设备开锁请求                      */
#define CANCMD_RESPONE_LOCKNUM              0xCB   /* 设备开锁请求应答                    */
#define CANCMD_REQUIRE_TURNOFFLOCK          0xC8   /* 设备关锁请求                      */
#define CANCMD_RESPONE_TURNOFFLOCK          0xC9   /* 设备关锁请求应答                    */
#define CANCMD_ENTER_UPDATE                 0xBE   /* 进入升级模式                      */
#define CANCMD_RESPONE_ENTER_UPDATE         0xBF   /* 进入升级模式应答                    */
#define CANCMD_UPDATE_WRITEDATA             0xBC   /* 写升级数据                       */
#define CANCMD_RESPONE_WRITEDATA            0xBD   /* 写升级数据应答                     */
#define CANCMD_UPDATE_FINISH                0xBA   /* 升级结束                        */
#define CANCMD_RESPONE_UPDATE_FINISH        0xBB   /* 升级结束应答                      */



#ifdef __cplusplus
}
#endif

#endif


