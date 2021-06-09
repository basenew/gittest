#ifndef COMMON_H
#define COMMON_H

#include <unistd.h>
#include <stdio.h>
//#define ROS_WARN printf
//#define ROS_INFO printf
//#define ROS_ERROR printf
//#define ROS_WARN(...) ROS_LOG(::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define CAN_INVALID_MODE  1
typedef enum
{
    IDLE_TASK			= 0,
    RECEIVE_ATTRIBUTE_TASK ,
    UPDATE_TASK,
    SET_PARAMETER_TASK,

}NODE_TASK;


typedef unsigned char                   u8;     /**<  8bit unsigned integer type */
typedef unsigned short                  u16;    /**< 16bit unsigned integer type */
typedef unsigned int                   u32;    /**< 32bit unsigned integer type */


//HTN = HOST TO NODE
//NTH = NOTE TO HOST
//FE	请求属性指令	OUT	主控制器向节点请求上传节点的节点属性	节点属性格式
#define HTN_ATTRIBUTE_REQUEST   0XFE
//FD	请求属性应答	IN	向主控制器报告上传节点属性结束
#define NTH_ATTRIBUTE_SEND_END_RESPONSE   0XFD
//FC	请求erase升级数据区	OUT	主控制器向节点请求erase升级数据区
#define HTN_ERASE_UPDATE_FEILD_REQUEST   0XFC
//FB	请求erase升级数据区应答	IN	向主控制器报告节点erase升级数据结束
#define NTH_ERASE_UPDATE_FEILD_RESPONSE   0XFB
//FA	请求发送升级数据指令	OUT	主控制器向节点请求发送升级数据,并下传升级数据包大小
#define HTN_SEND_UPDATE_DATA_REQUEST   0XFA
//F9	请求发送升级数据应答	IN	向主控制器报告节点接收升级数据包就绪与否，已就绪就要附带节点升级数据区大小
#define NTH_SEND_UPDATE_DATA_RESPONSE   0XF9
//F8	发送升级数据结束指令	OUT	主控制器向节点说明升级数据包传输结束
#define HTN_SEND_UPDATE_END_REQUEST  0XF8
//F7	发送升级数据结束应答	IN	向主控制器报告节点报告节点接收升级数据上否OK
#define NTH_SEND_UPDATE_END_RESPONSE   0XF7
//F6	块接收异常	OUT	主控制器向节点说明接收数据时有块丢失 Abnormal data receiving
#define HTN_RECEIVE_ABNORMAL				   0XF6
//F5	块接收异常	IN	向主控制器报告节点说明接收数据时有块丢失
#define NTH_RECEIVE_ABNORMAL   0XF5
//F4	请求节点接收设置参数指令	OUT	主控制器向节点说明下面的块数据中参数设置数据，要节点接收	参数建议为字符串
#define HTN_RECEIVE_PARAMETER_REQUEST  0XF4
//F3	节点接收设置参数应答	IN	向主控制器报告节点已是否准备好接收参数设置数据
#define NTH_RECEIVE_PARAMETER_RESPONSE  0XF3
//F2	请求设置节点参数指令	OUT	主控制器向节点要求把刚才缓冲区接收的参数，保存进参数表
#define HTN_SET_PARAMETER_REQUEST  0XF2
//F1	设置节点参数应答	IN	向主控制器报告节点保存参数是否OK
#define NTH_SET_PARAMETER_RESPONSE  0XF1
//F0	请求节点把参数复位成出厂值指令	OUT	主控制器向节点要求把参数恢复成出厂值	Mobile phone restore factory default
#define HTN_RESTORE_FACTORY_DEFAULT_REQUEST  0XF0
//EF	节点把参数复位成出厂值应答	IN	向主控制器报告节点恢复成出厂值是否OK
#define NTH_RESTORE_FACTORY_DEFAULT_RESPONSE  0XEF

//EE	节点复位请求	OUT	主控制器要求节点复位	该请求须要二次确认
#define HTN_RESET_REQUEST  0XEE
//ED	节点复位应答	IN	向主控制器报告节点准备复位
#define NTH_RESET_RESPONSE  0XED
//EC	节点准备就绪	IN	向主控制器报告节点节点已准备就绪
#define NTH_NODE_READY  0XEC
//EB	主控制器确认节点就绪	OUT	主控制器确认节点就绪状态
#define HTN_CONFIRM_READY  0XEB

//EC	要求节点上报状态	OUT	查询节点状态，在线离线，有无异常 	使用广播帧和单播帧  report state
#define HTN_REPORT_STATE_REQUEST  0XEC
//EB	节点上报状态	IN	上报节点状态，在线离线，有无异常
#define NTH_REPORT_STATE_RESPONSE 0XEB

//EA	主控制器心跳
#define HTN_HEART  0XEA
//E9  节点心跳
#define NTH_HEART  0XE9

//E8   请求上传节点串码
#define HTN_SERIAL_REQUEST  0XE8

#define HIN_SET_TIMER_REPORT 0X19//设置上报频率

#define CAN_UPGRADE_RETRY   true

#define CAN_EXPECT_STATUS_PRE 0 
#define CAN_EXPECT_STATUS_TMP 1

#define UPGRADE_MAX_PRIT_SIZE 256


#pragma pack(1)
typedef struct
{
    union HEAD{
        struct ID{
            u32  channel:16;
            u32  index:8;
            u32  end:1;
            u32  can:1;
            u32  mode:3;
            u32  size:3;
        }id;
        u32 h32;
    }head;
    u8   data[8];
}CAN_PACKAGE;

#pragma pack()

#pragma pack(1)
typedef struct
{
    u16  idvender;
    u16  idProduct;
    u16  year;
    u8  month;
    u8  day;
    u16  serial_number;
}SERIAL_CODE;


#pragma pack()





#endif // COMMON_H
