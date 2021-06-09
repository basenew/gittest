#ifndef __BMS_H__
#define __BMS_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stdint.h"
#include "stm32f4xx_can.h"

#define BMS_LOG_CNTS_MAX 500


typedef enum
{
    BMS_CTRL_IDLE = 0,
    BMS_CTRL_SHUTDOWN,
    BMS_CTRL_SLEEP, 
} bms_ctrl_t;


typedef struct
{
    /*
    INT16U，0.1V/BIT，0xFFFF 无效  
    */
    uint16_t TotalVolt; //总电压
    /*
    INT16U，0.1A/BIT，充电为正，放电
    为负，偏移量 5000。例如 5030 表示
    3A 充电，4700 表示 30A 放电 
    */
    uint16_t TotalCurr; //总电流
    /*
    INT16U，1%/BIT，0xFFFF 无效
    */
    uint16_t SOC; //SOC
    /*
    INT16U，0.1%/BIT，0xFFFF 无效
    */
    uint16_t SOH; //SOH
    /*
    INT16U，0.1V/BIT，0xFFFF 无效
    */
    uint16_t PackVolt; //PACK 标称电压
    /*
    INT16U，0.1Ah/BIT，例如 16AH 写入为 160=0x00A0
    */
    uint16_t PackCap; //PACK 标称容量
    /*
    INT16U，1mV/BIT，例如：3600=0x0E10
    BYTE1=0x10，BYTE2=0x0E
    */
	uint8_t wakeup_source;  //BMS唤醒指示， 0x01:外部IO唤醒， 0x02:其他唤醒，  0xff:无效
    uint16_t CellVolt_High; //单体最高电压
    uint16_t CellVolt_Low; //单体最低电压
    /*
    INT8U，0~15，表示多包组合时的最
    高单体所在的 PACK，0xFF 表示无效
    */
    uint8_t PackNo_CellVoltHigh; //单体最高电压 PACK 编号
    uint8_t PackNo_CellVoltLow; //单体最低电压 PACK 编号
    /*
    INT8U，表示单体在 PACK 中的串号，
    0xFF 表示无效
    */
    uint8_t BunchNo_CellVoltHigh; //单体最高电压串号
    uint8_t BunchNo_CellVoltLow;  //单体最低电压串号
    /*
    INT8U，1 度/BIT，0xFF 无效，偏移
    量 40 度。例 35 度，上报为
    35+40=75=0x48
    解析时为 0x48-40=35 度
    */
    uint8_t CellTempHigh; //单体最高温度
    uint8_t CellTempLow; //单体最低温度
    /*
    INT8U，0~15，表示多包组合时的温度所在的 PACK，0xFF 表示无效
    */
    uint8_t PackNo_CellTempHigh; //单体最高温度 PACK 编号
    uint8_t PackNo_CellTempLow; //单体最低温度 PACK 编号
    /*
    INT8U，表示温度所在的位置编号，0xFF 表示无效
    */
    uint8_t BunchNo_CellTempHigh; //最高温度位置
    uint8_t BunchNo_CellTempLow; //最低温度位置
    /*
    INT8U，0～255，每次上传累加 1
    */
    uint8_t BmsLife; //BMS 生命值
    /*
    INT8U，0xAA 为断开，0xBB 为闭合 0xFF 为无效
    */
    uint8_t ChgMosStatus; //充电 MOS 状态
    uint8_t DisChgMosStatus; //放电 MOS 状态
    /*
    INT8U，0x00 为无动作，0x01 为开机，0x02 为关机，0x03 为复位，0xFF 无效
    */
    uint8_t ExIoStatus; //外部 IO 状态指示
    /*
    INT16U，0xFFFF 无效。
    */
    uint16_t ChgDisChg_Cnts; //充放电次数
    /*
    INT8U， 0xBB 表示 BMS 即将进行休眠动作（延时 60 秒），0xFF 无效。
    */
    uint8_t SleepFlag; //关机指示
    /*
    INT8U，0x00 表示无充电，0x01 表示正在充电中，0x02 表示充电完成，0xFF 无效
    */
    uint8_t ChgStatus; //充电指示

    union 
    {
        uint8_t Warn[8];
        struct
        {
            uint8_t WarnStatus     : 8; //告警状态 有告警时值为 0x2F，无告警时值为0x3F
            uint8_t WarnLevel_H    : 8; //最高告警等级 无告警时为 0，最高告警等级为 1 级 为 0x01，最低告警等级为 2 级为0x02

            /*
            长度为 2 个 BIT;
            0 为无此告警;
            1 为此告警已经产生且等级为 1;
            2 为此告警已经产生且等级为 2;
            */
            uint8_t CellVolt_H     : 2; //单体高压
            uint8_t CellVolt_L     : 2; //单体低压
            uint8_t CellVolt_Diff  : 2; //单体压差
            uint8_t ChgTemp_High   : 2; //充电高温
            uint8_t ChgTemp_Low    : 2; //充电低温
            uint8_t DisChgTemp_High: 2; //放电高温
            uint8_t DisChgTemp_Low : 2; //放电低温
            uint8_t CellTemp_Diff  : 2; //单体温差
            uint8_t SOC_TooHigh    : 2; //SOC过高
            uint8_t SOC_TooLow     : 2; //SOC过低
            uint8_t TotalVolt_TooHigh : 2; //总压过高
            uint8_t TotalVolt_TooLow  : 2; //总压过低
            uint8_t SteadyChg_OverCurr: 2; //稳态充电过流
            uint8_t SteadyDisChg_OverCurr: 2; //稳态放电过流
            uint8_t PowerBoard_TempHigh  : 2; //功率板温度过高
            uint8_t BalancedMOS_TempHigh : 2; //均衡MOS温度过高
            
            /*
            此类告警不区分等级，只分有和无;
            0 为无此告警;
            1 为此告警已经产生且等级为 1;
            */
            uint8_t TransientChg_OverCurr: 1; //瞬态充电过流
            uint8_t TransientDisChg_OverCurr: 1; //瞬态放电过流
            uint8_t FrontChip_Fault: 1; //前端芯片异常
            uint8_t ShortCircuit_Fault: 1; //短路故障
            uint8_t ChgMOS_Fault: 1; //充电MOS故障
            uint8_t DisChgMOS_Fault: 1; //放电MOS故障
            uint8_t RS485_Comm_Timeout: 1; //RS485通信超时
            uint8_t Reserved1: 1; //预留
            uint8_t Reserved2: 8; //预留
        } sWarn;
    } uWarn;

    /*
    INT16U，1mV/BIT，例如：3600=0x0E10 BYTE1=0x10，BYTE2=0x0E
    */
    uint16_t CellVolt[16]; //电芯电压
    /*
    INT8U，1 度/BIT，0xFF 无效，偏移 量 40 度。例 35 度，上报为 35+40=75=0x48 解析时为 0x48-40=35 度
    */
    uint8_t  CellTemp[16]; //温度点温度
} bms_data_t;


typedef union 
{
    uint8_t data[4][8];
    struct
    {
        union 
        {
            uint8_t data0[8];
            struct
            {
                uint8_t PkgNo    : 8; //包序号
                /*
                与告警信息报文中的 BYTE3~BYTE7 相对应
                */
                uint8_t Warn1_7_1: 8;   //告警码 1，7-0 位
                uint8_t Warn1_15_8: 8;  //告警码 1，15-8 位
                uint8_t Warn1_23_16: 8; //告警码 1，23-16 位
                uint8_t Warn1_31_24: 8; //告警码 1，31-24 位
                uint8_t Warn2_7_0: 8;   //告警码 2，7-0 位
                /*
                INT16U，1mV/BIT，0xFFFF 无效
                */
                uint8_t CellVolt_High_L: 8; //单体最高电压，低字节
                uint8_t CellVolt_High_H: 8; //单体最高电压，高字节
            } sData0;
        } uData0;
        
        union 
        {
            uint8_t data1[8];
            struct
            {
                uint8_t PkgNo :8; //包序号
                /*
                INT16U，1mV/BIT，0xFFFF 无效
                */
                uint8_t CellVolt_Low_L: 8; //单体最低电压，低字节
                uint8_t CellVolt_Low_H: 8; //单体最低电压，高字节
                /*
                INT16U，0.1V/BIT，0xFFFF 无效
                */
                uint8_t TotalVolt_L: 8; //总电压 低字节 
                uint8_t TotalVolt_H: 8; //总电压 高字节 
                /*
                INT16U，0.1A/BIT，充电为正，放电为负，偏移量 5000
                */
                uint8_t TotalCurr_L: 8; //总电流 低字节 
                uint8_t TotalCurr_H: 8; //总电流 高字节 
                uint8_t Reserved: 8;    //预留
            } sData1;
        } uData1;
        
        union 
        {
            uint8_t data2[8];
            struct
            {
                uint8_t PkgNo: 8; //包序号
                /*
                INT8U，1%/BIT，0xFF 无效
                */
                uint8_t SOC: 8;   //SOC
                /*
                INT16U，0.1V/BIT，0xFFFF 无效
                */
                uint8_t PackVolt_L: 8; //PACK 电压，低字节
                uint8_t PackVolt_H: 8; //PACK 电压，高字节
                /*
                INT8U，1 度/BIT, 偏移量 40 度
                */
                uint8_t CellTempHigh: 8;   //单体最高温度
                uint8_t CellTempLow: 8;    //单体最低温度
                uint8_t PowerBoardTemp: 8; //功率板温度
                /*
                高 3 位为单体最高温度串号，低 5 位为单体最高电压的串号，从第 1 串开始
                */
                uint8_t BunchNumbers: 8; //单体最高温度与单体最高电压的串号
            } sData2;
        } uData2;
        
        union 
        {
            uint8_t data3[8];
            struct
            {
                uint8_t PkgNo: 8; //包序号
                /*
                高 3 位为单体最低温度串号，低 5 位 为单体最低电压的串号
                */
                uint8_t BunchNumbers: 8; //单体最低温度与单体最低电 压的串号
                /*
                INT8U，使用年的最后两位，如 2020年，则发送 20 即 0x14,0xFF 无效
                */
                uint8_t year: 8;  //年
                uint8_t month: 8; //月
                uint8_t day: 8;   //日
                uint8_t hour: 8;  //时
                uint8_t min: 8;   //分
                uint8_t sec: 8;   //秒
            } sData3;
        } uData3;
        
    } sLogData;
} bms_logdata_t;

typedef struct
{
    bms_logdata_t info[BMS_LOG_CNTS_MAX];
    uint16_t tar_cnts;
    uint16_t cnts;
    uint8_t  idx;
} bms_log_t;


typedef enum
{
    E_BMS_VER_SOFT = 0xDD,
    E_BMS_VER_HARD = 0xBB,
} bms_ver_idx_t;

typedef struct
{
    uint8_t frame_cnts;
    uint8_t size;
    char version[80];
} _ver_t;

typedef struct
{
    _ver_t soft;
    _ver_t hard;
} bms_ver_t;




int32_t bms_init(void);                    //bms初始化
int32_t bms_control(bms_ctrl_t idx);       //mcu发送控制命令到BMS
int32_t bms_sync(void);                    //bms定时同步函数，1s
//int32_t bms_getlog(void);                //获取bms内部日志
int32_t bms_getversion(bms_ver_idx_t idx); //获取BMS版本信息
void copy_datum(CanRxMsg rxmsg);           //bms CAN接收函数
bms_data_t* bat_get_bms_data_ptr(void);    //获取bms信息指针地址
uint8_t bms_get_wakeup_source(void);       //查询bms开机唤醒原因，按键、非按键（充电等）
void get_bms_hw_version(char* _version);   //获取bms硬件版本信息
void get_bms_sw_version(char* _version);   //获取bms软件版本信息
uint16_t get_bms_CellVolt(uint8_t _ch);    //获取bms单体电压信息
uint16_t get_bms_TotalVolt(void);          //获取bms总压信息
uint16_t get_bms_TotalCurr(void);          //获取bms总电流信息


#ifdef __cplusplus
}
#endif

#endif






