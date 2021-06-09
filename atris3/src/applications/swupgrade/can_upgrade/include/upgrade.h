#ifndef INCLUDEUPGRADE_H
#define INCLUDEUPGRADE_H

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>

#include "upgradeCanPkg.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "boost/function.hpp"
#include "common.h"

//CF	设备码应答	IN	节点向主控制器上报设备的pid，vid，总线版本，应用区或升级区
#define NTH_PRODUCT_CODE_RESPONSE   0XCF

//CE	设备码请求	OUT	主控制器向节点请求设备码
#define HTN_PRODUCT_CODE_REQUEST   0XCE

//CD	版本应答 IN 节点向主控制器上报设备的硬件，软件版本。和指令集版本
#define NTH_VERSION_RESPONSE   0XCD

//CC 版本请求 OUT 主控制器向节点请求节点的版本信息
#define HTN_VERSION_REQUEST   0XCC


//FF 舵机版本请求回复
#define NTH_VERSION_RESPONSE_STEER   0XFF
//FE 舵机版本请求信息节点
#define HIN_VERSION_REQUEST_STEER  0XFE

//CB 设置锁开应答 IN 节点应答主控制器的开锁请求
#define NTH_GET_LOCK_KEY_RESPONSE   0XCB

//CA 设置锁开请求 OUT 主控制器向节点请求，开设置锁
#define HTN_GET_LOCK_KEY_REQUEST   0XCA

//C9	设置锁关应答	IN	节点应答主控制器的上锁请求
#define NTH_DISTORY_LOCK_KEY_RESPONSE   0XC9

//C8	设置锁关请求	OUT	主控制器向节点请求，上设置锁
#define HTN_DISTORY_LOCK_KEY_REQUEST   0XC8

//C7	生产流水应答	IN	节点应答主控制器的生产流水请求
#define NTH_SERIAL_CODE_RESPONSE   0XC7

//C6	生产流水请求或设置	OUT	向节点请求或设置生产流水
#define HTN_SERIAL_CODE_REQUEST   0XC6

//C5	设备名应答	IN	节点应答主控制器的设备名请求
#define NTH_PRODUCT_NAME_RESPONSE   0XC5

//C4	设备名请求或设置	OUT	向节点请求或设置设备名
#define HTN_PRODUCT_NAME_REQUEST   0XC4

//C3	设置参数应答	IN	节点应答主控制器设置节点参数请求
//#define NTH_SET_PARAMETER_RESPONSE   0XC3
//C2	设置参数请求	OUT	向节点请求设置节点中的参数，如舵机中的KP,KI和零点
//#define HTN_SET_PARAMETER_REQUEST   0XC2

//C1	读参数应答	IN	节点应答主控制器的读参数请求
#define NTH_GET_PARAMETER_RESPONSE   0XC1
//C0	读参数请求	OUT	向节点请求读参数
#define HTN_GET_PARAMETER_REQUEST   0XC0

//BF	进入升级模式应答	IN	节点应答主控制器的升级请求
#define NTH_UPGRADE_RESPONSE   0XBF
//BE	进入升级模式请求	OUT	向节点请求进入升级模式
#define HTN_UPGRADE_REQUEST   0XBE


//BD	写升级数据应答	IN	节点应答主控制器的升级数据包
#define NTH_PROGRAMMING_RESPONSE   0XBD
//BC	写升级数据请求	OUT	向节点写升级数据
#define HTN_PROGRAMMING_REQUEST   0XBC

//BB	升级结束应答	IN	节点应答主控制器的升级结束请求，验证升级区的数据
#define NTH_PROGRAM_END_RESPONSE   0XBB
//BA	升级结束请求	OUT	向节点发送升级结束请求
#define HTN_PROGRAM_END_REQUEST   0XBA


//0X01  底部电源板状态值查询    modified by marty.gong@ubtrobot.com:2017-9-19-17:31:24 
#define HTN_GET_POWER_STATE_REQUEST 0X01
#define HTN_GET_POWER_STATE_RESPONSE 0X02 

//0X03  底部电源板状态值设置 modified by marty.gong@ubtrobot.com:2017-9-19-17:32:5 
#define HTN_SET_POWER_STATE_REQUEST 0X03
#define HTN_SET_POWER_STATE_RESPONSE 0X04 

#define HIT_RECOVER_LOW_POWER 0XB1
#define DF_APP_INTER_BOOT          0
#define DF_BOOT_INTER_UPGRADE      1
#define DF_INSUFFICIENT_FLASH      2
#define DF_KEY_ERROR               3
#define DF_LOCK_CLOSE              4
#define DF_START_ERASE_FLASH       5
#define DF_ERASE_FLASH_OK          6

#define UP_GRADE_PERCENT_SETION   4

typedef enum{

	upgrade_app = 0x0,
	upgrade_boot = 0x1,
	upgrade_fail = 0x2,
	upgrade_keynum = 0x3,
	upgrade_nor_setkey = 0x4,
	upgrade_erase_begin = 0x5,
	upgrade_erase_ok = 0x6,
	upgrade_erase_base_percent = 100,
}CAN_UP_STS;


typedef enum{

	print_percent_30 = 30,
	print_percent_60 = 60,
	print_percent_100 = 100,
}PRIT_PERCNT;

#pragma pack(1)
typedef struct{
    u16 vid;
    u16 pid;
    u8  can_bus_protocol_low;
    u8  can_bus_protocol_high;
    char mode;
}PRODUCT_CODE_STRUCT;


typedef union{
  
struct{
	u8  hardware_version_low;
	u8  hardware_version_high;
	u8  software_version_low;
	u8  software_version_high;
	u8  instruction_version_low;
	u8  instruction_version_high;
	}norm;
struct{
  	u8 hardware_version;
	u8 software_version_1;
	u8 software_version_2;
	u8 software_version_3;
	u8 software_version_4;
	u8 software_version_5;
  }ster;

}VERSION_STRUCT;

typedef union{
   struct{

       u8 chassis_:1;
       u8 chassis_control_:1;
       u8 waist_:1;
       u8 x86_:1;
       u8 _5v_:1;
       u8 eth2Can_:1;
       u8 switch_:1;
       u8 lida_:1;
       u8 _12v_:1;
       u8 recharge_:1;
       u8 head_:1;
       
   }power;
  struct{
    u8 data0;
    u8 data1;
  }normal;
}ChassisPower;

typedef struct{
    u8  key;
    u8  key_time;

}LOCK_KEY_STRUCT;

typedef struct{
    u8  key;
    u16 year;
    u8  month;
    u8  day;
    u16 serial_number;

}SERIAL_CODE_STRUCT;

typedef struct{
    u8  key;
    char name[6];

}NAME_STRUCT;


typedef struct{
    u8  key;
    u8  prameter_number;
    u8  pameter[5];

}PRAMETER_STRUCT;

typedef struct{
    u32  status:8;
    u32  program_lenth:24;
    u8  hardware_version_low;
    u8  hardware_version_high;
}UPGRADE_RESPONSE_STRUCT;

typedef struct{
    u32  key:8;
    u32  program_length:24;
}UPGRADE_REQUEST_STRUCT;

typedef struct{
    u32  next_address:24;
    u32  pre_code;

}PROGRAMMING_RESPONSE_STRUCT;

typedef struct{
    u32  program_address:24;
    u32  code;

}PROGRAMMING_REQUEST_STRUCT;

typedef struct{
    u8  status;
    u32  receive_crc;

}PROGRAM_END_RESPONSE_STRUCT;

typedef struct{
    u32  send_crc;

}PROGRAM_END_REQUEST_STRUCT;


typedef struct
{
    u8   cmd;
    union BODY{
        PRODUCT_CODE_STRUCT product_code;
        VERSION_STRUCT      version;
        LOCK_KEY_STRUCT     lock_key;
        SERIAL_CODE_STRUCT  serial_code;
        NAME_STRUCT         name;
        PRAMETER_STRUCT     prameter;
        UPGRADE_RESPONSE_STRUCT     upgrade_response;
        UPGRADE_REQUEST_STRUCT      upgrade_request;
        PROGRAMMING_RESPONSE_STRUCT programming_response;
        PROGRAMMING_REQUEST_STRUCT  programming_request;
        PROGRAM_END_RESPONSE_STRUCT program_end_response;
        PROGRAM_END_REQUEST_STRUCT  program_end_request;
        u8 data[7];
    }body;
}CAN_PACKAGE_CMD_STRUCT;

typedef struct
{
    u8   cmd;
    union BODY{
        PRODUCT_CODE_STRUCT product_code;
        VERSION_STRUCT      version;
        LOCK_KEY_STRUCT     lock_key;
        SERIAL_CODE_STRUCT  serial_code;
        NAME_STRUCT         name;
        PRAMETER_STRUCT     prameter;
        UPGRADE_RESPONSE_STRUCT     upgrade_response;
        UPGRADE_REQUEST_STRUCT      upgrade_request;
        PROGRAMMING_RESPONSE_STRUCT programming_response;
        PROGRAMMING_REQUEST_STRUCT  programming_request;
        PROGRAM_END_RESPONSE_STRUCT program_end_response;
        PROGRAM_END_REQUEST_STRUCT  program_end_request;
        u8 data[7];
    }body;
    int bus;
    int channel;
}CAN_PACKAGE_STRUCT;

#pragma pack()

#define DF_UPGRADE_SIZE (128*1024)

class upGrade
{
public:

    upGrade(boost::function<void(std::string)> callback = NULL);

public:
    PRODUCT_CODE_STRUCT *getProductCode(int bus,int channel);
    VERSION_STRUCT *getVersion(int bus,int channel);
    LOCK_KEY_STRUCT *getKey(int bus,int channel);
    SERIAL_CODE_STRUCT *getSerialCode(int bus,int channel);
    NAME_STRUCT getname(int bus,int channel);
    int setUpgrade(int bus,int channel,int key,std::string hardwareVersion,int length);
    int programming(int bus,int channel,int address,u32 code,int time_out);
    std::string progradeEnd(int bus,int channel,u32 crc);
    void progressBar(std::string str);
    int getActionFromFile(const char *name,char **buffer);
    bool compare(std::string command,std::string cmd_name);
    std::string enterUpgrade(int bus, int channel,int vid,int pid,std::string hardware_version,int length,int 
    expectStatus);
    std::string checkhardvare(int bus, int channel,int vid,int pid,std::string hardware_version);
    std::string  upGrading(int bus, int channel,int vid,int pid,std::string hardware_version,std::string file);

    bool waitErase(int bus,int channel);

	//判断是否是舵机
	bool idIsSteer(int channel);
    bool idIsChassis(int channel);
    bool idIsHeadPower(int channel);
   bool PrintPercent(unsigned int percent);//decide whehter print the percent
   void InitPercentPara();
   void SetReportTimer(int timer);
   bool GetChassisPower(ChassisPower *chassis_power);
   bool ChassisPowerOff();
   bool ChassisPowerOn();
   bool resetChassisPower();
   void idPowerStartTimer(int bus,int channel);
   void idPowerStopTimer(int bus,int channel);
   bool sendPacketToCanBusWithoutPacket(unsigned int  bus, unsigned int channel,unsigned int mode,unsigned char cmd,unsigned int size = 0);
   bool sendPacketToCanBusWithUpgradePacket(unsigned int bus, unsigned int channel, unsigned int mode, unsigned char cmd, int key, int length);
   bool sendPacketToCanBusWithProgramPacket(unsigned int bus, unsigned int channel, unsigned int mode,unsigned char cmd, int address, u32 data);
   bool sendPacketToCanBusWithProgramEndPacket(unsigned int bus, unsigned int channel, unsigned int mode,unsigned char cmd, u32 crc);
   bool upgradeCheckOK(int bus, int channel);
   bool checkLowPower(int bus, int channel);
   bool recoverLowPowerConsumption();
   u8* getPacktFromCanBus(int bus,int channel,int condition,int timeWaite);
   void swtichPktRecvier(bool on_);
private:
    std::vector<CAN_PACKAGE_STRUCT> product_code_list_;
    std::vector<CAN_PACKAGE_STRUCT> version_list_;
    std::vector<CAN_PACKAGE_STRUCT> serial_code_list_;
    std::vector<CAN_PACKAGE_STRUCT> name_list_;
    boost::function<void(std::string)> callback_;
private:
    void cal_crc(unsigned int ptr);
    void reset_crc(void){CRC = 0xFFFFFFFF;}
    unsigned int get_crc(void){return CRC;}
    unsigned int    CRC;    // init
private:
    CanPkg package;
	bool  upgrade_percent[UP_GRADE_PERCENT_SETION];
    upgradeCanPkg canPkg;
};

#endif // INCLUDEUPGRADE_H
