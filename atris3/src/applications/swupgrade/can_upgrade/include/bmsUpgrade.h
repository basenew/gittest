#ifndef BMS_H_H
#define BMS_H_H


#include "yaml-cpp/yaml.h"
#include "upgradeCanPkg.h"

using namespace std;
#define BMS_UP_GRADE_PERCENT_SETION   4


typedef unsigned char u8;

typedef struct{
    int pid_;
    string ver_hard;
    string ver_soft;
    bool is_force;
    string bin_path;
}BMSCfg;

typedef struct{
    unsigned char cmd;
    unsigned char data[7];

}BMS_PACKAGE_STRUCT;

typedef struct{
    u8  pid_high;
    u8  pid_low;
	u8  hardware_version_high;
	u8  hardware_version_low;
	u8  software_version_high;
	u8  software_version_low;
}BMS_VERSION_STRUCT;

typedef enum{
	print_percent_30_ = 30,
	print_percent_60_ = 60,
	print_percent_100_ = 100,
}BMS_PRIT_PERCNT;


class BMSupgrade
{
    public:
        BMSupgrade();
        ~BMSupgrade();
        bool init();
        bool upgrade();
        bool getVersion(string & ver_);
        void switchPktRecvier(bool on_);
    private:
        bool versionCheck(BMSCfg &cfg);
        bool verGet(BMS_VERSION_STRUCT & ver_now, BMSCfg & cfg);
        bool verCfgRead(BMSCfg &cfg);
        bool verRunRead(BMS_VERSION_STRUCT &ver_now);
        bool startUpgrade(string& bin_path);
        bool verCompare(BMS_VERSION_STRUCT &ver_now,BMSCfg &cfg,bool &isNeedUp);
        bool upgrading(unsigned char* updata_data,int fileLength);
        void initPercentPara();
        bool printPercent(unsigned int percent);
        int  getDataFromFile(const char * name, char ** buffer);
        bool sendRebootTransCmd();
        bool sendUpgradeStartCmd();
        bool sendUpgradeEndCmd();
        bool sendUpgradedata(unsigned int bus, unsigned int channel_send,unsigned int channel_recv,unsigned char * data,int size,int timeWaite);
        bool sendPacketToCanBusWithoutPacket(unsigned int bus, unsigned int channel,unsigned char cmd);
        bool sendPacketToCanBusWithUpgradePacket(unsigned int bus, unsigned int channel,BMS_PACKAGE_STRUCT &data,int size);
        u8* getPacktFromCanBus(int bus,int channel,int condition,int timeWaite);
        bool sendUpgradeEndCmdwithRetry(unsigned int retry_num,unsigned long sleep_time);
        bool sendUpgradeStartCmdWithRetry(unsigned int retry_num,unsigned long sleep_time);
        bool verRunReadwithRetry(BMS_VERSION_STRUCT &ver_now,unsigned int retry_num,unsigned long sleep_time);
   private:
        string bin_file_name;
        CanPkg package;
        upgradeCanPkg canPkg;

        bool  upgrade_percent[BMS_UP_GRADE_PERCENT_SETION];
};

#endif
