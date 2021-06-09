#ifndef DATA_ROUTER_JOINT_SET_H
#define DATA_ROUTER_JOINT_SET_H

#include <stdio.h>
#include <math.h>
#include "tinyxml.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "upgrade.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <vector>
#include <fstream>
struct UPGRADE {
   std::string name;
   unsigned int channel;
   std::string can_birdge;
   unsigned int bus;
   std::string driver;
   bool upgraded;
};

struct PRODUCT {
    unsigned int pid;
    unsigned int vid;
    std::string hardware_version;
    std::string software_version;
	std::string software_upgrade_threshold;
    std::string bin_file;
    std::string product;
	bool forceUp;
	
    bool upgraded;
};
struct UPVER{
    char  name[64];
    unsigned int channel;
    unsigned int bus;
    char software_version[64];
	char hardware_version[64];
	char software_upgrade_threshold[64];
    char driver[64];
};

struct CHEKVSION{
   std::string name;
   unsigned int channel;
   unsigned int bus;
   std::string sotftversion;
   time_t time;
};



class servoCmd
{
public:

   servoCmd(std::string command,std::string explain){
        command_ = command;
        explain_ = explain;
    }

    std::string command_;
    std::string explain_;

};

class dataRouterJointSet
{
public:
    dataRouterJointSet();
    ~dataRouterJointSet();
    bool init(void);
    std::string upgradeMcu(bool isForce,bool isAll, unsigned int channel,bool &version_get_flag,std::string &failed_mcu);
    
    void setUpgradeFolder(std::string folder);
    
    bool getMcuVersion(int bus,int channel,std::string &ver);
    bool configServerSrv(std::string &req,
                               std::string &resq);
	bool productVersionOutput();//用于自检
	void packetReciveSwitch(bool on_);
private:
   VERSION_STRUCT* upgradeGetVersion(int bus, int channel);
    PRODUCT_CODE_STRUCT* upgradeGetProductCode(int bus, int channel);
    bool upgradeUpgrading(int bus, int channel,int vid,int pid,std::string hardversion ,std::string binfile);
    bool upgradeResetChassisPower(std::vector<CHEKVSION> *upok_list);
    bool  upgradeSoftwarerecheck(std::vector<CHEKVSION> *upok_list,std::string &failed_mcu,bool isForeced = false);
    void formDevicePackage(CAN_PACKAGE package);
    void formCanBusPackage(CAN_PACKAGE *package,int packageNum);
    void setUpgradeStatus(int channel,bool status);
    bool getBaseInfo(VERSION_STRUCT &ver_strt,PRODUCT_CODE_STRUCT &pro_strt,UPGRADE *upgrade,std::vector<std::string>*errlist,std::string norup);
    bool versionProcess(std::string &hard_ver, std::string &soft_ver,VERSION_STRUCT* ver,int channel);
    bool upgradeCheckSoftVersion(PRODUCT_CODE_STRUCT * Pproduct,PRODUCT *PproCfg,std::string software,bool isForce);
    void outPutInfoAftrupgrade(time_t* start_time,std::vector<CHEKVSION>* oklist,std::vector<std::string>*errlist,std::string 
    norlist);

    void SetMcuReportTimer(int timer );
    void StopPowerAndSensorReportTImer(int bus,int channel);
    void StartPowerAndSensorReportTImer(int bus,int channel);
private:
    std::string config_command_;
    std::vector<servoCmd> command_list_;
    std::string slamHostIP;
    std::string slamPort;
    std::string  folder_;
    std::string  upgrading_product_;
    std::string loadYaml(void);
	std::string parse_servo_yaml(const YAML::Node &doc);
    void listDir(std::string path,std::vector<std::string> &path_list);
    void folderTest(void);
    void mcuChannelIdShow(void);
    void initCommandList(void);
    bool compare(std::string command,std::string cmd_name);
    std::string runCommand(std::string command);
    std::string command_Completion(std::string command);
    bool checkHardWareVersion(std::string orgversion,std::string upversion);//升级的硬件版本要匹配
    bool checkSoftWareVersion(std::string orgversion,std::string upversion,std::string softthreshold,bool flag);//不能够向下升级
    void updateProgressBar(std::string str);
    void getVersion(int bus,int channel);
    std::string setIdTo40(void);
	
	bool create_multi_dir(const char *path);
    void getId(int bus,int channel = 0);
    
	bool productVersionCompare(int bus,int channel,char* name,char* hardware,char* software,char* 
	threshold,std::ofstream* of_c, std::ofstream* of_f);//用于自检
	bool chassisVersionCompare(int bus,int channel,char*name,char*driver,char* hardware,char* software,char* 
threshold,std::ofstream* of_c,std::ofstream* of_f);//用于电机多版本检查
    void logOut(std::string str);


    upGrade *upgrade_p_;
    
    boost::thread *spinner_thread_;

    std::vector<UPGRADE> upgrade_list_;
    std::vector<PRODUCT> product_list_;
    int angle_channel_;
    float angle_angle_;
    int version_bus_;
    int version_channel_;
    int version_hardware_;
    bool isloadcfg;    
};


#endif // DEVICEMANAGER_H
