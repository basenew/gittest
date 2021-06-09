/*
 * firmware.h
 *
 *  Created on: 2018-7-21
 *      Author: fupj
 */

#ifndef FIRMWARE_H_
#define FIRMWARE_H_
#include <boost/thread.hpp>
#include "can_upgrade/include/datarouterJointSet.h"
#include "gs_upgrade/gs_upgrade.h"
#include "can_upgrade/include/bmsUpgrade.h"
#include "PowerUpgrade.h"

typedef enum{
 UPGRADE_SUCCESS = 0,//升级成功
 UPGRADE_GS_FAILED =1,//高仙升级失败
 UPGRADE_POWER_FAILED =2 ,//电源板升级失败
 UPGRADE_BATTERY_MONITOR_FAILED = 4,//电池监控板升级失败
 UPGRADE_READ_MCU_CONFIG_FAILED = 8,//MCU配置文件读取失败
 UPGRADE_BMS_FAILED = 16//BMS升级失败
}UPGRADE_FLAG;


class Firmware {
public:
    Firmware();
    ~Firmware();
    static Firmware* get_instance(){
	    static Firmware singleton;
	    return &singleton;
    };
    int upgrade();
    int upgradeAllMcuManual();
    int upgradeSingleMcuManual(const std::string product_name); // testing interface
    //int32_t upgrade_motor(); // test set by jzx
    string getVersionGS();
    string getVersionPower();
    string getVersionBatteryMonitor();

    string getVersionBMS();

private:
    void versionGetThread();

private:
    dataRouterJointSet mcuUpgrade_;
    GSupgrade gsUpgrade_;
    BMSupgrade bmsUpgrade_;
    boost::thread *ver_get_thread_;
    string ver_gaussian_;
    string ver_power_;
    string ver_batteryMonitor_;
    string ver_bms_;
    static boost::mutex mutex_;
    bool is_upgrade;
    bool enter_waite_upgrade_;
};

#endif /* FIRMWARE_H_ */
