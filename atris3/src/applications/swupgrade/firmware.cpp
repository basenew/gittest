/*
 * firmware.cpp
 *
 *  Created on: 2018-7-21
 *      Author: fupj
 */

#include "firmware.h"
#include "string.h"
using namespace std;

boost::mutex Firmware::mutex_;
//Firmware* Firmware::firmware_ = NULL;

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>



#define HEAD_POWER_CHANNEL 0xe4
#define BATTERY_MONITOR_CHANNEL 0xe3

#define POWER_CHANNEL_STR "e4"
#define BATTERY_MONITOR_CHANNEL_STR "e3"
#define MCU_CONFIG_STR "e1"
#define HEAD_POWER_BUS 0x1



Firmware::Firmware() {
	// TODO Auto-generated constructor stub
    printf("firmware contructor called\r\n");
    ver_gaussian_ ="";
    ver_power_="";
    ver_batteryMonitor_="";
    ver_bms_ ="";
    is_upgrade = false;
    enter_waite_upgrade_ = false;
    //gsUpgrade_.init();
    //mcuUpgrade_.init();
    //bmsUpgrade_.init();
    //ver_get_thread_ =  new boost::thread( boost::bind( &Firmware::versionGetThread, this));
  
}

Firmware::~Firmware() {
	// TODO Auto-generated destructor stub
	printf("firmware destructor called\r\n");
}


int Firmware::upgrade() {
    
    is_upgrade = true;
    int upgrade_flag = 0;
    int iRet;
    bool mcu_up_res;
    string failed_mcu_flag="";
    #if 0
    //等待停止获取版本信息；
    int i = 0;
    while(true && i++ < 30)
    {
        if(true == enter_waite_upgrade_)
        {
            break;
        }
        sleep(1);
        
    }

    //高仙
    if(false == gsUpgrade_.upgrade())
    {
        upgrade_flag |= UPGRADE_GS_FAILED;
    }
    //bms
    #if 0
    bmsUpgrade_.switchPktRecvier(true);
    if(false == bmsUpgrade_.upgrade())
    {
        upgrade_flag |= UPGRADE_BMS_FAILED ;
    }
    bmsUpgrade_.switchPktRecvier(false);
    #endif
    //mcu
    mcuUpgrade_.packetReciveSwitch(true);
    string failed_mcu_flag="";
    string ret_str =  mcuUpgrade_.upgradeMcu(false,true,0,mcu_version_get_flag,failed_mcu_flag);
   
    if(NULL == strstr(ret_str.c_str(),"ok")|| false == mcu_version_get_flag)
    {
        if(failed_mcu_flag.find(POWER_CHANNEL_STR)!= string::npos)
        {
            upgrade_flag |= UPGRADE_POWER_FAILED;
        }
        if(failed_mcu_flag.find(BATTERY_MONITOR_CHANNEL_STR)!= string::npos)
        {
            upgrade_flag |= UPGRADE_BATTERY_MONITOR_FAILED;
        }
        if(failed_mcu_flag.find(MCU_CONFIG_STR)!= string::npos)
        {
            upgrade_flag |= UPGRADE_READ_MCU_CONFIG_FAILED;
        }
    }
    mcuUpgrade_.packetReciveSwitch(false);
    #endif
    iRet = PowerUpgrade::get_instance()->upgradeInit();
    if(iRet < 0)
    {
        log_error("%s power upgrade init failed , iRet = %d",__FUNCTION__, iRet);
        return -2;
    }

    mcu_up_res = PowerUpgrade::get_instance()->upgradeAllMcu(failed_mcu_flag);
    if(mcu_up_res)
    {
      upgrade_flag = 0;
    }
    else
    {
      upgrade_flag = -1;
    }
    
    printf("%s , %d , %s", __FUNCTION__, upgrade_flag, failed_mcu_flag.c_str());

    log_info("%s , %d , %s\r\n",__FUNCTION__, upgrade_flag, failed_mcu_flag.c_str());
    is_upgrade = false;
    return upgrade_flag;
 
}
// create interface to upgrade only system monitor
int Firmware::upgradeSingleMcuManual(const std::string product_name)
{
    bool mcu_up_res;
    int upgrade_flag = 0;
    int iRet;

    //PowerUpgrade::get_instance()->init();
    PowerUpgrade::get_instance()->setUpgradeManualCtrl(true);
    iRet = PowerUpgrade::get_instance()->upgradeInit();
    if(iRet < 0)
    {
        log_error("%s upgrade single mcu manual upgrade init failed iRet = %d",__FUNCTION__, iRet);
        return -2;
    }

    mcu_up_res = PowerUpgrade::get_instance()->upgradeMcuSingle(product_name);
    if(mcu_up_res)
    {
        upgrade_flag = 0;
    }
    else
    {
        upgrade_flag = -1;
    }

    log_info("%s , %d\r\n",__FUNCTION__, upgrade_flag);
    return upgrade_flag;
}

int Firmware::upgradeAllMcuManual()
{
    bool mcu_up_res;
    int upgrade_flag = 0;
    int iRet;
    std::string failed_mcu_flag = "";

    //PowerUpgrade::get_instance()->init();
    PowerUpgrade::get_instance()->setUpgradeManualCtrl(true);
    iRet = PowerUpgrade::get_instance()->upgradeInit();
    if(iRet < 0)
    {
        log_error("%s upgrade all mcu manual upgrade init failed iRet = %d",__FUNCTION__, iRet);
        return -2;
    }

    mcu_up_res = PowerUpgrade::get_instance()->upgradeAllMcu(failed_mcu_flag);
    if(mcu_up_res)
    {
        upgrade_flag = 0;
    }
    else
    {
        upgrade_flag = -1;
    }

    log_info("%s , %d , %s\r\n",__FUNCTION__, upgrade_flag,failed_mcu_flag.c_str());
    return upgrade_flag;
}


string Firmware::getVersionGS()
{
    return ver_gaussian_;
}
string Firmware::getVersionPower()
{
     return  ver_power_;
}

string Firmware::getVersionBatteryMonitor()
{
     return ver_batteryMonitor_;
}

string Firmware::getVersionBMS()
{
    return ver_bms_;
}
#if 0
Firmware* Firmware::get_instance() {
	if (!firmware_) {
		boost::unique_lock<boost::mutex> lock(mutex_);
		if (!firmware_) {
			firmware_ = new Firmware();
		}
	}
	return firmware_;
}
#endif
// private

void Firmware:: versionGetThread()
{
    while(true)
    {
            sleep(2);
            if(true == is_upgrade)
            {
                enter_waite_upgrade_ = true;
                ver_gaussian_ = "";
                ver_power_ = "";
                ver_batteryMonitor_ = "";
                ver_bms_ = "";
                continue;
                
            }
            enter_waite_upgrade_ =false;
            //start receive package
            if(ver_power_.empty() || ver_batteryMonitor_.empty())
            {

                mcuUpgrade_.packetReciveSwitch(true);//turn on mcu packet recieve
            }
            //gaussian
            if(ver_gaussian_.empty())
            {

                string gs_ver="";
                gsUpgrade_.getVersion(gs_ver);
                if( !gs_ver.empty()&&gs_ver != ver_gaussian_)
                {
                    ver_gaussian_ = gs_ver;
                }
            }
         //power
          if(ver_power_.empty())
          {
              string power_ver="";
              mcuUpgrade_.getMcuVersion(HEAD_POWER_BUS,HEAD_POWER_CHANNEL,power_ver);
               if( !power_ver.empty()&&power_ver != ver_power_)
                {
                    ver_power_ = power_ver;
                }
          }
          //jk
          if(ver_batteryMonitor_.empty())
          {
             string batterM_ver_="";
             mcuUpgrade_.getMcuVersion(HEAD_POWER_BUS,BATTERY_MONITOR_CHANNEL, batterM_ver_);
             if(!batterM_ver_.empty()&&batterM_ver_ != ver_batteryMonitor_)
             {
                ver_batteryMonitor_ = batterM_ver_;
             }
          }
          //bms
          #if 0
           if(ver_bms_.empty())
          {
             
             bmsUpgrade_.switchPktRecvier(true);//turn on bms packet receive
             string bms_ver_="";
             ver_bms_.getVersion(bms_ver_);
             if(!bms_ver_.empty()&&bms_ver_ != ver_bms_)
             {
                ver_bms_ = bms_ver_;
                bmsUpgrade_.switchPktRecvier(false);//turn off bms packet receive
             }
          }
           #endif
        //turn off package recieve
        if(!ver_power_.empty()&& !ver_batteryMonitor_.empty())
        {
            mcuUpgrade_.packetReciveSwitch(false);
        }
        
    }
}








