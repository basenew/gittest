/*
 * swupgrade.h
 *
 *  Created on: 2018-7-21
 *      Author: fupj
 */

#ifndef SWUPGRADE_H_
#define SWUPGRADE_H_
#include <boost/thread.hpp>
#include <json/json.h>
#include "ros/ros.h"
#include "log/log.h"
#include "database/sqliteengine.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetSwVersion.h"
#include "atris_msgs/GetOTAPreCondition.h"
#include "include/PowerUpgrade.h"

class SWDownload;
class PowerUpgrade;

class SWUpgrade {
public:
    virtual ~SWUpgrade();
    static SWUpgrade* get_instance();
    static void setUpgrading(bool upgrading);
    static void setUpgradeStatus(int status);
    std::string getImxUpgradeStatus();

private:
    SWUpgrade();
    bool doGetSwVersion(atris_msgs::GetSwVersion::Request& req, atris_msgs::GetSwVersion::Response& res);
    void messageInstantReceive(const atris_msgs::SignalMessage& msg);
    //void doRequestSwVersion(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void doRequestSwUpgrade(const atris_msgs::SignalMessage& msg, const Json::Value& values);
    void doFirmwareUpgradeThread();
    int doFirmwareUpgradeTask();
    bool initSwUpgradeTable(void);
    void reinitSwUpgradeDataBase(void);
    int updateLogSerialToDataBase(const std::string & log_serial_str);
    int queryLogSerialFromDataBase(std::string & query_log_serial_str);
    
    void setImxUpgradeStatus(std::string status);
    std::string getAtrisVersion();
    std::string getPowerVersion();
    std::string getGsVersion();
    std::string getBMVersion();
    std::string getBMSVersion();
    void responseResult(const atris_msgs::SignalMessage& origin,
      const Json::Value& content, std::string title);
    int judgeOTAPreCond();

private:
    friend class SWDownload;
    friend class PowerUpgrade;
    static boost::mutex mutex_;
    static SWUpgrade* swupgrade_;
    static bool upgrading_now_;
    std::string atris_version_;
    std::string log_serial_; // serial number for upgrading, use the same log serial after reboot to upgrade mcu
    boost::thread* firmware_thread_;
    boost::shared_ptr<SWDownload> obj_;
    ros::NodeHandle nh_;
    ros::Publisher signal_resp_pub_;
    ros::Publisher aisound_tts_pub_;
    ros::Subscriber signal_req_sub_;
    //ros::ServiceServer get_sw_version_srv_;
    ros::ServiceClient get_ota_pre_condition_srv_client_;
};

#endif /* SWUPGRADE_H_ */
