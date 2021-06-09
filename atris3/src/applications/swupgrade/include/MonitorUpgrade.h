#ifndef MONITOR_UPGRADE_H__
#define MONITOR_UPGRADE_H__

#include "tiny_ros/atris_msgs/CanPkg.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "tiny_ros/ros.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include <thread>
#include "UpgradeComm.h"
#include "MonitorUpgradeErrorCode.h"
#include "utils/utils.h"


#define UPGRADE_TIMEOUT  (60) // 60 seconds download timeout is set to 60 seconds
#define READ_VER_TIMEOUT  (3) // 3 seconds for read version timeout

// update the upgrade result to power upgrade class , upgrade status , product name , error code , error string , percentage and time stamp
//using MonitorUpStatusCB = std::function<void(const int &, const std::string& , const int& , const std::string& , const int& , const time_t&)>;
//using MonitorUpStatusCB = std::function<void(const UP_RESULT &)>;

class MonitorUpgrade
{
	public:
		MonitorUpgrade(std::string prod_name = "monitor");
		~MonitorUpgrade();
		void init(); // initialization
		bool startUpgrade(int & error_code, std::string & error_str); // start upgrade function
		bool readMonitorVer(std::string & current_soft_ver, std::string & target_soft_ver); // read current monitor board version
		void testUpgrade();
		bool queryVersionInfo(std::string & current_soft_ver, std::string & target_soft_ver);
		void waitUpgradeFinish(int32_t & error_code, std::string & error_str); // thread cb func


	private:
		void setProductName(std::string product_name){product_name_ = product_name;};
		std::string getProductName(void) const {return product_name_;};
		//std::string getProductName(void) const {return product_name_;};
		int sendUpgradeStart(const std::string & upgrade_url); // send upgrade start command
		int sendReadVersionPkt(); // send read monitor version package

		void setCurMonitorVer(std::string current_monitor_ver){current_version_ = current_monitor_ver;};
		std::string getCurMonitorVer() const {return current_version_;};

		void setTarMonitorVer(std::string target_monitor_ver){target_version_ = target_monitor_ver;};
		std::string getTarMonitorVer() const {return target_version_;};

		void setPercent(int upgrade_percentage){ upgrade_percentage_ = upgrade_percentage;};
		int getPercent(){ return upgrade_percentage_;};

		// if upgrade is active
		void setMonitorUpgradeFinish(bool is_active);
		bool isMonitorUpgradeFinish();

		// set and get last upgrade flag
		void setLastUpgradeFlag(int32_t last_upgrade_flag){last_upgrade_flag_ = last_upgrade_flag;};
		int32_t getLastUpgradeFlag() const {return last_upgrade_flag_;};

		void setTimeOutCount(int count){time_out_cnt_ = count;};
		int getTimeOutCount(){return time_out_cnt_;};

		//int32_t getUpgradeErrCode() { boost::unique_lock<boost::mutex> lock(upgrade_code_mutex_); return upgrade_finish_code_;};

		void on_recv_monitor_up_resp(const tinyros::atris_msgs::CanPkg &msg);

		void notifyReadMcuResp(const std::string & cur_software_version, const std::string & tar_software_version);
		int isUpgradeErrCodeValid(int32_t up_err_code);
		
		int get_upgrade_resp_info(const tinyros::atris_msgs::CanPkg &msg, std::string & current_version , std::string & target_version);

		//void startThread();
		// post upgrade status to its manage class
		//void postUpgradeStats(const UP_RESULT & upgrade_result);

		void setFinishCode(int upgrade_finish_code){ upgrade_finish_code_ = upgrade_finish_code;};
		int getFinishCode(){ return upgrade_finish_code_;};

		void setErrString(std::string err_str){ upgrade_error_str_ = err_str;};
		std::string getErrString(){ return upgrade_error_str_;};
		bool checkMonitorNetwork(void);
		
	private:
		std::string product_name_; // product name
		std::string target_version_; // target version for monitor board
		std::string current_version_; // current version of monitor board
		int upgrade_percent_; // upgrade progress
		volatile bool is_upgrade_finish_; // whether or not is during upgrade process

		int32_t upgrade_finish_code_; // upgrade error code , upgrade status code when finished
		std::string upgrade_error_str_; // error code translated to string

		int32_t last_upgrade_flag_; // last stored upgrade flag
		int32_t upgrade_percentage_; // 0 ~ 100 %
		//time_t upgrade_start_time_; // mark upgrade start time
		//time_t upgrade_finish_time_; // mark upgrade start time

		boost::mutex mutex_;
		boost::mutex upgrade_active_mutex_;
		boost::condition_variable_any upgrade_active_cond_;
        boost::condition_variable_any recv_ver_cond_;
		bool is_read_ver_success_;

		int time_out_sec_;
		int time_out_cnt_;
		Utils *utils;

		//MonitorUpStatusCB up_st_cb_;
		
		tinyros::Publisher monitor_upgrade_req_pub_;
		tinyros::Subscriber<tinyros::atris_msgs::CanPkg, MonitorUpgrade> monitor_upgrade_resp_sub_;
};

#endif
