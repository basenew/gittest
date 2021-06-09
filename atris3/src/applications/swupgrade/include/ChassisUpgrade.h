#ifndef CHASSIS_UPGRADE_H__
#define CHASSIS_UPGRADE_H__

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
#include "ChassisUpgradeErrorCode.h"
#include "utils/utils.h"


#define UPGRADE_TIMEOUT  (60) // 60 seconds download timeout is set to 60 seconds
#define READ_VER_TIMEOUT  (3) // 3 seconds for read version timeout

class ChassisUpgrade
{
	public:
		ChassisUpgrade(std::string prod_name = "chassis_controller");
		~ChassisUpgrade();
		void init(); // initialization
		bool startUpgrade(int & error_code, std::string & error_str); // start upgrade function
		bool readChassisVer(std::string & current_soft_ver, std::string & target_soft_ver); // read current chassis board version
		void testUpgrade();
		bool queryVersionInfo(std::string & current_soft_ver, std::string & target_soft_ver);
		void waitUpgradeFinish(int32_t & error_code, std::string & error_str); // thread cb func


	private:
		void setProductName(std::string product_name){product_name_ = product_name;};
		std::string getProductName(void) const {return product_name_;};
		//std::string getProductName(void) const {return product_name_;};
		int sendUpgradeStart(const std::string & upgrade_url); // send upgrade start command
		int sendReadVersionPkt(); // send read Chassis version package

		void setCurChassisVer(std::string current_chassis_ver){current_version_ = current_chassis_ver;};
		std::string getCurChassisVer() const {return current_version_;};

		void setTarChassisVer(std::string target_chassis_ver){target_version_ = target_chassis_ver;};
		std::string getTarChassisVer() const {return target_version_;};

		void setPercent(int upgrade_percentage){ upgrade_percentage_ = upgrade_percentage;};
		int getPercent(){ return upgrade_percentage_;};

		// if upgrade is active
		void setChassisUpgradeFinish(bool is_active);
		bool isChassisUpgradeFinish();

		// set and get last upgrade flag
		void setLastUpgradeFlag(int32_t last_upgrade_flag){last_upgrade_flag_ = last_upgrade_flag;};
		int32_t getLastUpgradeFlag() const {return last_upgrade_flag_;};

		void setTimeOutCount(int count){time_out_cnt_ = count;};
		int getTimeOutCount(){return time_out_cnt_;};

		//int32_t getUpgradeErrCode() { boost::unique_lock<boost::mutex> lock(upgrade_code_mutex_); return upgrade_finish_code_;};

		void on_recv_chassis_up_resp(const tinyros::atris_msgs::CanPkg &msg);

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
		bool checkChassisNetwork(void);
		
	private:
		std::string product_name_; // product name
		std::string target_version_; // target version for chassis board
		std::string current_version_; // current version of chassis board
		int upgrade_percent_; // upgrade progress
		volatile bool is_upgrade_finish_; // whether or not is during upgrade process

		int32_t upgrade_finish_code_; // upgrade error code , upgrade status code when finished
		std::string upgrade_error_str_; // error code translated to string

		int32_t last_upgrade_flag_; // last stored upgrade flag
		int32_t upgrade_percentage_; // 0 ~ 100 %

		boost::mutex mutex_;
		boost::mutex upgrade_active_mutex_;
		boost::condition_variable_any upgrade_active_cond_;
        boost::condition_variable_any recv_ver_cond_;
		bool is_read_ver_success_;

		int time_out_sec_;
		int time_out_cnt_;
		Utils *utils;
		
		tinyros::Publisher chassis_upgrade_req_pub_;
		tinyros::Subscriber<tinyros::atris_msgs::CanPkg, ChassisUpgrade> chassis_upgrade_resp_sub_;
};

#endif
