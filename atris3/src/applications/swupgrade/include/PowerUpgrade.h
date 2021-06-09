#ifndef ELECTRIC_POWER_UPGRADE_H__
#define ELECTRIC_POWER_UPGRADE_H__

#include "include/MonitorUpgrade.h"
#include "include/MonitorUpgradeErrorCode.h"
//#include "include/UpgradeBase.h"
#include "include/UpgradeComm.h"
#include "include/ChassisUpgrade.h"
#include "include/ChassisUpgradeErrorCode.h"

#define ELECTRIC_POWER_ATRIS_DEBUG false
// report the upgrade output to file for debugging purpose
#define UPGRADE_RESULT_FILE_NAME "upgradeOutputFile"
#define UPGRADE_OUT_FILE_PATH "/userdata/tmp/"
#define UPGRADE_INDICATOR_FILE_NAME "upgradeStatus"
#define UPGRADE_FOLDER "/home/atris/atris_app/firmware"

#define MONITOR_FIRMWARE_FOLDER_ORIG "/home/atris/atris_app/firmware/monitor"
#define CHASSIS_FIRMWARE_FOLDER_ORIG "/home/atris/atris_app/firmware/chassis_controller"

#define MONITOR_YAML_FILE_PATH "/home/atris/atris_app/bin/dist/hfs/monitor/config.yaml"
#define CHASSIS_YAML_FILE_PATH "/home/atris/atris_app/bin/dist/hfs/chassis_controller/config.yaml"

#define NUM_OF_BOARD_UPGRADE 2 // 2 boards need to be upgraded
#define UPGRADE_RECHECK_SWITCH 1 // whether or not we need to do the after upgrade recheck
#define UPGRADE_PRODUCT_LIST_DEBUG 1
typedef struct __PRODUCT_UPGRADE_INFO
{
    std::string product_name; // product name
    std::string target_part; // target part , from where (eg : download)
	std::string source_part; // source part , to where (eg : app, bl)
	std::string app_version; // app version
	std::string boot_version; // boot version
	std::string app_md5_str; // app md5 string
	std::string bl_md5_str; // boot md5 string
	bool forceUp; // force upgrade
    bool upgraded; // is upgraded
} PRODUCT_UPGRADE_INFO;

typedef struct __UPGRADE_RESULT
{
   std::string product_name;
   std::string target_part;
   std::string soft_version;
   int err_code;
   std::string error_string;
   // upgrade consumed time
   time_t time_start; 
   time_t time_finish;
   time_t time_consume;
   bool status; // whether or not the upgrade is success
} UPGRADE_RESULT;

typedef struct __PRODUCT_YAML_PATH
{
	std::string product_name;
	std::string product_yaml_config_path;
} PRODUCT_YAML_PATH;

// electric power atris upgrade
class PowerUpgrade
{

	public:
		PowerUpgrade();
		~PowerUpgrade();
		int upgradeInit(void); // upgrade init
				
		// output is the failed mcu list
		bool upgradeAllMcu(std::string & failed_mcu_list);
		// only upgrade one mcu at once
		bool upgradeMcuSingle(const std::string & product_name);

		static PowerUpgrade* get_instance(){
            static PowerUpgrade singleton;
            return &singleton;
        }

        bool getUpgradeManualCtrl(void){return is_upgrade_manual_;};
        void setUpgradeManualCtrl(bool bManual){is_upgrade_manual_ = bManual;};

	private:
		int init(); // copy from firware foldder to /userdata/tmp operation
		void setUpgradeBoardNum(int num){num_board_upgrade_ = num;};
		int getUpgradeBoardNum(void){return num_board_upgrade_;};
		int firmwarePreCheck(bool monitor_check, bool chassis_controller_check); // check whether or not the config yaml file exist
        bool checkFirmwareFileFolder();
        int copyUpgradeFirmwareToHfs();
		bool parse_yaml_str(PRODUCT_UPGRADE_INFO & product, const std::string yaml_str);
		bool createMultiDir(const char *path); // create directory
		bool compare(std::string command,std::string cmd_name);
		void print_product_list(void);
		void setUpgradeStatus(bool status);
		void outputUpgradeResult(/*const std::vector<UPGRADE_RESULT> & up_res_vec*/std::string & failed_list);
		bool resultRecheck(std::string product_name, int err_code, std::string err_string, time_t upgrade_time_start, time_t upgrade_time_finish, UPGRADE_RESULT & up_res_out);
		std::string loadYaml(void);
		//void upGradeProcessPrint(std::string product_name , int num, int stat_arr[]);
		int strFind(char * src , char  * dest);
		void enqueUpgradeResult(const UPGRADE_RESULT & upgrade_res);

		void sendAllUpFinToSer();
		void postMcuUpgradeStatusToWeb(const std::string & upgrade_status, const std::string & upgrade_result, int upgrade_progress);

		void constructFailUpResult(std::string product_name, int err_code , std::string err_string , time_t upgrade_time_start, time_t upgrade_time_finish, UPGRADE_RESULT & product_upgrade_result);
		PRODUCT_UPGRADE_INFO & getProdUpgInfoByName(std::string prod_name);
		void recordUpgradeStartTime(void);
		std::string getLocalTimeStr(time_t time_stamp);
	private:
		bool is_cfg_file_ok_; // check if all the yaml config file ok
		bool is_upgrading_;
		bool is_all_mcu_finish_; // if all mcu board finish upgrade
		bool is_upgrade_manual_; // if upgrade is manually
		int num_board_upgrade_; // num of boards need to be upgraded
		int num_of_config_file_; // num of yaml config file
		std::string upgrade_start_time_; // record the last date time upgrade start
		MonitorUpgrade monitor_upgrade_; // monitor upgrade instance
		ChassisUpgrade chassis_upgrade_; // chassis upgrade instance
		std::vector<PRODUCT_YAML_PATH> yaml_config_file_list_; // yaml config file list
		std::vector<PRODUCT_UPGRADE_INFO> product_list_; // product list that got upgrade info in it
		std::vector<UPGRADE_RESULT> upgrad_result_list_; // upgrade result list that have the mcu upgrade result
		//std::vector<UpgradeBase *> product_upgrade_list_; // products that need to be upgraded

		int num_board_upgraded_; // num of board upgraded in this round
};

#endif
