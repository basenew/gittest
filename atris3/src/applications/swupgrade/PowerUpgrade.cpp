#include "include/PowerUpgrade.h"
#include "include/MonitorUpgrade.h"
#include "include/ChassisUpgrade.h"
#include "swupgrade.h"
#include "imemory/atris_imemory_api.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <time.h>
#include <dirent.h>
#include "yaml-cpp/yaml.h"

#define MAX_PATH_LENGTH 128

template<typename T>  
void operator >> (const YAML::Node& node, T& i)  
{
    i = node.as<T>();  
}

PowerUpgrade::PowerUpgrade()
	:num_board_upgrade_(NUM_OF_BOARD_UPGRADE)
	,num_board_upgraded_(0)
	,is_all_mcu_finish_(false)
	,is_upgrading_(false)
	,is_cfg_file_ok_(false)
    ,is_upgrade_manual_(false)
{
    log_info("%s constructor called...",__FUNCTION__);
    int iRet;
    int i;
    #if 0
    if(!getUpgradeManualCtrl())
    {
        log_warn("%s not using manual upgrade ctrl , using firware file from firmware folder!!!",__FUNCTION__);
        iRet = init();
        if(0 > iRet)
        {
            log_error("%s init failed!!!",__FUNCTION__);
        }
        else
        {
            log_info("%s init success...",__FUNCTION__);
        }
    }
    else
    {
        log_warn("%s using manual upgrade ctrl , using the file copied manually from outside!!!",__FUNCTION__);
    }
    #endif

}

int PowerUpgrade::upgradeInit(void)
{
    int iRet;

    std::string up_out_dir_path(UPGRADE_OUT_FILE_PATH);
    if(!createMultiDir(up_out_dir_path.c_str())) // upgrade result output file
    {
        log_error("create result output file directory failed!!!");
        return -2;
    }

    if(!getUpgradeManualCtrl())
    {
        log_warn("%s not using manual upgrade ctrl , using firware file from firmware folder!!!",__FUNCTION__);
        iRet = init();
        if(0 > iRet)
        {
            log_error("%s init failed!!!",__FUNCTION__);
            return -1;
        }
        else
        {
            log_info("%s init success...",__FUNCTION__);
        }
    }
    else
    {
        log_warn("%s using manual upgrade ctrl , using the file copied manually from outside!!!",__FUNCTION__);
    }

    is_all_mcu_finish_ = false;
    yaml_config_file_list_.clear();
    product_list_.clear();
    upgrad_result_list_.clear();

    return 0;
}


PowerUpgrade::~PowerUpgrade()
{
    log_info("%s destructor called...",__FUNCTION__);
}

// check whether or not the firware upgrade foldder exist
bool PowerUpgrade::checkFirmwareFileFolder()
{
    std::string chassis_firware_folder_path(CHASSIS_FIRMWARE_FOLDER_ORIG);
    std::string monitor_firmware_folder_path(MONITOR_FIRMWARE_FOLDER_ORIG);

    if(access(chassis_firware_folder_path.c_str(), F_OK) == 0 && access(monitor_firmware_folder_path.c_str(), F_OK) == 0)
    {
        log_info("%s firmware folder path check ok...\r\n",__FUNCTION__);
    }
    else
    {
        log_error("%s firmware folder path check failed!!!\r\n",__FUNCTION__);
        return false;
    }

    return true;
}

int PowerUpgrade::copyUpgradeFirmwareToHfs()
{
    log_info("start copy firmware folder to hfs\r\n");

    //std::string cmd = "cp -r "
    std::string monitor_sour_pos(MONITOR_FIRMWARE_FOLDER_ORIG);
    std::string chassis_sour_pos(CHASSIS_FIRMWARE_FOLDER_ORIG);
    std::string dest_pos = "/userdata/tmp/";
    FILE * fp = NULL;

    std::string cmd;
    cmd = "cp -r " + monitor_sour_pos + " " + dest_pos;
    log_info("%s copy monitor firmware command string is %s",__FUNCTION__, cmd.c_str());
    if((fp = popen(cmd.c_str(), "r")) != NULL)
    {
        log_info("cp monitor firmware folder to destination success...\r\n");
        pclose(fp);
        fp = NULL;
    }
    else
    {
        log_error("cp monitor firmware folder to destination failed!!!\r\n");
        return -1;
    }

    cmd = "cp -r " + chassis_sour_pos + " " + dest_pos;
    if((fp = popen(cmd.c_str(), "r")) != NULL)
    {
        log_info("cp chassis firmware folder to destination success...\r\n");
        pclose(fp);
        fp = NULL;
    }
    else
    {
        log_error("cp chassis firmware folder to destination failed!!!\r\n");
        return -1;
    }

    return 0;
}

// create output file directory
int PowerUpgrade::init()
{
	// create upgrade output directory
    log_info("%s power upgrade init...\r\n",__FUNCTION__);
    //std::string up_out_dir_path(UPGRADE_OUT_FILE_PATH);
    //if(!createMultiDir(up_out_dir_path.c_str())) // upgrade result output file
    //{
    //    log_error("create result output file directory failed!!!");
    //    return -1;
    //}

    #if 1
    if(!checkFirmwareFileFolder())
    {
        log_error("fireware folder does not exist!!!!\r\n");
        return -2;
    }
    else
    {
        // copyt upgrade fire to hfs
        log_info("start copy fireware folder to hfs...\r\n");
        copyUpgradeFirmwareToHfs();
    }
    #endif

    //is_all_mcu_finish_ = false;
    //yaml_config_file_list_.clear();
    //product_list_.clear();
    //upgrad_result_list_.clear();
    return 0;
}


void PowerUpgrade::constructFailUpResult(std::string product_name, int err_code ,  std::string err_string , time_t upgrade_time_start, time_t upgrade_time_finish, UPGRADE_RESULT & product_upgrade_result)
{
    log_info("%s product name : %s, error code : %d , error string : %s\r\n",__FUNCTION__, product_name.c_str(), err_code, err_string.c_str());
    product_upgrade_result.product_name = product_name;
    product_upgrade_result.target_part = getProdUpgInfoByName(product_name).target_part;
    product_upgrade_result.soft_version = "";
    product_upgrade_result.err_code = err_code;
    product_upgrade_result.error_string = err_string;
    product_upgrade_result.time_start = upgrade_time_start;
    product_upgrade_result.time_finish = upgrade_time_finish;
    product_upgrade_result.time_consume = upgrade_time_finish - upgrade_time_start;
    product_upgrade_result.status = false;
}

void PowerUpgrade::enqueUpgradeResult(const UPGRADE_RESULT & upgrade_res)
{
    upgrad_result_list_.push_back(upgrade_res);
}



PRODUCT_UPGRADE_INFO & PowerUpgrade::getProdUpgInfoByName(std::string prod_name)
{
    std::vector<PRODUCT_UPGRADE_INFO>::iterator iter = product_list_.begin();
    while (iter!=product_list_.end())
    {
    	if ((*iter).product_name == prod_name)
    	{
    		return (*iter);
    	}

    	iter++;
    }
}


bool PowerUpgrade::resultRecheck(std::string product_name, int err_code, std::string err_string, time_t upgrade_time_start, time_t upgrade_time_finish, UPGRADE_RESULT & up_res_out)
{
    bool bRet;
    bool bQuery = false;
    std::string cur_part_version;
    std::string tar_part_version;
    PRODUCT_UPGRADE_INFO product_info = getProdUpgInfoByName(product_name);

    if(product_name == "monitor")
    {
        log_info("compare software version using monitor queryVersionInfo function");
        bQuery = monitor_upgrade_.queryVersionInfo(cur_part_version, tar_part_version);
        if(!bQuery)
        {
            log_error("monitor board query result failed , after receive upgrade finish flag\r\n");
        }
        else
        {
            log_info("monitor board query result success... , board name : %s , current software version : %s, target software version : %s\r\n",product_name.c_str(), cur_part_version.c_str(), tar_part_version.c_str());
        }
    }
    else if(product_name == "chassis_controller")
    {
        log_info("compare software version using chassis queryVersionInfo function");
        bQuery = chassis_upgrade_.queryVersionInfo(cur_part_version, tar_part_version);
        if(!bQuery)
        {
            log_error("chassis board query result failed , after receive upgrade finish flag\r\n");
        }
        else
        {
            log_info("chassis board query result success... , board name : %s , current software version : %s, target software version : %s\r\n",product_name.c_str(), cur_part_version.c_str(), tar_part_version.c_str());
        }
    }
    else
    {
        log_error("can not find product ptr according to product name\r\n");
    }

    // fill other result part
    up_res_out.product_name = product_name;
    up_res_out.target_part = product_info.target_part;
    up_res_out.err_code = err_code;
    up_res_out.error_string = err_string;
    up_res_out.time_start = upgrade_time_start;
    up_res_out.time_finish = upgrade_time_finish;
    up_res_out.time_consume = up_res_out.time_finish - up_res_out.time_start;

    // compare upgrade part version
    if(product_info.target_part == "app")
    {
        if(cur_part_version == product_info.app_version)
        {
            log_info("board name : %s upgrading app, version check success...\r\n",product_name.c_str());
            bRet = true;
        }
        else
        {
            log_error("board name : %s upgrading app, version check failed!!!\r\n",product_name.c_str());
            bRet = false;
        }
    }
    else
    {
        if(cur_part_version == product_info.boot_version)
        {
            log_info("board name : %s upgrading boot, version check success...\r\n",product_name.c_str());
            bRet = true;
        }
        else
        {
            log_error("board name : %s upgrading boot, version check failed!!!\r\n",product_name.c_str());
            bRet = false;
        }
    }

    if(bRet)
    {
        up_res_out.status = true;
        up_res_out.soft_version = cur_part_version;
    }
    else
    {
        up_res_out.status = false;
        up_res_out.soft_version = "";
    }

    return bRet;
}

// upgrade single mcu board interface
bool PowerUpgrade::upgradeMcuSingle(const std::string & product_name)
{
    std::string ret_str = "";
    std::string upgraded_version;
    time_t upgrade_start_time = 0;
    time_t upgrade_finish_time = 0;
    int err_code;
    std::string err_msg;
    int i;
    bool bRet = true;
    bool bRet1 = true;
    UPGRADE_RESULT upgrade_result;

    log_info("%s upgrade %s single start",__FUNCTION__, product_name.c_str());
    if(product_name == "monitor")
    {
        if(firmwarePreCheck(true,false)!=1)
        {
            log_error("system monitor pre check failed");
            return false;
        }
        else
        {
            log_info("system monitor pre check success");
        }
    }
    else if(product_name == "chassis_controller")
    {
        if(firmwarePreCheck(false,true)!=1)
        {
            log_error("chassis controller pre check failed");
            return false;
        }
        else
        {
            log_info("chassis controller pre check success");
        }
    }
    else
    {
        log_error("unrecoginized product name : %s", product_name.c_str());
        return false;
    }

    // load yaml file
    ret_str = loadYaml();
    if(!compare(ret_str, "ok"))
    {
        log_error("%s load yaml file failed!!!",__FUNCTION__);
        return false;
    }
    else
    {
        log_info("%s load yaml file success...",__FUNCTION__);
    }

    upgrade_start_time = time(NULL); // record the upgrade start time

    if(product_name == "monitor")
    {
        monitor_upgrade_.startUpgrade(err_code, err_msg);
    }
    else if(product_name == "chassis_controller")
    {
        chassis_upgrade_.startUpgrade(err_code, err_msg);
    }
    else
    {
        log_error("product name : %s unrecoginized, start upgrade failed\r\n", product_name.c_str());
        return false;
    }

    if(err_code != 0)
    {
        // deal with error here
        upgrade_finish_time = time(NULL);// record the upgrade finish time
        log_error("%s upgrade %s fail , err code : %d err msg : %s", __FUNCTION__, product_name.c_str(), err_code , err_msg.c_str());
        constructFailUpResult(product_name, err_code , err_msg, upgrade_start_time, upgrade_finish_time, upgrade_result);
        //enqueUpgradeResult(upgrade_result);
        bRet = false;
    }
    else
    {
        // deal with success here
        // version recheck after upgrade finish

        upgrade_finish_time = time(NULL);// record the upgrade finish time
        log_info("%s upgrade %s success , do the post software version check",__FUNCTION__, product_name.c_str());
        bRet1 = resultRecheck(product_name, err_code , err_msg, upgrade_start_time, upgrade_finish_time, upgrade_result);
        if(!bRet1)
        {
            log_error("%s software version recheck failed!!!",__FUNCTION__);
            bRet = false;
        }
        else
        {
            log_info("%s software version recheck success...",__FUNCTION__);
        }
        //enqueUpgradeResult(upgrade_result);

    }

    log_info("%s upgrade %s single end",__FUNCTION__, product_name.c_str());

    setUpgradeManualCtrl(false);
    log_info("%s set upgrade manual control to false!!!",__FUNCTION__);

    return bRet;    
}

// call this function to upgrade all the mcu boards in serial
// upgrade each mcu board
// 1. first we load each yaml file into product struct
// 2. loop for each product and call the corresponding upgrade function
// 3. after the upgrade, we compare if the version equals the version in the yaml file(if we upgrade boot we compare the bl version, 
// if we upgrade the app we compare the app version)

bool PowerUpgrade::upgradeAllMcu(std::string & failed_mcu_list)
{
    //std::vector<UPGRADE_RESULT> upgrade_result_list;
    std::string ret_str;
    //int iRet;
    std::string upgraded_version;
    time_t upgrade_start_time = 0;
    time_t upgrade_finish_time = 0;
    int err_code;
    std::string err_msg;
    int i;
    bool bRet = true;
    bool bRet1 = true;
    UPGRADE_RESULT upgrade_result;

    if(is_upgrading_)
    {
        log_error("power upgrade is upgrading\r\n");
        return false;
    }

    log_info("%s upgrade start",__FUNCTION__);
    //upgrade_result_list.clear(); // clear the the upgrade result of last time , and set it to unknown
    num_board_upgraded_ = 0;
    //log_info("upgrade result list size = %d\r\n",upgrade_result_list.size());
    //error_list.clear();
    setUpgradeStatus(true); // output put status to file
    // check if we got enought config yaml files to read
    log_info("num of boards need to be upgraded = %d", num_board_upgrade_);
    // in case we cannot get enough yaml file
    if(num_board_upgrade_ != firmwarePreCheck(true,true))
    {
        log_error("total num of yaml file not equal to num of board upgrade");
        return false;
    }
    else
    {
        log_info("num of yaml file check ok");
    }

    // load yaml file
    ret_str = loadYaml();
    if(!compare(ret_str, "ok"))
    {
        log_error("%s load yaml file failed!!!",__FUNCTION__);
        return false;
    }
    else
    {
        log_info("%s load yaml file success...",__FUNCTION__);
    }
	
    // print the yaml file
    if(UPGRADE_PRODUCT_LIST_DEBUG)
    {
        log_info("print product list");
        print_product_list();
    }

    recordUpgradeStartTime();

    // iterate for each product in the product list, try upgrade
    std::vector<PRODUCT_UPGRADE_INFO>::iterator iter = product_list_.begin();

    for(;iter != product_list_.end();iter++)
    {
        upgrade_start_time = time(NULL); // record the upgrade start time
        // record upgrade start time
        log_info("start upgrade board name : %s\r\n",(*iter).product_name.c_str());

        //(*iter)->startUpgrade(err_code, err_msg);
        if((*iter).product_name == "monitor")
        {
            postMcuUpgradeStatusToWeb("sys_monitor_start", "success", 100);
            log_info("start upgrade monitor!!!\r\n");
            monitor_upgrade_.startUpgrade(err_code, err_msg);
        }
        else if((*iter).product_name == "chassis_controller")
        {
            log_info("start upgrade chassis controller!!!\r\n");
            postMcuUpgradeStatusToWeb("chassis_control_start", "success", 100);
            chassis_upgrade_.startUpgrade(err_code, err_msg);
		}
        else
        {
            log_error("unrecoginized board name : %s, continue upgrade next board", (*iter).product_name.c_str());
            continue;
        }

        if(err_code != 0)
        {
            // deal with error here
            upgrade_finish_time = time(NULL);// record the upgrade finish time
            log_error("%s upgrade %s fail , err code : %d err msg : %s", __FUNCTION__, (*iter).product_name.c_str(), err_code , err_msg.c_str());
            constructFailUpResult((*iter).product_name, err_code , err_msg, upgrade_start_time, upgrade_finish_time, upgrade_result);
            //enqueUpgradeResult(upgrade_result);
            bRet = false;
        }
        else
        {
            // deal with success here
            // version recheck after upgrade finish

            upgrade_finish_time = time(NULL);// record the upgrade finish time
            log_info("%s upgrade %s success , do the post software version check",__FUNCTION__, (*iter).product_name.c_str());
            bRet1 = resultRecheck((*iter).product_name, err_code , err_msg, upgrade_start_time, upgrade_finish_time, upgrade_result);
            if(!bRet1)
            {
                
                log_error("%s board %s software version recheck failed!!!",__FUNCTION__, (*iter).product_name.c_str());
                if((*iter).product_name == "monitor")
                {
                    postMcuUpgradeStatusToWeb("sys_monitor_end", "monitor_upgrade_failed", 100);
                }
                else if((*iter).product_name == "chassis_controller")
                {
                    postMcuUpgradeStatusToWeb("chassis_control_end", "chassis_upgrade_failed", 100);
                }
                else
                {
                    log_error("result recheck unrecoginized board name : %s", (*iter).product_name.c_str());
                }

                bRet = false;
            }
            else
            {
                //postMcuUpgradeStatusToWeb("chassis_control_end", "success", 100.0f);
                log_info("%s board %s software version recheck success...",__FUNCTION__, (*iter).product_name.c_str());
                if((*iter).product_name == "monitor")
                {
                    postMcuUpgradeStatusToWeb("sys_monitor_end", "monitor_upgrade_success", 100);
                }
                else if((*iter).product_name == "chassis_controller")
                {
                    postMcuUpgradeStatusToWeb("chassis_control_end", "chassis_upgrade_success", 100);
                }
                else
                {
                    log_error("result recheck unrecoginized board name : %s", (*iter).product_name.c_str());
                }
            }
            //enqueUpgradeResult(upgrade_result);
        }

        enqueUpgradeResult(upgrade_result);

        log_info("finish upgrade board name : %s\r\n",(*iter).product_name.c_str());

        num_board_upgraded_++;
        log_info("num board upgraded : %d , total num of board : %d\r\n",num_board_upgraded_, num_board_upgrade_);
    }
    
    //sendAllUpFinToSer();

    setUpgradeStatus(false); // output status to file
    // output the final upgrade result to file for debugging purpose
    outputUpgradeResult(failed_mcu_list);

    log_info("%s upgrade end",__FUNCTION__);
    
    setUpgradeManualCtrl(false);
    log_info("%s set upgrade manual control to false!!!",__FUNCTION__);
    return bRet;	
}

// send final upgrade finish msg to signaling server
void PowerUpgrade::sendAllUpFinToSer(void)
{

}

void PowerUpgrade::postMcuUpgradeStatusToWeb(const std::string & upgrade_status, const std::string & upgrade_result, int upgrade_progress)
{
    log_info("%s",__FUNCTION__);
    int64_t now = (int64_t)(tinyros::Time::now().toSec() * 1000);
    std::stringstream id; id << now;

    Json::FastWriter fw;
    Json::Value response;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    atris_msgs::SignalMessage param;
    param.account = shmrbt.robot.receiver;

    response["id"] = id.str();
    response["timestamp"] = now;
    response["status"] = upgrade_status;
    response["result"] = upgrade_result;
    response["progress"] = upgrade_progress;

    SWUpgrade::get_instance()->responseResult(param, response, "response_sw_upgrade");
}

// generate upgrade result and recheck if the upgrade is success


// output the upgrade result to file for debugging purpose
void PowerUpgrade::outputUpgradeResult(/*const std::vector<UPGRADE_RESULT> & up_res_vec*/std::string & failed_list)
{
    log_info("size of the upgrade result vec : %d", upgrad_result_list_.size());
    std::string file_path;
    int i = 0;
    std::string up_out_file_path_dir(UPGRADE_OUT_FILE_PATH);
    std::string up_result_file_name(UPGRADE_RESULT_FILE_NAME);
    file_path = up_out_file_path_dir + up_result_file_name;

    std::ofstream fout(file_path.c_str());
    if(!fout)
    {
        log_error("error write upgrade status");
        return;
    }

    // start output
    if(fout.is_open())
    {
        std::vector<UPGRADE_RESULT>::iterator iter = upgrad_result_list_.begin();
        fout << "upgrade result:" << std::endl;
        fout << "upgrade start time: " << upgrade_start_time_;
        fout << std::endl;
        for(;iter != upgrad_result_list_.end();iter++)
        {
            fout << "product name: " << (*iter).product_name << std::endl;
            fout << "target part: " << (*iter).target_part << std::endl;
            fout << "software version: " << (*iter).soft_version << std::endl;
            fout << "error code: " << (*iter).err_code << std::endl;
            fout << "error message: " << (*iter).error_string << std::endl;
            //fout << "upgrade time start: " << (*iter).time_start << std::endl;
            //fout << "upgrade time finish: " << (*iter).time_finish << std::endl;
            fout << "upgrade time start: " << getLocalTimeStr((*iter).time_start) << std::endl;
            fout << "upgrade time finish: " << getLocalTimeStr((*iter).time_finish) << std::endl;
            fout << "upgrade time consume: " << (*iter).time_consume << std::endl;

            if((*iter).status == true)
            {
                fout << "upgrade result: " << "success" << std::endl;
            }
            else
            {
                fout << "upgrade result: " << "fail" << std::endl;
                failed_list += (*iter).product_name + " ";
            }

            if(i != num_board_upgrade_-1)
            {
                fout << "------------------------------" << std::endl;
            }

            i++;
        }
    }
    else
    {
        log_error("open upgrade output result file failed!!!\r\n");
    }

    // end output
    fout.close();
    system("sync");
}

// try to find each board's yaml file
// check if the yaml config file exist
// return the total num of yaml files
int PowerUpgrade::firmwarePreCheck(bool monitor_check, bool chassis_controller_check)
{
    int num_of_board = 0;
    //char file_path[MAX_PATH_LENGTH] = {0};
    std::string chassis_controller_yaml_file_path(CHASSIS_YAML_FILE_PATH);
    std::string monitor_yaml_file_path(MONITOR_YAML_FILE_PATH);

    PRODUCT_YAML_PATH product_yaml_str;


    if(monitor_check)
    {
        if(access(monitor_yaml_file_path.c_str(), F_OK) == 0)
        {
            log_info("monitor yaml file exist");
            num_of_board++;
            product_yaml_str.product_yaml_config_path = monitor_yaml_file_path;
            product_yaml_str.product_name = "monitor";
            yaml_config_file_list_.push_back(product_yaml_str);
        }
        else
        {
            log_error("monitor yaml config does not exist");
        }
    }

    if(chassis_controller_check)
    {
        if(access(chassis_controller_yaml_file_path.c_str(), F_OK) == 0)
        {
            log_info("chassis_controller yaml file exist");
            num_of_board++;
            product_yaml_str.product_yaml_config_path = chassis_controller_yaml_file_path;
            product_yaml_str.product_name = "chassis_controller";
            yaml_config_file_list_.push_back(product_yaml_str);
        }
        else
        {
            log_error("chassis_controller yaml config does not exist");
        }
    }
	
	
    return num_of_board;
}



// create multiple level directory function
bool PowerUpgrade::createMultiDir(const char *path)
{
    int i, len;
    char dir_path[256] = {0};
    len = strlen(path);
    dir_path[len] = '\0';
    strncpy(dir_path, path, len);
    log_info("%s directory path : %s",__FUNCTION__, path);

    for (i=0; i<len; i++)
    {
        if (dir_path[i] == '/' && i > 0)
        {
            dir_path[i]='\0';
            if (access(dir_path, F_OK) < 0)
            {
                if (mkdir(dir_path, 0777) < 0)
                {
                    log_error("mkdir=%s:msg=%s\n", dir_path, strerror(errno));
                    return false;
                }
                else
                {
                    log_info("make dir success , dir path : %s\r\n",dir_path);
                    return true;
                }
            }
            dir_path[i]='/';
        }
    }

    return true;
}

// show product list array
// for debugging purpose print the config file info
void PowerUpgrade::print_product_list(void)
{
    std::vector<PRODUCT_UPGRADE_INFO>::iterator iter = product_list_.begin();
    int i = 0;
    int output_to_terminal = 1;

    for(;iter != product_list_.end();iter++)
    {
        if(output_to_terminal)
        {
            std::cout << "product number : " << i << std::endl;
            std::cout << "product name : " << (*iter).product_name << std::endl;
            std::cout << "target part : " << (*iter).target_part << std::endl;
            std::cout << "source part : " << (*iter).source_part << std::endl;
            std::cout << "app version : " << (*iter).app_version << std::endl;
            std::cout << "boot version : " << (*iter).boot_version << std::endl;
            std::cout << "app md5 : " << (*iter).app_md5_str << std::endl;
            std::cout << "boot md5 : " << (*iter).bl_md5_str << std::endl;
            std::cout << "--------------------------------------------------" << std::endl;
            std::cout << std::endl << std::endl;
        }
        else
        {
            log_info("product number : %d \r\n",i);
            log_info("product name : %s\r\n",(*iter).product_name.c_str());
            log_info("target part : %s\r\n",(*iter).target_part.c_str());
            log_info("source part : %s\r\n",(*iter).source_part.c_str());
            log_info("app version : %s\r\n",(*iter).app_version.c_str());
            log_info("boot version : %s\r\n",(*iter).boot_version.c_str());
            log_info("app md5 : %s\r\n",(*iter).app_md5_str.c_str());
            log_info("boot md5 : %s\r\n",(*iter).bl_md5_str.c_str());
			
            log_info("--------------------------------------------------\r\n");
            log_info("\r\n\r\n");
        }
        i++;
    }
	
    //std::cout << std::endl;

    log_info("total num of yaml config file = %d\r\n", i);
}

int PowerUpgrade::strFind(char * src , char  * dest)
{
    int len = 0;
    if(src == NULL || dest == NULL)
    {
        return -1;
    }

    while(*src != ' ' && *src != '\r' && *src != '\n')
    {
        *dest++ = *src++;
        len++;
    }

    return len;
}

bool PowerUpgrade::parse_yaml_str(PRODUCT_UPGRADE_INFO & product, const std::string yaml_str)
{
    char * ptr_b = NULL;
    //const char * ptr_s = NULL;
    char str_buf[512] = {0};
    char DataBuffer[50] = {0};
    char tempBuf[50] = {0};
	
    std::string product_name;
    std::string target_part;
    std::string source_part;
    std::string app_version;
    std::string boot_version;
    std::string app_md5_str;
    std::string bl_md5_str;
    int len;
	
    strcpy(str_buf, yaml_str.c_str());
    str_buf[yaml_str.length()] = '\0';
    printf("yaml string = %s",str_buf);
	// get product name
	
    ptr_b = strstr(str_buf, "product:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("product:");
        //printf("product len = %d\r\n",len - strlen("product:"));
        memcpy(DataBuffer, ptr_b, len - strlen("product:"));
        DataBuffer[len - strlen("product:")] = '\0';
        product_name = DataBuffer;
        product.product_name = product_name;
        std::cout << "product name : " << product.product_name << std::endl;
    }
    else
    {
        log_error("cannot find product key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));

    // get target part
    ptr_b = strstr(str_buf, "target_part:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("target_part:");
        //printf("target part len = %d\r\n",len - strlen("target_part:"));
        memcpy(DataBuffer, ptr_b, len - strlen("target_part:"));
        DataBuffer[len - strlen("target_part:")] = '\0';
        target_part = DataBuffer;
        product.target_part = target_part;
        std::cout << "target part : " << product.target_part << std::endl;
    }
    else
    {
        log_error("cannot find target part key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));
    // get source part
    ptr_b = strstr(str_buf, "source_part:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("source_part:");
        //printf("source part len = %d\r\n",len - strlen("source_part:"));
        memcpy(DataBuffer, ptr_b, len - strlen("source_part:"));
        DataBuffer[len - strlen("source_part:")] = '\0';
        source_part = DataBuffer;
        product.source_part = source_part;
        std::cout << "source part : " << product.source_part << std::endl;
    }
    else
    {
        log_error("cannot find source part key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));

    // get app version

    ptr_b = strstr(str_buf, "app_version:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("app_version:");
        memcpy(DataBuffer, ptr_b, len - strlen("app_version:"));
        DataBuffer[len - strlen("app_version:")] = '\0';
        app_version = DataBuffer;
        product.app_version = app_version;
        std::cout << "app version : " << product.app_version << std::endl;
    }
    else
    {
        log_error("cannot find app version key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));
    // get boot version
    ptr_b = strstr(str_buf, "bl_version:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("bl_version:");
        memcpy(DataBuffer, ptr_b, len - strlen("bl_version:"));
        DataBuffer[len - strlen("bl_version:")] = '\0';
        boot_version = DataBuffer;
        product.boot_version = boot_version;
        std::cout << "boot version : " << product.boot_version << std::endl;
    }
    else
    {
        log_error("cannot find boot version key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));
    // get app md5 string
    ptr_b = strstr(str_buf, "app_md5:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("app_md5:");
        memcpy(DataBuffer, ptr_b, len - strlen("app_md5:"));
        DataBuffer[len - strlen("app_md5:")] = '\0';
        app_md5_str = DataBuffer;
        product.app_md5_str = app_md5_str;
        std::cout << "app md5 : " << product.app_md5_str << std::endl;
    }
    else
    {
        log_error("cannot find app md5 key in yaml string");
        return false;
    }

    memset(DataBuffer, 0, sizeof(DataBuffer));
    memset(tempBuf, 0, sizeof(tempBuf));
    // get boot md5 string
    ptr_b = strstr(str_buf, "bl_md5:");
    if (ptr_b)
    {
        len = strFind(ptr_b, tempBuf);
        //printf("len = %d\r\n",len);
        ptr_b += strlen("bl_md5:");
        memcpy(DataBuffer, ptr_b, len - strlen("bl_md5:"));
        DataBuffer[len - strlen("bl_md5:")] = '\0';
        bl_md5_str = DataBuffer;
        product.bl_md5_str = bl_md5_str;
        std::cout << "boot md5 : " << product.bl_md5_str << std::endl;
    }
    else
    {
        log_error("cannot find boot md5 key in yaml string");
        return false;
    }

    // default set forceUp to false
    product.forceUp = false;
    // upgraded is initialized to true
    product.upgraded = true;

    return true;
}


// load each yaml file into product struct
std::string PowerUpgrade::loadYaml(void)
{
    product_list_.clear();
    int size = 0;
	
    log_info("yaml config file list = %d\r\n",yaml_config_file_list_.size());
    BOOST_FOREACH (PRODUCT_YAML_PATH path, yaml_config_file_list_)
    {
        log_info("product name : %s , try to read yaml file path %s",path.product_name.c_str() , path.product_yaml_config_path.c_str());
        std::ifstream fin(path.product_yaml_config_path.c_str());
        std::string yaml_str;
        bool bRet;

        if(!fin)continue;
        YAML::Node config;
        config = YAML::Load(fin);

        PRODUCT_UPGRADE_INFO product;
        // yaml file strange , config file got only 1 element in the yaml array
        if(config)
        {
            if(config[0])
            {
                (config[0]) >> yaml_str;
                // parse the yaml string
                bRet = parse_yaml_str(product, yaml_str);
                if(bRet)
                {
                    // push product upgrade info into list
                    product_list_.push_back(product);
                }
                else
                {
                    log_error("parse yaml string failed");
                    return "error";
                }
            }
        }
        else
        {
            log_error("read yaml config error");
            return "error";
        }

        if(fin.is_open())
        {
            fin.close();
        }
    }

    //log_info("total config file num :%d\r\n", size);
    is_cfg_file_ok_ = true;

    return "ok";
}

// tool function , compare two string
bool PowerUpgrade::compare(std::string command,std::string cmd_name)
{
    if(command.size() < cmd_name.size()) return false;
    std::string tem = command.substr(0,cmd_name.size());
    std::transform(tem.begin(),tem.end(),tem.begin(),::toupper);
    std::transform(cmd_name.begin(),cmd_name.end(),cmd_name.begin(),::toupper);
    if(tem == cmd_name) return true;
    return false;
}

// set upgrade status
void PowerUpgrade::setUpgradeStatus(bool status)
{
    std::string file_path;
    std::string up_out_dir_path(UPGRADE_OUT_FILE_PATH);
    std::string up_ind_file_name(UPGRADE_INDICATOR_FILE_NAME);

    is_upgrading_ = status;// set power upgrade upgrading status
    file_path = up_out_dir_path + up_ind_file_name;

    std::ofstream fout(file_path.c_str());
    if(!fout)
    {
        log_error("error write upgrade status");
        return;
    }

    if(fout.is_open())
    {
        if(true == status)
        {
            log_info("setting upgrade status to 1");
            fout << "1";
        }
        else
        {
            log_info("setting upgrade status to 0");
            fout << "0";
        }
    }
    else
    {
        log_error("file is not opened!!!");
    }

    fout.close();
    system("sync");

}

void PowerUpgrade::recordUpgradeStartTime(void)
{
    time_t timep;
    time(&timep);
    struct tm * p;
    char cbuf[128] = {0};

    p = localtime(&timep);
    sprintf(cbuf, "%d/%d/%d %02d:%02d:%02d", 1900 + p->tm_year, 1+ p->tm_mon, p->tm_mday,p->tm_hour, p->tm_min, p->tm_sec);
    upgrade_start_time_ = cbuf;
    log_info("%s upgrade start date string: %s", __FUNCTION__, upgrade_start_time_.c_str());
}

std::string PowerUpgrade::getLocalTimeStr(time_t time_stamp)
{
    struct tm * p;
    char cbuf[128] = {0};
    std::string time_str;

    p = localtime(&time_stamp);
    sprintf(cbuf, "%d/%d/%d %02d:%02d:%02d", 1900 + p->tm_year, 1+ p->tm_mon, p->tm_mday,p->tm_hour, p->tm_min, p->tm_sec);
    time_str = cbuf;
    log_info("%s time point: %s", __FUNCTION__, time_str.c_str());
    return time_str;
}






