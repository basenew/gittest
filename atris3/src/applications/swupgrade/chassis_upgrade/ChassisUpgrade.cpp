#include "include/ChassisUpgrade.h"
#include "include/ChassisUpgradeErrorCode.h"
//#include "config/config.h"
#define TEST_UPGRADE_URL "http://10.20.18.2/hfs"
#define PRODUCT_NAME "chassis_controller"
#define CHASSIS_IP "10.20.18.10"

ChassisUpgrade::ChassisUpgrade(std::string prod_name)
	: chassis_upgrade_resp_sub_(TOPIC_CHASSIS_UPGRADE_RESP, &ChassisUpgrade::on_recv_chassis_up_resp, this)
	, chassis_upgrade_req_pub_(TOPIC_CHASSIS_UPGRADE_REQ, new tinyros::atris_msgs::CanPkg())
	, is_upgrade_finish_(false) // in upgrade or not true is active
	, target_version_("") // target version number
	, current_version_("") // current version number
	, is_read_ver_success_(true)
	, last_upgrade_flag_(0)
	, time_out_sec_(60) // config file to config this parameter TODO::
	, upgrade_percent_(0)
	, upgrade_finish_code_(-1)
	, product_name_(prod_name)
	, time_out_cnt_(0)
{
    log_info("ChassisUpgrade::ChassisUpgrade() called...\r\n");
    tinyros::nh()->advertise(chassis_upgrade_req_pub_);
    tinyros::nh()->subscribe(chassis_upgrade_resp_sub_);
    utils = Utils::get_instance();
}

ChassisUpgrade::~ChassisUpgrade()
{
    log_info("ChassisUpgrade::~ChassisUpgrade() called...\r\n");
}

// start upgrade mcu
bool ChassisUpgrade::startUpgrade(int & error_code, std::string & error_str)
{
    bool bRet;
    log_info("start of %s\r\n",__FUNCTION__);
    init();
    setTimeOutCount(0);

    bRet = checkChassisNetwork();
    if(!bRet)
    {
        log_error("%s check chassis network failed!!!",__FUNCTION__);
    }

    testUpgrade();

    waitUpgradeFinish(error_code, error_str);

    log_info("end of %s\r\n",__FUNCTION__);
    return true;

}

// test_function
void ChassisUpgrade::testUpgrade()
{
    std::string test_upgrade_url(TEST_UPGRADE_URL);
    sendUpgradeStart(test_upgrade_url);
}

// init function for upgrade
void ChassisUpgrade::init()
{
    bool bRet;
    log_info("chassis upgrade init");

}

// query mcu software info, periodically query mcu software version info
// also external function
bool ChassisUpgrade::queryVersionInfo(std::string & current_soft_ver, std::string & target_soft_ver)
{
    return readChassisVer(current_soft_ver, target_soft_ver);
}

// read current chassis board version (3 times retry)
// true if success, else false
// output is set to chassis ver out string
bool ChassisUpgrade::readChassisVer(std::string & current_soft_ver, std::string & target_soft_ver)
{
    int try_count = 3;
    log_info("%s -> read chassis software version\r\n",__FUNCTION__);
    boost::unique_lock<boost::mutex> lock(mutex_);

    if(is_read_ver_success_ == false) // other is reading chassis version now, just return false
    {
        current_soft_ver = "";
        target_soft_ver = "";
        return false;
    }

    if(is_read_ver_success_ == true)
    {
        is_read_ver_success_ = false;
        setCurChassisVer(""); // clear current software version string
        setTarChassisVer(""); // clear the target software version string
    }
	
    while(!is_read_ver_success_ && try_count != 0)
    {
        sendReadVersionPkt();

        // 3 seconds read version timeout
        if(recv_ver_cond_.timed_wait(lock, boost::get_system_time() + boost::posix_time::seconds(3))) 
        {
            if(is_read_ver_success_)
            {
                current_soft_ver = getCurChassisVer();
                target_soft_ver = getTarChassisVer();
                log_info("%s read chassis version info success... current chassis version = %s, target chassis version = %s\r\n", __FUNCTION__, current_soft_ver.c_str(), target_soft_ver.c_str());
                return true;
            }
            else
            {
                current_soft_ver = "";
                target_soft_ver = "";
                log_info("%s read chassis version info failed!!!\r\n",__FUNCTION__);
                return false;
            }
			
        }
        else
        {
            log_error("read chassis version timeout!!! try read again , try count = %d\r\n",try_count);
        }

        try_count--;
        //usleep(1000*1000);
    }

    if(try_count == 0)
    {
        log_error("read chassis software version failed after 3 times retry !!!");
        current_soft_ver = "";
        target_soft_ver = "";
        return false;
    }

    return true;
}


// get mcu upgrade response
int ChassisUpgrade::get_upgrade_resp_info(const tinyros::atris_msgs::CanPkg &msg, std::string & current_version , std::string & target_version)
{
    log_info("%s called!!!\r\n",__FUNCTION__);
    int iRet;
    uint32_t sec = ros::Time::now().sec;
    int32_t upgrade_flag;
    std::string err_str;
    time_t upgrade_time_stamp;
    int upgrade_percent;
    //UP_RESULT upgrade_info;

    upgrade_flag = msg.data_i[0]; // member variable to store the upgrade error code
    upgrade_percent = msg.data_i[1]; // upgrade percentage

    if(upgrade_percent > 100 || upgrade_percent < 0 || upgrade_flag < 0 || upgrade_flag > 0xFF)
    {
        log_error("%s upgrade chassis info invalid!!!!!!!!!!!!!!!",__FUNCTION__);
        return -1;
    }


    // check error code valid
    iRet = isUpgradeErrCodeValid(upgrade_flag);
    if(iRet < 0)
    {
        log_error("%s upgrade error code error , mcu upgrade failed!!!",__FUNCTION__);
        return -2;
    }
	
    log_info("%s chassis is upgrading(upgrade flag = %02x , upgrade percentage = %d %%)",__FUNCTION__, upgrade_flag, upgrade_percent);
	
    if(!isChassisUpgradeFinish())
    {
        setTimeOutCount(0); // clear time out counter

        // set upgrade percentage
        setPercent(upgrade_percent); // set percentage

        // judge the start and end of upgrade

        if(upgrade_flag == 0xFF && getLastUpgradeFlag() == 0)
        {
            // indicate upgrade start
            log_info("upgrade really start &&&&\r\n");

        }
        //else if((upgrade_flag == 0 && getLastUpgradeFlag() == 0xFF) || (upgrade_flag == 0 && getLastUpgradeFlag() == 0))
        else if(upgrade_flag == 0 && getLastUpgradeFlag() == 0xFF)
        {
            // indicate upgrade finish
            log_info("upgrade really finish &&&&\r\n");

            setFinishCode(upgrade_flag); // set upgrade error code to success
            setErrString(get_chassis_upgrade_err_msg(upgrade_flag));
            setChassisUpgradeFinish(true);

        }
        else
        {
            log_info("%s other case upgrade in progress , mcu upgrade error code = %02x, last upgrade error code = %02x\r\n", __FUNCTION__, upgrade_flag, last_upgrade_flag_);
        }

        setLastUpgradeFlag(upgrade_flag);
    }

    //mcu_version parse
    std::string::size_type pos = msg.data_s.find("&");
    if(pos != std::string::npos)
    {
        current_version = msg.data_s.substr(0, pos);
        target_version = msg.data_s.substr(pos+1);
        //log_info("current version str size = %d, target version str size = %d\r\n",current_version_.size(), target_version_.size());
        log_info("version string ok , pos = %d, current soft version = %s , target version = %s", pos, current_version.c_str(), target_version.c_str());
    }
    else
    {
        log_error("can not find & key word in soft version string!!!");
        current_version = "";
        target_version = "";
    }

    return 0;
}

// notify read mcu response
void ChassisUpgrade::notifyReadMcuResp(const std::string & cur_software_version, const std::string & tar_software_version)
{
    log_info("%s notifyReadMcuResp called ---------------------",__FUNCTION__);
    boost::unique_lock<boost::mutex> lock(mutex_);

    if(!is_read_ver_success_)
    {
        log_info("%s read soft version success\r\n", __FUNCTION__);
        is_read_ver_success_ = true;
        if(!cur_software_version.empty() && !tar_software_version.empty())
        {
            setCurChassisVer(cur_software_version);
            setTarChassisVer(tar_software_version);
        }

        recv_ver_cond_.notify_all();
    }
}

// on read chassis upgrade response package
void ChassisUpgrade::on_recv_chassis_up_resp(const tinyros::atris_msgs::CanPkg &msg)
{
    int iRet;
    std::string current_soft_ver = "";
    std::string target_soft_ver = "";

    //log_info("%s recv mcu upgrade response",__FUNCTION__);
    // only receive 0x02 cmd response
    if(msg.cmd != 0x02)
    {
        //log_error("chassis response cmd invalid\r\n");
        return;
    }

    iRet = get_upgrade_resp_info(msg, current_soft_ver, target_soft_ver);
    if(iRet < 0)
    {
        log_error("%s get upgrade response info failed, upgrade failed(%d)\r\n",__FUNCTION__, iRet);
        if(iRet!=0)
        {
            log_info("only not return if we get valid error code");
            return;
        }
    }

    // if read mcu version flag is set, set it to current version and signal it

    notifyReadMcuResp(current_soft_ver, target_soft_ver);

}

// send upgrade download request
int ChassisUpgrade::sendUpgradeStart(const std::string & upgrade_url)
{
    log_info("%s -> sendUpgradeStart, upgrade url = %s\r\n",__FUNCTION__, upgrade_url.c_str());

    setChassisUpgradeFinish(false);
    setLastUpgradeFlag(0);
    setPercent(0);
	
    tinyros::atris_msgs::CanPkg upgrade_msg;
    upgrade_msg.cmd = 0x01;
    upgrade_msg.data_s = upgrade_url;
    chassis_upgrade_req_pub_.publish(&upgrade_msg);
	
    return 0;
}

// send 0x03 cmd to read current software version
int ChassisUpgrade::sendReadVersionPkt()
{
    log_info("%s chassis upgrade -> sendReadVersionPkt",__FUNCTION__);
    tinyros::atris_msgs::CanPkg ver_read_msg;
    ver_read_msg.cmd = 0x03;
    chassis_upgrade_req_pub_.publish(&ver_read_msg);
	return 0;
}


// check if the upgrade error code is valid, if ok return 0, else return -1
int ChassisUpgrade::isUpgradeErrCodeValid(int32_t up_err_code)
{
    std::string up_err_str;
    time_t upgrade_time_stamp;
    //UP_RESULT upgrade_info;

    if(up_err_code == 0 || up_err_code == 0xFF)
    {
        // upgrade finish or status unknown , otherwise the uggrade is failed
        log_info("%s upgrade error code ok...", __FUNCTION__);
    }
    else
    {
        setFinishCode(up_err_code);
        setErrString(get_chassis_upgrade_err_msg(up_err_code));
        setChassisUpgradeFinish(true); // upgrade forced to finish because of wrong upgrade error code

        log_error("%s error code = %d , MCU UPGRADE FAILED ERROR STRING = %s",__FUNCTION__, up_err_code , get_chassis_upgrade_err_msg(up_err_code));

        return -1;
    }

    return 0;
}

// wait 60 secs to wait for mcu upgrade finish
void ChassisUpgrade::waitUpgradeFinish(int32_t & error_code, std::string & error_str)
{
    int count = 0;
    int nmax_count = time_out_sec_;
    bool retFlag = false;
	
    //std::string error_str;
    time_t upgrade_time_stamp;

    log_warn("%s wait chassis upgrade finish start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n",__FUNCTION__);

    boost::unique_lock<boost::mutex> check_upgrade_lock(upgrade_active_mutex_, boost::defer_lock);

    while(getTimeOutCount() < nmax_count)
    {
        check_upgrade_lock.lock();
        if(is_upgrade_finish_)
        {
            check_upgrade_lock.unlock();
            break;
        }

        upgrade_active_cond_.timed_wait(check_upgrade_lock, boost::get_system_time() + boost::posix_time::milliseconds(1000*1));
        if(is_upgrade_finish_)
        {
            check_upgrade_lock.unlock();
            break;
        }

        count = getTimeOutCount();
        count++;
        setTimeOutCount(count);

        check_upgrade_lock.unlock();

        log_info("%s check chassis upgrade wait , count: %d",__FUNCTION__, count);
    }

    // upgrade time out
    if(getTimeOutCount() == nmax_count)
    {
        error_code = CHASSIS_UPGRADE_TIMEOUT;
        error_str = get_chassis_upgrade_err_msg(CHASSIS_UPGRADE_TIMEOUT);
    	
        log_info("%s upgrade time out post CHASSIS UPGRADE TIMEOUT\r\n",__FUNCTION__);
        //UP_RESULT upgrade_info;
        check_upgrade_lock.lock();
        is_upgrade_finish_ = true;

        check_upgrade_lock.unlock();
        return;
    }

    error_code = getFinishCode();
    error_str = getErrString();

    log_warn("%s wait chassis upgrade finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n",__FUNCTION__);

    return;
}

void ChassisUpgrade::setChassisUpgradeFinish(bool is_active)
{
    boost::unique_lock<boost::mutex> lock(upgrade_active_mutex_); 
    is_upgrade_finish_ = is_active;
    if(is_upgrade_finish_ == true)
    {
        log_info("%s notify upgrade finish\r\n",__FUNCTION__);
        upgrade_active_cond_.notify_all();
    }
};


// true finish false not finish
bool ChassisUpgrade::isChassisUpgradeFinish()
{
    boost::unique_lock<boost::mutex> lock(upgrade_active_mutex_);
    return is_upgrade_finish_;
};

bool ChassisUpgrade::checkChassisNetwork(void)
{
    const char * chassis_ip = CHASSIS_IP;
    int times = 3;
    int count = 0;
    bool bRet = false;
    while (count < times)
    {
        if (!utils->check_network_state(chassis_ip))
        {
            log_error("%s chassis network unreachable",__FUNCTION__);
            count++;
        }
        else
        {
            log_info("%s chassis network ok", __FUNCTION__);
            bRet = true;
            break;
        }

        usleep(1000*1000);

    }

    if(count == times)
    {
        log_error("%s ping chassis reach maximum times : %d", __FUNCTION__, count);
    }

    return bRet;
}