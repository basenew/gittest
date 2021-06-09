
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
//#include "sdk.h"
#include "dbcom_signal.h"
#include <thread>
#include <mutex>

DbComManager::DbComManager()
  : device_ip("")
  , username("")
  , passwd("")
  , is_connected_(false)
  , is_login_(false)
  , need_reconnect_(true)
  , login_timeout_(90)
  , conn(NULL)
  , get_signal_strength_failed_cnt_(0)
  , dbcom_try_login_thread_exit_(false)
{
	int iRet;
	diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);

	cfg = Config::get_instance();
    utils = Utils::get_instance();

	iRet = Init();
	if(iRet < 0)
	{
		log_info("%s init failed!!!\r\n",__FUNCTION__);
	}

	is_connected_ = false;
    if(connect_thread != NULL){
        log_info("delete connect thread before.");
        sleep(2);
        delete connect_thread;
        connect_thread = NULL;
    }

    if(get_sig_thread != NULL)
    {
	    log_info("delete get sig thread");
	    sleep(2);
	    delete get_sig_thread;
	    get_sig_thread = NULL;
    }

    connect_thread = new boost::thread(boost::bind(&DbComManager::dbcom_login, this));
    get_sig_thread = new boost::thread(boost::bind(&DbComManager::dbcom_read_signal, this));
}


DbComManager::~DbComManager()
{
    sdk_disconnect(conn);
}

int DbComManager::Init()
{
    device_ip = cfg->dbcom_device_ip;
    username = cfg->dbcom_username;
    passwd = cfg->dbcom_passwd;
    login_timeout_ = cfg->dbcom_login_timeout;

    log_info("%s -----------------------------------------------------------",__FUNCTION__);
    log_info("%s device_ip = %s , username = %s , passwd = %s , login_timeout = %d",__FUNCTION__, device_ip.c_str(), username.c_str(), passwd.c_str(), login_timeout_);
    log_info("%s -----------------------------------------------------------",__FUNCTION__);

    return 0;
}

int DbComManager::dbcom_login()
{
    //struct sdk_connection *conn;
    //char dbcom_ip[50] = {0};
    int login_timeout = login_timeout_;
    bool login_status;
    bool connect_status;

    log_debug("%s:ip:%s,username:%s,passwd:%s,login timeout:%d", __FUNCTION__, device_ip.c_str(), username.c_str(), passwd.c_str(),login_timeout);
    while (!dbcom_try_login_thread_exit_)
    {
        //log_debug("%s dbcom try login thread , login_timeout : %d",__FUNCTION__, login_timeout);
        if(!need_reconnect_)
        {
            //log_info("%s dbcom do not need to reconnect, continue next loop",__FUNCTION__);
            sleep(5);
            continue;
        }

        if(login_timeout == 0)
        {
            dbcom_try_login_wait(); // we will wait a period of time to wait to try login again
            login_timeout = login_timeout_;
        }

        if (!utils->check_network_state(device_ip.c_str()))
        {
            log_error("\033[1;31mdbcom network unreachable.%s\033[0m",device_ip.c_str());
            sleep(2);
            login_timeout--;
            continue;
        }
        else
	    {
	        log_info("%s ping double com ip ok!!!\r\n",__FUNCTION__);
	    }
 
        conn = sdk_connect((char *)device_ip.c_str(), SDK_PORT); 
	    if (conn == NULL) 
	    {
		    log_info("%s connect to double com failed device ip = %s",__FUNCTION__, device_ip.c_str());
            is_connected_ = false;
            //sendDbComConnectStatus(false);
		    fprintf(stderr, "Error connecting to %s: %s\n", device_ip.c_str(), strerror(errno));
            login_timeout--;
            sleep(5);
            continue;
        }
        else
        {
            log_info("\033[1;32mconnect to doublecom successfully:login:%d", is_connected_);
            //sendDbComConnectStatus(true);
            is_connected_ = true;
        }

        if(is_connected_)
        {
        	if (sdk_login(conn, (char *)username.c_str(), (char *)passwd.c_str()))
        	{
        		log_info("&&&&&& %s dbcom login successfully, set login status to true &&&&&&", __FUNCTION__);
        		is_login_ = true;
                //sendDbComLoginStatus(true);
                need_reconnect_ = false;
        	}
        	else
        	{
                log_error("%s dbcom sdk login failed!!!", __FUNCTION__);
                is_login_ = false;
                login_timeout--;
                sleep(5);
                continue;
        	}
        }
        else
        {
        	// usually cannot reach here, dbcom not connected successfully
            log_error("%s dbcom not connected!!!",__FUNCTION__);
        }

        sleep(2);
    }

    return 0;
}

int DbComManager::dbcom_read_signal()
{
    struct sdk_result *res;
    char * signal_strength_str = NULL;
    std::string signal_str = "";
    int iRet;

    while(1)
    {
    	if(is_login_ != true)
    	{
    		log_error("%s not ok to read signal from dbcom since not login",__FUNCTION__);
    		usleep(5000*1000);
    		continue;
    	}

		res=sdk_send_command_wait(conn,(char *)"/interface/wireless/print",(char *)"=.proplist=.id,name,mac-address,disabled,ssid,frequency,band,channel-width,default-authentication,security-profile",NULL);
		while(res&&res->re){
			//log_info("%20s %20s %20s %20s %20s %20s %20s %20s %20s %20s\n",sdk_get(res,(char *)"=.id"),sdk_get(res,(char *)"=name"),sdk_get(res,(char *)"=mac-address"),sdk_get(res,(char *)"=disabled"),sdk_get(res,(char *)"=ssid"),sdk_get(res,(char *)"=frequency"),sdk_get(res,(char *)"=band"),sdk_get(res,(char *)"=channel-width"),sdk_get(res,(char *)"=default-authentication"),sdk_get(res,(char *)"=security-profile"));//.id 为无线接口id，配置的时候需要
			sdk_result_free(res);
			res=sdk_read_packet(conn);
		}
		
		sdk_result_free(res);

		res=sdk_send_command_wait(conn,(char *)"/interface/wireless/registration-table/print",(char *)"=.proplist=mac-address,rx-ccq,tx-ccq,tx-signal-strength,signal-strength,interface,uptime",NULL);
        if(res == NULL)
        {
            log_info("%s get registable print failed!!! get_signal_strength_failed_cnt_ : %d",__FUNCTION__, get_signal_strength_failed_cnt_);
            get_signal_strength_failed_cnt_++;
            if(get_signal_strength_failed_cnt_ == GET_SIG_FAIL_MAXIMUM)
            {
                need_reconnect_ = true;
            }
        }
        else
        {
		    while(res&&res->re)
            {
			    //log_info(" %20s %20s %20s %20s %20s %20s %20s\n",sdk_get(res,(char *)"=mac-address"),sdk_get(res,(char *)"=rx-ccq"),sdk_get(res,(char *)"=tx-ccq"),sdk_get(res,(char *)"=tx-signal-strength"),sdk_get(res,(char *)"=signal-strength"),sdk_get(res,(char *)"=interface"),sdk_get(res,(char *)"=uptime"));
                signal_strength_str = sdk_get(res,(char *)"=signal-strength");
                if(signal_strength_str == NULL)
                {
                    log_error("%s get doublecom signal strength string failed",__FUNCTION__);
                    iRet = sendDbComSignalStrength("");
                    if(iRet < 0)
                    {
                        log_error("%s send doublecom abnormal signal strength failed",__FUNCTION__);
                        usleep(5000*1000);
                        continue;
                    }
                    else
                    {
                        //log_info("%s send doublecom abnormal signal strength ok",__FUNCTION__);
                    }

                    usleep(5000*1000);
                    continue;
                }
                else
                {
                    signal_str = signal_strength_str;
                    iRet = sendDbComSignalStrength(signal_str);
                    if(iRet < 0)
                    {
                        log_error("%s send doublecom signal strength failed",__FUNCTION__);
                        usleep(5000*1000);
                        continue;
                    }
                    else
                    {
                        //log_info("%s send doublecom signal strength ok",__FUNCTION__);
                    }

                    get_signal_strength_failed_cnt_ = 0;
                }

			    sdk_result_free(res);
			    res=sdk_read_packet(conn);
		    }

		    sdk_result_free(res);
        }

		usleep(5000*1000);
	}

	return 0;
}
#if 0
void DbComManager::sendDbComConnectStatus(bool connect_status)
{
    Json::Value root;
    atris_msgs::RobotInfo doublecom_connect_status;  
    Json::FastWriter fw;
    if(connect_status != is_connected_)
    {
        log_info("%s connect status : %d, publish to diag", __FUNCTION__, connect_status?1:0);
        root["robot_info"]["doublecom"]["connect_status"] = connect_status?1:0;
        doublecom_connect_status.json = fw.write(root);
        diag_info_pub_.publish(doublecom_connect_status);
    }

    is_connected_ = connect_status;
}

void DbComManager::sendDbComLoginStatus(bool login_status)
{
    Json::Value root;
    atris_msgs::RobotInfo doublecom_login_status;  
    Json::FastWriter fw;
    if(login_status != is_login_)
    {
        log_info("%s login status : %d, publish to diag", __FUNCTION__, login_status?1:0);
        root["robot_info"]["doublecom"]["login_status"] = login_status?1:0;
        doublecom_login_status.json = fw.write(root);
        diag_info_pub_.publish(doublecom_login_status);
    }

    is_login_ = login_status;
}
#endif

int DbComManager::sendDbComSignalStrength(const std::string & signal_strength_str)
{
    std::string::size_type position;
    std::string signal_num = "";
    int signal_level;
    char chSignalStrength[20] = {0};

    int level;
    if(signal_strength_str.empty())
    {
        level = 0;
    }
    else
    {
        //log_info("%s signal string : %s ",__FUNCTION__, signal_strength_str.c_str());
        position = signal_strength_str.find("@");
        if (position != signal_strength_str.npos)
        {
            //log_info("%s position is : %d\n" , __FUNCTION__, position);
        }
        else
        {
            log_error("%s Not found the @ flag\n", __FUNCTION__);
            return -1;
        }

        signal_num = signal_strength_str.substr(0, position);
        //log_debug("%s signal string is %s", __FUNCTION__, signal_num.c_str());

        strcpy(chSignalStrength, signal_num.c_str());
        signal_level = atoi(chSignalStrength);
        //log_debug("%s signal level = %d", __FUNCTION__, signal_level);

        if(signal_level > -65)
        {
            level = 3;
            //log_debug("%s dbcom signal strength strong",__FUNCTION__);
        }
        else if(signal_level >= -80 && signal_level <= -65)
        {
            level = 2;
            //log_debug("%s dbcom signal strength medium",__FUNCTION__);
        }
        else if(signal_level < -80)
        {
            level = 1;
            //log_debug("%s dbcom signal strength weak",__FUNCTION__);
        }
        else
        {
            log_error("%s unknown level %d\r\n",__FUNCTION__, signal_level);
            level = 0;
        }

    }

    Json::Value root;
    atris_msgs::RobotInfo doublecom_signal_strength;  
    Json::FastWriter fw;
    root["robot_info"]["doublecom"]["level"] = level;
    root["robot_info"]["doublecom"]["connect_status"] = is_connected_?1:0;
    root["robot_info"]["doublecom"]["login_status"] = is_login_?1:0;
    root["robot_info"]["doublecom"]["sig_val"] = signal_level;

    doublecom_signal_strength.json = fw.write(root);
    diag_info_pub_.publish(doublecom_signal_strength);

    return 0;
}

void DbComManager::dbcom_try_login_wait(void)
{

    std::unique_lock<std::mutex> wlock(dbcom_login_mutex_);

    if(dbcom_login_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(10 * 1000)) == std::cv_status::timeout)
    {
        log_warn("%s wait try login timeout, start to try login again",__FUNCTION__);
    }
    else
    {
    }

}