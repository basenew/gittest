/*
 * swupgrade.cpp
 *
 *  Created on: 2018-7-21
 *      Author: fupj
 */
   
#include <signal.h>
#include <curl/curl.h>
#include "swupgrade.h"
#include <libgen.h>
#include "tiny_ros/ros/time.h"
#include "sound/sound.h"
#include "firmware.h"
#include "database/sqliteengine.h"
#include "sound/sound.h"
#include "tts_strings/tts_strings.h"
#include "transferfile/transferfile.h"
#include "task_manager/task_manager.h"
#include "imemory/atris_imemory_api.h"

#define SWUPGRADE_TAG    "SWUpgrade->"
#define SWUPGRADE_VERSION_FILE "/home/atris/atris_app/config/versionInfo.txt"
#define SWUPGRADE_DOWNLOAD_DIR "/userdata/tmp/upgrade/"
#define SWUPGRADE_SCRIPT "/home/atris/atris_app/scripts/atris_swupdate.sh "
#define SWUPGRADE_IMX_SUCCESS "success"
#define SWUPGRADE_IMX_FAILED "failed"
#define SWUPGRADE_IMX_STANDBY "standby"
#define SWUPGRADE_IMX_PROGRESS "progress"

#define SW_UPGRADE_TABLE_COLUMN 2

#define SW_UPGRADE_TABLE "CREATE TABLE IF NOT EXISTS [sw_upgrade] (" \
    "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
    "[logSerial] TEXT)"

boost::mutex SWUpgrade::mutex_;
SWUpgrade* SWUpgrade::swupgrade_ = NULL;
bool SWUpgrade::upgrading_now_ = false;

enum {
  STATUS_success = 0,
  STATUS_upgrading = 1,
  STATUS_fail_downloading = -1,
  STATUS_fail_no_found_tar = -2,
  STATUS_fail_untar = -3,
  STATUS_fail_no_found_swu = -4,
  STATUS_fail_no_found_swu_md5 = -5,
  STATUS_fail_check_md5 = -6,
  STATUS_fail_do_upgrade_script = -7,
  STATUS_fail_atris_family = -8,
  STATUS_fail_invalid_data = -9,
  STATUS_fail_firmware_upgrading = -10,
  STATUS_fail_imx_upgrading = -11,
  STATUS_fail_untar_swu = -12,
  STATUS_fail_fail_remove = -13,
  STATUS_fail_fail_untar_xs = -14,
  STATUS_fail_fail_untar_xs_bin = -15,
  STATUS_fail_unknow = -255,
};

class SWDownload: public FileObject {
private:
    uint32_t last_sec_;
    virtual void progress(double progress /* 0~100 */) {
        uint32_t now = ros::Time::now().sec;
        if ((now - last_sec_) >= 5 || progress >= 100) {
#if 1
            Json::Value response;
            response["id"] = origin.msgID;
            response["timestamp"] = origin.timestamp;
            response["status"] = "downloading";
            response["progress"] = (int)progress;
            response["result"] = "success";
            SWUpgrade::get_instance()->responseResult(origin, response, "response_sw_upgrade");
#endif
            last_sec_ = now;
        }
        if (now < last_sec_) {
            last_sec_ = now;
        }
    }

    virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
        Json::FastWriter fw;
        Json::Value upgrading, downloading;
        upgrading["id"] = origin.msgID;
        upgrading["timestamp"] = origin.timestamp;
        upgrading["status"] = "imx_upgrading";
        upgrading["result"] = "success";

        downloading["id"] = origin.msgID;
        downloading["timestamp"] = origin.timestamp;
        downloading["status"] = "downloading";
        downloading["progress"] = 0;
        downloading["result"] = "success";
        
        if (state == TRANSFER_FILE_ERROR) {
            if (!cancel) {
                downloading["progress"] = 100;
                downloading["result"] = "fail_downloading";
                SWUpgrade::get_instance()->responseResult(origin, downloading, "response_sw_upgrade");
            }
            
            downloading = false;

            SWUpgrade::setUpgrading(false);
            SWUpgrade::setUpgradeStatus(STATUS_fail_downloading);
        } else if (state == TRANSFER_FILE_STARTED) {
            SWUpgrade::get_instance()->responseResult(origin, downloading, "response_sw_upgrade");
        } else if (state == TRANSFER_FILE_COMPLETED) {
            FILE *fp = NULL;
            char *s = NULL;
            char buffer[1024];
            std::string doscript = std::string(SWUPGRADE_SCRIPT) + local_path;
            
            downloading = false;

            sleep(5);

            SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");

            memset(buffer, '\0', sizeof(buffer));
            if((fp = popen(doscript.c_str(), "r")))  {
                while(fgets(buffer, sizeof(buffer) - 1, fp)) 
                {
                    log_info("%s do script buffer : %s\r\n",__FUNCTION__, buffer);
                    if((s = strstr(buffer, "fail_no_found_tar"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_no_found_tar);
                        upgrading["result"] = "fail_no_found_tar";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "fail_atris_pack_untar"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_untar);
                        upgrading["result"] = "fail_atris_pack_untar";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "fail_no_found_swu"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_no_found_swu);
                        upgrading["result"] = "fail_no_found_swu";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "fail_no_found_swu_md5"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_no_found_swu_md5);
                        upgrading["result"] = "fail_no_found_swu_md5";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "fail_check_md5"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_check_md5);
                        upgrading["result"] = "fail_check_md5";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "fail_untar_swu"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_untar_swu);
                        upgrading["result"] = "fail_untar_swu";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
			            goto beach;
                    } else if ((s = strstr(buffer, "fail_remove"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_fail_remove);
                        upgrading["result"] = "fail_remove";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
			            goto beach;
                    } else if ((s = strstr(buffer, "fail_untar_xs_nav_pack"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_fail_untar_xs);
                        upgrading["result"] = "fail_untar_xs_nav_pack";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
			            goto beach;
                    } else if ((s = strstr(buffer, "fail_untar_xs_nav_bin_pack"))) {
                        SWUpgrade::setUpgrading(false);
                        SWUpgrade::setUpgradeStatus(STATUS_fail_fail_untar_xs_bin);
                        upgrading["result"] = "fail_untar_xs_nav_bin_pack";
                        SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
                        goto beach;
                    } else if ((s = strstr(buffer, "success"))) {
                        SWUpgrade::setUpgradeStatus(STATUS_upgrading);

                        system("tinyrosshutdown all; reboot -f");
                        goto beach;
                    }
                }
                
                SWUpgrade::setUpgrading(false);
                SWUpgrade::setUpgradeStatus(STATUS_fail_unknow);
                upgrading["result"] = "fail_unknow";
                SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");

                beach:
                pclose(fp);
            } else {
                SWUpgrade::setUpgrading(false);
                SWUpgrade::setUpgradeStatus(STATUS_fail_do_upgrade_script);
                upgrading["result"] = "fail_do_upgrade_script";
                SWUpgrade::get_instance()->responseResult(origin, upgrading, "response_sw_upgrade");
            }
        }
    }

public:
    SWDownload(): last_sec_(0), downloading(false) { }
    atris_msgs::SignalMessage origin;
    bool downloading;
};

SWUpgrade* SWUpgrade::get_instance() {
    if (!swupgrade_) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!swupgrade_) {
            swupgrade_ = new SWUpgrade();
        }
    }
    return swupgrade_;
}

SWUpgrade::SWUpgrade()
  : obj_(boost::shared_ptr<SWDownload> (new SWDownload()))
    , atris_version_("")
    , log_serial_("")
  {
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &SWUpgrade::messageInstantReceive, this);
    //get_sw_version_srv_ = nh_.advertiseService(SRV_GET_SW_VERSION, &SWUpgrade::doGetSwVersion, this);
    get_ota_pre_condition_srv_client_ = nh_.serviceClient<atris_msgs::GetOTAPreCondition>(SRV_GET_OTA_PRE_CONDITION);
    signal_resp_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    
    initSwUpgradeTable();

    std::string status = getImxUpgradeStatus();
    firmware_thread_ = NULL;
    SWUpgrade::setUpgrading(false);

    //Firmware::get_instance();
    
    Json::FastWriter fw;
    int iRet;
    if (status != SWUPGRADE_IMX_STANDBY) {
        SWUpgrade::setUpgrading(true);
        SWUpgrade::setUpgradeStatus(STATUS_upgrading);
        // here we read the log serial from database, since it is recorded before reboot
        iRet = queryLogSerialFromDataBase(log_serial_);
        if(iRet < 0)
        {
            log_error("%s query log serial from sw upgrade data base failed , iRet = %d",__FUNCTION__, iRet);
        }
        else
        {
            log_info("%s query log serial from sw upgrade data base success ... log serial num : %s", __FUNCTION__, log_serial_.c_str());
        }

        firmware_thread_ = new boost::thread(boost::bind(&SWUpgrade::doFirmwareUpgradeThread, this));
    } else {
        SWUpgrade::setUpgradeStatus(STATUS_success);
    }

    atris_version_ = getAtrisVersion();
    log_info(SWUPGRADE_TAG"%s version: %s", __FUNCTION__, atris_version_.c_str());
}

SWUpgrade::~SWUpgrade() {
    log_info(SWUPGRADE_TAG"%s", __FUNCTION__);
    if (firmware_thread_) {
        firmware_thread_->interrupt();
        firmware_thread_->join();
        delete firmware_thread_;
    }
}


void SWUpgrade::responseResult(const atris_msgs::SignalMessage& origin,
  const Json::Value& content, std::string title) {
    tinyros::Time now = tinyros::Time::now();
    Json::FastWriter jwriter;
    Json::Value root;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    
    root["title"] = Json::Value(title);
    root["accid"] = shmrbt.robot.sn;
    root["content"] = content;
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
    if (root["content"]["id"].isNull()) {
      std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
      root["content"]["id"] = uid.str();
    }

    root["content"]["log_serial"] = log_serial_;
    root["content"]["version"] = atris_version_;

    atris_msgs::SignalMessage resp;
    resp.type = origin.type;
    resp.account = origin.account;
    resp.msgID = root["content"]["id"].asString();
    resp.msg = jwriter.write(root);
    log_info("%s swupgrade response string : %s",__FUNCTION__, resp.msg.c_str());
    signal_resp_pub_.publish(resp);
}

#if 1
std::string SWUpgrade::getAtrisVersion() {
    FILE * fp;
    std::string result = "";
    char * line = NULL;
    size_t len = 0;

    if ((fp = fopen(SWUPGRADE_VERSION_FILE, "r")) != NULL) {
        getline(&line, &len, fp);
        if (line) {
            result = line;
            result.erase(result.find_last_not_of(" \t\f\v\n\r") + 1);
            result.erase(0, result.find_first_not_of(" \t\f\v\n\r"));
            free(line);
        }
        fclose(fp);
    }
    return result;
}
#endif

std::string SWUpgrade::getPowerVersion() {
    return "";//Firmware::get_instance()->getVersionPower();
}

std::string SWUpgrade::getGsVersion() {

    return "";//Firmware::get_instance()->getVersionGS();
}

std::string SWUpgrade::getBMVersion() {
    return "";//Firmware::get_instance()->getVersionBatteryMonitor();
}

std::string SWUpgrade::getBMSVersion() {
    return "";//Firmware::get_instance()->getVersionBMS();
}

void SWUpgrade::setUpgrading(bool upgrading) {
    SWUpgrade::upgrading_now_ = upgrading;
    
    shm::UpgradeStatus upgrade_status;
    shm::iMemory_read_UpgradeStatus(&upgrade_status);
    upgrade_status.upgrading = SWUpgrade::upgrading_now_;
    shm::iMemory_write_UpgradeStatus(&upgrade_status);
}

void SWUpgrade::setUpgradeStatus(int status) {
	log_info("%s start",__FUNCTION__);
    shm::UpgradeStatus upgrade_status;
    shm::iMemory_read_UpgradeStatus(&upgrade_status);
    upgrade_status.status = status;
    shm::iMemory_write_UpgradeStatus(&upgrade_status);
    log_info("%s end",__FUNCTION__);
}
#if 0
void SWUpgrade::doRequestSwVersion(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["imx"] = getImxVersion();
    response["power"] = getPowerVersion();
    response["gs"] = getGsVersion();
    response["battery_monitor"] = getBMVersion();
    response["bms"] = getBMSVersion();
    response["result"] = "success";
    responseResult(msg, response, "response_sw_version");
}
#endif

void SWUpgrade::doRequestSwUpgrade(const atris_msgs::SignalMessage& msg, const Json::Value& values) {
    Json::FastWriter fw;
    Json::Value response;
    int iRet;

    response["id"] = msg.msgID;
    response["timestamp"] = msg.timestamp;
    response["result"] = "success";

    if (!values["content"]["url"].isNull() && !values["content"]["log_serial"].isNull()) {
        shm::Robot shmrbt;
        shm::iMemory_read_Robot(&shmrbt);
        std::string url = values["content"]["url"].asString();
        url.erase(url.find_last_not_of(" \t\f\v\n\r") + 1);
        url.erase(0, url.find_first_not_of(" \t\f\v\n\r"));
        std::string name = basename((char*)(url.c_str()));
        std::string rbt_sn = shmrbt.robot.sn;

        log_serial_ = values["content"]["log_serial"].asString();
        log_info("%s url = %s",__FUNCTION__, url.c_str());
        log_info("-------------------- %s get log serial for this upgrade round , log serial = %s upgrade package name = %s---------------------", __FUNCTION__, log_serial_.c_str(), name.c_str());
        iRet = updateLogSerialToDataBase(log_serial_);
        if(iRet < 0)
        {
            log_error("%s upgrade log serial to database failed iRet = %d",__FUNCTION__, iRet);
        }
        else
        {
            log_info("%s upgrade log serial to database success",__FUNCTION__);
        }

        if (true/*(rbt_sn.find("DAA") != std::string::npos && name.find("Atris1C-") != std::string::npos)
          || (rbt_sn.find("DAA") != std::string::npos && name.find("Atris1CEvt2-") != std::string::npos)
          || (rbt_sn.find("DAA") != std::string::npos && name.find("Atris1CDvt1-") != std::string::npos)
          || (rbt_sn.find("DAA") != std::string::npos && name.find("Atris1CDvt2-") != std::string::npos)
          || (rbt_sn.find("DAF") != std::string::npos && name.find("Atris1CGa-") != std::string::npos)
          || (rbt_sn.find("DAD") != std::string::npos && name.find("Atris1W-") != std::string::npos)
          || (rbt_sn.find("DAH") != std::string::npos && name.find("Atris1W-") != std::string::npos)
          || (rbt_sn.find("DAG") != std::string::npos && name.find("Atris1WGa-") != std::string::npos)*/) {
            SWUpgrade::setUpgradeStatus(STATUS_upgrading);
            response["status"] = "started";
            SWUpgrade::get_instance()->responseResult(msg, response, "response_sw_upgrade");

            obj_->local_path = SWUPGRADE_DOWNLOAD_DIR + name;
            obj_->remote_path = url;
            obj_->origin = msg;
            obj_->downloading = true;
            obj_->cancel = false;
            obj_->breakpoint = true;
            TransferFile::download(obj_);
        } else {
            SWUpgrade::setUpgrading(false);
            response["result"] = "fail_atris_family";
            SWUpgrade::get_instance()->responseResult(msg, response, "response_sw_upgrade");
            SWUpgrade::setUpgradeStatus(STATUS_fail_atris_family);
        }
    } else {
        log_error("%s failed invalid content!!!!!!!!!!!!!!!!",__FUNCTION__);
        SWUpgrade::setUpgrading(false);
        SWUpgrade::setUpgradeStatus(STATUS_fail_invalid_data);
        response["result"] = "fail_invalid_data";
        responseResult(msg, response, "response_sw_upgrade");
    }

}

std::string SWUpgrade::getImxUpgradeStatus() {
    char buffer[1024];
    FILE *fp = NULL;
    char *s = NULL;
    std::string status = SWUPGRADE_IMX_STANDBY;

    memset(buffer, '\0', sizeof(buffer));
    if((fp = popen("atris_printenv recovery_status", "r"))) {
        while(fgets(buffer, sizeof(buffer) - 1, fp)) {
            if((s = strstr(buffer, SWUPGRADE_IMX_SUCCESS))) {
                status = SWUPGRADE_IMX_SUCCESS;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_FAILED))) {
                status = SWUPGRADE_IMX_FAILED;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_STANDBY))) {
                status = SWUPGRADE_IMX_STANDBY;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_PROGRESS))) {
                status = SWUPGRADE_IMX_PROGRESS;
                break;
            }
        }
        log_info(SWUPGRADE_TAG"%s status: %s", __FUNCTION__, status.c_str());
        pclose(fp);
    } else {
        log_error(SWUPGRADE_TAG"%s popen failed", __FUNCTION__);
    }
    return status;
}

void SWUpgrade::setImxUpgradeStatus(std::string status) {
    if (status == SWUPGRADE_IMX_STANDBY || status == SWUPGRADE_IMX_SUCCESS
            || status == SWUPGRADE_IMX_PROGRESS || status == SWUPGRADE_IMX_FAILED) {
        std::string cmd = std::string("atris_setenv recovery_status ") + status;
        system(cmd.c_str());
    } else {
        log_error(SWUPGRADE_TAG"%s unknow status: %s", __FUNCTION__, status.c_str());
    }
}

int SWUpgrade::doFirmwareUpgradeTask() {
    atris_msgs::AisoundTTS msg;
    msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SWU_INFO);
    aisound_tts_pub_.publish(msg);
    return 0;
}

void SWUpgrade::doFirmwareUpgradeThread() {
    int64_t now = (int64_t)(tinyros::Time::now().toSec() * 1000);
    std::stringstream id; id << now;

    Json::FastWriter fw;
    Json::Value response;
    response["id"] = id.str();
    response["timestamp"] = now;
    response["status"] = "firmware_upgrading";
    response["result"] = "success";
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    atris_msgs::SignalMessage param;
    param.account = shmrbt.robot.receiver;
    
    atris_msgs::AisoundTTS msg;
    msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    
    Sound::get_instance()->setVolume(30);
    
    TaskManager *swu_tm = TaskManager::get_instance();
    Task swu_task;
    swu_task.cb = boost::bind(&SWUpgrade::doFirmwareUpgradeTask, this);
    swu_tm->post_cycle(swu_task, 30*1000);
    sleep(15);
    
    std::string status = getImxUpgradeStatus();
    if (status == SWUPGRADE_IMX_SUCCESS) {
        if (shmrbt.robot.binded) {
            responseResult(param, response, "response_sw_upgrade");
        }
        if (!Firmware::get_instance()->upgrade()) {
            swu_tm->del(swu_task.id);
            swu_task.id = -1;
            setImxUpgradeStatus(SWUPGRADE_IMX_STANDBY);
            //if (shmrbt.robot.binded) {
                response["status"] = "finished";
                responseResult(param, response, "response_sw_upgrade");
            //}
            msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SWU_FINISHED);
            msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
            aisound_tts_pub_.publish(msg);
            SWUpgrade::setUpgradeStatus(STATUS_success);
        } else {
            swu_tm->del(swu_task.id);
            swu_task.id = -1;
            //if (shmrbt.robot.binded) {
                response["result"] = "fail_firmware_upgrading";
                responseResult(param, response, "response_sw_upgrade");
            //}
            msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SWU_FAIL_FIRMWARE);
            msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
            aisound_tts_pub_.publish(msg);
            SWUpgrade::setUpgradeStatus(STATUS_fail_firmware_upgrading);
        }
    } else if (status == SWUPGRADE_IMX_FAILED || status == SWUPGRADE_IMX_PROGRESS) {
        if (status == SWUPGRADE_IMX_PROGRESS) {
            FILE *fp = NULL;
            char *s = NULL;
            char buffer[1024];
            std::string doscript = std::string(SWUPGRADE_SCRIPT);
            memset(buffer, '\0', sizeof(buffer));
            if((fp = popen(doscript.c_str(), "r")))  {
                while(fgets(buffer, sizeof(buffer) - 1, fp)) {
                    if ((s = strstr(buffer, "success"))) {
                        system("tinyrosshutdown all; reboot -f");
                        return;
                    }
                }
                pclose(fp);
            }
        }

        swu_tm->del(swu_task.id);
        swu_task.id = -1;
        //if (shmrbt.robot.binded) {
            //response["status"] = "imx_upgrading";
            response["status"] = "finished";
            response["result"] = "fail_imx_upgrading";
            responseResult(param, response, "response_sw_upgrade");
        //}
        msg.text = TTSStrings::text(TTSStrings::TTS_KEY_SWU_FAIL_FIRMWARE);
        msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
        aisound_tts_pub_.publish(msg);
        SWUpgrade::setUpgradeStatus(STATUS_fail_imx_upgrading);
    }

    if (swu_task.id != -1) {
        swu_tm->del(swu_task.id);
        swu_task.id = -1;
    }
    
    // response_robot_status
    if (shmrbt.robot.binded) {
        response["status"] = "online";
        response["result"] = "success";
        responseResult(param, response, "response_robot_status");
    }

    SWUpgrade::setUpgrading(false);

    sleep(5);
    shm::iMemory_read_Robot(&shmrbt);
    Sound::get_instance()->setVolume(shmrbt.appdata.volume);
}

void SWUpgrade::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
    Json::Reader reader;
    Json::Value root;
    Json::FastWriter writer;
    std::string upgrade_content_str = "";
    int iRet;
    if (msg.title == "request_sw_upgrade") {
        log_info("****************** %s recv software upgrade*****************",__FUNCTION__);
        log_info("****************** %s recv software upgrade*****************",__FUNCTION__);
        log_info("****************** %s recv software upgrade*****************",__FUNCTION__);
        log_info("****************** %s recv software upgrade*****************",__FUNCTION__);

        if (SWUpgrade::upgrading_now_) {
            log_error("&&&&&&&&&&& %s already in upgrading process &&&&&&&&&&&&&&", __FUNCTION__);
            //Json::Value response;
            //response["id"] = msg.msgID;
            //response["timestamp"] = msg.timestamp;
            //response["result"] = "fail_upgrading_now";
            //responseResult(msg, response, "response_sw_upgrade");
            return;
        }


        SWUpgrade::setUpgrading(true);

        reader.parse(msg.msg, root);
        upgrade_content_str = writer.write(root);
        log_info("%s upgrade content string : %s",__FUNCTION__, upgrade_content_str.c_str());

        doRequestSwUpgrade(msg, root);
    } else if (msg.title == "request_sw_cancel") {
        obj_->cancel = true;
        
        Json::Value response;
        response["id"] = msg.msgID;
        response["timestamp"] = msg.timestamp;
        if (obj_->downloading) {
            obj_->downloading = false;
            response["result"] = "success";
        } else {
            response["result"] = "fail_no_download";
        }
        responseResult(msg, response, "response_sw_cancel");
    }
}

#if 0
bool SWUpgrade::doGetSwVersion(atris_msgs::GetSwVersion::Request& req,
  atris_msgs::GetSwVersion::Response& res) {
    res.imx = getImxVersion();
    res.power = getPowerVersion();
    res.gs = getGsVersion();
    res.battery_monitor = getBMVersion();
    res.bms = getBMSVersion();
    return true;
}
#endif

bool SWUpgrade::initSwUpgradeTable(void)
{
    char **result;
    int row, column;
    int retry_times = 3;
    int iRet;

    log_info("%s called", __FUNCTION__);

    // retry initialize the diag event database
    do
    {
        iRet = SqliteEngine::execSQL(SW_UPGRADE_TABLE, SQLITE_SW_UPGRADE_FILE);
        if(iRet != SQLITE_OK)
        {
            log_info("%s create software upgrade table failed!!! iRet = %d", __FUNCTION__, iRet);
            reinitSwUpgradeDataBase();
        }
        else
        {
            log_info("initialize software upgrade database table success ...iRet = %d", iRet);
            break;
        }

        retry_times--;
    }while(retry_times != 0);

    if(retry_times == 0)
    {
        // we initialize failed here , do not need to do the following steps
        log_error("init sw upgrade data base failed after three times!!!");
        return false;
    }
    else
    {
        log_info("%s retry times not exceeded , retry times = %d", __FUNCTION__, retry_times);
    }

    // from here the data base file exist
    iRet = SqliteEngine::query("SELECT * FROM sw_upgrade", &result, &row, &column, SQLITE_SW_UPGRADE_FILE);
    log_info("%s check if data base format ok , SELECT * FROM sw_upgrade row = %d , column = %d\r\n", __FUNCTION__, row, column);
    if(iRet == SQLITE_OK)
    {
        if (row > 0 && column != SW_UPGRADE_TABLE_COLUMN) 
        {
            // query data base 
            // data base format wrong
            log_error("sw upgrade database format error!!!");
            log_error("%s database format wrong column shoud be %d , but row = %d , column = %d\r\n", __FUNCTION__, SW_UPGRADE_TABLE_COLUMN, row , column);
            SqliteEngine::execSQL("DROP TABLE sw_upgrade");
            SqliteEngine::freeQuery(result);
            // reinit the database
            reinitSwUpgradeDataBase();
            return false;
        }
        else if(row <= 0)
        {
            // init sw upgrade data base
            log_info("%s init sw upgrade database , just put empty log serial num into database", __FUNCTION__);
            log_info("%s sw upgrade data base file format ok!!!",__FUNCTION__);
            log_info("%s row = %d , column = %d\r\n",__FUNCTION__, row , column);
            SqliteEngine::execSQL("INSERT INTO sw_upgrade(logSerial) VALUES('" +log_serial_+"')", SQLITE_SW_UPGRADE_FILE);
        }
        else if(row > 0 && column == SW_UPGRADE_TABLE_COLUMN)
        {
            // just print row num for debug use
            log_info("%s log serial num already stored in table , row num = %d", __FUNCTION__, row);
        }
    }
    else
    {
        log_info("%s query event from sw upgrade data base failed",__FUNCTION__);
        SqliteEngine::freeQuery(result);
        return false;
    }

    SqliteEngine::freeQuery(result);
    return true;
}

// sw upgrade table format is wrong , reinit the sw upgrade database
void SWUpgrade::reinitSwUpgradeDataBase(void)
{
    log_error("%s ", __FUNCTION__);
    std::string sw_upgrade_db_path = SQLITE_SW_UPGRADE_FILE;
    if(access(sw_upgrade_db_path.c_str(), F_OK)==0)
    {
        // if the data base file exist, rename the data base file to .bak
        log_warn("%s rename the sw upgrade data base file to bak!!!!!!",__FUNCTION__);
        std::string cmd = "mv " + std::string(SQLITE_SW_UPGRADE_FILE) + " " + std::string(SQLITE_SW_UPGRADE_FILE) + ".bak";
        system(cmd.c_str());
        SqliteEngine::execSQL(SW_UPGRADE_TABLE,  SQLITE_SW_UPGRADE_FILE);
    }
    else
    {
        // create the data base file if it doest not exist
        log_error("%s sw upgrade database file does not exist",__FUNCTION__);
        SqliteEngine::execSQL(SW_UPGRADE_TABLE,  SQLITE_SW_UPGRADE_FILE); // create database file
    }
}

// update log serial number to sw upgrade database
// result 0 success , else failed
int SWUpgrade::updateLogSerialToDataBase(const std::string & log_serial_str)
{
    log_info("%s called",__FUNCTION__);
    int ret = SQLITE_OK;
    char **result;
    int row = 0, column = 0;

    log_info("%s log serial : %s", __FUNCTION__, log_serial_str.c_str());

    ret = SqliteEngine::query("SELECT * FROM sw_upgrade", &result, &row, &column, SQLITE_SW_UPGRADE_FILE);
    log_info("%s select * from sw_upgrade , row : %d , column : %d\r\n",__FUNCTION__, row, column);
    if(ret == SQLITE_OK)
    {
        if(row > 0 && column != SW_UPGRADE_TABLE_COLUMN)
        {
            // usually cannot reach here, format wrong
            SqliteEngine::execSQL("DROP TABLE sw_upgrade");
            SqliteEngine::freeQuery(result);
            reinitSwUpgradeDataBase();
            log_error("%s column num does not match", __FUNCTION__);
            return -1;
        }
        else if(row > 0 && column == SW_UPGRADE_TABLE_COLUMN)
        {

            // update the log serial entry
            log_info("%s update the log serial entry where log serial string = %s",__FUNCTION__, log_serial_str.c_str());
            SqliteEngine::execSQL("UPDATE sw_upgrade SET logSerial='"+ log_serial_str +"'", SQLITE_SW_UPGRADE_FILE);

        }
        else if(row <= 0)
        {
            // cant find any log serial in the data base just reinitialize the database
            SqliteEngine::freeQuery(result);
            reinitSwUpgradeDataBase();
            log_error("%s sw upgrade data base table is empty", __FUNCTION__);
            return -2;
        }
        else
        {
            SqliteEngine::freeQuery(result);
            log_error("%s unknown errro, something wrong just return!!!",__FUNCTION__);
            return -3;
        }
    }
    else
    {
        log_error("%s select * from sw upgrade data base failed ret = %d", __FUNCTION__, ret);
        SqliteEngine::freeQuery(result);
        return -4;
    }

    SqliteEngine::freeQuery(result);
    
    return 0;
}

// query log serial number from sw upgrade database
// result 0 success , else failed
int SWUpgrade::queryLogSerialFromDataBase(std::string & query_log_serial_str)
{
    int iRet;
    char **result1 = nullptr;
    int row1 = 0;
    int column1 = 0;

    iRet = SqliteEngine::query("SELECT * FROM sw_upgrade", &result1, &row1, &column1, SQLITE_SW_UPGRADE_FILE);
    if(iRet == SQLITE_OK)
    {
        if (row1 <= 0) 
        {

            log_error("%s sw upgrade table is empty, no log serial record in the data base", __FUNCTION__);
            SqliteEngine::freeQuery(result1);
            return -1;
        }
        else if(row1 > 0 && column1 != SW_UPGRADE_TABLE_COLUMN)
        {
            log_error("%s format wrong , drop the table and reinit the data base", __FUNCTION__);
            SqliteEngine::execSQL("DROP TABLE sw_upgrade");
            SqliteEngine::freeQuery(result1);
            // reinit the database
            reinitSwUpgradeDataBase();
            return -2;

        }
        else if(row1 == 1 && column1 == SW_UPGRADE_TABLE_COLUMN)
        {
            // correct row num, print the result 
            // get last round log serial from the data base
            log_info("%s row1 = %d\r\n", __FUNCTION__, row1);

            query_log_serial_str = result1[column1+1];
            log_info("%s query success , get query result , log serial num : %s", __FUNCTION__, query_log_serial_str.c_str());

        }
        else
        {
            // something wrong here, just print for debug use
            log_error("%s unknown error , something wrong just return!!!",__FUNCTION__);
            SqliteEngine::freeQuery(result1);
            return -3;
        }

    }
    else
    {
        log_info("%s query log serial from database failed iRet = %d\r\n",__FUNCTION__, iRet);
        SqliteEngine::freeQuery(result1);
        return -4;
    }

    SqliteEngine::freeQuery(result1);
    return 0;
}

// check ota condition on robot
// if robot not braked or robot bat level is not more than 30% , return negative response to pc
int SWUpgrade::judgeOTAPreCond()
{
    atris_msgs::GetOTAPreCondition otapre;
    int battery_level = -1;
    bool brake_status = false;

    if (get_ota_pre_condition_srv_client_.call(otapre))
    {
        brake_status = otapre.response.brake_status;
        battery_level = otapre.response.battery_level;
    }

    #if 1
    if(!brake_status)
    {
        log_warn("%s failed robot not braked!!!",__FUNCTION__);
        return -2;
    }

    if(battery_level < 30)
    {
        log_warn("%s failed robot battery level less than 30 percent!!!",__FUNCTION__);
        return -1;
    }
    #endif

    return 0;
}

#if 0
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "swupgrade");
    tinyros::init("swupgrade");
    ros::Time::init();
    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);
    shm::iMemory_init();
    SWUpgrade::get_instance();
    ros::MultiThreadedSpinner s(10);
    ros::spin(s);
    curl_global_cleanup();

    return 0;
}
#endif
