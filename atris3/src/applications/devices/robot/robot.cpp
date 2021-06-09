/*
 * robot.cpp
 *
 *  Created on: 2018-6-21
 *      Author: fupj
 */

#include "robot.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "eeprom/eeprom.h"
#include "flash_lamp/flash_lamp.h"
#include "abi/abi.h"
#include "libmediaplayer/mediaplayer.h"
#include "libxml/xml.h"
#include "imemory/atris_imemory_api.h"
#include "utils/utils.h"
#include "voip/voip.h"
#include "can_service/can_service.h"
#include "task_manager/task_manager.h"
#include "tts_strings/tts_strings.h"

#define ROBOT_TAG      "Robot->"
#define ROBOT_TIME_ACCURACY 1000
#define ROBOT_TABLE_COLUMN  9
#define ROBOT_TABLE "CREATE TABLE IF NOT EXISTS [robot] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[sn] TEXT NOT NULL," \
        "[name] TEXT NOT NULL," \
        "[pwd] TEXT NOT NULL," \
        "[channel] TEXT," \
        "[pc] TEXT," \
        "[binded] INTEGER," \
        "[accid] TEXT," \
        "[token] TEXT)"

#define APPDATA_TABLE_COLUMN  15
#define APPDATA_TABLE "CREATE TABLE IF NOT EXISTS [appdata] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[speed] DOUBLE," \
        "[ptz_step] FLOAT," \
        "[volume] INTEGER," \
        "[play_interval] INTEGER," \
        "[muted] INTEGER," \
        "[shaked] INTEGER," \
        "[tts_lng] INTEGER," \
        "[tts_speaker] INTEGER," \
        "[tts_enable] INTEGER," \
        "[sip_volume] INTEGER," \
        "[power_warning_value] INTEGER," \
        "[power_warning_charge] INTEGER," \
        "[sip_num] TEXT," \
        "[speed_level] TEXT)"

#define DEFAULT_SIP_NUM  ("100000")
#define DEFAULT_SPEED_LEVEL ("slow")

boost::mutex Robot::mutex_;
Robot* Robot::robot_ = NULL;
int64_t Robot::calibration_value = 0;
bool Robot::calibrated = false;

Robot::Robot() {
    log_info(ROBOT_TAG"%s", __FUNCTION__);
    stop_voice_chat_srv_client_ = nh_.serviceClient<atris_msgs::StopVoiceChat>(SRV_STOP_VOICE_CHAT);
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    time_calibrated_pub_ = nh_.advertise<atris_msgs::TimeCalibrated>(TOPIC_TIME_CALIBRATED_MESSAGE, 100);
    robot_shaked_pub_ = nh_.advertise<atris_msgs::RobotShaked>(TOPIC_ROBOT_SHAKED_MESSAGE, 100);
    nav_reset_pub_ = nh_.advertise<atris_msgs::NavReset>(TOPIC_NAV_RESET, 100);
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Robot::messageInstantReceive, this);
    yunxin_token_sub_ = nh_.subscribe(TOPIC_YUNXIN_TOKEN_UPDATE, 100, &Robot::doYunxinTokenUpdate, this);

    memset(&shmrobot_, 0, sizeof(shm::Robot));
    std::string strsn = getSnFromEeprom();
    snprintf(shmrobot_.robot.sn, sizeof(shmrobot_.robot.sn), "%s", strsn.c_str());
    snprintf(shmrobot_.robot.name, sizeof(shmrobot_.robot.name), "%s", strsn.c_str());
    snprintf(shmrobot_.robot.pwd, sizeof(shmrobot_.robot.pwd), "%s", Config::get_instance()->default_psw.c_str());
    snprintf(shmrobot_.appdata.sip_num, sizeof(shmrobot_.appdata.sip_num), "%s", DEFAULT_SIP_NUM);
    snprintf(shmrobot_.appdata.speed_level, sizeof(shmrobot_.appdata.speed_level), "%s", DEFAULT_SPEED_LEVEL);
    shmrobot_.appdata.chassis_speed = 0.3;
    shmrobot_.appdata.ptz_step = 2.0;
    shmrobot_.appdata.volume = 30;
    shmrobot_.appdata.tts_lng = 0;
    shmrobot_.appdata.tts_speaker = 0;
    shmrobot_.appdata.tts_enable = 1;
    shmrobot_.appdata.power_warning_value = 40;
    shmrobot_.appdata.power_warning_charge = 30;
    
    initRobotTable();
    initAppdataTable();
    shm::iMemory_write_Robot(&shmrobot_);

    abiEnable_ = false;
    player_ =  MediaPlayer::createMediaPlayer();
}

Robot::~Robot() {
    log_info(ROBOT_TAG"%s", __FUNCTION__);
    Abi::deleteInstance();
    if (player_) {
        delete player_;
        player_ = NULL;
    }
}

Robot* Robot::get_instance() {
    if (!robot_) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!robot_) {
            robot_ = new Robot();
        }
    }
    return robot_;
}

void Robot::init() {
    log_debug(ROBOT_TAG"%s", __FUNCTION__);

    if (Config::get_instance()->abi_enable) {
        abiEnable_ = true;
    } else {
        abiEnable_ = false;
    }
    Abi::createInstance(shmrobot_.robot.sn);
}

void Robot::initRobotTable() {
    char **result;
    int row, column;

    retry:
    SqliteEngine::execSQL(ROBOT_TABLE);
    SqliteEngine::query("SELECT * FROM robot", &result, &row, &column);
    log_debug(ROBOT_TAG"%s row: %d, column: %d", __FUNCTION__, row, column);
    if (row <= 0) {
        std::stringstream sbinded; sbinded<<shmrobot_.robot.binded;
        SqliteEngine::execSQL("INSERT INTO robot(sn, name, pwd, channel, pc, binded, accid,token) VALUES('"
                +std::string(shmrobot_.robot.sn)+"', '"
                +std::string(shmrobot_.robot.name)+"', '"
                +std::string(shmrobot_.robot.pwd)+"', '"
                +std::string(shmrobot_.robot.channel)+"', '"
                +std::string(shmrobot_.robot.receiver)+"', "
                +sbinded.str()+", '"
                +std::string(shmrobot_.robot.accid)+"', '"
                +std::string(shmrobot_.robot.token)+"')");
    } else {
        if (column != ROBOT_TABLE_COLUMN) {
            SqliteEngine::execSQL("DROP TABLE robot");
            goto retry;
        } else {
            std::string rbtsn = result[column + 1];
            snprintf(shmrobot_.robot.name, sizeof(shmrobot_.robot.name), "%s", result[column + 2]);
            snprintf(shmrobot_.robot.pwd, sizeof(shmrobot_.robot.pwd), "%s", result[column + 3]);
            snprintf(shmrobot_.robot.channel, sizeof(shmrobot_.robot.channel), "%s", result[column + 4]);
            snprintf(shmrobot_.robot.receiver, sizeof(shmrobot_.robot.receiver), "%s", result[column + 5]);
            shmrobot_.robot.binded = atoi(result[column + 6]);
            snprintf(shmrobot_.robot.accid, sizeof(shmrobot_.robot.accid), "%s", result[column + 7]);
            snprintf(shmrobot_.robot.token, sizeof(shmrobot_.robot.token), "%s", result[column + 8]);
            if (rbtsn != std::string(shmrobot_.robot.sn)) {
                shmrobot_.robot.token[0] = '\0';
                SqliteEngine::execSQL("UPDATE robot SET sn='"+std::string(shmrobot_.robot.sn)+"'");
                SqliteEngine::execSQL("UPDATE robot SET token='"+std::string(shmrobot_.robot.token)+"'");
            }
        }
    }

    SqliteEngine::freeQuery(result);
    log_debug(ROBOT_TAG"%s sn: %s, name: %s, channel: %s, pc: %s, binded: %d", __FUNCTION__,
      shmrobot_.robot.sn, shmrobot_.robot.name, shmrobot_.robot.channel, shmrobot_.robot.receiver, shmrobot_.robot.binded);
}


void Robot::initAppdataTable() {
    char **result;
    int row, column;
    int sip_volume = 0;
    int iRet;
    Voip::get_instance()->GetVolume(&sip_volume);
    shmrobot_.appdata.sip_volume = sip_volume;

    retry:
    SqliteEngine::execSQL(APPDATA_TABLE);
    iRet = SqliteEngine::query("SELECT * FROM appdata", &result, &row, &column, SQLITE_DATA_FILE);
    log_debug(ROBOT_TAG"%s row: %d, column: %d", __FUNCTION__, row, column);
    if(iRet == SQLITE_OK)
    {
        if (row <= 0) {

            std::stringstream sspeed; sspeed<<shmrobot_.appdata.chassis_speed;
            std::stringstream sstep; sstep<<shmrobot_.appdata.ptz_step;
            std::stringstream svolume; svolume<<shmrobot_.appdata.volume;
            std::stringstream smuted; smuted<<shmrobot_.appdata.muted;
            std::stringstream splay_interval; splay_interval<<shmrobot_.appdata.play_interval;
            std::stringstream sshaked; sshaked<<shmrobot_.appdata.shaked;
            std::stringstream stts_lng; stts_lng<<shmrobot_.appdata.tts_lng;
            std::stringstream stts_speaker; stts_speaker<<shmrobot_.appdata.tts_speaker;
            std::stringstream stts_enable; stts_enable<<shmrobot_.appdata.tts_enable;
            std::stringstream ssip_volume; ssip_volume<<shmrobot_.appdata.sip_volume;
            std::stringstream spower_warning_value; spower_warning_value<<shmrobot_.appdata.power_warning_value;
            std::stringstream spower_warning_charge; spower_warning_charge<<shmrobot_.appdata.power_warning_charge;

            // if column number match , insert the default value into the app database
            SqliteEngine::execSQL("INSERT INTO appdata(speed, ptz_step, volume, play_interval, muted, shaked, tts_lng, tts_speaker, tts_enable, sip_volume, power_warning_value, power_warning_charge, sip_num, speed_level) VALUES("
                +sspeed.str()+", "
                +sstep.str()+", "
                +svolume.str()+", "
                +splay_interval.str()+", "
                +smuted.str()+", "
                +sshaked.str()+", "
                +stts_lng.str()+", "
                +stts_speaker.str()+", "
                +stts_enable.str()+", "
                +ssip_volume.str()+", "
                +spower_warning_value.str()+", "
                +spower_warning_charge.str()+", '"
                +std::string(shmrobot_.appdata.sip_num)+"', '"
                +std::string(shmrobot_.appdata.speed_level)+"')");

        } else {
            if (column != APPDATA_TABLE_COLUMN) {
                SqliteEngine::execSQL("DROP TABLE appdata");
                reinitAppDataDb();
                goto retry;
            } else {
                shmrobot_.appdata.chassis_speed = atof(result[column + 1]);
                shmrobot_.appdata.ptz_step = atof(result[column + 2]);
                shmrobot_.appdata.volume = atoi(result[column + 3]);
                shmrobot_.appdata.play_interval = atoi(result[column + 4]);
                shmrobot_.appdata.muted = atoi(result[column + 5]);
                shmrobot_.appdata.shaked = atoi(result[column + 6]);
                shmrobot_.appdata.tts_lng = atoi(result[column + 7]);
                shmrobot_.appdata.tts_speaker = atoi(result[column + 8]);
                shmrobot_.appdata.tts_enable = atoi(result[column + 9]);
                shmrobot_.appdata.sip_volume = atoi(result[column + 10]);
                shmrobot_.appdata.power_warning_value = atoi(result[column + 11]);
                shmrobot_.appdata.power_warning_charge = atoi(result[column + 12]);
                snprintf(shmrobot_.appdata.sip_num, sizeof(shmrobot_.appdata.sip_num), "%s", result[column + 13]);
                snprintf(shmrobot_.appdata.speed_level, sizeof(shmrobot_.appdata.speed_level), "%s", result[column + 14]);

                if (sip_volume != shmrobot_.appdata.sip_volume) {
                    shmrobot_.appdata.sip_volume = sip_volume;
                    std::stringstream ssip_volume; ssip_volume<<shmrobot_.appdata.sip_volume;
                    SqliteEngine::execSQL("UPDATE appdata SET sip_volume="+ssip_volume.str());
                }
            }
        }
    }
    else
    {
    }
    // init soundcard
    shmrobot_.appdata.muted = 0;
    Sound::get_instance()->setMuted(shmrobot_.appdata.muted);
    Sound::get_instance()->setVolume(shmrobot_.appdata.volume);

    SqliteEngine::freeQuery(result);
}


std::string Robot::getSnFromEeprom() {
    char buffer[100];
    FILE *fp = NULL;
    std::string sn = "";

    memset(buffer, '\0', sizeof(buffer));
    if((fp = popen("GetSN", "r"))) {
        while(fgets(buffer, sizeof(buffer) - 1, fp));
        pclose(fp);
    }
    sn = buffer;
    sn.erase(sn.find_last_not_of(" \t\f\v\n\r") + 1);
    sn.erase(0, sn.find_first_not_of(" \t\f\v\n\r"));
    std::transform(sn.begin(), sn.end(), sn.begin(), ::toupper);
    if (sn.empty()) {
        log_error(ROBOT_TAG"%s please set robot sn", __FUNCTION__);
    }
    return sn;
}

void Robot::doResetRobot(int clear) {
    atris_msgs::NavReset nav_reset_msg;
    
    // stop voice chat
    atris_msgs::StopVoiceChat stopvoice;
    stop_voice_chat_srv_client_.call(stopvoice);

    // stop play propaganda
    Propaganda::get_instance()->stop();
    //close FlashLamp
    //FlashLamp::get_instance()->close_all_lamp();
    
    FILE *fp = NULL;
    if (clear == 1) { // reset factory
        nav_reset_msg.reset = atris_msgs::NavReset::NAV_RESET;
        nav_reset_pub_.publish(nav_reset_msg);
        
        // reset appdata
        SqliteEngine::execSQL("DELETE FROM appdata");

        if((fp = popen("rm -rf /userdata/impdata/robot.cfg", "r")) != NULL) {
            pclose(fp);
        }

        if((fp = popen("rm -rf /userdata/impdata/rosshttpd.db", "r")) != NULL) {
            pclose(fp);
        }
    } else if (clear == 2) { // reset factory and clear data
        nav_reset_msg.reset = atris_msgs::NavReset::NAV_RESET_CLEAR;
        nav_reset_pub_.publish(nav_reset_msg);
        
        // backup tts lisense
        if((fp = popen("mv /userdata/impdata/aisound /home/atris", "r")) != NULL) {
            pclose(fp);
        }
   
        if((fp = popen("rm -rf /userdata/*", "r")) != NULL) {
            pclose(fp);
        }

        // restore tts lisense
        if((fp = popen("mkdir -p /userdata/impdata", "r")) != NULL) {
            pclose(fp);
        }
        if((fp = popen("mv /home/atris/aisound /userdata/impdata", "r")) != NULL) {
            pclose(fp);
        }

    }

    if (clear > 0) {
        if((fp = popen("sync", "r")) != NULL) {
            pclose(fp);
        }

        sleep(2);

        if((fp = popen("tinyrosshutdown all; reboot -f", "r")) != NULL) {
            pclose(fp);
        }
    }
}

void Robot::doResetRobotByAbi()
{
    if ((shmrobot_.robot.binded != 1) ||
        Abi::getInstance()->isofflineLogined()) {
        log_info("abi not need reset");
    }  else {
        Json::Value notify;
        ros::Time now = ros::Time::now();
        std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
        notify["id"] = uid.str();
        notify["timestamp"] = now.toSec() * 1000;
        notify["sn"] = std::string(shmrobot_.robot.sn);
        notify["client"] = "";
        atris_msgs::SignalMessage param;
        param.account = std::string(shmrobot_.robot.receiver);
        param.type = Config::get_instance()->server_type;
        Utils::get_instance()->responseResult(param, notify, "notify_other_unbound");

        shmrobot_.robot.binded = 0;
        shmrobot_.robot.accid[0] = '\0';
        shmrobot_.robot.channel[0] = '\0';
        shmrobot_.robot.receiver[0] = '\0';
        shm::iMemory_write_Robot(&shmrobot_);
        
        std::stringstream ss;
        ss << shmrobot_.robot.binded;
        SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
        SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
        SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
        SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");

        // reset robot
        this->doResetRobot(0);
    }
}

int64_t Robot::timeCalibrated() {
    int64_t now = (int64_t)ros::Time::now().toSec() * 1000;
    if (Robot::calibrated) {
        return (Robot::calibration_value + now);
    }
    return 0;
}

// request get robot state
void Robot::doRequestGetRobotState(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    char **result;
    int row, column;
    int iRet;

    if(root["accid"].isNull() || root["content"]["id"].isNull() || root["content"]["timestamp"].isNull())
    {
        log_error("%s fail invalid data",__FUNCTION__);
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_get_robot_state");
    }
    else
    {
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "success";

        // some initial fake value
        response["power_warning_value"] = 20;
        response["power_warning_charge"] = 30;
        response["chassis_speed"] = 1.1f;
        response["speed_level"] = "slow";
        response["ptz_step"] = 2.0f;
        response["speaker_volume"] = 30;

        iRet = SqliteEngine::query("SELECT * FROM appdata", &result, &row, &column, SQLITE_DATA_FILE);
        log_debug(ROBOT_TAG"%s row: %d, column: %d", __FUNCTION__, row, column);
        //log_debug("%s request data field ok!!!",__FUNCTION__);
        if(iRet == SQLITE_OK)
        {
            if(row<=0)
            {
                log_error("%s appdata database is empty!!!!!!!",__FUNCTION__);
                response["result"] = "fail_invalid_data";

            }
            else
            {
                if(column != APPDATA_TABLE_COLUMN)
                {
                    SqliteEngine::execSQL("DROP TABLE appdata", SQLITE_DATA_FILE);
                    reinitAppDataDb();
                    initAppdataTable();
                    log_error("%s column num does not match", __FUNCTION__);
                    response["result"] = "fail_invalid_data";
                }
                else
                {
                    log_info("%s get robot setting from database success",__FUNCTION__);
                    response["power_warning_value"] = atoi(result[column + 11]);
                    response["power_warning_charge"] = atoi(result[column + 12]);
                    response["chassis_speed"] = atof(result[column + 1]);
                    response["speed_level"] = result[column + 14];
                    response["ptz_step"] = atof(result[column + 2]);
                    response["speaker_volume"] = atoi(result[column + 3]);
                }
            }
        }
        else
        {
            response["result"] = "fail_invalid_data";
            SqliteEngine::execSQL("DROP TABLE appdata", SQLITE_DATA_FILE);
            reinitAppDataDb();
            initAppdataTable();
        }

        Utils::get_instance()->responseResult(msg, response, "response_get_robot_state");
        SqliteEngine::freeQuery(result);
    }
}

// request change robot state
void Robot::doRequestChangeRobotState(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    int iRet;
    Json::Value response;
    std::string message_content = "";
    Json::FastWriter jwriter;
    std::string default_speed_level = "slow"; // just for test default use
    char **result;
    int row, column;
    int power_warning_value;
    int power_warning_charge;
    float ptz_step;
    double chassis_speed;
    std::string speed_level;
    int speaker_volume;

    message_content = jwriter.write(root);
    log_info("%s message json : %s",__FUNCTION__, message_content.c_str());

    if(root["accid"].isNull() 
        || root["content"]["id"].isNull() 
        || root["content"]["timestamp"].isNull()
        || root["content"]["power_warning_value"].isNull()
        || root["content"]["power_warning_charge"].isNull()
        || root["content"]["chassis_speed"].isNull()
        || root["content"]["ptz_step"].isNull()
        || root["content"]["speed_level"].isNull()
        || root["content"]["speaker_volume"].isNull())
    {
        log_debug("%s fail invalid data",__FUNCTION__);
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_change_robot_state");
    }
    else
    {
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "success";

        // just print the value, and update the database, and config variable
        // we will read this variable from database when program start next cycle
        log_debug("%s power warning value : %d , power warning charge : %d , chassis speed : %lf , speed level : %s , ptz step : %f , speaker volume : %d",
            __FUNCTION__, root["content"]["power_warning_value"].asInt(), root["content"]["power_warning_charge"].asInt(), root["content"]["chassis_speed"].asDouble()
            , root["content"]["speed_level"].isNull()?default_speed_level.c_str():root["content"]["speed_level"].asString().c_str(), root["content"]["ptz_step"].asFloat(), root["content"]["speaker_volume"].asInt());

        //Config::get_instance()->low_battery_charge = root["content"]["power_warning_charge"].asInt();
        //Config::get_instance()->low_battery_warn = root["content"]["power_warning_value"].asInt();

        // here we update the database
        //updateRobotState()
        iRet = SqliteEngine::query("SELECT * FROM appdata", &result, &row, &column, SQLITE_DATA_FILE);
        log_debug(ROBOT_TAG"%s row: %d, column: %d", __FUNCTION__, row, column);
        if(iRet == SQLITE_OK)
        {
            if(row<=0)
            {
                log_error("%s appdata database is empty!!!!!!!",__FUNCTION__);
                response["result"] = "sucess";

                power_warning_value = root["content"]["power_warning_value"].asInt();
                power_warning_charge = root["content"]["power_warning_charge"].asInt();
                ptz_step = root["content"]["ptz_step"].asFloat();
                chassis_speed = root["content"]["chassis_speed"].asDouble();
                speed_level = root["content"]["speed_level"].asString();
                speaker_volume = root["content"]["speaker_volume"].asInt();
            
                // write it to share mem
                shmrobot_.appdata.chassis_speed = chassis_speed;
                shmrobot_.appdata.ptz_step = ptz_step;
                shmrobot_.appdata.volume = speaker_volume;
                shmrobot_.appdata.power_warning_value = power_warning_value;
                shmrobot_.appdata.power_warning_charge = power_warning_charge;
                snprintf(shmrobot_.appdata.speed_level, sizeof(shmrobot_.appdata.speed_level), "%s", speed_level.c_str());
                shm::iMemory_write_Robot(&shmrobot_);


                std::stringstream spower_warning_value; spower_warning_value<<power_warning_value;
                std::stringstream spower_warning_charge; spower_warning_charge<<power_warning_charge;
                std::stringstream sstep; sstep<<ptz_step;
                std::stringstream schassis_speed; schassis_speed<<chassis_speed;
                std::stringstream sspeaker_volume; sspeaker_volume<<speaker_volume;

                SqliteEngine::execSQL("UPDATE appdata SET power_warning_value='"+spower_warning_value.str()+"'", SQLITE_DATA_FILE);
                SqliteEngine::execSQL("UPDATE appdata SET power_warning_charge='"+spower_warning_charge.str()+"'", SQLITE_DATA_FILE);
                SqliteEngine::execSQL("UPDATE appdata SET ptz_step='"+sstep.str()+"'",SQLITE_DATA_FILE);
                SqliteEngine::execSQL("UPDATE appdata SET speed='"+schassis_speed.str()+"'",SQLITE_DATA_FILE);
                SqliteEngine::execSQL("UPDATE appdata SET speed_level='"+speed_level+"'",SQLITE_DATA_FILE);
                SqliteEngine::execSQL("UPDATE appdata SET volume='"+sspeaker_volume.str()+"'",SQLITE_DATA_FILE);
                
            }
            else
            {
                if(column != APPDATA_TABLE_COLUMN)
                {
                    response["result"] = "fail_invalid_data";
                    SqliteEngine::execSQL("DROP TABLE appdata", SQLITE_DATA_FILE);
                    reinitAppDataDb();
                    initAppdataTable();
                }
                else
                {
                    // update the database


                    power_warning_value = root["content"]["power_warning_value"].asInt();
                    power_warning_charge = root["content"]["power_warning_charge"].asInt();
                    ptz_step = root["content"]["ptz_step"].asFloat();
                    chassis_speed = root["content"]["chassis_speed"].asDouble();
                    speed_level = root["content"]["speed_level"].asString();
                    speaker_volume = root["content"]["speaker_volume"].asInt();
                
                    // write it to share mem
                    shmrobot_.appdata.chassis_speed = chassis_speed;
                    shmrobot_.appdata.ptz_step = ptz_step;
                    shmrobot_.appdata.volume = speaker_volume;
                    shmrobot_.appdata.power_warning_value = power_warning_value;
                    shmrobot_.appdata.power_warning_charge = power_warning_charge;
                    snprintf(shmrobot_.appdata.speed_level, sizeof(shmrobot_.appdata.speed_level), "%s", speed_level.c_str());
                    shm::iMemory_write_Robot(&shmrobot_);


                    std::stringstream spower_warning_value; spower_warning_value<<power_warning_value;
                    std::stringstream spower_warning_charge; spower_warning_charge<<power_warning_charge;
                    std::stringstream sstep; sstep<<ptz_step;
                    std::stringstream schassis_speed; schassis_speed<<chassis_speed;
                    std::stringstream sspeaker_volume; sspeaker_volume<<speaker_volume;

                    response["result"] = "success";
                    SqliteEngine::execSQL("UPDATE appdata SET power_warning_value='"+spower_warning_value.str()+"'", SQLITE_DATA_FILE);
                    SqliteEngine::execSQL("UPDATE appdata SET power_warning_charge='"+spower_warning_charge.str()+"'", SQLITE_DATA_FILE);
                    SqliteEngine::execSQL("UPDATE appdata SET ptz_step='"+sstep.str()+"'",SQLITE_DATA_FILE);
                    SqliteEngine::execSQL("UPDATE appdata SET speed='"+schassis_speed.str()+"'",SQLITE_DATA_FILE);
                    SqliteEngine::execSQL("UPDATE appdata SET speed_level='"+speed_level+"'",SQLITE_DATA_FILE);
                    SqliteEngine::execSQL("UPDATE appdata SET volume='"+sspeaker_volume.str()+"'",SQLITE_DATA_FILE);
                }
            }
        }
        else
        {
            response["result"] = "fail_invalid_data";
            SqliteEngine::execSQL("DROP TABLE appdata", SQLITE_DATA_FILE);
            reinitAppDataDb();
            initAppdataTable();
        }
        
        response["power_warning_value"] = root["content"]["power_warning_value"];
        response["power_warning_charge"] = root["content"]["power_warning_charge"];
        response["chassis_speed"] = root["content"]["chassis_speed"];
        response["speed_level"] = root["content"]["speed_level"].isNull()?default_speed_level:root["content"]["speed_level"];
        response["ptz_step"] = root["content"]["ptz_step"];
        response["speaker_volume"] = root["content"]["speaker_volume"];
        //response["result"] = "";
        Utils::get_instance()->responseResult(msg, response, "response_change_robot_state");
        SqliteEngine::freeQuery(result);
    }
}

void Robot::reinitAppDataDb(void)
{
    log_error("%s", __FUNCTION__);
    std::string appdata_db_path = SQLITE_DATA_FILE;
    if(access(appdata_db_path.c_str(), F_OK)==0)
    {
        log_warn("%s rename the appdata data base file to bak!!!!!!",__FUNCTION__);
        std::string cmd = "mv " + std::string(SQLITE_DATA_FILE) + " " + std::string(SQLITE_DATA_FILE) + ".bak";
        system(cmd.c_str());
        SqliteEngine::execSQL(APPDATA_TABLE);
    }
    else
    {
        log_error("%s appdata database file does not exist",__FUNCTION__);
        SqliteEngine::execSQL(APPDATA_TABLE); // create database file
    }
}

void Robot::doRequestLoginRobot(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    std::string accid = root["accid"].isNull() ? "" : root["accid"].asString();
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    EepromData *atris_info = &Eeprom::get_instance()->atris_info;
    response["camera_reg_id"] = Json::Value(atris_info->camera_reg_id);
    response["token"]= "";

    if ((shmrobot_.robot.binded) && (accid != std::string(shmrobot_.robot.accid))) {
        response["result"] = "fail_other_bound";
    } else {
        if (!root["content"]["account"].isNull() &&
            !root["content"]["pwd"].isNull()) {
            std::string abiAccount = root["content"]["account"].asString();
            std::string pwd = root["content"]["pwd"].asString();

            bool ret = Abi::getInstance()->isRightUserInfo(abiAccount, pwd);
            if (ret) {
                Abi::getInstance()->updateAbiOffLineToken();
                Abi::getInstance()->updateAbiofflineLogined(true);
                response["token"] = Abi::getInstance()->getAbiOffLineToken();
                response["companyid"] = Abi::getInstance()->getAbiCompanyId();
                response["result"] = "success";
            } else {
                response["result"] = "fail_invalid_user";
            }
        } else {
            response["result"] = "fail_invalid_data";
        }
    }
    Utils::get_instance()->responseResult(msg, response, "response_login_robot");

}

void Robot::doRequestBoundRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["pwd"].isNull() &&
        !root["content"]["channel"].isNull()) {
        if (std::string(shmrobot_.robot.pwd) == root["content"]["pwd"].asString()) {
            std::string  straccid = root["accid"].isNull() ? "" : root["accid"].asString();
            if (shmrobot_.robot.binded) {
                if (straccid == std::string(shmrobot_.robot.accid)) {
                    std::string strchannel = root["content"]["channel"].asString();
                    snprintf(shmrobot_.robot.channel, sizeof(shmrobot_.robot.channel), "%s", strchannel.c_str());
                    SqliteEngine::execSQL("UPDATE robot SET channel='"+strchannel+"'");
                    Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
                } else {
                    response["result"] = "fail_other_bound";
                    Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
                }
            } else {
                std::string strreceiver = msg.account;
                std::string  strchannel = root["content"]["channel"].asString();
                snprintf(shmrobot_.robot.receiver, sizeof(shmrobot_.robot.receiver), "%s", strreceiver.c_str());
                snprintf(shmrobot_.robot.accid, sizeof(shmrobot_.robot.accid), "%s", straccid.c_str());
                snprintf(shmrobot_.robot.channel, sizeof(shmrobot_.robot.channel), "%s", strchannel.c_str());
                if (Config::get_instance()->default_psw != std::string(shmrobot_.robot.pwd)) {
                    shmrobot_.robot.binded = 1;
                }
                shm::iMemory_write_Robot(&shmrobot_);
                
                std::stringstream ss; ss << shmrobot_.robot.binded;
                SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
                SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");
                SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
                SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
                EepromData *atris_info = &Eeprom::get_instance()->atris_info;
                response["camera_reg_id"] = Json::Value(atris_info->camera_reg_id);
                Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
            }
        } else {
            response["result"] = "fail_invalid_pwd";
            Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
        }
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
    }
}

void Robot::doRequestBoundRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    EepromData *atris_info = &Eeprom::get_instance()->atris_info;
    response["camera_reg_id"] = Json::Value(atris_info->camera_reg_id);
    response["bindmode"] = root["content"]["bindmode"];
    response["companyid"] = Abi::getInstance()->getAbiCompanyId();


    bool bindSuccess = false;
    if (!root["content"]["account"].isNull() &&
        !root["content"]["pwd"].isNull()     &&
        !root["content"]["channel"].isNull() &&
        !root["accid"].isNull()) {
        std::string abiAccount = root["content"]["account"].asString();
        std::string pwd = root["content"]["pwd"].asString();
        std::string accid = root["accid"].asString();
        int bindMode = root["content"]["bindmode"].isNull() ? 0 : root["content"]["bindmode"].asInt();
        if (bindMode == 0) {
            bool abiConnected = Abi::getInstance()->getAbiIsConnected();
            if (abiConnected) {
                int ret = Abi::getInstance()->postRequest(abiAccount, pwd, OPERATETYPEBIND);
                if (ret == 0) {
                    bindSuccess = true;
                    response["result"] = "success";
                } else {
                    std::string string;
                    covertAbiHttpErrToFailString(ret, string);
                    response["result"] = string;
                }
            } else {
                response["result"] = "fail_abi_offline";
            }
        } else {
            if (shmrobot_.robot.binded) {
                if (accid == std::string(shmrobot_.robot.accid)) {
                    bool ret = Abi::getInstance()->isRightUserInfo(abiAccount, pwd);
                    if (ret) {
                        bindSuccess = true;
                        response["result"] = "success";
                    } else {
                        response["result"] = "fail_abi_invalid_account_or_pwd";
                    }
                } else {
                    response["result"] = "fail_other_bound";
                }
            } else {
                bool ret = Abi::getInstance()->isRightUserInfo(abiAccount, pwd);
                if (ret) {
                    bindSuccess = true;
                    response["result"] = "success";
                } else {
                    response["result"] = "fail_abi_invalid_account_or_pwd";
                }
            }
        }
        Utils::get_instance()->responseResult(msg, response, "response_bound_robot");

        if (bindSuccess) {
            std::string strsipnum = root["content"]["phoneNum"].isNull() ? DEFAULT_SIP_NUM : root["content"]["phoneNum"].asString();
            std::string strchannel = root["content"]["channel"].asString();
            snprintf(shmrobot_.appdata.sip_num, sizeof(shmrobot_.appdata.sip_num), "%s", strsipnum.c_str());
            snprintf(shmrobot_.robot.channel, sizeof(shmrobot_.robot.channel), "%s", strchannel.c_str());
            snprintf(shmrobot_.robot.accid, sizeof(shmrobot_.robot.accid), "%s", accid.c_str());
            snprintf(shmrobot_.robot.receiver, sizeof(shmrobot_.robot.receiver), "%s", msg.account.c_str());
            shmrobot_.robot.binded = 1;
            shm::iMemory_write_Robot(&shmrobot_);
            
            std::stringstream ss; ss << shmrobot_.robot.binded;
            SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");
            SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
            SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
            SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
            SqliteEngine::execSQL("UPDATE appdata SET sip_num='"+std::string(shmrobot_.appdata.sip_num)+"'");
            Voip::get_instance()->SetSipNum(strsipnum);
        }
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_bound_robot");
    }
}

void Robot::doRequestChangePwd(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (abiEnable_) {
        response["result"] = "fail_abi_enable";
        Utils::get_instance()->responseResult(msg, response, "response_change_pwd");
        return;
    }

    if (!root["content"]["pwd"].isNull()) {
        std::string accid = root["accid"].asString();
        if (accid == std::string(shmrobot_.robot.accid)) {
            std::string strpwd = root["content"]["pwd"].asString();
            snprintf(shmrobot_.robot.pwd, sizeof(shmrobot_.robot.pwd), "%s", strpwd.c_str());
            shmrobot_.robot.binded = 1;
            shm::iMemory_write_Robot(&shmrobot_);
            
            std::stringstream ss; ss << shmrobot_.robot.binded;
            SqliteEngine::execSQL("UPDATE robot SET pwd='"+std::string(shmrobot_.robot.pwd)+"'");
            SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());

            EepromData *atris_info = &Eeprom::get_instance()->atris_info;
            response["camera_reg_id"] = Json::Value(atris_info->camera_reg_id);
            Utils::get_instance()->responseResult(msg, response, "response_change_pwd");
        } else {
            response["result"] = "fail_other_bound";
            Utils::get_instance()->responseResult(msg, response, "response_change_pwd");
        }
        Utils::get_instance()->responseResult(msg, response, "response_change_pwd");
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_change_pwd");
    }
}

void Robot::doRequestUnboundRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    
    shmrobot_.robot.binded = 0;
    shmrobot_.robot.accid[0] = '\0';
    shmrobot_.robot.channel[0] = '\0';
    shmrobot_.robot.receiver[0] = '\0';
    shm::iMemory_write_Robot(&shmrobot_);
    
    std::stringstream ss; ss << shmrobot_.robot.binded;
    SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
    SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
    SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
    SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");
    Utils::get_instance()->responseResult(msg, response, "response_unbound_robot");

    // stop voice chat
    atris_msgs::StopVoiceChat stopvoice;
    stop_voice_chat_srv_client_.call(stopvoice);

    // stop play propaganda
    Propaganda::get_instance()->stop();
    
    //close FlashLamp
    FlashLamp::get_instance()->close_all_lamp();

    //stop all task
    atris_msgs::NavReset nav_reset_msg;
    nav_reset_msg.reset = atris_msgs::NavReset::NAV_STOP;
    nav_reset_pub_.publish(nav_reset_msg);
}

void Robot::doRequestUnboundRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) 
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["bindmode"] = root["content"]["bindmode"];

    bool unbindSuccess = false;
    if (!root["content"]["account"].isNull() &&
        !root["content"]["pwd"].isNull() &&
        !root["accid"].isNull()) {
        std::string accid = root["accid"].asString();
        int bindMode = root["content"]["bindmode"].isNull() ? 0 : root["content"]["bindmode"].asInt();
        if (bindMode == 0) {
            std::string abiAccount = root["content"]["account"].asString();
            std::string pwd = root["content"]["pwd"].asString();
            bool abiConnected = Abi::getInstance()->getAbiIsConnected();
            if (abiConnected) {
                int ret = Abi::getInstance()->postRequest(abiAccount, pwd, OPERATETYPEUNBIND);
                if (ret == 0) {
                    unbindSuccess = true;
                    response["result"] = "success";
                } else {
                    std::string string;
                    covertAbiHttpErrToFailString(ret, string);
                    response["result"] = string;
                }
            } else {
                response["result"] = "fail_abi_offline";
            }
        } else {
            if (shmrobot_.robot.binded) {
                if (accid == std::string(shmrobot_.robot.accid)) {
                    unbindSuccess = true;
                    response["result"] = "success";
                    Abi::getInstance()->updateAbiofflineLogined(false);
                } else {
                    response["result"] = "fail_other_bound";
                }
            } else {
                response["result"] = "success";
            }
        }
        Utils::get_instance()->responseResult(msg, response, "response_unbound_robot");

        if (unbindSuccess) {
            shmrobot_.robot.binded = 0;
            shmrobot_.robot.accid[0] = '\0';
            shmrobot_.robot.channel[0] = '\0';
            shmrobot_.robot.receiver[0] = '\0';
            shm::iMemory_write_Robot(&shmrobot_);
            
            std::stringstream ss; ss << shmrobot_.robot.binded;
            SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
            SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
            SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
            SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");
            
            // stop voice chat
            atris_msgs::StopVoiceChat stopvoice;
            stop_voice_chat_srv_client_.call(stopvoice);

            // stop play propaganda
            Propaganda::get_instance()->stop();
            
            //close FlashLamp
            FlashLamp::get_instance()->close_all_lamp();

            atris_msgs::NavReset nav_reset_msg;
            nav_reset_msg.reset = atris_msgs::NavReset::NAV_STOP;
            nav_reset_pub_.publish(nav_reset_msg);
        }
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_unbound_robot");
    }
}

void Robot::doRequestResetRobotDisableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["clear"] = 0;
    response["result"] = "success";

    if (!root["content"]["account"].isNull() && 
        !root["content"]["pwd"].isNull()) {
        if (Config::get_instance()->super_name == root["content"]["account"].asString()) {
            if (Config::get_instance()->super_psw == root["content"]["pwd"].asString()) {
                if (shmrobot_.robot.binded) {
                    std::string accid = root["accid"].asString();
                    if (accid != std::string(shmrobot_.robot.accid)) {
                        Json::Value notify;
                        notify["id"] = root["content"]["id"];
                        notify["timestamp"] = root["content"]["timestamp"];
                        notify["sn"] = std::string(shmrobot_.robot.sn);
                        notify["client"] = msg.account;

                        atris_msgs::SignalMessage temp = msg;
                        temp.account = std::string(shmrobot_.robot.receiver);
                        Utils::get_instance()->responseResult(temp, notify, "notify_other_unbound");
                        return;
                    }
                }

                std::string strpwd = Config::get_instance()->default_psw;
                snprintf(shmrobot_.robot.pwd, sizeof(shmrobot_.robot.pwd), "%s", strpwd.c_str());
                shmrobot_.robot.binded = 0;
                shmrobot_.robot.receiver[0] = '\0';
                shmrobot_.robot.accid[0] = '\0';
                shmrobot_.robot.channel[0] = '\0';
                shm::iMemory_write_Robot(&shmrobot_);
                
                std::stringstream ss;ss << shmrobot_.robot.binded;
                SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
                SqliteEngine::execSQL("UPDATE robot SET pwd='"+std::string(shmrobot_.robot.pwd)+"'");
                SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
                SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
                SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");

                // reset robot
                int clear = root["content"]["clear"].isNull() ? 0 : root["content"]["clear"].asInt() ;
                response["clear"] = clear;
                Utils::get_instance()->responseResult(msg, response, "response_reset_robot");
                if (clear > 0) {
                    new boost::thread(boost::bind(&Robot::doResetRobot, this, clear));
                } else {
                    this->doResetRobot(clear);
                }
            } else {
                response["result"] = "fail_invalid_pwd";
                Utils::get_instance()->responseResult(msg, response, "response_reset_robot");
            }
        } else {
            response["result"] = "fail_invalid_user";
            Utils::get_instance()->responseResult(msg, response, "response_reset_robot");
        }
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_reset_robot");
    }
}

void Robot::doRequestResetRobotEnableAbi(const atris_msgs::SignalMessage& msg, const Json::Value& root) 
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["clear"] = 0;
    response["timestamp"] = root["content"]["timestamp"];
    response["bindmode"] = root["content"]["bindmode"];

    bool resetSuccess = false;
    int clear = root["content"]["clear"].isNull() ? 0 : root["content"]["clear"].asInt();
    response["clear"] = clear;
    if (!root["content"]["account"].isNull() &&
        !root["content"]["pwd"].isNull()) {
        std::string abiAccount = root["content"]["account"].asString();
        std::string pwd = root["content"]["pwd"].asString();
        int bindMode = root["content"]["bindmode"].isNull() ? 0 : root["content"]["bindmode"].asInt();
        if (bindMode == 0) {
            bool abiConnected = Abi::getInstance()->getAbiIsConnected();
            if (abiConnected) {
                int ret = Abi::getInstance()->postRequest(abiAccount, pwd, OPERATETYPERESET);
                if (ret == 0) {
                    resetSuccess = true;
                    response["result"] = "success";
                } else {
                    std::string string;
                    covertAbiHttpErrToFailString(ret, string);
                    response["result"] = string;
                }
            } else {
                response["result"] = "fail_abi_offline";
            }
        } else {
            bool ret = Abi::getInstance()->isSuperUser(abiAccount, pwd);
            if (ret) {
                resetSuccess = true;
                response["result"] = "success";
                Abi::getInstance()->updateAbiofflineLogined(false);
            } else {
                response["result"] = "fail_abi_usr_has_no_right";
            }
        }
        if (resetSuccess) {
            std::string accid = root["accid"].asString();
            if ((shmrobot_.robot.binded) && (accid != std::string(shmrobot_.robot.accid))) {
                Json::Value notify;
                notify["id"] = root["content"]["id"];
                notify["timestamp"] = root["content"]["timestamp"];
                notify["sn"] = std::string(shmrobot_.robot.sn);
                notify["client"] = msg.account;
                atris_msgs::SignalMessage tmp = msg;
                tmp.account = std::string(shmrobot_.robot.receiver);
                Utils::get_instance()->responseResult(tmp, notify, "notify_other_unbound");
            }

            shmrobot_.robot.receiver[0] = '\0';
            shmrobot_.robot.accid[0] = '\0';
            shmrobot_.robot.channel[0] = '\0';
            shmrobot_.robot.binded = 0;
            shm::iMemory_write_Robot(&shmrobot_);
            
            std::stringstream ss;ss << shmrobot_.robot.binded;
            SqliteEngine::execSQL("UPDATE robot SET binded="+ss.str());
            SqliteEngine::execSQL("UPDATE robot SET pc='"+std::string(shmrobot_.robot.receiver)+"'");
            SqliteEngine::execSQL("UPDATE robot SET accid='"+std::string(shmrobot_.robot.accid)+"'");
            SqliteEngine::execSQL("UPDATE robot SET channel='"+std::string(shmrobot_.robot.channel)+"'");

            // reset robot
            if (clear > 0) {
                new boost::thread(boost::bind(&Robot::doResetRobot, this, clear));
            } else {
                this->doResetRobot(clear);
            }
        }
    } else {
        response["result"] = "fail_invalid_data";
    }
    Utils::get_instance()->responseResult(msg, response, "response_reset_robot");
}

void Robot::doRequestTimeCalibration(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    int64_t now = (int64_t)ros::Time::now().toSec() * 1000;
    int64_t timestamp = root["content"]["timestamp"].asInt64();
    int64_t calibration = Robot::calibration_value;
    int64_t interval =  timestamp - now;

    Robot::calibrated = false;
    Robot::calibration_value = interval;

    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    if (llabs(calibration - interval) >= ROBOT_TIME_ACCURACY) {
        response["result"] = "fail_try_again";
    } else {
        Robot::calibrated = true;
    }
    
    shm::TimeCalibrated tc;
    tc.calibrated = Robot::calibrated;
    tc.interval = Robot::calibration_value;
    shm::iMemory_write_TimeCalibrated(&tc);

    atris_msgs::TimeCalibrated tc_msg;
    tc_msg.calibrated = Robot::calibrated;
    tc_msg.interval = Robot::calibration_value;
    time_calibrated_pub_.publish(tc_msg);
    
    Utils::get_instance()->responseResult(msg, response, "response_time_calibration");
}

void Robot::doRequestBoundInfo(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["binded"] = shmrobot_.robot.binded;
    response["result"] = "success";
    if (shmrobot_.robot.binded) {
        response["pc"] = std::string(shmrobot_.robot.accid);
    } else {
        response["pc"] = "";
    }
    Utils::get_instance()->responseResult(msg, response, "response_bound_info");
}

void Robot::doRequestRobotStatus(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];

    if (shmrobot_.robot.binded) {
        std::string accid = root["accid"].isNull() ? "" : root["accid"].asString();
        if (accid == std::string(shmrobot_.robot.accid)) {
            shm::UpgradeStatus status;
            if (shm::iMemory_read_UpgradeStatus(&status)) {
                response["status"] = status.upgrading ? "upgrading": "online";
            }
        } else {
            response["status"] = "fail_other_bound";
        }
    } else {
        response["status"] = "not_bound";
    }
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_robot_status");
}

void Robot::doRequestSetVolume(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["volume"].isNull()) {
        shmrobot_.appdata.volume = root["content"]["volume"].asInt();
        shm::iMemory_write_Robot(&shmrobot_);

        Sound::get_instance()->setVolume(shmrobot_.appdata.volume);

        std::stringstream svolume; svolume<<shmrobot_.appdata.volume;
        SqliteEngine::execSQL("UPDATE appdata SET volume="+svolume.str());

        Utils::get_instance()->responseResult(msg, response, "response_set_volume");
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_set_volume");
    }
}

void Robot::doRequestSetMute(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["muted"].isNull()) {
        shmrobot_.appdata.muted = root["content"]["muted"].asInt();
        shm::iMemory_write_Robot(&shmrobot_);

        Sound::get_instance()->setMuted(shmrobot_.appdata.muted);

        std::stringstream smuted; smuted<<shmrobot_.appdata.muted;
        SqliteEngine::execSQL("UPDATE appdata SET muted="+smuted.str());

        Utils::get_instance()->responseResult(msg, response, "response_set_mute");
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_set_mute");
    }
}

void Robot::doRequestGetVolume(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    response["volume"] = shmrobot_.appdata.volume;
    response["muted"] = shmrobot_.appdata.muted;
    Utils::get_instance()->responseResult(msg, response, "response_get_volume");
}

void Robot::doRequestGetMute(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    response["muted"] = shmrobot_.appdata.muted;
    Utils::get_instance()->responseResult(msg, response, "response_get_mute");
}

void Robot::doRequestSetPlayInterval(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["play_interval"].isNull()) {
        shmrobot_.appdata.play_interval = root["content"]["play_interval"].asInt() ;
        shm::iMemory_write_Robot(&shmrobot_);

        std::stringstream splay_interval; splay_interval<<shmrobot_.appdata.play_interval;
        SqliteEngine::execSQL("UPDATE appdata SET play_interval="+splay_interval.str());

        Utils::get_instance()->responseResult(msg, response, "response_set_play_interval");
    } else {
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_set_play_interval");
    }
}

void Robot::doRequestGetPlayInterval(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    response["play_interval"] = shmrobot_.appdata.play_interval;
    Utils::get_instance()->responseResult(msg, response, "response_get_play_interval");
}

void Robot::doRequestShakeDown(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (shmrobot_.appdata.shaked) {
        response["result"] = "fail_already_shaked";
    } else {
        // stop voice chat
        atris_msgs::StopVoiceChat stopvoice;
        stop_voice_chat_srv_client_.call(stopvoice);

        // stop play propaganda
        Propaganda::get_instance()->stop();
        
        //close FlashLamp
        FlashLamp::get_instance()->close_all_lamp();

        atris_msgs::NavReset nav_reset_msg;
        nav_reset_msg.reset = atris_msgs::NavReset::NAV_STOP;
        nav_reset_pub_.publish(nav_reset_msg);

        shmrobot_.appdata.shaked = 1;
        shm::iMemory_write_Robot(&shmrobot_);
        std::stringstream sshaked; sshaked<<shmrobot_.appdata.shaked;
        SqliteEngine::execSQL("UPDATE appdata SET shaked="+sshaked.str());
    }

    atris_msgs::RobotShaked shaked_msg;
    shaked_msg.shaked = shmrobot_.appdata.shaked;
    robot_shaked_pub_.publish(shaked_msg);
    
    Utils::get_instance()->responseResult(msg, response, "response_shake_down");
}

void Robot::doRequestShakeUp(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (shmrobot_.appdata.shaked) {
        shmrobot_.appdata.shaked = 0;
        shm::iMemory_write_Robot(&shmrobot_);
        std::stringstream sshaked; sshaked<<shmrobot_.appdata.shaked;
        SqliteEngine::execSQL("UPDATE appdata SET shaked="+sshaked.str());
    } else {
        response["result"] = "fail_no_shaked";
    }

    atris_msgs::RobotShaked shaked_msg;
    shaked_msg.shaked = shmrobot_.appdata.shaked;
    robot_shaked_pub_.publish(shaked_msg);

    Utils::get_instance()->responseResult(msg, response, "response_shake_up");
}

void Robot::doRequestShakeDie(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    
    shmrobot_.appdata.shaked = 0;
    shm::iMemory_write_Robot(&shmrobot_);
    std::stringstream sshaked; sshaked<<shmrobot_.appdata.shaked;
    SqliteEngine::execSQL("UPDATE appdata SET shaked="+sshaked.str());

    CanPkg package = {0};
    package.head.id.channel = CH_SHAKE_DIE;
    package.data[0] = 0xF5;
    CanService::get_instance()->send(&package, 1);

    Utils::get_instance()->responseResult(msg, response, "response_shake_die");
}

void Robot::doRequestShakeStatus(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["shaked"] = shmrobot_.appdata.shaked;
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_shake_status");
}

void Robot::doRequestSetRobotSn(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    std::string robotSn = "";
    if (!root["content"]["sn"].isNull()) {
        robotSn = root["content"]["sn"].asString();
        const char *sn = robotSn.c_str();
        if ((strncmp(sn, "DA", 2) != 0) ||
           (strncmp(sn+6, "UBT", 3) != 0)) {
            response["result"] = "fail_sn_is_invaild";
        } else {
            std::string setSnCmd = "SetSN " + robotSn;
            FILE *fp = popen(setSnCmd.c_str(), "r");
            if (fp == NULL) {
                log_error("%s set robot sn failed!", __FUNCTION__);
                response["result"] = "fail_set_sn";
            } else {
                shmrobot_.robot.token[0] = '\0';
                snprintf(shmrobot_.robot.sn, sizeof(shmrobot_.robot.sn), "%s", robotSn.c_str());
                shm::iMemory_write_Robot(&shmrobot_);
                
                SqliteEngine::execSQL("UPDATE robot SET sn='" +std::string(shmrobot_.robot.sn)+"'");
                SqliteEngine::execSQL("UPDATE robot SET token='"+std::string(shmrobot_.robot.token)+"'");
            }
            pclose(fp);
        }
    } else {
        response["result"] = "fail_sn_is_empty";
    }

    Utils::get_instance()->responseResult(msg, response, "response_set_robot_sn");
}

int Robot::closeMicAfterRecordFile()
{
    return 0;
}

void Robot::doRequestRecordFactoryFile(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if ((!root["content"]["duration"].isNull()) && 
        (!root["content"]["file_name"].isNull())) {
        std::string duration = root["content"]["duration"].asString();
        std::string fileName = root["content"]["file_name"].asString();
        std::string recordCmd = "arecord -d " + duration + 
                                " -r 48000 -f S16_LE /userdata/impdata/" + fileName + "&";
       
        FILE *fp = popen(recordCmd.c_str(), "r");
        if (fp == NULL) {
            log_error("%s record audio file failed!", __FUNCTION__);
            response["result"] = "fail_record_audio_file";
        }
        pclose(fp);

        int delayMicroSec = atoi(duration.c_str()) * 1000;
        Task task;
        task.cb = boost::bind(&Robot::closeMicAfterRecordFile, this);
        TaskManager::get_instance()->post_delay(task, delayMicroSec);
    } else {
        response["result"] = "fail_no_invalid_params";
    }

    Utils::get_instance()->responseResult(msg, response, "response_record_factory_file");
}

void Robot::doRequestPlayFactoryFile(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    if (!root["content"]["file_name"].isNull()) {
        std::string fileName = root["content"]["file_name"].asString();
        std::string filePath = "/userdata/impdata/" + fileName;
        player_->setDataSource(filePath);
        player_->prepare();
        player_->start();
    } else {
        response["result"] = "fail_file_name_is_empty";
    }

    Utils::get_instance()->responseResult(msg, response, "response_play_factory_file");
}

void Robot::doRequestStopAllTask(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    this->doResetRobot(0);
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_stop_all_task");
}

void Robot::doRequesSetVoipAttributes(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    std::string voipVolume = root["content"]["volume"].isNull() ? "" : root["content"]["volume"].asString();
    if (voipVolume != "") {
        int volume = atoi(voipVolume.c_str());
        if ((volume < 1) || (volume > 9)) {
            response["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, response, "response_set_voip_attributes");
            return;
        }

        int ret = Voip::get_instance()->SetVolume(volume);
        if (ret == 0) {
            response["result"] = "success";
            
            shmrobot_.appdata.sip_volume = volume;
            shm::iMemory_write_Robot(&shmrobot_);
            std::stringstream ssip_volume; ssip_volume<<shmrobot_.appdata.sip_volume;
            SqliteEngine::execSQL("UPDATE appdata SET sip_volume="+ssip_volume.str());
        } else {
             response["result"] = "fail_inner_error";
        }
    } else {
         response["result"] = "fail_invalid_data";
    }

    Utils::get_instance()->responseResult(msg, response, "response_set_voip_attributes");
}

void Robot::doRequesGetVoipAttributes(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["volume"] = shmrobot_.appdata.sip_volume;
    response["result"] = "success";

    Utils::get_instance()->responseResult(msg, response, "response_get_voip_attributes");
}

void Robot::doRequesHangupVoip(const atris_msgs::SignalMessage& msg, const Json::Value& root)
{
    Json::Value response;
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["result"] = "success";

    int ret = Voip::get_instance()->CancelCall();
    if (ret == 0) {
        response["result"] = "success";
    } else {
        response["result"] = "failed";
    }

    Utils::get_instance()->responseResult(msg, response, "response_hangup_voip");
}

void Robot::doNoticeVoipCall()
{
    if (!shmrobot_.robot.binded) {
        Voip::get_instance()->CancelCall();

        atris_msgs::AisoundTTS msg;
        msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
        msg.text = TTSStrings::text(TTSStrings::TTS_KEY_HIGH_CALL_VOIP_NO_BOUND);
        aisound_tts_pub_.publish(msg);
        log_info("NO bound,CancelCall");
    }
}

void Robot::doYunxinTokenUpdate(const std_msgs::String& msg) {
    snprintf(shmrobot_.robot.token, sizeof(shmrobot_.robot.token), "%s", msg.data.c_str());
    shm::iMemory_write_Robot(&shmrobot_);
    SqliteEngine::execSQL("UPDATE robot SET token='"+std::string(shmrobot_.robot.token)+"'");
}

void Robot::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
    Json::Reader reader;
    Json::Value root, response;
    if (msg.title == "request_login_robot") {
        reader.parse(msg.msg, root);
        doRequestLoginRobot(msg, root);
    } else if (msg.title == "request_bound_robot") {
        reader.parse(msg.msg, root);
        if (abiEnable_) {
            doRequestBoundRobotEnableAbi(msg, root);
        } else {
            doRequestBoundRobotDisableAbi(msg, root);
        }
    } else if (msg.title == "request_change_pwd") {
        reader.parse(msg.msg, root);
        doRequestChangePwd(msg, root);
    } else if (msg.title == "request_unbound_robot") {
        reader.parse(msg.msg, root);
        if (abiEnable_) {
            doRequestUnboundRobotEnableAbi(msg, root);
        } else {
            doRequestUnboundRobotDisableAbi(msg, root);
        }
    } else if (msg.title == "request_reset_robot") {
        reader.parse(msg.msg, root);
        if (abiEnable_) {
            doRequestResetRobotEnableAbi(msg, root);
        } else {
            doRequestResetRobotDisableAbi(msg, root);
        }
    } else if (msg.title == "request_time_calibration") {
        reader.parse(msg.msg, root);
        doRequestTimeCalibration(msg, root);
    } else if (msg.title == "request_bound_info") {
        reader.parse(msg.msg, root);
        doRequestBoundInfo(msg, root);
    } else if (msg.title == "request_robot_status") {
        reader.parse(msg.msg, root);
        doRequestRobotStatus(msg, root);
    } else if (msg.title == "request_set_volume") {
        reader.parse(msg.msg, root);
        doRequestSetVolume(msg, root);
    } else if (msg.title == "request_set_mute") {
        reader.parse(msg.msg, root);
        doRequestSetMute(msg, root);
    } else if (msg.title == "request_get_volume") {
        reader.parse(msg.msg, root);
        doRequestGetVolume(msg, root);
    } else if (msg.title == "request_get_mute") {
        reader.parse(msg.msg, root);
        doRequestGetMute(msg, root);
    } else if (msg.title == "request_set_play_interval") {
        reader.parse(msg.msg, root);
        doRequestSetPlayInterval(msg, root);
    } else if (msg.title == "request_get_play_interval") {
        reader.parse(msg.msg, root);
        doRequestGetPlayInterval(msg, root);
    } else if (msg.title == "request_shake_down") {
        reader.parse(msg.msg, root);
        doRequestShakeDown(msg, root);
    } else if (msg.title == "request_shake_up") {
        reader.parse(msg.msg, root);
        doRequestShakeUp(msg, root);
    } else if (msg.title == "request_shake_die") {
        reader.parse(msg.msg, root);
        doRequestShakeDie(msg, root);
    } else if (msg.title == "request_shake_status") {
        reader.parse(msg.msg, root);
        doRequestShakeStatus(msg, root);
    } else if (msg.title == "request_stop_all_task") {
        reader.parse(msg.msg, root);
        doRequestStopAllTask(msg, root);
    } else if (msg.title == "request_set_robot_sn") {
        reader.parse(msg.msg, root);
        doRequestSetRobotSn(msg, root);
    } else if (msg.title == "request_record_factory_file") {
        reader.parse(msg.msg, root);
        doRequestRecordFactoryFile(msg, root);
    } else if (msg.title == "request_play_factory_file") {
        reader.parse(msg.msg, root);
        doRequestPlayFactoryFile(msg, root);
    } else if (msg.title == "notice_voip_outgoing_call") {
        doNoticeVoipCall();
    } else if (msg.title == "request_set_voip_attributes") {
        reader.parse(msg.msg, root);
        doRequesSetVoipAttributes(msg, root);
    } else if (msg.title == "request_get_voip_attributes") {
        reader.parse(msg.msg, root);
        doRequesGetVoipAttributes(msg, root);
    } else if (msg.title == "request_hangup_voip") {
        reader.parse(msg.msg, root);
        doRequesHangupVoip(msg, root);
    } else if (msg.title == "request_set_tts_language") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "fail_invalid_data";
        if (!root["content"]["language"].isNull()) {
          int lng = root["content"]["language"].asInt();
          if (lng >= 0 && lng < TTSStrings::TTS_LANGUAGE_MAX) {
              response["result"] = "success";
              shmrobot_.appdata.tts_lng = lng;
              shm::iMemory_write_Robot(&shmrobot_);
              std::stringstream ss; ss << lng;
              SqliteEngine::execSQL("UPDATE appdata SET tts_lng="+ss.str());
          }
        }
        Utils::get_instance()->responseResult(msg, response, "response_set_tts_language");
    } else if (msg.title == "request_get_tts_language") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["language"] = shmrobot_.appdata.tts_lng;
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_get_tts_language");
    } else if (msg.title == "request_set_tts_speaker") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "fail_invalid_data";
        if (!root["content"]["speaker"].isNull()) {
            response["result"] = "success";
            shmrobot_.appdata.tts_speaker = root["content"]["speaker"].asInt();
            shm::iMemory_write_Robot(&shmrobot_);
            std::stringstream ss; ss << shmrobot_.appdata.tts_speaker;
            SqliteEngine::execSQL("UPDATE appdata SET tts_speaker="+ss.str());
        }
        Utils::get_instance()->responseResult(msg, response, "response_set_tts_speaker");
    } else if (msg.title == "request_get_tts_speaker") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["speaker"] = shmrobot_.appdata.tts_speaker;
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_get_tts_speaker");
    }  else if (msg.title == "request_tts_enable") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "fail_invalid_data";
        if (!root["content"]["enable"].isNull()) {
            response["result"] = "success";
            shmrobot_.appdata.tts_enable = root["content"]["enable"].asInt();
            shm::iMemory_write_Robot(&shmrobot_);
            std::stringstream ss; ss << shmrobot_.appdata.tts_enable;
            SqliteEngine::execSQL("UPDATE appdata SET tts_enable="+ss.str());
        }
        Utils::get_instance()->responseResult(msg, response, "response_tts_enable");
    } else if (msg.title == "request_tts_enable_status") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["enable"] = shmrobot_.appdata.tts_enable;
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_tts_enable_status");
    } else if (msg.title == "request_set_speed") {
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["result"] = "fail_invalid_data";
        if(!root["content"]["speed"].isNull()) {
            response["result"] = "success";
            shmrobot_.appdata.chassis_speed = root["content"]["speed"].asDouble();
            shm::iMemory_write_Robot(&shmrobot_);
            std::stringstream ss; ss << shmrobot_.appdata.chassis_speed;
            SqliteEngine::execSQL("UPDATE appdata SET speed="+ss.str());
        }
        Utils::get_instance()->responseResult(msg, response, "response_set_speed");
    } else if (msg.title == "request_get_speed"){
        reader.parse(msg.msg, root);
        response["id"] = root["content"]["id"];
        response["timestamp"] = root["content"]["timestamp"];
        response["speed"] = shmrobot_.appdata.chassis_speed;
        response["result"] = "success";
        Utils::get_instance()->responseResult(msg, response, "response_get_speed");
    } else if (msg.title == "notice_voip_call_established") {
        Voip::get_instance()->SetCallingStatus(true);
    } else if (msg.title == "notice_voip_call_terminated") {
        Voip::get_instance()->SetCallingStatus(false);
    } else if (msg.title == "request_get_robot_state") {
        reader.parse(msg.msg, root);
        doRequestGetRobotState(msg, root);
    } else if (msg.title == "request_change_robot_state") {
        reader.parse(msg.msg, root);
        doRequestChangeRobotState(msg, root);
    }

}

void Robot::covertAbiHttpErrToFailString(int abiHttpErr, std::string &string)
{
    if ((abiHttpErr == ABI_HTTP_ERR_NO_REOPONSE) ||
        (abiHttpErr == ABI_HTTP_ERR_INVALID_JSON_STRING)) {
        string = "fail_abi_inner_service error";
    } else if (abiHttpErr == ABI_HTTP_ERR_EXCEPTION_ACCOUNT) {
        string = "fail_abi_exception_account";
    } else if (abiHttpErr == ABI_HTTP_ERR_INVALID_ACCOUNT_PWD) {
        string = "fail_abi_invalid_account_or_pwd";
    } else if (abiHttpErr == ABI_HTTP_ERR_INVALID_TOKEN) {
        string = "fail_abi_invalid_token";
    } else if (abiHttpErr == ABI_HTTP_ERR_INVALID_BIND_USER) {
        string = "fail_abi_invalid_bind_user";
    } else if (abiHttpErr == ABI_HTTP_ERR_INVALID_ROBOT_SN) {
        string = "fail_abi_invalid_robot_sn";
    } else if (abiHttpErr == ABI_HTTP_ERR_ROBOT_IS_BINDED) {
        string = "fail_other_bound";
    } else if (abiHttpErr == ABI_HTTP_ERR_USER_HAS_NO_RIGHT) {
        string = "fail_abi_usr_has_no_right";
    } else if (abiHttpErr == ABI_HTTP_ERR_AUTHENTICATION) {
        string = "fail_abi_authentication";
    } else if (abiHttpErr == ABI_HTTP_ERR_INTERNAL_ERROR) {
        string = "fail_abi_internal_error";
    } else {
        string = "fail_abi_unexpected_error";
    }
}
