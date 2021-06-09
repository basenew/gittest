#include "abi.h"

#include <time.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "database/sqliteengine.h"
#include "utils/utils.h"
#include "md5/md5.h"
#include "log/log.h"
#include "config/config.h"
#include "imemory/atris_imemory_api.h"

#define TIMEOUTSEC       15
#define RANDSTRINGLENGTH 11

#define ABIOFFLINE_TABLE_COLUMN 4

#define ABIUSER_TABLE "CREATE TABLE IF NOT EXISTS [abiuser] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[account] TEXT NOT NULL," \
        "[password] TEXT NOT NULL," \
        "[salt] TEXT NOT NULL," \
        "[role] TEXT NOT NULL)" 

#define ABIOFFLINE_TABLE "CREATE TABLE IF NOT EXISTS [abioffline] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[token] TEXT NOT NULL," \
        "[company_id] TEXT NOT NULL," \
        "[offlineLogined] INTEGER)"

boost::mutex Abi::instanceMutex_;
boost::mutex Abi::userTableMutex_;

Abi* Abi::sInstance_ = NULL;

Abi::Abi(const std::string & robotSn)
    : deviceId_(robotSn),
      abiConnected_(false),
      start_thread_(NULL),
      abiOfflineToken_("abitoken"),
      offlineLogined_(false)
{
    
    diag_info_srv_client_ = nh_.serviceClient<atris_msgs::GetDiagnosticsInfo>(SRV_GET_DIAGNOSTICS_INFO);
    
    abiId_ = Config::get_instance()->abi_id;
    abiKey_ = Config::get_instance()->abi_key;
    abiVersion_ = Config::get_instance()->abi_version;
    baseUrl_ = Config::get_instance()->base_url;
    timestampUrl_ = Config::get_instance()->timestamp_url;
    robotInfoUrl_ = Config::get_instance()->robotinfo_url;
    userListSec_ = Config::get_instance()->abi_get_user_list_period_sec;
    timestampSec_ = Config::get_instance()->abi_get_timestamp_period_sec;
    superName_ = Config::get_instance()->abi_super_name;
    superPwd_ = Config::get_instance()->abi_super_pwd;
    
    snprintf(abiCompanyId_.company_id, sizeof(abiCompanyId_.company_id), "%s", "UBT_ATRIS");
    
    initAbiOfflineTable();
    
    shm::iMemory_write_CompanyId(&abiCompanyId_);
    
    SqliteEngine::execSQL(ABIUSER_TABLE, SQLITE_ABI_FILE);
    start_thread_ = new boost::thread(boost::bind(&Abi::startThread, this));
}

Abi::~Abi()
{
    if (start_thread_) {
        start_thread_->interrupt();
        start_thread_->join();
        delete start_thread_;
    }
}

void Abi::startThread() 
{
    std::vector<abiUser> user;
    int getUserInterval = 0;
    int getTimestampInterval = 0;
    int postRobotInfoInterval = 10;
    int running = 1;
    int i = 0;

    while (running == 1) {
        if ((getUserInterval % userListSec_) == 0) {
            getUserInterval = 0;
            int ret = getUserList(user);
            if (ret == 0) {
                updateABIUserTable(user);
            } else {
                log_info("get user list error is %d", ret);
            }
        }
        getUserInterval++;

        if ((getTimestampInterval % timestampSec_) == 0) {
            getTimestampInterval = 0;
            if (getTimestamp() == 0) {
                abiConnected_ = true;
            } else {
                abiConnected_ = false;
            }
        }
        getTimestampInterval++;

        if ((postRobotInfoInterval % 30) == 0) {
            postRobotInfoInterval = 0;
            postRobotInfo();
        }
        postRobotInfoInterval++;
        sleep(1);
    }
}

void Abi::test()
{
    std::vector<abiUser> user;
    if (getUserList(user) == 0) {
        updateABIUserTable(user);
    }

    std::string bindUser = "hcq";
    std::string pwd = "89e89c17f877ca2821b557f633cec3253b0aa941";
    if (isSuperUser(bindUser, pwd)) {
        std::cout << "find right user" << std::endl;
    } else {
        std::cout << "can not find right user" << std::endl;
    }

    std::cout << "operateType is " << OPERATETYPERESET << std::endl;
    if (postRequest(bindUser, pwd, OPERATETYPERESET) == 0) {
        std::cout << "postBindRequest success" << std::endl;
    }

    std::cout << "operateType is " << OPERATETYPEBIND << std::endl;
    if (postRequest(bindUser, pwd, OPERATETYPEBIND) == 0) {
        std::cout << "####Bind success##" << std::endl;
    } else {
         std::cout << "###post Bind failed######" << std::endl;
    }

    std::cout << "operateType is " << OPERATETYPEUNBIND << std::endl;
    if (postRequest(bindUser, pwd, OPERATETYPEUNBIND) == 0) {
        std::cout << "####post unBind success" << std::endl;
    } else {
         std::cout << "post unBind failed######" << std::endl;
    }

    if (getTimestamp() == 0) {
        std::cout << "getTimestamp success" << std::endl;
    } else {
        std::cout << "getTimestamp failed" << std::endl;
    }
    std::string abiSign;
    getAbiSign(abiSign);
    std::cout << abiSign << std::endl;
    std::cout << deviceId_ << std::endl;

    updateAbiOffLineToken();
    std::cout << "set token is " << abiOfflineToken_ << std::endl;
}

void Abi::createInstance(const std::string & robotSn)
{
    if (!sInstance_) {
        boost::unique_lock<boost::mutex> lock(instanceMutex_);
        if (!sInstance_) {
            sInstance_ = new Abi(robotSn);
        }
    }
}

Abi* Abi::getInstance()
{
    assert(sInstance_);
    return sInstance_;
}

void Abi::deleteInstance() 
{
    if (sInstance_ != nullptr) {
        delete sInstance_;
        sInstance_ = nullptr;
    }
}

void Abi::updateABIUserTable(const std::vector<abiUser> &user) 
{
    int userNum = user.size();
    if (userNum <= 0) {
        return;
    }

    boost::unique_lock<boost::mutex> lock(userTableMutex_);
    SqliteEngine::execSQL("DELETE FROM abiuser", SQLITE_ABI_FILE);
    for (int i = 0; i < userNum; i++) {
        SqliteEngine::execSQL("INSERT INTO abiuser(account, password, salt, role) VALUES('"
                +user[i].account+"', '"
                +user[i].password+"', '"
                +user[i].salt+"', '"
                +user[i].role+"')",
                SQLITE_ABI_FILE);
    }
}

bool Abi::isRightUserInfo(const std::string &account, const std::string &pwd)
{
    char **result;
    int row = 0;
    int column = 0;
    bool isRightInfo = false;

    if ((account != "") && (strcasecmp(account.c_str(), superName_.c_str()) == 0) &&
        (pwd != "")     && (strcasecmp(pwd.c_str(), superPwd_.c_str()) == 0)) {
        return true;
    }

    SqliteEngine::query("SELECT * FROM abiuser WHERE account='"+account+"'",
                        &result, &row, &column, SQLITE_ABI_FILE);
    if (row > 0) {
        const std::string salt = result[column+3];
        std::string stringCalcMd5 = pwd + salt;

        CMD5 md5obj;
        md5obj.GenerateMD5((unsigned char*)stringCalcMd5.c_str(), stringCalcMd5.size());
        std::string stringMd5 = md5obj.ToString();
        std::string password = result[column+2];
        if (strcasecmp(stringMd5.c_str(), password.c_str()) == 0) {
            isRightInfo = true;
        } else {
            isRightInfo = false;
        }
    } else {
        isRightInfo = false;
    }
    
    SqliteEngine::freeQuery(result);
    return isRightInfo;
}

bool Abi::isSuperUser(const std::string &account, const std::string &pwd)
{
    char **result;
    int row = 0;
    int column = 0;
    bool isSuperUser = false;


    if ((account != "") && (strcasecmp(account.c_str(), superName_.c_str()) == 0) &&
        (pwd != "")     && (strcasecmp(pwd.c_str(), superPwd_.c_str()) == 0)) {
        return true;
    }

    if (isRightUserInfo(account, pwd)) {
        SqliteEngine::query("SELECT * FROM abiuser WHERE account='"+account+"'",
                            &result, &row, &column, SQLITE_ABI_FILE);
        if (row > 0) {
            std::string role = result[column+4];
            if (role == "0") {
                isSuperUser = true;
            } else {
                isSuperUser = false;
            }
        } else {
            isSuperUser = false;
        }

        SqliteEngine::freeQuery(result);
    } else {
        isSuperUser = false;
    }

    return isSuperUser;
}

void Abi::getRandString(char *str)
{ 
    if (str == NULL) {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    int randNum =  tv.tv_sec + tv.tv_usec;
    srand(randNum);

    int len = rand() % 10 + 1;
    for (int i = 0;i < len; i++) {
        switch((rand()%3)) {
        case 1:
            str[i] = 'A' + rand() % 26;
            break;
        case 2:
            str[i]= 'a' + rand() % 26;
            break;
        default:
            str[i]= '0' + rand() % 10;
            break;
        }
    }
}

void Abi::getAbiSign(std::string &abiSign)
{
    std::string deviceId  = deviceId_;
    std::string abiKey = abiKey_;
    std::string abiVersionNum = abiVersion_;
    std::string bankSpace = " ";

    ros::Time timeNow = ros::Time::now();
    std::stringstream streamNow;
    streamNow  << (uint64_t)(timeNow.toSec());
    std::string stringNow = streamNow.str();

    char randStr[RANDSTRINGLENGTH] = {'\0'};
    getRandString(randStr);
    std::string stringRand = randStr;

    std::string stringCalcMd5;
    stringCalcMd5 = stringNow + abiKey + stringRand + deviceId;

    CMD5 md5obj; 
    md5obj.GenerateMD5((unsigned char*)stringCalcMd5.c_str(), stringCalcMd5.size());
    std::string stringMd5 = md5obj.ToString();


    abiSign = stringMd5 + bankSpace + stringNow + bankSpace + 
              stringRand + bankSpace + abiVersionNum;

    // std::cout << "abiSign is "  << abiSign.c_str() << std::endl;
}

void Abi::getAbiHttpHeader(std::vector<std::string> &httpHeader)
{
    std::string abiSignString = "X-UBT-Sign";
    std::string abiIdString = "X-UBT-AppId";
    std::string deviceIdString = "X-UBT-DeviceId";
    std::string colonString = ": ";
    std::string deviceId  = deviceId_;
    std::string abiId     = abiId_;

    std::string abiSign;
    getAbiSign(abiSign);
    std::string headerAbiSign;
    headerAbiSign = abiSignString + colonString + abiSign;
    httpHeader.push_back(headerAbiSign);

    std::string headerAbiId;
    headerAbiId = abiIdString + colonString + abiId;
    httpHeader.push_back(headerAbiId);

    std::string headerDeviceId;
    headerDeviceId = deviceIdString + colonString + deviceId;
    httpHeader.push_back(headerDeviceId);
}

int  Abi::getUserList(std::vector<abiUser> &user)
{
    std::string url = baseUrl_ + "/user?" + "robotSn=" + deviceId_;
    std::vector<std::string> httpHeader;
    int result = 0;

    Utils *utils = Utils::get_instance();
    std::string resp;

    static bool hasGotCompanyId = false;
    int times = 3;
    while (times > 0) {
        times --;
        httpHeader.clear();
        getAbiHttpHeader(httpHeader);
        int ret = utils->http_get(url, resp, httpHeader, TIMEOUTSEC);
        if (ret == 0) {
            Json::Reader reader;
            Json::Value root;
            if (reader.parse(resp.c_str(), root)) {
                std::string code = root["code"].isNull() ? "" : root["code"].asString();
                std::string msg  = root["msg"].isNull() ? "" : root["msg"].asString();
                if ((code == "200") && (msg == "success")) {
                    if (hasGotCompanyId == false) {
                        std::string company_id = root["data"]["enterpriseNum"].isNull() ? "" : root["data"]["enterpriseNum"].asString();
                        if (company_id != "") {
                            snprintf(abiCompanyId_.company_id, sizeof(abiCompanyId_.company_id), "%s", company_id.c_str());
                            shm::iMemory_write_CompanyId(&abiCompanyId_);
                            
                            SqliteEngine::execSQL("UPDATE abioffline SET company_id='"+company_id+"'", SQLITE_ABI_FILE);
                            hasGotCompanyId = true;
                        }
                    }

                    std::string bindUser = root["data"]["bindUser"].isNull() ? "" :
                                           root["data"]["bindUser"].asString();
                    Json::Value array = root["data"]["user"];
                    abiUser abiUser;

                    for (unsigned int i = 0; i < array.size(); i++) {
                        abiUser.account = array[i]["account"].isNull() ? "" : array[i]["account"].asString();
                        abiUser.password = array[i]["password"].isNull() ? "" : array[i]["password"].asString();
                        abiUser.salt = array[i]["salt"].isNull() ? "" : array[i]["salt"].asString();
                        abiUser.role = array[i]["role"].isNull() ? "" : array[i]["role"].asString();
                        user.push_back(abiUser);
                    }
                    result =  0;
                } else {
                    result =  atoi(code.c_str()); // 400x
                }
            } else {
                result = ABI_HTTP_ERR_INVALID_JSON_STRING;
            }
        } else {
            result = ABI_HTTP_ERR_NO_REOPONSE;
        }

        if (result >= 0) {
            break;
        }
    }

    return result;
}

int Abi::postRequest(const std::string &account, const std::string &pwd, int operateType)
{
    int result = -1;
    std::string url = baseUrl_ + "/bind";
    std::vector<std::string> httpHeader;
    
    Utils *utils = Utils::get_instance();
    std::string resp;
    Json::Reader reader;
    Json::Value root;
    root["bindUser"] = Json::Value(account);
    root["operateType"] = Json::Value(std::to_string(operateType));
    root["robotSn"] = Json::Value(deviceId_);
    root["uesrPwd"] = Json::Value(pwd);

    Json::FastWriter fw;
    std::string param = fw.write(root);

    int times = 3;
    while (times > 0) {
        times --;
        httpHeader.clear();
        httpHeader.push_back("Content-Type:application/json");
        httpHeader.push_back("Accept: application/json");
        getAbiHttpHeader(httpHeader);
        int ret = utils->http_post(url, param, resp, httpHeader, TIMEOUTSEC);
        if (ret == 0) {
            if (reader.parse(resp.c_str(), root)) {
                std::string code = root["code"].isNull() ? "" : root["code"].asString();
                std::string msg  = root["msg"].isNull() ? "" : root["msg"].asString();
                if ((code == "200") && (msg == "success")) {
                    result =  0;
                } else {
                    result =  atoi(code.c_str()); // 400x
                }
            } else {
                result = ABI_HTTP_ERR_INVALID_JSON_STRING;
            }
        } else {
            result = ABI_HTTP_ERR_NO_REOPONSE;
        }

        if (result >= 0) {
            break;
        }
    }

    return result;
}

int Abi::getTimestamp()
{
    int result = -1;
    std::string url = timestampUrl_;
    Utils *utils = Utils::get_instance();
    std::string resp;
    std::vector<std::string> header;

    int times = 3;
    while (times > 0) {
        times --;
        int ret = utils->http_get(url, resp, header, TIMEOUTSEC);
        if (ret == 0) {
            Json::Reader reader;
            Json::Value root;
            if (reader.parse(resp.c_str(), root)) {
                std::string timestamp = root["timestamp"].asString();
                result = 0;
            } else {
                result = -1;
            }
        } else {
            result = -1;
        }

        if (result == 0) {
            break;
        }
    }

    return result;
}

void Abi::updateAbiOffLineToken()
{
    boost::uuids::uuid a_uuid = boost::uuids::random_generator()();
    abiOfflineToken_ = boost::uuids::to_string(a_uuid);
    SqliteEngine::execSQL("UPDATE abioffline SET token='"+abiOfflineToken_+"'", SQLITE_ABI_FILE);
}

void Abi::updateAbiofflineLogined(bool offlineLogined)
{
    offlineLogined_ = offlineLogined;
    std::stringstream sofflineLogined;
    sofflineLogined << (offlineLogined_ ? 1 : 0);
    SqliteEngine::execSQL("UPDATE abioffline SET offlineLogined="+sofflineLogined.str(), SQLITE_ABI_FILE);
}

void Abi::initAbiOfflineTable() 
{
    char **result;
    int row, column;

    retry:
    SqliteEngine::execSQL(ABIOFFLINE_TABLE, SQLITE_ABI_FILE);
    SqliteEngine::query("SELECT * FROM abioffline", &result, &row, &column, SQLITE_ABI_FILE);
    std::stringstream sofflineLogined;
    sofflineLogined << (offlineLogined_ ? 1 : 0);
    if (row <= 0) {
        SqliteEngine::execSQL("INSERT INTO abioffline(token, company_id, offlineLogined) VALUES('"
                        +abiOfflineToken_+"', '"
                        +std::string(abiCompanyId_.company_id)+"', '"
                        +sofflineLogined.str()+"')",
                        SQLITE_ABI_FILE);
    } else {
        if (column != ABIOFFLINE_TABLE_COLUMN) {
            SqliteEngine::execSQL("DROP TABLE abioffline", SQLITE_ABI_FILE);
            goto retry;
        } else {
            abiOfflineToken_ = result[column + 1];
            snprintf(abiCompanyId_.company_id, sizeof(abiCompanyId_.company_id), "%s", result[column + 2]);
            offlineLogined_ = result[column + 3];
        }
    }

    SqliteEngine::freeQuery(result);
}

void Abi::postRobotInfo()
{
    Json::Reader reader;
    Json::Value robotInfo, diag_info;
    atris_msgs::GetDiagnosticsInfo diag;
    if (diag_info_srv_client_.call(diag) && !diag.response.data.empty()) {
        if (!reader.parse(diag.response.data, diag_info)) {
            log_warn("%s RobotInfo parse fail.", __FUNCTION__);
            return;
        }
    } else {
        log_warn("%s diag_info_srv_client call fail.", __FUNCTION__);
        return;
    }

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    robotInfo["title"] = "response_robot_info";
    robotInfo["accid"] = shmrbt.robot.sn;
    robotInfo["content"]= diag_info;
    Json::FastWriter jwriter;
    std::string content = jwriter.write(robotInfo);
    std::string url = robotInfoUrl_;
    std::string resp;
    std::vector<std::string> httpHeader;
    Json::Value root;
    int result = 0;
    int times = 3;
    unsigned int i = 0;

    while (times > 0) {
        times --;
        httpHeader.clear();
        httpHeader.push_back("Content-Type:application/json");
        httpHeader.push_back("Accept: application/json");
        getAbiHttpHeader(httpHeader);
        getPostRobotInfoHttpHeader(httpHeader);
        int ret = Utils::get_instance()->http_post(url, content, resp, httpHeader, TIMEOUTSEC);
        if (ret == 0) {
            if (reader.parse(resp.c_str(), root)) {
                std::string code = root["code"].isNull() ? "" : root["code"].asString();
                std::string msg  = root["msg"].isNull() ? "" : root["msg"].asString();
                if ((code == "200") && (msg == "success")) {
                    result =  0;
                } else {
                    result =  atoi(code.c_str()); // 400x
                    log_info("post robot info error code is %d", result);
                }
            } else {
                result = ABI_HTTP_ERR_INVALID_JSON_STRING;
            }
        } else {
            result = ABI_HTTP_ERR_NO_REOPONSE;
        }

        if (result >= 0) {
            break;
        }
    }
}

void Abi::getPostRobotInfoHttpHeader(std::vector<std::string> &httpHeader)
{
    std::string modelName = "atris";
    std::string snString = "sn";
    std::string colonString = ": ";
    std::string deviceId  = deviceId_;

    std::string headerCollectionName;
    headerCollectionName = "collectionName" + colonString + modelName;
    httpHeader.push_back(headerCollectionName);

    std::string headerDeviceId;
    headerDeviceId = snString + colonString + deviceId;
    httpHeader.push_back(headerDeviceId);
}