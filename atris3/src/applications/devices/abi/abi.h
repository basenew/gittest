#ifndef ABI_H_
#define ABI_H_

#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <json/json.h>
#include "ros/ros.h"
#include "imemory/atris_imemory_api.h"
#include "atris_defines.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/GetVoiceChatStatus.h"
#include "atris_msgs/GetLampStatus.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/GetDisperseStatus.h"
#include "atris_msgs/GetPatrolStatus.h"
#include "atris_msgs/GetPtzStatus.h"
#include "atris_msgs/GetSwVersion.h"
#include "atris_msgs/GetChassisInfo.h"
#include "atris_msgs/GetDiagnosticsInfo.h"

enum {
    OPERATETYPEBIND = 1,
    OPERATETYPEUNBIND,
    OPERATETYPERESET
};

struct abiUser {
    std::string account;
    std::string password;
    std::string salt;
    std::string role;
};

#define ABI_HTTP_ERR_NO_REOPONSE              -1
#define ABI_HTTP_ERR_INVALID_JSON_STRING      -2
#define ABI_HTTP_ERR_AUTHENTICATION           1003
#define ABI_HTTP_ERR_INTERNAL_ERROR           1007
#define ABI_HTTP_ERR_EXCEPTION_ACCOUNT        4000
#define ABI_HTTP_ERR_INVALID_ACCOUNT_PWD      4001
#define ABI_HTTP_ERR_INVALID_TOKEN            4002
#define ABI_HTTP_ERR_INVALID_BIND_USER        4003
#define ABI_HTTP_ERR_INVALID_ROBOT_SN         4004
#define ABI_HTTP_ERR_ROBOT_IS_BINDED          4005
#define ABI_HTTP_ERR_USER_HAS_NO_RIGHT        4006

class Abi
{
public:
    virtual ~Abi();
    static void createInstance(const std::string & robotSn);
    static void deleteInstance();
    static Abi* getInstance();
    void test();
    int postRequest(const std::string &account, const std::string &pwd, int operateType);
    bool isRightUserInfo(const std::string &account, const std::string &pwd);
    bool isSuperUser(const std::string &account, const std::string &pwd);
    bool getAbiIsConnected() {return abiConnected_;}
    void updateAbiOffLineToken();
    std::string getAbiOffLineToken() {return abiOfflineToken_;}
    std::string getAbiCompanyId() {return std::string(abiCompanyId_.company_id);}
    void updateAbiofflineLogined(bool offlineLogined);
    bool isofflineLogined() {return offlineLogined_;}

private:
    Abi(const std::string & robotSn);
    void startThread();
    int getUserList(std::vector<abiUser> &user);
    int getTimestamp();
    void updateABIUserTable(const std::vector<abiUser> &user);
    void getRandString(char *str);
    void getAbiSign(std::string &abiSign);
    void getAbiHttpHeader(std::vector<std::string> &httpHeader);
    void initAbiOfflineTable();
    void getPostRobotInfoHttpHeader(std::vector<std::string> &httpHeader);
    void postRobotInfo();

private:
    static boost::mutex instanceMutex_;
    static boost::mutex userTableMutex_;
    static Abi * sInstance_;
    std::string deviceId_;
    std::string abiId_;
    std::string abiKey_;
    std::string abiVersion_;
    std::string baseUrl_;
    std::string timestampUrl_;
    std::string robotInfoUrl_;
    int userListSec_;
    int timestampSec_;
    bool abiConnected_;
    boost::thread* start_thread_;
    std::string abiOfflineToken_;
    bool offlineLogined_;

    std::string superName_;
    std::string superPwd_;

    Json::Value robotInfo_;
    shm::CompanyId abiCompanyId_;
    ros::NodeHandle nh_;
    ros::ServiceClient diag_info_srv_client_;
};
#endif