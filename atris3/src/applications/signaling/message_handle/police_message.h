#ifndef MESSAGEHANDLE_H_
#define MESSAGEHANDLE_H_

#include <string>
#include <json/json.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include "log/log.h"
#include "libxml/xml.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/NavToPoint.h"
#include "atris_msgs/GetNavPointList.h"

class WsClient;

class MessageHandle {
public:
    virtual ~MessageHandle();
    static void createInstance();
    static void deleteInstance();
    static MessageHandle* getInstance();

private:
    MessageHandle();
    void initReqTypeResTitleMap();
    void findResTitleFromReqType(const std::string &reqType, std::string &resTitle);
    
    void wsXmlClientConnect();
    void wsJsonClientConnect();
    void parseXmlMessage(const std::string &string);
    void parseJsonMessage(const std::string &string);
    void messageResponse(const atris_msgs::SignalMessage& msg);
    int convertXmlStringToRobotJson(const std::string &xmlString,
                                    std::string &title, Json::Value &root);
    int convertJsonStringToRobotJson(const std::string &jsonString,
                                     std::string &title, Json::Value &root);
    void addHeadNodeXml(std::string &xmlString, const std::string &parNodeName);
    void addBodyNodeXml(std::string &xmlString, const std::string &parNodeName, 
                        const std::string &jsonString);
    void createResJson(std::string &jsonString, const std::string &message);
    bool sendMessPlayMusic();
    bool sendMessStopMusic();
    int diffTimeFromNow(const std::string &dateTime);
    std::string timestampToDate(time_t time);
    void initNewsParamsTable();

private:
    static boost::mutex instanceMutex_;
    static MessageHandle * sInstance_;

    WsClient *wsXmlClient_;
    WsClient *wsJsonClient_;

    std::string robotid_;
    std::string userid_;
    std::string id_;
    std::string xmlTimestamp_;
    std::string xmlRequestType_;
    std::string jsonRequestType_;
    std::string navType_;
    std::string cameraId_;
    std::string newTaskId_;
    std::string ruleType_;
    std::string beginMusicDateTime_;
    std::string endMusicDateTime_;
    std::string musicTimeList_;
    
    String_Map reqTypeRespTitleMap_;
    Json::Value playMusicValue_;
    ros::NodeHandle nh_;
    ros::Publisher signal_req_pub_;
    ros::Subscriber signal_resp_sub_;
    ros::ServiceClient nav_to_point_srv_client_;
    ros::ServiceClient get_nav_point_list_srv_client_;
};

#endif