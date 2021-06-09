#include "police_message.h"

#include <libwebsockets.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include "wsclient/wsclient.h"

#include "config/config.h"
#include "task_manager/task_manager.h"
#include "database/sqliteengine.h"

#define WS_XML_PROTOCOL_NAME    ("ws")
#define WS_JSON_PROTOCOL_NAME    ("ws")

#define SPECIAL_BROADCAST_NEWS_TASK   ("0")
#define RECYCLE_BROADCAST_NEWS_TASK   ("1")
#define TIMEING_BROADCAST_NEWS_TASK   ("2")
#define LOCATION_BROADCAST_NEWS_TASK  ("3")

#ifdef _CHASSIS_JC_
    #define DEFAULT_FOAWARD_SPEED            (0.5)
    #define DEFAULT_BACKWARD_SPEED           (0.3)
    #define DEFAULT_LEFT_SPEED               (0.2)
    #define DEFAULT_FOAWARD_LEFT_SPEED       (0.0)
    #define DEFAULT_RIGHT_SPEED              (0.2)
    #define DEFAULT_FOAWARD_RIGHT_SPEED      (0.0)
#else
    #define DEFAULT_FOAWARD_SPEED            (1.0)
    #define DEFAULT_BACKWARD_SPEED           (0.6)
    #define DEFAULT_LEFT_SPEED               (0.2)
    #define DEFAULT_FOAWARD_LEFT_SPEED       (0.5)
    #define DEFAULT_RIGHT_SPEED              (0.2)
    #define DEFAULT_FOAWARD_RIGHT_SPEED      (0.5)
#endif

#define NEWSPARAMS_TABLE_COLUMN  7
#define NEWSPARAMS_TABLE "CREATE TABLE IF NOT EXISTS [newsparams] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[fileurl] TEXT," \
        "[ruletype] TEXT," \
        "[playmusicjsonvalue] TEXT," \
        "[begintime] TEXT," \
        "[endtime] TEXT," \
        "[timelist] TEXT)"

boost::mutex MessageHandle::instanceMutex_;
MessageHandle* MessageHandle::sInstance_ = NULL;

MessageHandle::MessageHandle()
  : wsXmlClient_(new WsClient()),
    wsJsonClient_(new WsClient()),
    robotid_(""),
    userid_(""),
    id_(""),
    xmlTimestamp_(""),
    xmlRequestType_(""),
    jsonRequestType_(""),
    cameraId_(""),
    newTaskId_(""),
    ruleType_(""),
    beginMusicDateTime_(""),
    musicTimeList_("") {
    signal_resp_sub_ = nh_.subscribe(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100, &MessageHandle::messageResponse, this);
    signal_req_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
    nav_to_point_srv_client_ = nh_.serviceClient<atris_msgs::NavToPoint>(SRV_NAV_TO_POINT);
    get_nav_point_list_srv_client_ = nh_.serviceClient<atris_msgs::GetNavPointList>(SRV_GET_NAV_POINT_LIST);
    wsXmlClientConnect();
    wsJsonClientConnect();
    initReqTypeResTitleMap();
    initNewsParamsTable();
}

MessageHandle::~MessageHandle()
{
    if (wsXmlClient_) {
        delete wsXmlClient_;
        wsXmlClient_ = nullptr;
    }
    if (wsJsonClient_) {
        delete wsJsonClient_;
        wsJsonClient_ = nullptr;
    }
}

void MessageHandle::createInstance()
{
    if (!sInstance_) {
        boost::unique_lock<boost::mutex> lock(instanceMutex_);
        if (!sInstance_) {
            sInstance_ = new MessageHandle();
        }
    }
}

MessageHandle* MessageHandle::getInstance()
{
    assert(sInstance_);
    return sInstance_;
}

void MessageHandle::deleteInstance() 
{
    if (sInstance_ != NULL) {
        delete sInstance_;
        sInstance_ = NULL;
    }
}

void MessageHandle::initReqTypeResTitleMap()
{
    reqTypeRespTitleMap_.insert(String_Pair("immediaterecharge", "response_recharge"));
    reqTypeRespTitleMap_.insert(String_Pair("stoprecharge", "response_recharge"));
    reqTypeRespTitleMap_.insert(String_Pair("move", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("stopmove", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("movehead", "response_move_ptz"));
    reqTypeRespTitleMap_.insert(String_Pair("resethead", "response_reset_ptz"));
    reqTypeRespTitleMap_.insert(String_Pair("floodlight", "response_switch_light"));
    reqTypeRespTitleMap_.insert(String_Pair("flashlight", "response_switch_light"));
    reqTypeRespTitleMap_.insert(String_Pair("disperse", "response_sonic_disperse"));
    reqTypeRespTitleMap_.insert(String_Pair("mediavolume", "response_set_volume"));
    reqTypeRespTitleMap_.insert(String_Pair("stopAllTask", "response_stop_all_task"));
    reqTypeRespTitleMap_.insert(String_Pair("setrobotlocation", "response_nav_to"));
    reqTypeRespTitleMap_.insert(String_Pair("broadcast_new", "response_music_transport"));
    reqTypeRespTitleMap_.insert(String_Pair("broadcast_action", "response_music_play"));

    reqTypeRespTitleMap_.insert(String_Pair("forward", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("backward", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("left", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("right", "response_robot_move"));
    reqTypeRespTitleMap_.insert(String_Pair("stop", "response_robot_move"));
}

void  MessageHandle::findResTitleFromReqType(const std::string &reqType, std::string &resTitle)
{
    resTitle = "title";
    String_Map::iterator it = reqTypeRespTitleMap_.find(reqType);
    if (it != reqTypeRespTitleMap_.end()) {
        resTitle =  it->second;
    }
}

void MessageHandle::wsXmlClientConnect()
{
    wsClientParam param;
    param.path = Config::get_instance()->ws_xml_path;
    param.protocolsName = WS_XML_PROTOCOL_NAME;
    param.addr = Config::get_instance()->ws_xml_ip;
    param.port = Config::get_instance()->ws_xml_port;

    wsXmlClient_->setParams(param);
    wsXmlClient_->setSsl(false);
    wsXmlClient_->setReadCallback(boost::bind(&MessageHandle::parseXmlMessage, this, _1));
    wsXmlClient_->start();
}

void MessageHandle::wsJsonClientConnect()
{
    wsClientParam param;
    param.path = Config::get_instance()->ws_json_path;
    param.protocolsName = WS_JSON_PROTOCOL_NAME;
    param.addr = Config::get_instance()->ws_json_ip;
    param.port = Config::get_instance()->ws_json_port;

    wsJsonClient_->setParams(param);
    wsJsonClient_->setSsl(false);
    wsJsonClient_->setReadCallback(boost::bind(&MessageHandle::parseJsonMessage, this, _1));
    wsJsonClient_->start();
}

void MessageHandle::parseXmlMessage(const std::string &string) 
{
    atris_msgs::SignalMessage message;
    message.title = "";

    Json::FastWriter jwriter;
    Json::Value root;
    std::string title;
    std::string nodeTypeString;
    Xml::queryNodeText(string, "type", nodeTypeString);
 
    int ret = convertXmlStringToRobotJson(string, title, root);
    if (ret == 0) {
        message.title = title;
        message.msg = jwriter.write(root);
        log_info("xml message is \n");
        log_info("%s\n", message.msg.c_str());
        signal_req_pub_.publish(message);
    } else {
        std::string status = "101";
        std::string result = "";

        switch(ret) {
            case 1:
                status = "100";
                result = "success";
            break;
            case -1:
                status = "101";
                result = "wait for last command response";
            break;
            case -2:
                status = "108";
                result = "do not support this command";
            break;
            case -3:
                status = "101";
                result = "command has wrong params";
            break;
            case -4:
                status = "101";
                result = "robot inner error";
            break;
            case -5:
                status = "101";
                result = "command is timeout";
            break;
            default:
                log_info("ret is %d\n", ret);
            break;
        }

        std::string xmlString;
        std::string strRootNodeName = "response";
        Xml::createBaseXml(xmlString, strRootNodeName);
        addHeadNodeXml(xmlString, strRootNodeName);

        std::string nodeBody = "body";
        Xml::addNodeText(xmlString, strRootNodeName, nodeBody, "");
        String_Map bodyMap;
        bodyMap.insert(String_Pair("type", xmlRequestType_));
        bodyMap.insert(String_Pair("status", status));
        bodyMap.insert(String_Pair("result", result));
        String_Map::iterator iter;
        for (iter = bodyMap.begin(); iter != bodyMap.end(); iter++) {
            Xml::addNodeText(xmlString, nodeBody,
                             iter->first, iter->second);
        }
        if (nodeTypeString == "getlocationlist") {
            atris_msgs::GetNavPointList pointlist;
            get_nav_point_list_srv_client_.call(pointlist);
            
            String_Map attrMap;
            std::string location = "location";
            for (unsigned int i = 0; i < pointlist.response.points.size(); i++) {
                attrMap.clear();
                attrMap.insert(String_Pair("name", pointlist.response.points[i].name));
                attrMap.insert(String_Pair("x", std::to_string(pointlist.response.points[i].x)));
                attrMap.insert(String_Pair("y", std::to_string(pointlist.response.points[i].y)));
                Xml::addNodeAttribute(xmlString, nodeBody, location, attrMap);
            }
        }
        wsXmlClient_->sendMessageToServer(xmlString);
        xmlRequestType_ = "";
    }
}

void MessageHandle::parseJsonMessage(const std::string &string)
{
    atris_msgs::SignalMessage message;
    message.title = "";

    Json::FastWriter jwriter;
    Json::Value root;
    std::string title;
 
    int ret = convertJsonStringToRobotJson(string, title, root);
    if (ret == 0) {
        message.title = title;
        message.msg = jwriter.write(root);
        log_info("json message is \n");
        log_info("%s\n", message.msg.c_str());
        signal_req_pub_.publish(message);
    } else {
        std::string status = "101";
        std::string result = "";

        switch(ret) {
            case -1:
                status = "101";
                result = "wait for last command response";
            break;
            case -2:
                status = "101";
                result = "command type is not supported";
                break;
            case -3:
                status = "101";
                result = "json string is not right";
                break;
            case -4:
                status = "101";
                result = "navigation is not ready";
                break;
            default:
                break;
        }

        std::string jsonString;
        Json::Value writeValue;
        writeValue["type"] = Json::Value(jsonRequestType_);
        writeValue["status"] = Json::Value(status);
        writeValue["result"] = Json::Value(result);

        ros::Time now = ros::Time::now();
        std::string time = timestampToDate(now.toSec());
        writeValue["stamp"] = Json::Value(time);

        Json::FastWriter fw;
        jsonString = fw.write(writeValue);
        wsJsonClient_->sendMessageToServer(jsonString);
        jsonRequestType_ = "";
    }
}

void MessageHandle::messageResponse(const atris_msgs::SignalMessage& msg)
{
    Json::Reader reader;
    Json::Value root;
    std::string resTitle = "";
    if (reader.parse(msg.msg, root)) {
       resTitle = root["title"].isNull() ? "" : root["title"].asString();
    }

    if (resTitle == "response_music_transport") {
        std::string result = root["content"]["result"].asString();
        int progress =  (int)root["content"]["progress"].asFloat();
        if ((result == "success") &&
            (progress == 100)) {
            Task task;
            task.cb = boost::bind(&MessageHandle::sendMessPlayMusic, this);
            if (ruleType_ == SPECIAL_BROADCAST_NEWS_TASK) {
                TaskManager::get_instance()->post(task);
            } else if (ruleType_ == RECYCLE_BROADCAST_NEWS_TASK) {
                int delaySec = diffTimeFromNow(beginMusicDateTime_);
                if (delaySec >= 0) {
                    TaskManager::get_instance()->post_delay(task, delaySec*1000);
                }
            } else if (ruleType_ == TIMEING_BROADCAST_NEWS_TASK) {
                int delaySec = 0;
                char *p = nullptr;
                const char *delim = ",";
                p = strtok((char *)musicTimeList_.c_str(), delim);
                while (p != nullptr) {
                    delaySec = diffTimeFromNow(p);
                    if (delaySec >= 0) {
                        TaskManager::get_instance()->post_delay(task, delaySec*1000);
                    }
                    p = strtok(nullptr, delim);
                }
            }
        }
    }

    if (xmlRequestType_ != "") {
        std::string title;
        findResTitleFromReqType(xmlRequestType_, title);
        if (title == resTitle) {
            std::string xmlString;
            std::string strRootNodeName = "response";
            Xml::createBaseXml(xmlString, strRootNodeName);
            addHeadNodeXml(xmlString, strRootNodeName);
            addBodyNodeXml(xmlString, strRootNodeName, msg.msg);
            log_info("send sip message is :\n");
            log_info("%s\n", xmlString.c_str());
            wsXmlClient_->sendMessageToServer(xmlString);
            xmlRequestType_ = "";
        }
    }

    if (navType_ != "") {
        std::string title;
        int state;
        if (resTitle == "response_nav_state") {
            state = root["content"]["state"].isNull() ? 0 : root["content"]["state"].asInt();
            if ((state == 302) ||
                (state == 306)) {
                std::string xmlString;
                std::string strRootNodeName = "response";
                Xml::createBaseXml(xmlString, strRootNodeName);
                addHeadNodeXml(xmlString, strRootNodeName);
                addBodyNodeXml(xmlString, strRootNodeName, msg.msg);
                wsXmlClient_->sendMessageToServer(xmlString);
                if (state == 306) {
                    navType_ = "";
                }
            }
        }
    }

    if (jsonRequestType_ != "") {
        std::string title;
        findResTitleFromReqType(jsonRequestType_, title);
        if (title == resTitle) {
            std::string jsonString;
            createResJson(jsonString, msg.msg);
            wsJsonClient_->sendMessageToServer(jsonString);
            log_info("send json string is :\n");
            log_info("%s\n", jsonString.c_str());
            jsonRequestType_ = "";
        }
    }
}

int MessageHandle::convertXmlStringToRobotJson(const std::string &xmlString, 
                                               std::string &title, Json::Value &root)
{
    if ((xmlRequestType_ != "") ||
        (jsonRequestType_ != "")) {
        return -1;
    }

    log_info("######xmlString is :\n");
    log_info("%s\n", xmlString.c_str());
    std::string nodeTypeString;
    Xml::queryNodeText(xmlString, "robotid", robotid_);
    Xml::queryNodeText(xmlString, "userid", userid_);
    Xml::queryNodeText(xmlString, "id", id_);
    Xml::queryNodeText(xmlString, "time", xmlTimestamp_);
    long long timestamp = atoll(xmlTimestamp_.c_str());
    std::string date = timestampToDate(timestamp / 1000);
    int diffTime = diffTimeFromNow(date);
    diffTime = abs(diffTime);
#if 0
    if (diffTime > 60) {
        return -5;
    }
#endif
    root["content"]["id"] = id_;
    root["content"]["timestamp"] = Json::Value(atoi(xmlTimestamp_.c_str()));;

    Xml::queryNodeText(xmlString, "type", nodeTypeString);
    if (nodeTypeString.empty()) {
        return -2;
    } 
    xmlRequestType_ = nodeTypeString;

    if (nodeTypeString == "immediaterecharge") {
        title = "request_recharge";
        root["content"]["switch"] = Json::Value(1);
        navType_ = nodeTypeString;
    } else if (nodeTypeString == "stoprecharge") {
        title = "request_recharge";
        root["content"]["switch"] = Json::Value(0);
    } else if (nodeTypeString == "move") {
        title = "request_robot_move";
        std::string angularspeed;
        Xml::queryNodeText(xmlString, "angularspeed", angularspeed);
        std::string linespeed;
        Xml::queryNodeText(xmlString, "linespeed", linespeed);
        if ((angularspeed.empty()) ||
            (linespeed.empty())) {
            return -3;
        }
        root["content"]["v_linear"] = Json::Value(atof(linespeed.c_str()));
        root["content"]["v_angular"] = Json::Value(atof(angularspeed.c_str()));
    } else if (nodeTypeString == "stopmove") {
        title = "request_robot_move";
        root["content"]["v_linear"] = Json::Value(0);
        root["content"]["v_angular"] = Json::Value(0);
    } else if (nodeTypeString == "movehead") {
        title = "request_move_ptz";
        Xml::queryNodeText(xmlString, "cameraid", cameraId_);
        std::string direction; 
        Xml::queryNodeText(xmlString, "direction", direction);
        std::string angle; 
        Xml::queryNodeText(xmlString, "angle", angle);
        if (direction.empty() || angle.empty()) {
           log_info("movehead angle is empty\n"); 
           return -3;
        }
        if (direction == "up") {
            root["content"]["verticalangel"] = Json::Value(0);
            root["content"]["horizontalangel"] = Json::Value(-atoi(angle.c_str()));
        } else if (direction == "under") {
            root["content"]["verticalangel"] = Json::Value(0);
            root["content"]["horizontalangel"] = Json::Value(atoi(angle.c_str()));
        } else if (direction == "left") {
            root["content"]["verticalangel"] = Json::Value(atoi(angle.c_str()));
            root["content"]["horizontalangel"] = Json::Value(0);
        } else if (direction == "right") {
            root["content"]["verticalangel"] = Json::Value(-atoi(angle.c_str()));
            root["content"]["horizontalangel"] = Json::Value(0);
        } else {
            log_info("direction is not right\n");
            return -3;
        }
    } else if (nodeTypeString == "resethead") {
        title = "request_reset_ptz";
        Xml::queryNodeText(xmlString, "cameraid", cameraId_);
        if (cameraId_.empty()) {
            return -3;
        }
    } else if (nodeTypeString == "floodlight") {
        title = "request_switch_light";
        std::string switchLight; 
        Xml::queryNodeText(xmlString, "switch", switchLight);
        if ((switchLight.empty())||
            ((switchLight != "0") && (switchLight != "1"))) {
            log_info("switch param is not right\n");
            return -3;
        } 
        root["content"]["switch"] = Json::Value(atoi(switchLight.c_str()));
        root["content"]["lamp"] = Json::Value(0);
    } else if (nodeTypeString == "flashlight") {
        title = "request_switch_light";
        std::string switchLight; 
        Xml::queryNodeText(xmlString, "switch", switchLight);
        if ((switchLight.empty())||
            ((switchLight != "0") && (switchLight != "1"))) {
            log_info("switch param is not right\n");
            return -3;
        } 
        root["content"]["switch"] = Json::Value(atoi(switchLight.c_str()));
        root["content"]["lamp"] = Json::Value(1);
    } else if (nodeTypeString == "disperse") {
        title = "request_sonic_disperse";
        std::string switchDisperse; 
        Xml::queryNodeText(xmlString, "switch", switchDisperse);
        if ((switchDisperse.empty())||
            ((switchDisperse != "0") && (switchDisperse != "1"))) {
            log_info("switch param is not right\n");
            return -3;
        } 
        root["content"]["switch"] = Json::Value(atoi(switchDisperse.c_str()));
    }  else if (nodeTypeString == "mediavolume") {
        title = "request_set_volume";
        std::string volume; 
        Xml::queryNodeText(xmlString, "volume", volume);
        if (volume.empty()) {
            log_info("volume is empty\n");
            return -3;
        } 
        int volumeValue = atoi(volume.c_str());
        if ((volumeValue < 0) || (volumeValue > 100)) {
            return -3;
        }
        root["content"]["volume"] = Json::Value(volumeValue);
    } else if (nodeTypeString == "stopAllTask") {
        title = "request_stop_all_task";
    } else if (nodeTypeString == "setrobotlocation") {
        navType_ = nodeTypeString;

        unsigned int i = 0;
        std::string para = "";
        int xPos = 0;
        int yPos = 0;
        Xml::queryNodeText(xmlString, "para", para);
        log_info("para is %s", para.c_str());
        char *p = nullptr;
        const char *delim = ",";
        std::string::size_type idx = para.find(delim);
        if (idx != std::string::npos) {
            p = strtok((char *)para.c_str(), delim);
            if (p != nullptr) {
                xPos = atoi(p);
                p = strtok(nullptr, delim);
                if (p != nullptr) {
                    yPos = atoi(p);
                }
            }
        }
        
        atris_msgs::NavToPoint point;
        point.request.name = para; 
        point.request.x = xPos;
        point.request.y = yPos;
        nav_to_point_srv_client_.call(point);
        return (point.response.result ? 1 : -4);
    } else if (nodeTypeString == "getlocationlist") {
         return 1;
    } else if (nodeTypeString == "broadcast_new") {
        std::string taskId;
        std::string fileUrl;
        std::string interval;
        std::string times;
        std::string timeList;
        Xml::queryNodeText(xmlString, "taskid", taskId);
        Xml::queryNodeText(xmlString, "fileurl", fileUrl);
        Xml::queryNodeText(xmlString, "ruletype", ruleType_);
        Xml::queryNodeText(xmlString, "begintime", beginMusicDateTime_);
        Xml::queryNodeText(xmlString, "endtime", endMusicDateTime_);
        Xml::queryNodeText(xmlString, "interval", interval);
        Xml::queryNodeText(xmlString, "times", times);
        Xml::queryNodeText(xmlString, "timelist", musicTimeList_);

        if (ruleType_.empty()) {
            ruleType_ = SPECIAL_BROADCAST_NEWS_TASK;
        }
        SqliteEngine::execSQL("UPDATE newsparams SET ruletype='"+ruleType_+"'");

        if (ruleType_ == RECYCLE_BROADCAST_NEWS_TASK) {
            if ((beginMusicDateTime_.empty()) ||
                (endMusicDateTime_.empty())) {
                return -2;
            } else {
                SqliteEngine::execSQL("UPDATE newsparams SET begintime='"+beginMusicDateTime_+"'");
                SqliteEngine::execSQL("UPDATE newsparams SET endtime='"+endMusicDateTime_+"'");
            }
        } else if (ruleType_ == TIMEING_BROADCAST_NEWS_TASK) {
            if (musicTimeList_.empty()) {
                return -2;
            } else {
                SqliteEngine::execSQL("UPDATE newsparams SET timelist='"+musicTimeList_+"'");
            }
        }

        title = "request_music_transport";
        std::string musicName = "police.mp3";
        if (fileUrl.empty()) {
            return -3;
        } else {
            SqliteEngine::execSQL("UPDATE newsparams SET fileurl='"+fileUrl+"'");
            Json::Value array = root["content"]["list"];
            Json::Value item;
            const char *ptr = strrchr(fileUrl.c_str(), '/');
            /*if (ptr) {
                ptr++;
                musicName = ptr;
                item["name"] = musicName;
            }*/
            item["name"] = musicName;
            item["url"] = fileUrl;
            array.append(item);
            root["content"]["list"] = array;
        } 

        playMusicValue_["content"]["action"] = "play";
        if (!interval.empty()) {
            playMusicValue_["content"]["play_interval"] = Json::Value(atoi(interval.c_str()));
        }
        playMusicValue_["content"]["mode"] = Json::Value("recycle");
        if (!times.empty()) {
            playMusicValue_["content"]["times"] = Json::Value(atoi(times.c_str()));
        }
        Json::Value listValue;
        Json::Value listArray = playMusicValue_["content"]["play_list"];
        listArray.clear();
        Json::Value listItem;
        listItem = musicName;
        listArray.append(listItem);
        playMusicValue_["content"]["play_list"] = listArray;

        Json::FastWriter jwriter;
        std::string playMusicJsonValue = jwriter.write(playMusicValue_);
         SqliteEngine::execSQL("UPDATE newsparams SET playmusicjsonvalue='"+playMusicJsonValue+"'");

        if (ruleType_ == RECYCLE_BROADCAST_NEWS_TASK) {
            int delaySec = diffTimeFromNow(endMusicDateTime_); 
            if (delaySec >= 0) {
                Task task;
                task.cb = boost::bind(&MessageHandle::sendMessStopMusic, this);
                TaskManager::get_instance()->post_delay(task, delaySec*1000);
            }
        }
    } else if (nodeTypeString == "broadcast_action") {
        title = "request_music_play";
        std::string action; 
        Xml::queryNodeText(xmlString, "action", action);
        if (action.empty()) {
            return -3;
        }
        root["content"]["action"] = action;
        root["content"]["play_interval"] = 0;
    } else {
        log_info("type is %s", nodeTypeString.c_str());
        return -2;
    }

    return 0;
}

int MessageHandle::convertJsonStringToRobotJson(const std::string &jsonString,
                                                std::string &title, Json::Value &root)
{
    if ((xmlRequestType_ != "") ||
        (jsonRequestType_ != "")) {
        return -1;
    }

    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;

    Json::Reader reader;
    Json::Value readValue;
    if (reader.parse(jsonString, readValue)) {
        jsonRequestType_ = readValue["type"].isNull() ? "" : readValue["type"].asString();
        if (jsonRequestType_ == "forward") {
            title = "request_robot_move";
            std::string linespeed = readValue["speed"].isNull() ? "" : readValue["speed"].asString();
            if (linespeed != "") {
                root["content"]["v_linear"] = Json::Value(atof(linespeed.c_str()));
            } else {
                root["content"]["v_linear"] = Json::Value(DEFAULT_FOAWARD_SPEED);
            }
            root["content"]["v_angular"] = Json::Value(0.0);
        } else if (jsonRequestType_ == "backward") {
            title = "request_robot_move";
            std::string linespeed = readValue["speed"].isNull() ? "" : readValue["speed"].asString();
            if (linespeed != "") {
                root["content"]["v_linear"] = Json::Value(-atof(linespeed.c_str()));
            } else {
                root["content"]["v_linear"] = Json::Value(-DEFAULT_BACKWARD_SPEED);
            }
            root["content"]["v_angular"] = Json::Value(0.0);
        } else if (jsonRequestType_ == "left") {
            title = "request_robot_move";
            std::string angularspeed = readValue["palstance"].isNull() ? "" : readValue["palstance"].asString();
            root["content"]["v_linear"] = Json::Value(DEFAULT_FOAWARD_LEFT_SPEED);
            if (angularspeed != "") {
                root["content"]["v_angular"] = Json::Value(atof(angularspeed.c_str()));
            } else {
                root["content"]["v_angular"] = Json::Value(DEFAULT_LEFT_SPEED);
            }
        } else if (jsonRequestType_ == "right") {
            title = "request_robot_move";
            std::string angularspeed = readValue["palstance"].isNull() ? "" : readValue["palstance"].asString();
            root["content"]["v_linear"] = Json::Value(DEFAULT_FOAWARD_RIGHT_SPEED);
            if (angularspeed != "") {
                root["content"]["v_angular"] = Json::Value(-atof(angularspeed.c_str()));
            } else {
                root["content"]["v_angular"] = Json::Value(-DEFAULT_RIGHT_SPEED);
            }
        } else if (jsonRequestType_ == "stop") {
            title = "request_robot_move";
            root["content"]["v_linear"] = Json::Value(0.0);
            root["content"]["v_angular"] = Json::Value(0.0);
        } else {
            log_info("command type is not supported\n");
            return -2;
        }
    } else {
        log_info("json string is not right\n");
        return -3;
    }

    return 0;
}

void MessageHandle::addHeadNodeXml(std::string &xmlString, const std::string &parNodeName)
{
    std::string nodeHeader = "header";
    Xml::addNodeText(xmlString, parNodeName, nodeHeader, "");

    String_Map headerMap;
    headerMap.clear();
    headerMap.insert(String_Pair("robotid", robotid_));
    headerMap.insert(String_Pair("userid" , userid_));
    headerMap.insert(String_Pair("id" , id_));
    headerMap.insert(String_Pair("time" , xmlTimestamp_));
    headerMap.insert(String_Pair("version" , "1.0"));

    String_Map::iterator iter;
    for (iter = headerMap.begin(); iter != headerMap.end(); iter++) {
        Xml::addNodeText(xmlString, nodeHeader,
                         iter->first, iter->second);
    }
}

void MessageHandle::addBodyNodeXml(std::string &xmlString, const std::string &parNodeName, 
                                   const std::string &jsonString)
{
    std::string nodeBody = "body";
    Xml::addNodeText(xmlString, parNodeName, nodeBody, "");

    String_Map bodyMap;
    if (xmlRequestType_ != "") {
        bodyMap.insert(String_Pair("type", xmlRequestType_));
    }

    Json::Reader reader;
    Json::Value root;
    std::string status = "101";
    std::string result = "";
    if (reader.parse(jsonString, root)) {
        result = root["content"]["result"].isNull() ? "" : root["content"]["result"].asString();
        if (result == "success") {
            status = "100";
        }
    }

    if (root["title"] == "response_nav_state") {
        if (navType_ != "") {
            bodyMap.insert(String_Pair("type", navType_));
        }
        int state = root["content"]["state"].isNull() ? 0 : root["content"]["state"].asInt();
        if (state == 302) {
            status = "104"; 
            result = "avoiding obstacles";    
        } else if (state == 306) {
            status = "103";
            result = "task end";
        }
    }

    if ((root["title"] == "response_reset_ptz")  ||
        (root["title"] == "response_move_ptz")) {
        bodyMap.insert(String_Pair("cameraid", cameraId_));  
    }

    if (root["title"] == "response_music_transport") {
        bodyMap.insert(String_Pair("taskid", newTaskId_));
    }
    
    bodyMap.insert(String_Pair("status", status));
    bodyMap.insert(String_Pair("result", result));
    String_Map::iterator iter;
    for (iter = bodyMap.begin(); iter != bodyMap.end(); iter++) {
        Xml::addNodeText(xmlString, nodeBody,
                         iter->first, iter->second);
    }
}

void MessageHandle::createResJson(std::string &jsonString, const std::string &message)
{
    Json::Reader reader;
    Json::Value readValue;
    std::string status = "101";
    std::string result = "";
    if (reader.parse(message, readValue)) {
        result = readValue["content"]["result"].isNull() ? "" : readValue["content"]["result"].asString();
        if (result == "success") {
            status = "100";
        }
    }
   
    Json::Value writeValue;
    writeValue["type"] = Json::Value(jsonRequestType_);
    writeValue["status"] = Json::Value(status);
    writeValue["result"] = Json::Value(result);

    ros::Time now = ros::Time::now();
    std::string time = timestampToDate(now.toSec());
    writeValue["stamp"] = Json::Value(time);
    //writeValue["stamp"] = Json::Value(now.toSec() * 1000);

    Json::FastWriter fw;
    jsonString = fw.write(writeValue);
}

bool MessageHandle::sendMessPlayMusic()
{
    atris_msgs::SignalMessage message;
    std::string musicTitle = "request_music_play";
    Json::FastWriter jwriter;

    message.title = musicTitle;
    message.msg = jwriter.write(playMusicValue_);
    signal_req_pub_.publish(message);

    return true;
}

bool MessageHandle::sendMessStopMusic()
{
    atris_msgs::SignalMessage message;
    std::string musicTitle = "request_music_play";
    Json::FastWriter jwriter;

    Json::Value root;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["content"]["timestamp"] = now.toSec() * 1000;
    root["content"]["action"] = "stop";
    message.title = musicTitle;
    message.msg = jwriter.write(root);
    signal_req_pub_.publish(message);

    return true;
}

int MessageHandle::diffTimeFromNow(const std::string &dateTime)
{
    time_t now;
    time(&now);  

    tm time;
    memset(&time, 0, sizeof(time));
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;

    if (sscanf(dateTime.c_str(), "%d-%d-%d %d:%d:%d",
        &year, &month, &day, &hour, &min, &sec) == 6) {
        if ((year < 2019) ||
            (month < 0)   || (month > 12) ||
            (day < 0)     || (day > 31)   ||
            (hour < 0)    || (hour > 24)  ||
            (min < 0)     || (min > 59)   ||
            (sec < 0)     || (sec > 59)) {
            return -1;
        }
    } else {
        return -1;
    }

    time.tm_year = year - 1900;
    time.tm_mon  = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min  = min;
    time.tm_sec  = sec;
  
    time_t t = mktime(&time);
    int diff = t - now;

    return diff;
}

std::string  MessageHandle::timestampToDate(time_t time)
{
        struct tm *t = localtime(&time);
        char dateBuf[128] = "0";
        snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,
                t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

        std::string date(dateBuf);
        return date;
}

void MessageHandle::initNewsParamsTable()
{
    char **result;
    int row, column;
    std::string fileUrl = "";
    std::string playMusicJsonValue = "";

    retry:
    SqliteEngine::execSQL(NEWSPARAMS_TABLE);
    SqliteEngine::query("SELECT * FROM newsparams", &result, &row, &column);
    if (row <= 0) {
        SqliteEngine::execSQL("INSERT INTO newsparams(fileurl, ruletype, playmusicjsonvalue, begintime, endtime, timelist) VALUES('"
                +fileUrl+"', '"
                +ruleType_+"', '"
                +playMusicJsonValue+"', '"
                +beginMusicDateTime_+"', '"
                +endMusicDateTime_+"', '"
                +musicTimeList_+"')");
    } else {
        if (column != NEWSPARAMS_TABLE_COLUMN) {
            SqliteEngine::execSQL("DROP TABLE robot");
            goto retry;
        } else {
            fileUrl = result[column + 1];
            ruleType_ = result[column + 2];
            playMusicJsonValue = result[column + 3];
            beginMusicDateTime_ = result[column + 4];
            endMusicDateTime_ = result[column + 5];
            musicTimeList_ = result[column + 6];
        }
    }

    SqliteEngine::freeQuery(result);

    Json::Reader reader;
    Json::Value value;
    if (reader.parse(playMusicJsonValue.c_str(), value)) {
        playMusicValue_ = value;
    }

    if (ruleType_ == RECYCLE_BROADCAST_NEWS_TASK) {
        int delaySec = diffTimeFromNow(endMusicDateTime_); 
        if (delaySec >= 0) {
            Task task;
            task.cb = boost::bind(&MessageHandle::sendMessStopMusic, this);
            TaskManager::get_instance()->post_delay(task, delaySec*1000);
        }
    }

    if ((ruleType_ == RECYCLE_BROADCAST_NEWS_TASK) ||
        (ruleType_ == TIMEING_BROADCAST_NEWS_TASK)) {
        Json::Value root;
        ros::Time now = ros::Time::now();
        std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
        root["content"]["id"] = uid.str();
        root["content"]["timestamp"] = now.toSec() * 1000;

        Json::Value array = root["content"]["list"];
        const char *ptr = strrchr(fileUrl.c_str(), '/');
        std::string musicName;
        Json::Value item;
        if (ptr) {
            ptr++;
            musicName = ptr;
            item["name"] = musicName;
        }
        item["url"] = fileUrl;
        array.append(item);
        root["content"]["list"] = array;

        Json::FastWriter jwriter;
        atris_msgs::SignalMessage message;
        message.title = "request_music_transport";
        message.msg = jwriter.write(root);
        signal_req_pub_.publish(message);
    }
}
