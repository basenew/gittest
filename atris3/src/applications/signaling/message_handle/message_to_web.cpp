#include "message_to_web.h"
#include "tiny_ros/ros/time.h"
#include "utils/utils.h"
#include "md5/md5.h"
#include "config/config.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"

#define POST_QUEUE_MAX 200

MessageToWeb::MessageToWeb() 
    : post_thread_pool_(1)
    , post_queue_count_(0)
{
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100, &MessageToWeb::ReceiveSingalMessage, this);
    get_gps_position_srv_client_ = nh_.serviceClient<atris_msgs::GetGpsPos>(SRV_GET_GPS_POSITION);
}

void MessageToWeb::ReceiveSingalMessage(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value root;
    Json::FastWriter writer;
    
    if (msg.title.find("notify_event") != std::string::npos) {
        if (reader.parse(msg.msg, root)) {
            if ((!root.isNull()) &&
                (!root["content"].isNull()) &&
                (!root["content"]["event_message"].isNull())) {
                boost::unique_lock<boost::mutex> lock(post_queue_mutex_); 
                std::string event_content = writer.write(root["content"]["event_message"]);

                if (post_queue_count_ < POST_QUEUE_MAX) {
                    post_queue_count_ += 1;
                    post_thread_pool_.schedule(boost::bind(&MessageToWeb::PostMessageToWeb, this, event_content));
                    log_info("post_queue_count: %d, event_content is %s", post_queue_count_, event_content.c_str());
                } else {
                    log_once_warn("Queue overflow(%d), event_content is %s", post_queue_count_, event_content.c_str());
                }
            }
        }
    }
}

void  MessageToWeb::PostMessageToWeb(const std::string &content)
{
    int times = 3;
    std::string url = Config::get_instance()->post_event_url;
    log_info("---------------%s post event url : %s---------------",__FUNCTION__, url.c_str());
 
    std::vector<std::string> http_header;
    std::string resp;
    Json::Reader reader;
    Json::Value root;
    int result = -1;
    long http_res_code = 0;

    while (times > 0) {
        times --;
        http_header.clear();
        http_header.push_back("Content-Type:application/json");
        http_header.push_back("Accept: application/json");
        GetHttpHeader(http_header);
        int ret = Utils::get_instance()->http_post(url, content, resp, http_header, http_res_code, kHttpTimeoutSec);
        if (ret == 0) 
        {

            if (http_res_code == 201) 
            {
                result =  0; // 100x
                log_info("%s PostMessageToWeb success, event_content:%s\r\n", __FUNCTION__, content.c_str());
            } else {
                if (reader.parse(resp.c_str(), root)) 
                {
                    std::string code = root["code"].isNull() ? "" : root["code"].asString();
                    std::string msg  = root["msg"].isNull() ? "" : root["msg"].asString();
                    log_error("PostMessageToWeb fail , http response code %ld, code : %s, msg: %s\r\n",http_res_code,code.c_str(),msg.c_str());
                    result = -1;
                }
                else
                {
                    log_error("post message to web , resp json string parse failed!!!\r\n");
                    result = -3;
                }

            }
        } else {
            result = -2;
        }

        if (result >= 0) {
            break;
        }
    }
    
    boost::unique_lock<boost::mutex> lock(post_queue_mutex_);   
    post_queue_count_ -= 1;
}

int MessageToWeb::InitRemoteMqttTopic()
{
    int times = 1;
    std::string url = Config::get_instance()->remote_mqtt_topic_url_;
    Json::Value root;
    Json::FastWriter writer;
    std::vector<std::string> http_header;
    Json::Reader reader;
    std::string resp;
    int result = -1;

    root["domain"] = "setting";
    root["productName"] = "Atris";

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string sn = std::string(shmrbt.robot.sn);
    root["sn"] = sn;

    root["push"].append("");
    std::string content = writer.write(root);

    while (times > 0) {
        times --;
        http_header.clear();
        http_header.push_back("Content-Type:application/json");
        http_header.push_back("Accept: application/json");
        GetHttpHeader(http_header);
        int ret = Utils::get_instance()->http_post(url, content, resp, http_header, kHttpTimeoutSec);
        log_info("resp is %s", resp.c_str());
        if (ret == 0) {
            if (reader.parse(resp.c_str(), root)) {
                std::string code = root["code"].isNull() ? "" : root["code"].asString();
                std::string msg  = root["msg"].isNull() ? "" : root["msg"].asString();
                if ((code == "0") && (msg == "success")) {
                    result =  0;
                } else {
                    result =  atoi(code.c_str());
                }
            } else {
                result = -1;
            }
        } else {
            result = -2;
        }

        if (result >= 0) {
            break;
        }
    }

    return result;
}

void MessageToWeb::GetHttpHeader(std::vector<std::string> &http_header)
{
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string sign_string = "X-UBT-Sign";
    std::string appid_string = "X-UBT-AppId";
    std::string deviceid_string = "X-UBT-DeviceId";
    std::string colon_string = ": ";
    std::string deviceid  = shmrbt.robot.sn;
    std::string appid = Config::get_instance()->abi_id;

    std::string sign;
    GetHttpHeaderSign(sign);
    std::string header_sign = sign_string + colon_string + sign;
    http_header.push_back(header_sign);

    std::string header_appid = appid_string + colon_string + appid;
    http_header.push_back(header_appid);

    std::string header_deviceid= deviceid_string + colon_string + deviceid;
    http_header.push_back(header_deviceid);
}

void MessageToWeb::GetHttpHeaderSign(std::string &sign)
{
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string device_id  = shmrbt.robot.sn;
    log_info("%s device id = %s", __FUNCTION__, device_id.c_str());
    std::string key = Config::get_instance()->abi_key;
    std::string version_num = Config::get_instance()->abi_version;
    std::string bank_space = " ";

    tinyros::Time now_time = tinyros::Time::now();
    std::stringstream now_stream;
    now_stream  << (uint64_t)(now_time.toSec());
    std::string nowtime_string = now_stream.str();

    char rand_str[16] = {'\0'};
    GetRandString(rand_str);
    std::string rand_string = rand_str;

    std::string calc_md5_string;
    calc_md5_string = nowtime_string + key + rand_string + device_id;

    CMD5 md5obj; 
    md5obj.GenerateMD5((unsigned char*)calc_md5_string.c_str(), calc_md5_string.size());
    std::string md5_string = md5obj.ToString();


    sign = md5_string + bank_space + nowtime_string + bank_space + 
           rand_string + bank_space + version_num;

}


void MessageToWeb::GetRandString(char *str)
{ 
    if (str == NULL) {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    int rand_num =  tv.tv_sec + tv.tv_usec;
    srand(rand_num);

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


