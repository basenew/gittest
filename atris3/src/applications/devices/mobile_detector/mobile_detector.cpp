#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <json/json.h>

#include "mobile_detector.h"
#include "config/config.h"
#include "flash_lamp/flash_lamp.h"
#include "imsi_data.h"
#include "utils/utils.h"
#include "robot/robot.h"
#include "transferfile/transferfile.h"
#include "imemory/atris_imemory_api.h"


boost::asio::io_service detect_service;
//boost::asio::ip::udp::endpoint detect_ep( boost::asio::ip::address::from_string("192.168.8.2"), 33082);
boost::asio::ip::udp::socket detect_sock(detect_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 33082) );


#define MOBILE_DETECT_TAG  "mobile_detect->"

MobileDetect::MobileDetect() {
    log_info(MOBILE_DETECT_TAG"%s", __FUNCTION__);
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &MobileDetect::on_recv_mobile_detect_ctrl, this);
    ImsiData::init();
    new boost::thread(boost::bind(&MobileDetect::udp_receive_thread, this));
    new boost::thread(boost::bind(&MobileDetect::data_process_thread, this));
}

MobileDetect::~MobileDetect()
{
    log_info(MOBILE_DETECT_TAG"%s", __FUNCTION__);
}

int MobileDetect::init(void)
{
    log_info(MOBILE_DETECT_TAG"%s", __FUNCTION__);
    return 0;
}

void MobileDetect::udp_receive_thread(void)
{
    boost::asio::ip::udp::endpoint sender_ep;
    int bytes =0;
    char *buff = NULL;
   
    while(1)
    {
        char buff[1024]={0};
        char *data = NULL;
        //char  detect_data[2048] ={0};

        //log_info(MOBILE_DETECT_TAG"%s.", __FUNCTION__);
        bytes = detect_sock.receive_from(boost::asio::buffer(buff), sender_ep);
        if(bytes != 0){
            //log_debug(MOBILE_DETECT_TAG"%s receive data size: %d",__FUNCTION__,bytes);
            boost::lock_guard<boost::mutex> lock(mobile_decected_data_lock);
            
            //log_debug(MOBILE_DETECT_TAG"%s detected data: %s",__FUNCTION__, buff);
            data = (char *)malloc(1024);
            if(data == NULL){
                log_error("malloc failed");
                break;
            }
            memcpy(data, buff, sizeof(buff));
            msg_list.add(data);
        }
    }
}

void MobileDetect::data_process_thread(void)
{
    char *buff = NULL;
    while(1){
        buff = msg_list.get();
        //log_info(MOBILE_DETECT_TAG":%s", buff);
        if(!parser_detected_data(buff)){
            notify_detected_data();
        }
        free(buff);
    }
}

int MobileDetect::parser_detected_data(std::string databuff)
{
    //log_debug(MOBILE_DETECT_TAG"%s ",__FUNCTION__);
    Json::Reader reader;
    Json::Value root;
    
    if(!reader.parse(databuff, root)){
        log_error("parse mobile detected data  fail.");
        return -1;
    }
    
    if(root.isMember("id"))
    {   
#if 0
        if("MSG_GPS_RPT" == root["id"].asString()){
            if(!root.isMember("latitude") && !root.isMember("longitude")){
                log_error("no gps data");
            }else{
                log_debug("mobile detector have gps data");
                mobileDetect_data.latitude = root["latitude"].asString();
                mobileDetect_data.longitude = root["longitude"].asString();
                log_info("mobile detect gps: latitude data: %s, longitude data: %s",
                    mobileDetect_data.latitude.c_str(), mobileDetect_data.longitude.c_str());
            }
            return -1;
        }
#endif
        if("MSG_UEID_RPT" == root["id"].asString()){
            if(!root.isMember("imsi")){
                log_error("no imsi");
                return -1;
            }else{
                mobileDetect_data.imsi = root["imsi"].asString();
                log_info("imsi :%s", mobileDetect_data.imsi.c_str());
                return 0;   
            }

        }
        
    }
    return -1;
}

void MobileDetect::notify_detected_data(void)
{
    //log_debug(MOBILE_DETECT_TAG"%s ",__FUNCTION__);
    Json::Value content;

    Json::FastWriter fw;

    int64_t now = (int64_t)(ros::Time::now().toSec() * 1000);
    struct DangerImsiListData data;
    struct ImsiDetectionListData detectData;
    data.imsi = mobileDetect_data.imsi;
    data.danger_type = DANGER_TYPE_NORMAL;

    if(ImsiData::IsImsiInDangerImsiList(data)){
        //root["title"] = "notify_mobile_imsi";
        content["imsi"] = data.imsi.c_str();
        content["type"] = data.danger_type;
        content["detection_timestamp"] = now;
        Utils::get_instance()->NotifyRobotStatus("notify_mobile_imsi", content);

        if(Config::get_instance()->open_red_blue_light_imsi_detection){
            if(data.danger_type == DANGER_TYPE_ONE){
                FlashLamp::get_instance()->set_red_blue_flash_status(FL_ON, 2, 10);
            }else if(data.danger_type == DANGER_TYPE_TWO){
                FlashLamp::get_instance()->set_red_blue_flash_status(FL_ON, 1.6, 10);
            }else if(data.danger_type == DANGER_TYPE_THREE){
                FlashLamp::get_instance()->set_red_blue_flash_status(FL_ON, 1.2, 10);
            }
        }
    }
    detectData.detection_timestamp = now;
    detectData.item = data;
    ImsiData::UpdateImsiDetectionListDataBase(detectData);
}


void MobileDetect::on_recv_mobile_detect_ctrl(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value req, resp;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if (msg.title == "request_mobile_imsi_bootup") {
        if(!reader.parse(msg.msg, req)){
            log_error("parse json fail.");
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_mobile_imsi_bootup");
            return;
        }
        int num = req["content"]["req_number"].asInt();
        int cur_page = req["content"]["cur_page"].asInt();
        int total_page = 0;
        std::vector<ImsiDetectionListData> vector_data;
        int ret = ImsiData::QueryNumberofDetectionImsi(cur_page, num, vector_data, &total_page);
        if(ret < 0){
            resp["result"] = "fail_inner_error";
            Utils::get_instance()->responseResult(msg, resp, "response_mobile_imsi_bootup");
            return ;
        }
        resp["req_number"] = num;
        resp["total_page"] = total_page;
        resp["resp_number"] = vector_data.size();
        resp["cur_page"] = cur_page;
        Json::Value imsi_array;
        Json::Value item;
        for(std::vector<struct ImsiDetectionListData>::iterator it = vector_data.begin(); 
            it!=vector_data.end(); it++)
        {
            item["detection_timestamp"] = Json::Value(it->detection_timestamp);
            item["imsi"] = it->item.imsi.c_str();
            item["type"] = it->item.danger_type;
            imsi_array.append(item);
        };
        resp["imsi_array"] = imsi_array;
        Utils::get_instance()->responseResult(msg, resp, "response_mobile_imsi_bootup");
    }
    else if(msg.title == "request_get_detection_imsi_file_url"){
        if(!reader.parse(msg.msg, req)){
            log_error("parse json fail.");
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_get_detection_imsi_file_url");
            return;
        }

        new boost::thread(boost::bind(&MobileDetect::imsi_detection_file_export_thread, this, msg));
        resp["result"] = "success";
        Utils::get_instance()->responseResult(msg, resp, "response_get_detection_imsi_file_url");
    }
    else if(msg.title == "request_imsi_file_transport"){
        log_info(MOBILE_DETECT_TAG"%s", __FUNCTION__);
        std::string local_path;
        std::string url;
        if(!reader.parse(msg.msg, req)){
            log_error("parse json fail.");
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_imsi_file_transport");
            return;
        }

        resp["result"] = "success";
        Utils::get_instance()->responseResult(msg, resp, "response_imsi_file_transport");

        std::string name;
        for(unsigned int i=0; i<req["content"]["list"].size(); i++){
            name = req["content"]["list"][i]["name"].asString();
            url = req["content"]["list"][i]["url"].asString();
            TransferFile::DownloadFile(name, url, download_state_handler);
        }
    }
}

void MobileDetect::imsi_detection_file_export_thread(atris_msgs::SignalMessage &export_msg)
{
    Json::Reader reader;
    Json::Value req, resp;

    resp["id"] = export_msg.msgID;
    resp["timestamp"] = export_msg.timestamp;
    resp["result"] = "success";

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string local_dir = "/tmp/";
    std::string file_name = std::string(shmrbt.robot.sn) + "_IMSI";
    std::string local_path = local_dir + file_name + ".txt";
    
    if(ImsiData::GenerateDetectionImsiTXTFile(local_path) < 0){
        resp["result"] = "fail_inner_error";
        Utils::get_instance()->responseResult(export_msg, resp, "notify_get_detection_imsi_file_url");
    }else{
        log_info(MOBILE_DETECT_TAG"local_path=%s", local_path.c_str());
        FILE *fp = NULL;
        std::string path = local_dir + file_name + ".tar.gz";
        std::string cmd = "cd " + local_dir + "; tar -cvf " + file_name + ".tar.gz " + file_name + ".txt" + ";cd -";
        if((fp = popen(cmd.c_str(), "r")) != NULL){
            pclose(fp);
        }else{
            resp["result"] = "fail_inner_error";
            Utils::get_instance()->responseResult(export_msg, resp, "notify_get_detection_imsi_file_url");
            return;
        }
        TransferFile::UploadFile(path, upload_state_handler);
    }
}

void MobileDetect::upload_state_handler(TransferStates state, std::string &path)
{
    Json::Value content;
    if(state == TRANSFER_FILE_COMPLETED){
        content["progress"] = 100;
        if(Config::get_instance()->hfs_url.find("upload") != std::string::npos ||
           Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos )
        {
            // use fastdfs or udfs-tracer
            content["url"] = Utils::get_instance()->get_fileurl();
        } 
        else
        {
            content["url"] = path;
        }
        content["result"] = "success";
    }
    else if(state == TRANSFER_FILE_ERROR)
    {
        content["result"] = "fail_upload_error";
    }
    else
    {
        return;
    }
    
    Utils::get_instance()->NotifyRobotStatus("notify_get_detection_imsi_file_url", content, "");
}

void MobileDetect::download_state_handler(TransferStates state, std::string &path)
{
    //log_info(MOBILE_DETECT_TAG"%s", __FUNCTION__);
    Json::Value content;
    if(state == TRANSFER_FILE_COMPLETED){
        log_info("Download finish, path=%s", path.c_str());
        std::string suffixStr = path.substr(path.find_last_of('.') + 1);
        if(suffixStr == "txt" || suffixStr == "TXT"){
            ImsiData::ParseDangerImsiTXTfile(path);
            content["result"] = "success";
        }else{
            content["result"] = "fail_path_error";
        }
    }
    else if(state == TRANSFER_FILE_ERROR)
    {
        log_info("Download error");
        content["result"] = "fail_download_error";
    }
    else
    {
        return;
    }
    Utils::get_instance()->NotifyRobotStatus("notify_imsi_file_transport", content, "");
}
