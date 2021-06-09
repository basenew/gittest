#pragma once
#include <functional>

#include <json/json.h>

#include "transferfile/transferfile.h"

#define DOWNLOAD_PROGRESS_INTERVAL 1   // seconds

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[trn_ntf]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[trn_ntf]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[trn_ntf]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[trn_ntf]"#format, ##args)

using namespace Json;
//using namespace Robot;
using namespace atris_msgs;

namespace nav{

class TransNotifyer:public FileObject{
private:
    uint32_t last_sec_;

    void resp_progress(string result, string status, double progress){
        Value resp;
        int64_t now = (int64_t)(ros::Time::now().toSec() * 1000);
        resp["id"] = origin.msgID;
        resp["name"] = name;
        resp["timestamp"] = Value(now);
        resp["progress"] = progress;
        resp["status"] = status;
        resp["result"] = result;
        resp["url"] = remote_path;
        Utils::get_instance()->responseResult(origin, resp, "response" + origin.title.substr(7));
    }

    virtual void progress(double progress){
        uint32_t now = ros::Time::now().sec;
        if ((now - last_sec_) >= DOWNLOAD_PROGRESS_INTERVAL){
            resp_progress("success", status_str, progress);
            last_sec_ = now;
        }

        if (now < last_sec_) {
            last_sec_ = now;
        }
    }

    virtual void notify(TransferStates st, string msg = "", int code = 0){
        state = st;
        if (state == TRANSFER_FILE_ERROR){
	    Lerror("trans fail");
            resp_progress(result_fail, "", 0);
        }else if (state == TRANSFER_FILE_STARTED){
            resp_progress("success", started_str, 0);
        }else if (state == TRANSFER_FILE_COMPLETED){
            if(Config::get_instance()->hfs_url.find("upload") != std::string::npos
            || Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos){//todo zy maybe set config file type
                remote_path = Utils::get_instance()->get_fileurl();
		Linfo("server gen url:%s", remote_path.c_str());
            };
            if (cb)cb(remote_path);
            resp_progress("success", result_success, 100);
        }
    }

public:
    //typedef void (*ON_FINISHED)();
    TransNotifyer(bool download)
    :last_sec_(0)
    ,state(TRANSFER_FILE_STARTED)
    ,started_str(download?"started":"start_upload")
    ,status_str(download?"downloading":"uploading")
    ,result_fail(download?"fail_download_error":"fail_upload_error")
    ,result_success(download?"finished":"finish_upload")
    {};

    TransferStates state;
    string resp_str;
    string started_str;
    string status_str;
    string result_fail;
    string result_success;
    string name;
    string station_id;
    SignalMessage origin;
    //ON_FINISHED   cb;
    function<int(string)> cb;
};


}

