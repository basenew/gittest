#include <boost/thread/thread.hpp>
#include "gs_api.h"
#include "gs_nav.h"
#include "nav_error_code.h"
#include "transferfile/transferfile.h"
#include "utils/utils.h"
#include <iomanip>

#define DOWNLOAD_PROGRESS_INTERVAL 1   // seconds
#define MAX_RECONNECT_TIMES     20
#define MAP_UPLOAD_TMP_PATH "/userdata/tmp/maps"

#define CHECK_STATE_TIMEOUT  3600 //seconds

class GsMapDownload: public FileObject {
    private:
        uint32_t last_sec_;

        void resp_download_progress(std::string result, std::string status, double progress ) {
            log_info("downloading %s progress:%f", name.c_str(), progress);
            Json::Value resp;
            int64_t now = (int64_t)(ros::Time::now().toSec() * 1000);
            resp["id"] = origin.msgID;
            resp["timestamp"] = Json::Value(now);
            resp["url"] = remote_path;
            resp["name"] = name;
            resp["progress"] = progress;
            resp["status"] = status;
            resp["result"] = result;
            Utils::get_instance()->responseResult(origin, resp, "response_sync_map");
        }

        virtual void progress(double progress)  {
            uint32_t now = ros::Time::now().sec;
            if ((now - last_sec_) >= DOWNLOAD_PROGRESS_INTERVAL) {
                resp_download_progress("success", "downloading", progress);
                last_sec_ = now;
            }

            if (now < last_sec_) {
                last_sec_ = now;
            }
        }

        virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
            if (state == TRANSFER_FILE_ERROR) {
                resp_download_progress("fail_download_error", "", 0);
            } else if (state == TRANSFER_FILE_STARTED) {
                resp_download_progress("success", "started", 0);
            } else if (state == TRANSFER_FILE_COMPLETED) {
                bool ret = GsApi::get_instance()->map_upload(name);//上传文件到导航控制盒
                if(!ret){
                    resp_download_progress("success", "fail_upload", 100);
                }else{
                    resp_download_progress("success", "finished", 100);
                }
            }
        }

    public:
        GsMapDownload(): last_sec_(0) { }
        atris_msgs::SignalMessage origin;
        std::string name;
};

class GsMapUpload: public FileObject {
    private:
        void resp_upload_progress(std::string result, std::string status, double progress ) {
            log_info("downloading %s progress:%f", name.c_str(), progress);
            Json::Value resp;

            int64_t now = (int64_t)(ros::Time::now().toSec() * 1000);
            resp["id"] = origin.msgID;
            resp["timestamp"] = Json::Value(now);
            resp["url"] = remote_path;
            resp["name"] = name;
            resp["progress"] = progress;
            resp["status"] = status;
            resp["result"] = result;
            Utils::get_instance()->responseResult(origin, resp, "response_load_map");
        }

        virtual void progress(double progress)  {
            uint32_t now = ros::Time::now().sec;
            if ((now - last_sec_) >= DOWNLOAD_PROGRESS_INTERVAL) {
                resp_upload_progress("success", "uploading", progress);
                last_sec_ = now;
            }

            if (now < last_sec_) {
                last_sec_ = now;
            }
        }

        virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
            if(state == TRANSFER_FILE_ERROR){
                //resp_upload_files("fail_inner_error");
                resp_upload_progress("fail_inner_error", "fail_upload", 0);
                log_info("upload error:%s", local_path.c_str());
            }
            else if(state == TRANSFER_FILE_COMPLETED){
                //resp_upload_files("success"); 
                resp_upload_progress("success", "finish_upload", 100);
                log_info("upload completed:%s", local_path.c_str());
            }
            else if(state == TRANSFER_FILE_STARTED){
                resp_upload_progress("success", "start_upload", 0);
                log_info("upload start:%s", local_path.c_str());
            }
            this->state = state;
        }

    public:
        TransferStates state;
        GsMapUpload(): last_sec_(0), state(TRANSFER_FILE_STARTED) { }
        atris_msgs::SignalMessage origin;
        std::string name;
        uint32_t last_sec_;
};

#define DB_MOD_NAV_NAME            "nav"
#define DB_KEY_USING_MAP           "using_map"
#define GET_USING_MAP_FROM_DB()    nav_db.get(DB_KEY_USING_MAP)
#define SET_USING_MAP_TO_DB(name)  nav_db.set(DB_KEY_USING_MAP, name)

GsNav::GsNav() {
    log_info("%s", __FUNCTION__);
    get_gps_position_srv_ = nh_.advertiseService(SRV_GET_GPS_POSITION, &GsNav::doGetGpsPosition, this);
    diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
    dev_state_path = "/gs-robot/notice/device_status";
    state_path = "/gs-robot/notice/status";
    health_state_path =  "/gs-robot/notice/system_health_status";
    state_monitor_id = 0;
    dev_status_thread = health_thread = NULL;
    ws_cli_state = ws_cli_health_state = NULL;
    lidar_slave_state = lidar_master_state =
    rgbd_state = gyro_state = gyro_state_data = odom_delta_spd = -1;
    gsnav_state =false;
    gsapi = GsApi::get_instance();
    cfg = Config::get_instance();
    utils = Utils::get_instance();
    ultrasonic_state = 0xFF;
    nav_db.select(DB_MOD_NAV_NAME);

    // init gps data
    lati_ = 0.0;
    long_ = 0.0;
    alti_ = 0.0;
    gpsstatus = -1;
    lws_cli_count = 0;

    gps_data_thread_ = new boost::thread(boost::bind(&GsNav::update_gps_thread, this));
    using_map = GET_USING_MAP_FROM_DB();
    log_info("atris using map:%s", using_map.c_str());
}

void GsNav::update_gps_thread() {
    while (1) {
        int status;
        gpsstatus = -1; // gps device unavailable
        if (gsapi->get_gps_data(lati_, long_, alti_, status) == ERR_OK) {
            if (status == 2) {
                gpsstatus = 0; // 正常
            } else {
                gpsstatus = 1; // 定位精度太差
            }
        }
        gps_data_publish();
        sleep(3);
    }
}

bool GsNav::check_state()
{
    log_info("check gs nav state.");
    std::string version;

    int i = 0;
    while(i < CHECK_STATE_TIMEOUT){
        gsnav_state = gsapi->get_version(version) == ERR_OK;
        if(gsnav_state){
            log_debug("gs nav work fine.");
            break;
        }
        else
            log_debug("gs nav device not work count:%d", i);
        sleep(3);
        i += 3;
    }

    return gsnav_state;
}

bool GsNav::get_state()
{
    return gsnav_state;
}

static int lws_callback_status(struct lws *wsi, enum lws_callback_reasons reason,
        void *user, void *in, size_t len)
{
    switch (reason) {
        /* because we are protocols[0] ... */
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            {
                log_error("status CLIENT_CONNECTION_ERROR: %s",
                        in ? (char *)in : "(null)");
                GsNav * nav = GsNav::get_instance();
                nav->ws_cli_state = NULL;
                sleep(1);
                nav->create_state_monitor(1);
            }
            break;

        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            {
                log_warn("%s: established", __func__);
            }
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            //log_info("%s", (const char *)in);
            GsNav::get_instance()->on_recv_status((char*)in);
            break;

        case LWS_CALLBACK_CLIENT_CLOSED:
            GsNav::get_instance()->ws_cli_state = NULL;
            break;

        default:
            break;
    }

    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

static int lws_callback_device_status(struct lws *wsi, enum lws_callback_reasons reason,
        void *user, void *in, size_t len)
{
    switch (reason) {

        /* because we are protocols[0] ... */
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            log_error("dev CLIENT_CONNECTION_ERROR: %s",
                    in ? (char *)in : "(null)");
            {
                GsNav *nav = GsNav::get_instance();
                //nav->ws_cli_dev_state = NULL;
                //sleep(1);
                //nav->create_state_monitor(1);
            }
            break;

        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            log_warn("%s: established", __func__);
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            //log_info("%s", (const char *)in);
            break;

        case LWS_CALLBACK_CLIENT_CLOSED:
            GsNav::get_instance()->ws_cli_dev_state = NULL;
            break;

        default:
            break;
    }

    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

static int lws_callback_health_status(struct lws *wsi, enum lws_callback_reasons reason,
        void *user, void *in, size_t len)
{
    switch (reason) {
        /* because we are protocols[0] ... */
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            {
                log_error("health  CLIENT_CONNECTION_ERROR: %s",
                        in ? (char *)in : "(null)");
                GsNav *nav = GsNav::get_instance();
                nav->ws_cli_health_state = NULL;
                sleep(1);
                nav->create_state_monitor(2);
            }
            break;
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            {
                log_warn("%s: established", __func__);
            }
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            {
                //log_info("%s", (const char *)in);
                std::string test = (char*)(in);
                GsNav::get_instance()->on_recv_dev_status((const char *)in);
            }
            break;

        case LWS_CALLBACK_CLIENT_CLOSED:
            GsNav::get_instance()->ws_cli_health_state = NULL;
            break;

        default:
            break;
    }

    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

static const struct lws_protocols dev_status_protocols[] = {
    {
        "gs-dev-state-protocol",
        lws_callback_device_status,
        0,
        0,
    },
    { NULL, NULL, 0, 0 }
};

static const struct lws_protocols status_protocols[] = {
    {
        "gs-state-protocol",
        lws_callback_status,
        0,
        0,
    },
    { NULL, NULL, 0, 0 }
};

static const struct lws_protocols health_status_protocols[] = {
    {
        "gs-health-state-protocol",
        lws_callback_health_status,
        0,
        0,
    },
    { NULL, NULL, 0, 0 }
};

int GsNav::ws_cli_health_connect(const struct lws_protocols *protocols, struct lws *client, const char *path)
{
    struct lws_context_creation_info info;
    struct lws_client_connect_info i;
    struct lws_context *context;
    const char *p;
    int n = 0, logs = LLL_USER | LLL_ERR | LLL_WARN | LLL_NOTICE;

    thread_health_state = THREAD_RUNNING;
    //lws_set_log_level(logs, NULL);

    //boost::unique_lock<boost::mutex> lock(mutex);
    memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */
    info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    info.port = CONTEXT_PORT_NO_LISTEN; /* we do not run any server */
    info.protocols = protocols;

    context = lws_create_context(&info);
    if (!context) {
        lwsl_err("lws init failed\n");
        return -1;
    }

    memset(&i, 0, sizeof i); /* otherwise uninitialized garbage */
    i.context = context;
    i.port = cfg->gs_ws_port;
    i.address = cfg->gs_ip.c_str();
    i.path = path;
    i.host = i.address;
    i.origin = i.address;
    i.ssl_connection = 0;
    i.protocol = protocols[0].name;
    i.pwsi = &client;

    lws_client_connect_via_info(&i);

    cond.notify_one();
    //lock.unlock();

    lws_cli_count ++;
    while (n >= 0 && client != NULL && lws_srv_health_state)
        n = lws_service(context, 100);

    //lock.lock();
    lws_context_destroy(context);
    //lock.unlock();
    lws_cli_count --;
    log_info("ws_cli_health_connect exit");
    client = NULL;

    thread_health_state = THREAD_IDLE;

    return 0;
}

int GsNav::ws_cli_dev_connect(const struct lws_protocols *protocols, struct lws *client, const char *path)
{
    struct lws_context_creation_info info;
    struct lws_client_connect_info i;
    struct lws_context *context;
    const char *p;
    int n = 0, logs = LLL_USER | LLL_ERR | LLL_WARN | LLL_NOTICE;

    thread_dev_state = THREAD_RUNNING;
    //lws_set_log_level(logs, NULL);

    //boost::unique_lock<boost::mutex> lock(mutex);
    memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */
    info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    info.port = CONTEXT_PORT_NO_LISTEN; /* we do not run any server */
    info.protocols = protocols;

    context = lws_create_context(&info);
    if (!context) {
        lwsl_err("lws init failed\n");
        return -1;
    }

    memset(&i, 0, sizeof i); /* otherwise uninitialized garbage */
    i.context = context;
    i.port = cfg->gs_ws_port;
    i.address = cfg->gs_ip.c_str();
    i.path = path;
    i.host = i.address;
    i.origin = i.address;
    i.ssl_connection = 0;
    i.protocol = protocols[0].name;
    i.pwsi = &client;

    lws_client_connect_via_info(&i);

    cond.notify_one();
    //lock.unlock();

    lws_cli_count ++;
    while (n >= 0 && client != NULL && lws_srv_dev_state)
        n = lws_service(context, 100);

    //lock.lock();
    lws_context_destroy(context);
    //lock.unlock();
    lws_cli_count --;
    log_info("ws_cli_dev_connect exit");
    client = NULL;

    thread_dev_state = THREAD_IDLE;
    return 0;
}

int GsNav::init()
{
    log_info("gs nav init");
    ws_cli_state = ws_cli_health_state = NULL;
    ws_cli_dev_state = NULL;
    gs_host = "http://" + cfg->gs_ip + ":" + std::to_string(cfg->gs_web_port);
    mls_state = MLS_IDLE;
    return 0;
}

int GsNav::create_state_monitor(int type)
{
    if(type == 1){
        boost::unique_lock<boost::mutex> lock(mutex);
        lws_srv_dev_state = true;
        dev_status_thread = new boost::thread(boost::bind(&GsNav::ws_cli_dev_connect, this,
                    status_protocols, ws_cli_state, state_path.c_str()));
        cond.wait(lock);

        log_info("create dev state ws");
    }
    else if(type == 2){
        boost::unique_lock<boost::mutex> lock(mutex);
        lws_srv_health_state = true;
        health_thread = new boost::thread(boost::bind(&GsNav::ws_cli_health_connect, this,
                    health_status_protocols, ws_cli_health_state, health_state_path.c_str()));
        cond.wait(lock);

        log_info("create health state ws");
    }

    return 0;
}


void GsNav::gps_data_publish()
{
    Json::FastWriter fw;
    Json::Value rbtInfoValue;
    atris_msgs::RobotInfo rbtInfo;
    std::stringstream slati; slati << setprecision(20) << lati_;
    std::stringstream slong; slong << setprecision(20) << long_;
    std::stringstream salti; salti << setprecision(20) << alti_;

    rbtInfoValue["robot_info"]["gps"]["error"] = gpsstatus;
    rbtInfoValue["robot_info"]["gps"]["lati"] = slati.str();
    rbtInfoValue["robot_info"]["gps"]["long"] = slong.str();
    rbtInfoValue["robot_info"]["gps"]["alti"] = salti.str();
    rbtInfo.json = fw.write(rbtInfoValue);
    diag_info_pub_.publish(rbtInfo);
}

bool GsNav::doGetGpsPosition(atris_msgs::GetGpsPos::Request& req,   atris_msgs::GetGpsPos::Response& res) 
{

    std::stringstream slati; slati << setprecision(20) << lati_;
    std::stringstream slong; slong << setprecision(20) << long_;

    res.status = gpsstatus;
    res.latitude = slati.str();
    res.longitude = slong.str();

    return true;
}

void GsNav::on_recv_dev_status(const char *data)
{
    Json::Reader reader;
    Json::Value root, rbtInfoValue;
    Json::FastWriter fw;
    atris_msgs::RobotInfo rbtInfo;

    if(!reader.parse(data, root)){
        log_error("parse device health state fail.");
        return ;
    }

    if(!root["laserTopic"].isNull()){
        lidar_master_state = root["laserTopic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["main_lidar"]["error"] = lidar_master_state;
    }

    if(!root["laser2Topic"].isNull()){
        lidar_slave_state = root["laser2Topic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["slave_lidar"]["error"] = lidar_slave_state;
    }

    if(!root["imuTopic"].isNull()) {
        rbtInfoValue["robot_info"]["gyro"]["error"] = root["imuTopic"].asBool() ? 0 : 1;
    }

    if(!root["odomDeltaSpeed"].isNull()){
        odom_delta_spd = root["odomDeltaSpeed"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["odom"]["error"] = odom_delta_spd;
    }

    if(!root["pointcloudTopic"].isNull()){
        rgbd_state = root["pointcloudTopic"].asBool() ? 0 : 1;
        rbtInfoValue["robot_info"]["rgbd"]["error"] = rgbd_state;
    }

    rbtInfoValue["robot_info"]["gaussian_status"] = root;
    rbtInfo.json = fw.write(rbtInfoValue);
    diag_info_pub_.publish(rbtInfo);
}

void GsNav::on_recv_status(char *data)
{
	boost::unique_lock<boost::mutex> lock(mutex);
    std::list<NavStateMonitor>::iterator it;
    for(it = state_monitor_list.begin(); it != state_monitor_list.end(); it ++){
        it->cb(data);
    }
}

void GsNav::add_state_monitor(NavStateMonitor &state_monitor)
{
    log_info("add state monitor success.");
	boost::unique_lock<boost::mutex> lock(mutex);
    state_monitor_id ++;
    state_monitor.id = state_monitor_id;
    state_monitor_list.push_back(state_monitor);
}

int GsNav::del_state_monitor(int id)
{
    if(id == 0)
        return 0;

	boost::unique_lock<boost::mutex> lock(mutex);
    std::list<NavStateMonitor>::iterator it;
    for(it = state_monitor_list.begin(); it != state_monitor_list.end(); it ++){
        if(it->id == id){
            log_info("del state monitor success.");
            state_monitor_list.erase(it);
            return 0;
        }
    }

    return -1;

}

