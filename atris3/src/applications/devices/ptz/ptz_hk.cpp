#include <boost/thread.hpp>
#include <json/json.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "tiny_ros/ros.h"
#include "ptz_hk.h"

#define ISAPI_OUT_LEN 3 * 1024 * 1024
long lUserID;
long irealplayhandle_visible;
long irealplayhandle_audio;
long irealplayhandle_ir;
LONG current_stream_totalsize = 0;
LONG current_stream_size = 0;
char current_time[128] = {0};
char current_time_ir[128] = {0};
Json::Value response_record;
Json::Value root, response_rp;
std::string file_record_video;
std::string file_record_video_ir;
std::string file_record_video_rp;
std::string file_record_video_rp_ir;
std::string file_record_audio;
float m_ir_temperature_max,m_ir_temperature_min,m_ir_temperature_ave,m_ir_temperature_diff;
//#define QINIU_BUCKET_NAME "http://video.ubtrobot.com/"
#define QINIU_BUCKET_NAME "http://10.20.18.188:9020/v1/udfs-tracer"
const std::string FFMPEG_CMD = "/home/atris/atris_app/lib/ffmpeg/ffmpeg";
const std::string FFPROBE_CMD = "/home/atris/atris_app/lib/ffmpeg/ffprobe";
class UploadCaptureFile : public FileObject
{
    virtual void notify(TransferStates state, std::string msg = "", int code = 0)
    {
        this->state = state;
    }

public:
    UploadCaptureFile() : state(TRANSFER_FILE_STARTED) {}
    TransferStates state;
};

Ptz_hk::Ptz_hk()
{
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Ptz_hk::on_recv_ptz_ctrl, this);
    get_ptz_camera_srv_ = nh_.advertiseService(SRV_GET_PTZ_STATUS, &Ptz_hk::doGetPtzStatus, this);
    ptz_control_srv_ = nh_.advertiseService(SRV_PTZ_CONTROL, &Ptz_hk::on_recv_ptz_ctrl, this);
    ptz_temperature_srv_ = nh_.advertiseService(SRV_GET_PTZ_TEMPERATURE, &Ptz_hk::on_recv_get_ptz_temperature, this);
    //init();
    cfg = Config::get_instance();
    utils = Utils::get_instance();
    default_point = 1;
    ptz_state = false;
    is_login = false;
    login_thread = NULL;
    rotate_thread = NULL;
    h_angle_ = 0;
    v_angle_ = 0;
    m_light_status = 0;
    m_wiper_status = 0;
    _channel = 1;
    wait_ms_ = 5000; //default 2s
    is_rotate = false;
    is_rotate_async = false;
    ptz_rotate_state_ = PTZ_ROTATE_IDLE;
    ptz_rotate_data_ = NULL;
    movePtzData_ = NULL;
    capture_uploading_ = false;
    record_uploading_ = false;
    h_speed_ = 15;
    v_speed_ = 15;
    m_save_video_data = true;
    m_is_recording = false;
    m_is_recording_real_play = false;
    m_is_uploading_record = false;
    m_is_uploading_realplay = false;
}

int Ptz_hk::raw_login(const std::string &ip, const int port, const std::string &user, const std::string &passwd)
{

    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;
    struLoginInfo.wPort = port;

    memcpy(struLoginInfo.sDeviceAddress, ip.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, user.c_str(), NAME_LEN);
    memcpy(struLoginInfo.sPassword, passwd.c_str(), NAME_LEN);
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    return lUserID;
}

void Ptz_hk::ptz_login()
{
    is_reconnect = true;
    //NET_DEVICEINFO stDevInfo = {0};
    login_retry = cfg->ptz_login_timeout;
    log_debug("%s:ip:%s,user:%s,pws:%s", __PRETTY_FUNCTION__, cfg->ptz_ip.c_str(), cfg->ptz_user.c_str(), cfg->ptz_psw.c_str());
    while (true){
        if (!utils->check_network_state(cfg->ptz_ip.c_str())){
            log_error("\033[1;31mptz network unreachable.%s\033[0m", cfg->ptz_ip.c_str());
			is_login = false;
            ptz_state = false;
            sleep(2);
            continue;
        }else{
            if (!is_login){
                lUserID = raw_login(cfg->ptz_ip.c_str(), 8000, cfg->ptz_user.c_str(), cfg->ptz_psw.c_str());
                if (lUserID < 0){
                    log_error("\033[1;31mlogin ptz fail:%d, retry, %d\033[0m", lUserID, login_retry);
					is_login = false;
            		ptz_state = false;
                    sleep(1);
                    continue;
                }else{
                    log_info("\033[1;32mlogin to ptz successfully:login:%d,ptz_state:%d\033[0m", is_login, ptz_state);
                    init_hk_compress_info();
                    init_hk_poweroff_mode();
                    init_hk_audio();
                    is_login = true;
                    ptz_state = true;
                    static bool first_login = true;
                    if (first_login){
                        first_login = false;
                        ptz_thread_ = new boost::thread(boost::bind(&Ptz_hk::ptz_rotate_async, this));
                        rotate_thread = new boost::thread(boost::bind(&Ptz_hk::ptz_rotate_run, this));
                        realplay_thread = new boost::thread(boost::bind(&Ptz_hk::ptz_realplay_run, this));
                    }
                }
            }else{
                sleep(10);
            }
        }
    }
}

//0:auto mode,1:manual mode
int Ptz_hk::set_focus_mode(int mode)
{
    unsigned char tmp = (unsigned char)mode;
    //camera focus mode as auto
    DWORD dwReturn = 0;
    NET_DVR_FOCUSMODE_CFG struFocusModeCfg;
    bool ret = NET_DVR_GetDVRConfig(lUserID, NET_DVR_GET_FOCUSMODECFG, _channel, &struFocusModeCfg, sizeof(struFocusModeCfg), &dwReturn);
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
        {
            is_login = false;
            log_debug("[%s] NET_DVR_GetDVRConfig  FOCUSMODECFG err:%d", __FUNCTION__, NET_DVR_GetLastError());
            return -1;
        }
    }
    else
    {
        log_debug("focus mode:%d,%d\n", struFocusModeCfg.byFocusMode, struFocusModeCfg.byAutoFocusMode);
    }
    struFocusModeCfg.byFocusMode = (BYTE)tmp;  //focusmode 0:auto, 1:manual
    struFocusModeCfg.wMinFocusDistance = 0x0a; //MinFocusDistance 10cm;
    ret = NET_DVR_SetDVRConfig(lUserID, NET_DVR_SET_FOCUSMODECFG, _channel, &struFocusModeCfg, sizeof(struFocusModeCfg));
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
        {
            is_login = false;
            log_debug("[%s] NET_DVR_GetDVRConfig  FOCUSMODECFG err:%d", __FUNCTION__, NET_DVR_GetLastError());
            return -2;
        }
        return -1;
    }
    ret = NET_DVR_GetDVRConfig(lUserID, NET_DVR_GET_FOCUSMODECFG, _channel, &struFocusModeCfg, sizeof(struFocusModeCfg), &dwReturn);
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
        {
            is_login = false;
            log_debug("[%s] NET_DVR_GetDVRConfig  FOCUSMODECFG err:%d", __FUNCTION__, NET_DVR_GetLastError());
            return -2;
        }
        return -1;
    }
    else
    {
        log_debug("2focus mode:%d,%d\n", struFocusModeCfg.byFocusMode, struFocusModeCfg.byAutoFocusMode);
    }
    return 0;
}

bool Ptz_hk::get_ptz_status(NET_DVR_PTZABSOLUTEEX_CFG &struBuiltinPTZABSOLUTEEX)
{
    if (!is_login)
        return false;
    DWORD Bytesreturned;
    bool ret = false;
    char m_szStatusBuf[ISAPI_STATUS_LEN];

    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);

    NET_DVR_STD_CONFIG struCfg = {0};
    LONG m_lChannel = _channel;
    struCfg.lpCondBuffer = &m_lChannel;
    struCfg.dwCondSize = sizeof(m_lChannel);
    struCfg.lpOutBuffer = &struBuiltinPTZABSOLUTEEX;
    struCfg.dwOutSize = sizeof(struBuiltinPTZABSOLUTEEX);

    memset(m_szStatusBuf, 0, ISAPI_STATUS_LEN);
    struCfg.lpStatusBuffer = m_szStatusBuf;
    struCfg.dwStatusSize = ISAPI_STATUS_LEN;

    ret = NET_DVR_GetSTDConfig(lUserID, NET_DVR_GET_PTZABSOLUTEEX, &struCfg);
    log_debug("%s%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
            is_login = false;
        return false;
    }
    h_angle_ = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan;
    v_angle_ = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt;
    zoom_ = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom;
    focus_ = struBuiltinPTZABSOLUTEEX.struPTZCtrl.dwFocus;
    log_debug("\033[1;32m pan:%f,tilt:%f,zoom:%f,focus:%f,fspeed:%f,h_speed_:%f\033[0m", h_angle_, v_angle_, zoom_, focus_, struBuiltinPTZABSOLUTEEX.fHorizontalSpeed, h_speed_);
    return true;
}

bool Ptz_hk::set_ptz_status(NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX)
{
    if (!is_login)
        return false;
    NET_DVR_STD_CONFIG struCfg = {0};
    char szStatusBuf[ISAPI_STATUS_LEN];
    memset(szStatusBuf, 0, ISAPI_STATUS_LEN);
    struCfg.lpCondBuffer = &_channel;
    struCfg.dwCondSize = sizeof(_channel);
    struCfg.lpInBuffer = &struBuiltinPTZABSOLUTEEX;
    struCfg.dwInSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struCfg.lpStatusBuffer = szStatusBuf;
    struCfg.dwStatusSize = ISAPI_STATUS_LEN;
    bool ret = NET_DVR_SetSTDConfig(lUserID, NET_DVR_SET_PTZABSOLUTEEX, &struCfg);
    log_debug("%s%s:%d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
            is_login = false;

        return false;
    }
    return true;
}

bool Ptz_hk::get_thermal_ability()
{
    if (!is_login)
        return false;
    DWORD Bytesreturned;
    bool ret = false;
    char m_szStatusBuf[ISAPI_STATUS_LEN];
    return true;
}
bool Ptz_hk::set_thermal_ability()
{
    if (!is_login)
        return false;
    return true;
}

void Ptz_hk::get_dvr_Config()
{
    char szFilename[64] = "config.txt";
    bool ret = NET_DVR_GetConfigFile(lUserID, szFilename);
    log_debug("%s:%d\n", __PRETTY_FUNCTION__, ret);
}

void Ptz_hk::ptz_rotate_async()
{
    int vel_angle, hor_angle;
    int interval;
    Json::Reader reader;
    Json::Value value;
    boost::unique_lock<boost::mutex> lock(ptz_mutex_, boost::defer_lock);
    while (true)
    {
        lock.lock();
        if ((!ptz_rotate_data_) || (!movePtzData_))
        {
            ptz_cond_.wait(lock);
        }
        if (movePtzData_)
        {
            reader.parse(movePtzData_->msg, value);
            int vertical = value["content"]["verticalangel"].asInt();
            int horizon = value["content"]["horizontalangel"].asInt();
            is_rotate_async = true;
            ptz_move(vertical, horizon, 0, 300);
            is_rotate_async = false;
            delete movePtzData_;
            movePtzData_ = NULL;
        }

        is_rotate_async = false;

        if (ptz_rotate_data_)
        {
            reader.parse(ptz_rotate_data_->msg, value);
            interval = value["content"]["interval"].asInt();
            vel_angle = value["content"]["verticalangel"].asInt();
            hor_angle = value["content"]["horizontalangel"].asInt();
            default_point = value["content"]["defaultpoint"].asInt();

            delete ptz_rotate_data_;
            ptz_rotate_data_ = NULL;

            is_rotate_async = true;
        }

        lock.unlock();

        int64_t now, last, diff;
        bool rotate_reset = true;
        int vel[3] = {-vel_angle, vel_angle * 2, -vel_angle};
        while (is_rotate_async)
        {
            if (rotate_reset)
            {
                rotate_reset = false;
                ptz_move_point(default_point);
            }

            for (size_t i = 0; i < 3 && is_rotate_async; i++)
            {
                now = ros::Time::now().toSec() * 1000;
                ptz_move(0, vel[i], 0);
                lock.lock();
                if (is_rotate_async)
                {
                    last = ros::Time::now().toSec() * 1000;
                    diff = last - now;
                    if (diff > 0 && diff < interval)
                    {
                        ptz_cond_.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(interval - diff));
                    }
                }
                lock.unlock();
            }

            if (is_rotate_async)
            {
                now = ros::Time::now().toSec() * 1000;
                ptz_move(hor_angle, 0, 0);
                lock.lock();
                if (is_rotate_async)
                {
                    last = ros::Time::now().toSec() * 1000;
                    diff = last - now;
                    if (diff > 0 && diff < interval)
                    {
                        ptz_cond_.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(interval - diff));
                    }
                }
                lock.unlock();
            }
        }
        ptz_stop();
    }
}

void Ptz_hk::change_ptz_speed(float h_speed, float v_speed)
{
    log_info("h:%f,v:%f\n", h_speed, v_speed);
    if (h_speed < 0)
        return;
    if (v_speed < 0)
        return;
    h_speed_ = h_speed;
    v_speed_ = v_speed;
}

bool Ptz_hk::ptz_rotate_run()
{
    boost::unique_lock<boost::mutex> rotate_lock(ptz_mutex_, boost::defer_lock);
    boost::unique_lock<boost::mutex> pause_lock(ptz_wait_mutex_, boost::defer_lock);

    while (1)
    {
        rotate_lock.lock();
        ptz_rotate_con_.wait(rotate_lock);

        ptz_rotate_state_ = PTZ_ROTATE_RUNING;
        is_rotate = true;
        pause_lock.lock();
        if (ptz_rotate_pause_flag)
        {
            ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
            ptz_pause_con_.wait(pause_lock);
        }
        pause_lock.unlock();
        ptz_move_point(def_point_);
        int v_count = 0;
        float vel[3] = {-v_angle_, v_angle_ * 2, -v_angle_};

        int times = 0;
        if (h_angle_ > 0 && h_angle_ <= 180)
        {
            times = 360 / h_angle_;
        }
        else if (h_angle_ > 180 && h_angle_ < 360)
        {
            times = 1;
            h_angle_ = h_angle_ - 360;
        }
        else if (h_angle_ == 0 || h_angle_ == 360)
        {
            times = 0;
        }
        rotate_lock.unlock();

        v_count = 0;
        while (v_count < 3 && is_rotate)
        {
            if (v_angle_ == 0)
                break;
            log_info("ptz v_angle=%d", v_angle_);
            ptz_move(0, vel[v_count], zoom_);
            v_count++;
            rotate_lock.lock();
            if (is_rotate && wait_ms_ > 0)
            {
                ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
            }
            rotate_lock.unlock();
            pause_lock.lock();
            if (ptz_rotate_pause_flag)
            {
                ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                ptz_pause_con_.wait(pause_lock);
                ptz_rotate_state_ = PTZ_ROTATE_RUNING;
            }
            pause_lock.unlock();
        }

        log_info("ptz times=%d, pauseflag=%d", times, ptz_rotate_pause_flag);
        while (is_rotate && times != 0)
        {
            if (h_angle_ != 0)
            {
                log_info("ptz h_angle=%d", h_angle_);
                ptz_move(h_angle_, 0, zoom_);
                times--;
                rotate_lock.lock();
                if (is_rotate && wait_ms_ > 0)
                {
                    ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
                }
                rotate_lock.unlock();
                pause_lock.lock();
                if (ptz_rotate_pause_flag)
                {
                    ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                    ptz_pause_con_.wait(pause_lock);
                    ptz_rotate_state_ = PTZ_ROTATE_RUNING;
                }
                pause_lock.unlock();
            }
            else
            {
                break;
            }

            v_count = 0;
            while (v_count < 3 && is_rotate)
            {
                if (v_angle_ == 0)
                    break;
                log_info("ptz v_angle=%d", v_angle_);
                ptz_move(0, vel[v_count], zoom_);
                v_count++;
                rotate_lock.lock();
                if (is_rotate && wait_ms_ > 0)
                {
                    ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
                }
                rotate_lock.unlock();
                pause_lock.lock();
                if (ptz_rotate_pause_flag)
                {
                    ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                    ptz_pause_con_.wait(pause_lock);
                    ptz_rotate_state_ = PTZ_ROTATE_RUNING;
                }
                pause_lock.unlock();
            }
        }
        pause_lock.lock();
        if (ptz_rotate_pause_flag)
        {
            ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
            ptz_pause_con_.wait(pause_lock);
            ptz_rotate_state_ = PTZ_ROTATE_RUNING;
        }
        pause_lock.unlock();
        ptz_move_point(def_point_);
        is_rotate = false;
        ptz_rotate_state_ = PTZ_ROTATE_STOP;
    }
    return true;
}

bool Ptz_hk::ptz_realplay_run()
{
    while (1)
    {
    }
}

bool Ptz_hk::ptz_rotate_stop()
{
    if (!is_login)
    {
        log_info("ptz not login");
        return false;
    }

    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    is_rotate = false;
    ptz_rotate_pause_flag = false;
    //ptz_rotate_con_.notify_all();
    ptz_pause_con_.notify_all();
    return true;
}

bool Ptz_hk::ptz_is_idle()
{
    return (ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP);
}

bool Ptz_hk::ptz_rotate(int h_angle, int v_angle, int zoom, int def_point)
{
    if (!is_login)
    {
        log_info("ptz not login");
        return false;
    }

    if (is_rotate)
    {
        ptz_rotate_stop();
    }
    while (ptz_rotate_state_ != PTZ_ROTATE_STOP && ptz_rotate_state_ != PTZ_ROTATE_IDLE)
    {
        usleep(100 * 1000);
    }
    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    ptz_rotate_state_ = PTZ_ROTATE_PREPARE;
    this->h_angle_ = h_angle;
    this->v_angle_ = v_angle;
    this->zoom_ = zoom;
    this->def_point_ = def_point;
    ptz_rotate_con_.notify_all();
    return true;
}

bool Ptz_hk::ptz_rotate_pause(int onoff)
{
    if (ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP)
    {
        return false;
    }
    log_info("ptz pause:%d, state=%d", onoff, ptz_rotate_state_);
    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    if (onoff == 1)
    {
        ptz_rotate_pause_flag = true;
        if (ptz_rotate_state_ == PTZ_ROTATE_PAUSED)
        {
            return true;
        }
    }
    else
    {
        ptz_rotate_pause_flag = false;
        if (ptz_rotate_state_ != PTZ_ROTATE_PAUSED)
        {
            return true;
        }
        ptz_pause_con_.notify_all();
    }
    return true;
}

bool Ptz_hk::ptz_rotate_pause()
{
    if (ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP)
    {
        return false;
    }
    log_info("ptz pause state=%d", ptz_rotate_state_);
    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    if (ptz_rotate_state_ == PTZ_ROTATE_PAUSED)
    {
        ptz_rotate_pause_flag = false;
        ptz_pause_con_.notify_all();
    }
    else if (ptz_rotate_state_ == PTZ_ROTATE_PREPARE || ptz_rotate_state_ == PTZ_ROTATE_RUNING)
    {
        ptz_rotate_pause_flag = true;
    }
    return true;
}

bool Ptz_hk::doGetPtzStatus(atris_msgs::GetPtzStatus::Request &req,
                            atris_msgs::GetPtzStatus::Response &res)
{


    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        log_info("%s:%d get %s", __PRETTY_FUNCTION__, __LINE__, ret ? "OK" : "Fail");
        return false;
    }
    res.pan_angle = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan;
    res.tilt_angle = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt;
    res.zoom_value = struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom;
    res.focus_value = struBuiltinPTZABSOLUTEEX.struPTZCtrl.dwFocus;
    res.light_status = m_light_status;
    res.wiper_status = m_wiper_status;


    if (get_state())
    {
        if (is_rotate_async)
            res.status = atris_msgs::GetPtzStatus::Response::PTZ_ROTATE_STATUS;
        else
            res.status = atris_msgs::GetPtzStatus::Response::PTZ_IDLE_STATUS;
    }
    else
    {
        res.status = atris_msgs::GetPtzStatus::Response::PTZ_NOT_LOGIN_STATUS;
    }

    return true;
}

bool Ptz_hk::on_recv_get_ptz_temperature(atris_msgs::GetPtzTemperature::Request& req,
  atris_msgs::GetPtzTemperature::Response& res) {
	if(!is_login){
    	log_error("ptz not login");
        return false;
    } 

	log_info("%s start[%.3f,%.3f] end[%.3f,%.3f]",__FUNCTION__,
			 req.start_point_x,req.start_point_y,req.end_point_x, req.end_point_y);
			
	int start_x = Utils::get_instance()->float2int(req.start_point_x);
	int start_y = Utils::get_instance()->float2int(req.start_point_y);
	int end_x = Utils::get_instance()->float2int(req.end_point_x);
	int end_y = Utils::get_instance()->float2int(req.end_point_y);

	float min = 0.0f;
	float max = 0.0f;
	float avg = 0.0f;
	float dif = 0.0f;
    DWORD minLocate = 0;
    DWORD maxLocate = 0;

	ptz_get_range_temperature(start_x, start_y, end_x, end_y, min, max, avg, dif, minLocate, maxLocate);
    res.temperature_max  = max;
    res.temperature_min  = min;
    res.temperature_ave  = avg;
    res.temperature_diff = dif;
    res.temperature_min_locate = minLocate;
    res.temperature_max_locate = maxLocate;
    return true;
}

bool Ptz_hk::ptz_get_range_temperature(int start_x, int start_y, int end_x, int end_y,
                                       float &min, float &max, float &avg, float &dif, DWORD &min_locate, DWORD &max_locate)
{
    if (!is_login){
		log_info("%s not login", __FUNCTION__);
        return false;
	}

	if (start_x > end_x || start_y > end_y){
		log_info("invalid point range");
		return false;
	}

    NET_DVR_JPEGPICTURE_WITH_APPENDDATA struJpegPictureWithAppendAata = {0};
    char szLan[256] = {0};
    if (struJpegPictureWithAppendAata.pJpegPicBuff == NULL){
        struJpegPictureWithAppendAata.pJpegPicBuff = new char[2 * 1024 * 1024];
        memset(struJpegPictureWithAppendAata.pJpegPicBuff, 0, 2 * 1024 * 1024);
    }

    if (struJpegPictureWithAppendAata.pP2PDataBuff == NULL){
        struJpegPictureWithAppendAata.pP2PDataBuff = new char[2 * 1024 * 1024];
        memset(struJpegPictureWithAppendAata.pP2PDataBuff, 0, 2 * 1024 * 1024);
    }

    bool bret = NET_DVR_CaptureJPEGPicture_WithAppendData(lUserID, 2, &struJpegPictureWithAppendAata);
    if (!bret){
        log_info("[%s]NET_DVR_CaptureJPEGPicture_WithAppendData err:%u=%s", __FUNCTION__, NET_DVR_GetLastError(), NET_DVR_GetErrorMsg());
        return false;
    }

    int length = struJpegPictureWithAppendAata.dwP2PDataLen / sizeof(float);
    float *pTemperatureData = (float *)struJpegPictureWithAppendAata.pP2PDataBuff;
    int img_width = struJpegPictureWithAppendAata.dwJpegPicWidth;//640*512
    int img_height = struJpegPictureWithAppendAata.dwJpegPicHeight;
    float min_tmp = 1000.0f;
    float max_tmp = 0.0f;
    float cur_tmp = 0.0f;
    double sum_tmp = 0.0f;
    DWORD minLocate = 0;
    DWORD maxLocate = 0;
	DWORD tmp_count = 0, idx_w, idx;

    log_info("%s start[%d,%d] end[%d,%d] w:%d h:%d len:%d", __FUNCTION__, start_x, start_y, end_x, end_y, img_width, img_height, length);
	if (length < img_width * img_height){
    	log_info("invalid tmp data buffer");
		return false;
	}
	if (start_x < 0) start_x = 0;
	if (start_y < 0) start_y = 0;
	if (end_x > img_width) end_x = img_width;
	if (end_y > img_height) end_y = img_height;

    if (length > 0 && pTemperatureData != NULL){
		
        for (int h = start_y; h <= end_y; ++h){
			idx_w = h * img_width;
			for (int w = start_x; w < end_x; w++){
			    idx = idx_w + w;
				cur_tmp = pTemperatureData[idx];
        	    if (cur_tmp > max_tmp){
        	        max_tmp = cur_tmp;
        	        maxLocate = idx;
        	    }
        	    if (cur_tmp < min_tmp){
        	        min_tmp = cur_tmp;
        	        minLocate = idx;
        	    }
				++tmp_count;
				sum_tmp += cur_tmp;
        	}
		}

		min = min_tmp;
		max = max_tmp;
    	dif = fabs(min - max);
		avg = sum_tmp/tmp_count;
        min_locate = minLocate;
        max_locate = maxLocate;

        log_info("%s min:[%.3f,%d] max:[%.3f,%d] dif:%.3f avg:%.3f",
				 __FUNCTION__, min_tmp, minLocate, max_tmp, maxLocate, dif, avg);
		return true;
    }else{
        log_info("[%s]Temperature data is invalid.", __FUNCTION__);
        return false;
    }
    return true;
}

bool Ptz_hk::on_recv_ptz_ctrl(atris_msgs::PtzControl::Request& req,
  atris_msgs::PtzControl::Response& res) {
  if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ROTATE_START) {
    res.result = ptz_rotate(req.h_angle, req.v_angle, req.zoom, req.default_point);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ROTATE_PAUSE) {
    res.result = ptz_rotate_pause(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ROTATE_RESUME) {
    res.result = ptz_rotate_pause(0);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ROTATE_STOP) {
    res.result = ptz_rotate_stop();
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_DEFAULT_POINT) {
    res.result = ptz_move_point(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE) {
    res.result = ptz_move(req.h_angle, req.v_angle, req.zoom);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_CONTROL) {
    //TODO 
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_CAPTURE) {
    std::string visiable_light_url,infrared_url, imagebuf;

    res.result = ptz_capture2(visiable_light_url, infrared_url, imagebuf);
    res.visible_light_url = visiable_light_url;
    res.infrared_url = infrared_url;
    res.imagebuf = imagebuf;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_RECORD_VIDEO_ON) {
    record_thread_ = new boost::thread(boost::bind(&Ptz_hk::start_preview_v40, this));
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_RECORD_VIDEO_OFF) {
    std::string visible_light_video_url, infrared_video_url;
    res.result = stop_save_realdata_record(visible_light_video_url, infrared_video_url);
    res.visible_light_video_url = visible_light_video_url;
    res.infrared_video_url = infrared_video_url;
  }else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_RECORD_AUDIO_ON) {
    audio_thread_ = new boost::thread(boost::bind(&Ptz_hk::start_record_audio, this));
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_RECORD_VIDEO_OFF) {
    std::string audio_url;
    res.result = stop_save_audio_record(audio_url);
    log_debug("###audio_url : %s",audio_url.c_str());
    res.audio_url = audio_url;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ZOOM_CONTROL) {
    //TODO
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_FOCUS_CONTROL) {
    //TODO
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_WIPER_CONTROL) {
    //TODO
    res.result = true;
  }

  return true;
}

//recv action control from remote mqtt
void Ptz_hk::on_recv_ptz_ctrl(const atris_msgs::SignalMessage &msg)
{
    log_info("[%s]:%s\n", __PRETTY_FUNCTION__, msg.title.c_str());
    Json::Reader reader;
    Json::Value req, resp;
    std::string resp_name;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";
    if (msg.title == "request_start_rotate_camera") {
        resp_name = "response_start_rotate_camera";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["interval"].isNull() || req["content"]["verticalangel"].isNull()
                || req["content"]["horizontalangel"].isNull() || req["content"]["defaultpoint"].isNull()) {
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }

        if (!is_rotate_async) {
            boost::unique_lock<boost::mutex> lock(ptz_mutex_);
            if (ptz_rotate_data_) {
                delete ptz_rotate_data_;
            }
            ptz_rotate_data_ = new atris_msgs::SignalMessage();
            *ptz_rotate_data_ = msg;
            ptz_cond_.notify_all();
            Utils::get_instance()->responseResult(msg, resp, resp_name);
        } else {
            resp["result"] = "fail_already_rotate";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
        }
    } else if(msg.title == "request_stop_rotate_camera") {
        
        resp_name = "response_stop_rotate_camera";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (is_rotate_async) {
            boost::unique_lock<boost::mutex> lock(ptz_mutex_);
            is_rotate_async = false;
            ptz_cond_.notify_all();
        }
        Utils::get_instance()->responseResult(msg, resp, resp_name);
        log_error("%s,resp:%s",__PRETTY_FUNCTION__,resp.asString().c_str());
    } else if(msg.title == "request_start_ptz_rotate") {
        resp_name = "response_start_rotate";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        ptz_rotate(req["h"].asInt(), req["v"].asInt(), req["z"].asInt(), req["d"].asInt());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_stop_ptz_rotate") {
        resp_name = "response_stop_ptz_rotate";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        ptz_rotate_stop();
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_pause_ptz_rotate") {
        log_info("ptz pause");
        resp_name = "response_pause_ptz_rotate";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        ptz_rotate_pause(1);
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_resume_ptz_rotate") {
        resp_name = "response_resume_ptz_rotate";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        ptz_rotate_pause(0);
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_prs_ptz_rotate") {
        resp_name = "response_prs_ptz_rotate";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        ptz_rotate_pause();
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_rotate_camera_status") {
        resp_name = "response_rotate_camera_status";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        resp["status"] = is_rotate_async ? 1 : 0;
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_reset_ptz") {
        resp_name = "response_reset_ptz";
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (is_rotate_async) {
            boost::unique_lock<boost::mutex> lock(ptz_mutex_);
            is_rotate_async = false;
            ptz_cond_.notify_all();
        }
        ptz_move_point(def_point_);
        resp["result"] = "success";
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_move_ptz") {
        resp_name = "response_move_ptz";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["verticalangel"].isNull() ||
            req["content"]["horizontalangel"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
            if (!is_rotate_async) {
                boost::unique_lock<boost::mutex> lock(ptz_mutex_);
                if (movePtzData_) {
                    delete movePtzData_;
                }
                movePtzData_ = new atris_msgs::SignalMessage();
                *movePtzData_ = msg;
                ptz_cond_.notify_all();
                resp["result"] = "success";
            } else {
                resp["result"] = "fail_already_rotate";
            }
        }
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }else if (msg.title == "request_ptz_move_control") {
        resp_name = "response_ptz_move_control";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["direction"].isNull())
        {
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            resp["result"] = "success";
            ptz_move_control(req["content"]["direction"].asInt());
        }
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }else if (msg.title == "request_ptz_stop_control") {
        resp_name = "response_ptz_stop_control";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
    
        resp["result"] = "success";
        ptz_move_control(4);
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if (msg.title == "request_ptz_hv_angle")
    {
        resp_name = "response_ptz_hv_angle";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["angle"].isNull())
        {
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            resp["result"] = "success";
        }
        switch (req["content"]["direction"].asInt())
        {
            case 0:
                ptz_left_ex(req["content"]["angle"].asFloat());
                break;
            case 1:
                ptz_up_ex(req["content"]["angle"].asFloat());
                break;
            default:
                log_info("index:%d\n",req["content"]["direction"].asInt());
        };
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_ptz_capture") {
        resp_name = "response_ptz_capture";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        } else {
            resp["visible_light_url"] = "";
            resp["infrared_url"] = "";
            resp["result"] = "success";
            if (0 ==uploading_status)
            {
                  if (capture_thread_)
                {
                    delete capture_thread_;
                    capture_thread_ = NULL;
                }
                capture_thread_ = new boost::thread(boost::bind(&Ptz_hk::ptz_capture, this, msg));
            }
            else
            {
                switch(uploading_status){
                    case -1:
                        resp["result"] = "fail_upload";
                        break;
                    case 1:
                        resp["result"] = "fail_visible_video_uploading";
                        break;
                    case 2:
                        resp["result"] = "fail_infrared_video_uploading";
                        break;
                    case 3:
                        resp["result"] = "fail_capture_visible_uploading";
                        break;
                    case 4:
                        resp["result"] = "fail_capture_infrared_uploading";
                        break;
                    case 5:
                        resp["result"] = "fail_record_audio_uploading";
                        break;
                    default:
                        break;
                }
                Utils::get_instance()->responseResult(msg, resp, resp_name);
            }
        }
    } else if (msg.title == "request_ptz_focus_control") {
        resp_name = "response_ptz_focus_control";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["operation"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
            /*
            TODO 
            */
            resp["result"] = "success";
        }
        ptz_focus_control(req["content"]["operation"].asInt());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_ptz_zoom_control") {
        resp_name = "response_ptz_zoom_control";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["operation"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
            resp["result"] = "success";
            ptz_zoom_control(req["content"]["operation"].asInt());
        }
	    Utils::get_instance()->responseResult(msg, resp, resp_name);
    }else if (msg.title == "request_ptz_zoom_value")
    {
        resp_name = "response_ptz_zoom_value";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["zoom"].isNull())
        {
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            ptz_zoom(req["content"]["zoom"].asInt());
            resp["result"] = "success";
        }
    
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if (msg.title == "request_ptz_record")
    {
        resp_name = "response_ptz_record";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["switch"].isNull())
        {
            resp["result"] = "fail_invalid_data";
        }
        else
        {
      if (0 == uploading_status){

                 switch (req["content"]["switch"].asInt())
                {
                    case 0:
                    {  
                            std::string video_light_url, video_infrared_url;
                            if(0 == stop_save_realdata_record(video_light_url, video_infrared_url)){
                                resp["result"] = "success";
                                resp["switch_ret"] = 0;
                                resp["visible_light_video_url"] = video_light_url;
                                resp["infrared_video_url"] = video_infrared_url;
                            }else{
                                resp["result"]="fail_upload_fail";
                                resp["switch_ret"] = 0;
                            }

                            Utils::get_instance()->responseResult(msg, resp, resp_name);
                    }
                    break;
                    case 1:
                    {
                            log_info("%s,%d\n",__FUNCTION__,__LINE__);
                            if(0 == start_preview_v40()){
                                // if (0 == save_real_data())
                                //     {
                                        resp["result"] = "success";
                                        resp["switch_ret"] = 1;
                                    }
                                    else
                                    {
                                        resp["result"] = "fail_invalid_data";
                                    }
                                    Utils::get_instance()->responseResult(msg, resp, resp_name); 
                            // }else
                            // {
                            //     resp["result"] = "fail_invalid_data";
                            // } 
                    }
                    break;
                    case 2:
                    {
                        cancel_save_realdata_record();
                        resp["result"] = "success";
                        resp["switch_ret"] = 2;
                        Utils::get_instance()->responseResult(msg, resp, resp_name);
                    }
                    break;
                    default:
                        log_error("undefine case%d", req["content"]["switch"]);
                        break;
                }
                //resp[""]
          }else{
                switch(uploading_status){
                    case -1:
                        resp["result"] = "fail_upload";
                        break;
                    case 1:
                        resp["result"] = "fail_visible_video_uploading";
                        break;
                    case 2:
                        resp["result"] = "fail_infrared_video_uploading";
                        break;
                    case 3:
                        resp["result"] = "fail_capture_visible_uploading";
                        break;
                    case 4:
                        resp["result"] = "fail_capture_infrared_uploading";
                        break;
                    case 5:
                        resp["result"] = "fail_record_audio_uploading";
                        break;
                    default:
                        break;
                }
                Utils::get_instance()->responseResult(msg, resp, resp_name);
                uploading_status = 0;
            }
        }
    }
    else if (msg.title == "request_sound_record")
    {
        resp_name = "response_sound_record";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["switch"].isNull())
        {
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            if( 0 == uploading_status){
            switch (req["content"]["switch"].asInt())
            {
                case 0:
                {
                    std::string audio_url;
                    stop_save_audio_record(audio_url);
                    resp["result"] = "success";
                    resp["switch_ret"] = 0;
                    log_info("\033[1;32m-----%s------\033[0m", audio_url.c_str());
                    resp["url"] = audio_url;
                    Utils::get_instance()->responseResult(msg, resp, resp_name);
                }
                break;
                case 1:
                {
                    if (0 == start_record_audio())
                    {
                        resp["result"] = "success";
                        resp["switch_ret"] = 1;
                    }
                    else
                    {
                        resp["result"] = "fail_invalid_data";
                    }
                    Utils::get_instance()->responseResult(msg, resp, resp_name);
                }
                break;
                case 2:
                {
                    cancel_save_audio();
                    resp["result"] = "success";
                    resp["switch_ret"] = 2;
                    Utils::get_instance()->responseResult(msg, resp, resp_name);
                }
                break;
                default:
                    log_error("undefine case%d", req["content"]["switch"]);
                    break;
            }   
            }else{
            switch(uploading_status){
                    case -1:
                        resp["result"] = "fail_upload";
                        break;
                    case 1:
                        resp["result"] = "fail_visible_video_uploading";
                        break;
                    case 2:
                        resp["result"] = "fail_infrared_video_uploading";
                        break;
                    case 3:
                        resp["result"] = "fail_capture_visible_uploading";
                        break;
                    case 4:
                        resp["result"] = "fail_capture_infrared_uploading";
                        break;
                    case 5:
                        resp["result"] = "fail_record_audio_uploading";
                        break;
                    default:
                        break;
                }
                Utils::get_instance()->responseResult(msg, resp, resp_name);
                uploading_status = 0;
            }
            //resp[""]
        }
    } else if (msg.title == "request_switch_ptz_wiper") {
        resp_name = "response_switch_ptz_wiper";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["switch"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
        	if(ptz_wiper_control(req["content"]["switch"].asInt())){
                m_wiper_status = req["content"]["switch"].asInt();
            }
            resp["result"] = "success";
        }

        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_switch_ptz_light") {
        resp_name = "response_switch_ptz_light";
        reader.parse(msg.msg, req);
        if (!is_login){
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["switch"].isNull()){
            resp["result"] = "fail_invalid_data";
        }else{
          if(ptz_light_control(req["content"]["switch"].asInt())){
            m_light_status = req["content"]["switch"].asInt();  
          }
          resp["result"] = "success";
        }
 
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if (msg.title == "request_ptz_param")
    {
        resp_name = "response_ptz_param";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        get_current_ptz_info();
        resp["pan"] = h_angle_;
        resp["tilt"] = v_angle_;
        resp["zoom"] = zoom_;
        resp["focus"] = focus_;

        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if (msg.title == "request_ptz_hv_speed")
    {
        resp_name = "response_ptz_speed";
        reader.parse(msg.msg, req);
        if (!is_login)
        {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        change_ptz_speed(req["content"]["h_speed"].asFloat(), req["content"]["v_speed"].asFloat());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }else if(msg.title=="request_ptz_ir_temperature_info"){
        reader.parse(msg.msg,req);
        if(!is_login){
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
			log_info("not login");
            return;
        } 

		int start_x = req["content"]["start_point"]["x"].asInt();
		int start_y = req["content"]["start_point"]["y"].asInt();
		int end_x = req["content"]["end_point"]["x"].asInt();
		int end_y = req["content"]["end_point"]["y"].asInt();
		log_info("req start[%d,%d] end[%d,%d]", start_x, start_y, end_x, end_y);

		float min = 0.0f, max = 0.0f, avg = 0.0f, dif = 0.0f;
        DWORD minLocate = 0;
        DWORD maxLocate = 0;
        ptz_get_range_temperature(start_x, start_y, end_x, end_y, min, max, avg, dif, minLocate, maxLocate);
        resp["temperature_max"]= max;
        resp["temperature_min"] = min;
        resp["temperature_ave"] = avg;
        resp["temperature_diff"] = dif;
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    /*
    else
    {
        log_debug("\033[1;33m unknow title command !\033[0m\n");
    }
    */
}

bool Ptz_hk::ptz_temperature(){
   ;// get_ptz_status();
}

bool Ptz_hk::check_state()
{
    bool ret = false;
    long handle;
    is_reconnect = false;

    log_info("check ptz state.");
    if (!cfg->ptz_ip.empty() && !cfg->ptz_user.empty() && !cfg->ptz_psw.empty())
    {
        lUserID = raw_login(cfg->ptz_ip.c_str(), 8000, cfg->ptz_user.c_str(), cfg->ptz_psw.c_str());
        if (lUserID < 0)
        {
            log_error("ptz not work");
            ret = false;
        }
        else{
            log_debug("ptz work fine.");
            ret = true;
        }
    } else {
        log_error("%s ptz config error.", __FUNCTION__);
    }
    ptz_state = ret;
    log_info("check ptz state end.");
    return ret;
}

bool Ptz_hk::get_state()
{
    return ptz_state;
}

int Ptz_hk::init()
{
    log_info("ptz init");
    //hk sdk init
    NET_DVR_Init();
    NET_DVR_SetLogPrint(true);
    NET_DVR_SetConnectTime(2000,1);
    NET_DVR_SetReconnect(10000,true);
    unsigned int uhkVersion = NET_DVR_GetSDKBuildVersion();
    log_debug("HCNetSDK:V%u.%u.%u.%u",(0xff000000 & uhkVersion)>>24,(0x00ff0000&uhkVersion)>>16,(0x0000ff00&uhkVersion)>>8,(0x000000ff & uhkVersion));

    is_login = false;
    if(login_thread != NULL){
        log_info("delete login thread before.");
        login_retry = -1;
        sleep(2);
        delete login_thread;
        login_thread = NULL;
    }
    login_thread = new boost::thread(boost::bind(&Ptz_hk::ptz_login, this));
    return 0;
}
int  Ptz_hk::init_hk_audio(){
    NET_DVR_AUDIO_INPUT_PARAM audio_input = {0};
    DWORD dwChannel = 1;
    DWORD dwReturned = 0;
    int ret = NET_DVR_GetDVRConfig(lUserID,NET_DVR_GET_AUDIO_INPUT,1,&audio_input,sizeof(NET_DVR_AUDIO_INPUT_PARAM),&dwReturned);
    audio_input.byAudioInputType = 0;
    audio_input.byVolume = 100;
    audio_input.byEnableNoiseFilter = 1;
    ret = NET_DVR_SetDVRConfig(lUserID,NET_DVR_SET_AUDIO_INPUT,1,&audio_input,sizeof(audio_input));

    NET_DVR_COMPRESSION_AUDIO audio_compress = {0};
    ret = NET_DVR_GetDVRConfig(lUserID,NET_DVR_GET_COMPRESSCFG_AUD,1,&audio_compress,sizeof(audio_compress),&dwReturned);
    audio_compress.byAudioEncType = 7;
    ret = NET_DVR_SetDVRConfig(lUserID,NET_DVR_SET_COMPRESSCFG_AUD,1,&audio_compress,sizeof(audio_compress));

    NET_DVR_MULTI_STREAM_COMPRESSIONCFG  stream_compressinfo= {0};//3216 远程获取多码流压缩参数 
    ret = NET_DVR_GetDVRConfig(lUserID,NET_DVR_GET_MULTI_STREAM_COMPRESSIONCFG,1,&stream_compressinfo,sizeof(stream_compressinfo),&dwReturned);
    stream_compressinfo.struStreamPara.byVideoEncType = 1;
    stream_compressinfo.struStreamPara.byAudioEncType = 7;
    ret = NET_DVR_SetDVRConfig(lUserID,NET_DVR_SET_MULTI_STREAM_COMPRESSIONCFG,1,&stream_compressinfo,sizeof(stream_compressinfo));


    return ret;

}

int Ptz_hk::init_hk_compress_info(){
    NET_DVR_MULTI_STREAM_COMPRESSIONCFG  stream_compressinfo= {0};//3216 远程获取多码流压缩参数 
    DWORD dwChannel = 1;
    DWORD dwReturned = 0;
    int ret = NET_DVR_GetDVRConfig(lUserID,NET_DVR_GET_MULTI_STREAM_COMPRESSIONCFG,1,&stream_compressinfo,sizeof(stream_compressinfo),&dwReturned);
    stream_compressinfo.struStreamPara.byVideoEncType = 1;
    stream_compressinfo.struStreamPara.byAudioEncType = 7;
    ret = NET_DVR_SetDVRConfig(lUserID,NET_DVR_SET_MULTI_STREAM_COMPRESSIONCFG,1,&stream_compressinfo,sizeof(stream_compressinfo));
    return ret;
}

int Ptz_hk::init_hk_poweroff_mode(){
    NET_DVR_PTZ_POWEROFFMEMCFG poweroffmode = {0};
    DWORD dwChannel = 1;
    DWORD dwReturned = 0;
    int ret = NET_DVR_GetDVRConfig(lUserID,NET_DVR_GET_POWEROFFMEMCFG,1,&poweroffmode,sizeof(poweroffmode),&dwReturned);
    poweroffmode.byResumeTimePoint = 0xff;
    ret = NET_DVR_SetDVRConfig(lUserID,NET_DVR_SET_POWEROFFMEMCFG,1,&poweroffmode,sizeof(poweroffmode));
    return ret;
}

int Ptz_hk::set_horiz(int degree)
{
    return 0;
}

int Ptz_hk::set_vertical(int degree)
{
    return 0;
}
#if 1
void CALLBACK GetThermInfoCallback(DWORD dwType, void *lpBuffer, DWORD dwBufLen, void *pUserData)
{
    if (dwType == NET_SDK_CALLBACK_TYPE_DATA)
    {
        LPNET_DVR_THERMOMETRY_UPLOAD lpThermometry = new NET_DVR_THERMOMETRY_UPLOAD;
        memcpy(lpThermometry, lpBuffer, sizeof(*lpThermometry));

        if ((lpThermometry->byRuleCalibType == 1) || (lpThermometry->byRuleCalibType == 2)) //???/线测???
        {
            m_ir_temperature_max = lpThermometry->struLinePolygonThermCfg.fMaxTemperature;
            m_ir_temperature_min = lpThermometry->struLinePolygonThermCfg.fMinTemperature;
            m_ir_temperature_ave = lpThermometry->struLinePolygonThermCfg.fAverageTemperature;
            m_ir_temperature_diff = lpThermometry->struLinePolygonThermCfg.fTemperatureDiff;
            log_debug("fMaxTemperature[%f]fMinTemperature[%f]fAverageTemperature[%f]fTemperatureDiff[%f]\n",
                   lpThermometry->struLinePolygonThermCfg.fMaxTemperature, lpThermometry->struLinePolygonThermCfg.fMinTemperature,
                   lpThermometry->struLinePolygonThermCfg.fAverageTemperature, lpThermometry->struLinePolygonThermCfg.fTemperatureDiff);
        }

        if (lpThermometry != NULL)
        {
            delete lpThermometry;
            lpThermometry = NULL;
        }
    }
    else if (dwType == NET_SDK_CALLBACK_TYPE_STATUS)
    {
        DWORD dwStatus = *(DWORD *)lpBuffer;
        if (dwStatus == NET_SDK_CALLBACK_STATUS_SUCCESS)
        {
            log_debug("dwStatus:NET_SDK_CALLBACK_STATUS_SUCCESS\n");
        }
        else if (dwStatus == NET_SDK_CALLBACK_STATUS_FAILED)
        {
            DWORD dwErrCode = *(DWORD *)((char *)lpBuffer + 4);
            log_debug("NET_DVR_GET_MANUALTHERM_INFO failed, Error code %d\n", dwErrCode);
        }
    }
}

void CALLBACK RealDataCallback(int lRealHandle, unsigned int dwDataType, unsigned char *pBuffer, unsigned int dwBufSize, void *pUser)
{
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD:
        break;
    case NET_DVR_STREAMDATA:
        if (current_stream_totalsize > 512 * 1024)
        {
            current_stream_totalsize = 0;
            //fclose(current_time,"w");
            log_debug("[%s]type:%d,size:%d\n", __FUNCTION__, dwDataType, dwBufSize);
            //infile.close();
            time_t tt = time(NULL);
            tm *t = localtime(&tt);
            sprintf(current_time, "/userdata/tmp/img/%d%02d%02d%02d%02d%02d.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
            sprintf(current_time_ir, "/userdata/tmp/img/%d%02d%02d%02d%02d%02d_ir.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
        }
        else
        {
            log_debug("[%s]type:%d,size:%d\n", __FUNCTION__, dwDataType, dwBufSize);
            current_stream_totalsize += dwBufSize;
            current_stream_size = dwBufSize;
        }
        break;
    case NET_DVR_AUDIOSTREAMDATA:
        break;
    case NET_DVR_PRIVATE_DATA:
        break;
    default:
        log_debug("type:%d\n", dwDataType);
    }
}

bool Ptz_hk::upload_file(const atris_msgs::SignalMessage &msg, std::string filepath)
{
    Json::Reader reader;
    Json::Value root; //, response;
    reader.parse(msg.msg, root);
    response_record["id"] = root["content"]["id"];
    response_record["timestamp"] = root["content"]["timestamp"];
    response_record["visible_light_video_url"] = "";
    response_record["infrared_video_url"] = "";
    response_record["result"] = "success";
    do
    {
        boost::shared_ptr<UploadCaptureFile> upload_file = boost::shared_ptr<UploadCaptureFile>(new UploadCaptureFile());
        upload_file->local_path = filepath;
        if (Config::get_instance()->hfs_type == "qiniu")
        {
            upload_file->remote_path = QINIU_BUCKET_NAME + filepath;
        }
        else
        {
            if (*(Config::get_instance()->hfs_url.end() - 1) != '/')
            {
                upload_file->remote_path = Config::get_instance()->hfs_url + "/" + filepath.c_str();
            }
            else
            {
                upload_file->remote_path = Config::get_instance()->hfs_url + filepath.c_str();
            }
        }
        upload_file->deleteDays = 7;
        upload_file->state = TRANSFER_FILE_STARTED;
        TransferFile::upload(upload_file);
        while (true)
        {
            if (upload_file->state == TRANSFER_FILE_COMPLETED) {
                if (Config::get_instance()->hfs_url.find("upload") != std::string::npos)
                {
                    log_debug("use go-fastdfs...");
                    upload_file->remote_path = Utils::get_instance()->get_fileurl();
                }
                else if (Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos)
                {
                    log_debug("use udfs-tracer...");
                    upload_file->remote_path = Utils::get_instance()->get_fileurl();
                }

                 std::remove(filepath.c_str());
                break;
            } else if (upload_file->state == TRANSFER_FILE_ERROR ) {
                log_error("upload capture files fail.");
                response_record["result"] = "fail_upload_record";
                Utils::get_instance()->responseResult(msg, response_record, "response_ptz_record");
                break;
            }
            usleep(500 * 1000);
        }
    } while (0);
    std::remove(filepath.c_str());
    capture_uploading_ = false;
}
int   Ptz_hk::upload_file(const std::string& file_local_path,std::string& file_remote_url,int type){
    uploading_status = type;
    do
    {
        FILE *fp = NULL;
        boost::shared_ptr<UploadCaptureFile> upload_file = boost::shared_ptr<UploadCaptureFile>(new UploadCaptureFile());
        upload_file->local_path = file_local_path;
        if (Config::get_instance()->hfs_type == "qiniu")
        {
            upload_file->remote_path = QINIU_BUCKET_NAME + file_local_path;
        }
        else
        {
            if (*(Config::get_instance()->hfs_url.end() - 1) != '/')
            {
                upload_file->remote_path = Config::get_instance()->hfs_url + "/" + file_local_path.c_str();
            }
            else
            {
                upload_file->remote_path = Config::get_instance()->hfs_url + file_local_path.c_str();
            }
        }
        upload_file->deleteDays = 7;
        upload_file->state = TRANSFER_FILE_STARTED;
        record_uploading_ = true;
        TransferFile::upload(upload_file);
        while (true)
        {
            if (upload_file->state == TRANSFER_FILE_COMPLETED)
            {
                log_info("\033[1;32mline:%d,%d\033[0m", __LINE__, upload_file->state);
                if (Config::get_instance()->hfs_url.find("upload") != std::string::npos)
                {
                    //log_debug("use go-fastdfs...");
                    upload_file->remote_path = Utils::get_instance()->get_fileurl();
                }
                else if (Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos)
                {
                    //log_debug("use udfs-tracer...");
                    upload_file->remote_path = Utils::get_instance()->get_fileurl();
                }
                file_remote_url = upload_file->remote_path;
                uploading_status = 0;
                break;
            }
            else if (upload_file->state == TRANSFER_FILE_ERROR)
            {
                log_error("upload capture files fail.");
                uploading_status = -1;
                return -1;
            }
            //usleep(5 * 1000);
        }
    } while (0);
    std::remove(file_local_path.c_str());
    return 0;
}

int Ptz_hk::start_preview_v40()
{
    NET_DVR_PREVIEWINFO stru_play_infovisible = {0};
    stru_play_infovisible.hPlayWnd = {0};
    stru_play_infovisible.lChannel = 1;
    stru_play_infovisible.dwStreamType = 0;
    stru_play_infovisible.dwLinkMode = 0;
    stru_play_infovisible.bBlocked = 1;
    tinyros::Time now = tinyros::Time::now();
    double time_d = now.toMSec();

    irealplayhandle_visible = NET_DVR_RealPlay_V40(lUserID, &stru_play_infovisible, NULL, NULL);
    log_debug("[%s]result:%s,%d,%d\n", __FUNCTION__,irealplayhandle_visible < 0 ? "error" : "success",irealplayhandle_visible, NET_DVR_GetLastError());
   
    NET_DVR_PREVIEWINFO stru_play_info_ir = {0};
    stru_play_info_ir.hPlayWnd = {0};
    stru_play_info_ir.lChannel = 2;
    stru_play_info_ir.dwStreamType = 0;
    stru_play_info_ir.dwLinkMode = 0;
    stru_play_info_ir.bBlocked = 1;
    irealplayhandle_ir = NET_DVR_RealPlay_V40(lUserID, &stru_play_info_ir, NULL, NULL);
    log_debug("[%s]result:%s:%d,%d\n",  __FUNCTION__,irealplayhandle_ir < 0 ? "error" : "success",irealplayhandle_ir, NET_DVR_GetLastError());
    usleep(50);
    save_real_data();
    return 0;
}

int Ptz_hk::start_preview_v40(const atris_msgs::SignalMessage &msg)
{
    NET_DVR_PREVIEWINFO stru_play_infovisible = {0};
    stru_play_infovisible.hPlayWnd = {0};
    stru_play_infovisible.lChannel = 1;
    stru_play_infovisible.dwStreamType = 0;
    stru_play_infovisible.dwLinkMode = 0;
    stru_play_infovisible.bBlocked = 1;
    tinyros::Time now = tinyros::Time::now();
    double time_d = now.toMSec();
    time_t tt = time(NULL);
    tm *t = localtime(&tt);
    char current_time[128];
    char current_time_ir[128];
    sprintf(current_time, "/userdata/tmp/video/%d%02d%02d%02d%02d%02d_visible_record.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    sprintf(current_time_ir, "/userdata/tmp/video/%d%02d%02d%02d%02d%02d_ir_record.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    irealplayhandle_visible = NET_DVR_RealPlay_V40(lUserID, &stru_play_infovisible, NULL, NULL);
    log_debug("result:%s:%d\n", irealplayhandle_visible < 0 ? "error" : "success", NET_DVR_GetLastError());

    file_record_video = current_time;
    if (NET_DVR_SaveRealData(irealplayhandle_visible, const_cast<char *>(file_record_video.c_str())))
    {
        ;
    }

    //LONG irealplayhandle_ir;
    NET_DVR_PREVIEWINFO stru_play_info_ir = {0};
    stru_play_info_ir.hPlayWnd = {0};
    stru_play_info_ir.lChannel = 2;
    stru_play_info_ir.dwStreamType = 0;
    stru_play_info_ir.dwLinkMode = 0;
    stru_play_info_ir.bBlocked = 1;

    irealplayhandle_ir = NET_DVR_RealPlay_V40(lUserID, &stru_play_info_ir, NULL, NULL);
    log_debug("result:%s:%d\n", irealplayhandle_ir < 0 ? "error" : "success", NET_DVR_GetLastError());
    string file_record_video_ir = current_time_ir;
    return 0;
}

int Ptz_hk::save_real_data()
{
    time_t tt = time(NULL);
    tm *t = localtime(&tt);
    char current_time[128];
    char current_time_ir[128];
    sprintf(current_time, "/userdata/tmp/video/%d%02d%02d%02d%02d%02d_visible_record_.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    sprintf(current_time_ir, "/userdata/tmp/video/%d%02d%02d%02d%02d%02d_ir_record_.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    file_record_video_ir = current_time_ir;
    if (!NET_DVR_SaveRealData(irealplayhandle_ir, const_cast<char *>(file_record_video_ir.c_str())))
    {
        log_error("\033[1;31msave real ir data failure:%d\033[0m",NET_DVR_GetLastError());
        return -2;
    } 
    file_record_video = current_time;
    if (!NET_DVR_SaveRealData(irealplayhandle_visible, const_cast<char *>(file_record_video.c_str())))
    {
        log_error("\033[1;31msave real visible data failure!:%d\033[0m",NET_DVR_GetLastError());
        return -1;
    }  
    //sleep(3);
   
    log_info("visible:%s,%s,ir:%s,%s\n",current_time,file_record_video.c_str(),current_time_ir,file_record_video_ir.c_str());       
    return 0;
}

int Ptz_hk::start_record_audio()
{
    NET_DVR_PREVIEWINFO stru_play_info = {0};
    stru_play_info.hPlayWnd = {0};
    stru_play_info.lChannel = 2;
    stru_play_info.dwStreamType = 0;
    stru_play_info.dwLinkMode = 0;
    stru_play_info.bBlocked = 1;
    tinyros::Time now = tinyros::Time::now();
    double time_d = now.toMSec();

    irealplayhandle_audio = NET_DVR_RealPlay_V40(lUserID, &stru_play_info, NULL, NULL);
    log_debug("result:%s:%d,%d\n", irealplayhandle_audio < 0 ? "error" : "success",irealplayhandle_audio, NET_DVR_GetLastError());
    save_audio_record();
    return 0;
}

int Ptz_hk::save_audio_record()
{
    time_t tt = time(NULL);
    tm *t = localtime(&tt);
    char current_time[128];
    sprintf(current_time, "/userdata/tmp/audio/%d%02d%02d%02d%02d%02d_audio_record_.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    file_record_audio = current_time;
    if (!NET_DVR_SaveRealData(irealplayhandle_audio, const_cast<char *>(file_record_audio.c_str())))
    {
        log_error("\033[1;31msave real audio data failure!\033[0m");
        return -1;
    }
    return 0;
}

int Ptz_hk::start_preview_rp()
{
    NET_DVR_PREVIEWINFO stru_play_infovisible_rp = {0};
    stru_play_infovisible_rp.hPlayWnd = {0};
    stru_play_infovisible_rp.lChannel = 1;
    stru_play_infovisible_rp.dwStreamType = 0;
    stru_play_infovisible_rp.dwLinkMode = 0;
    stru_play_infovisible_rp.bBlocked = 1;

    time_t tt = time(NULL);
    tm *t = localtime(&tt);
    char current_time[128];
    char current_time_ir[128];
    sprintf(current_time, "/userdata/tmp/rp/%d%02d%02d%02d%02d%02d_visible_rp.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    sprintf(current_time_ir, "/userdata/tmp/rp/%d%02d%02d%02d%02d%02d_ir_rp.mp4", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    irealplayhandle_visible_rp = NET_DVR_RealPlay_V40(lUserID, &stru_play_infovisible_rp, NULL, NULL);
    log_debug("result:%s:%d\n", irealplayhandle_visible_rp < 0 ? "error" : "success", NET_DVR_GetLastError());

    file_record_video_rp = current_time;
    if (NET_DVR_SaveRealData(irealplayhandle_visible_rp, const_cast<char *>(file_record_video_rp.c_str())))
    {
        ;
    }

    NET_DVR_PREVIEWINFO stru_play_info_ir_rp = {0};
    stru_play_info_ir_rp.hPlayWnd = {0};
    stru_play_info_ir_rp.lChannel = 2;
    stru_play_info_ir_rp.dwStreamType = 0;
    stru_play_info_ir_rp.dwLinkMode = 0;
    stru_play_info_ir_rp.bBlocked = 1;

    irealplayhandle_ir_rp = NET_DVR_RealPlay_V40(lUserID, &stru_play_info_ir_rp, NULL, NULL);
    log_debug("result:%s:%d\n", irealplayhandle_ir_rp < 0 ? "error" : "success", NET_DVR_GetLastError());
    string file_record_video_rp_ir = current_time_ir;
    if (NET_DVR_SaveRealData(irealplayhandle_ir_rp, const_cast<char *>(file_record_video_rp_ir.c_str())))
    {
        ;
    }
    return 0;
}

int Ptz_hk::convert_to_mp4(std::string &source, std::string &target)
{

    if (source.empty())
    {
        return -1;
    }
    std::string tmp = source;
    chmod(source.c_str(), 0666);
    target = source.replace(source.find("_.mp4"), source.find("_.mp4") + 5, ".mp4");
    std::string cmd = FFMPEG_CMD + " -y -i " + tmp + " -vcodec copy -f mp4  " + target;
    int ret = system(cmd.c_str());
    if(0 == ret){
        if(0!=std::remove(tmp.c_str())){
            log_error("\033[1;31mremove file:%s,ret:%d\n\033[0m",tmp.c_str(),ret);     
        }
    }else{
        log_debug("[%s]cmd;%s ,ret:%d\n", __FUNCTION__, cmd.c_str(), ret);
    } 
    return ret;
}

int Ptz_hk::convert_to_mp3(std::string &source, std::string &target)
{
    if (source.empty())
    {
        return -1;
    }
    std::string tmp = source;
    target = source.replace(source.find(".mp4"), source.find(".mp4") + 4, ".aac");
    std::string cmd = FFMPEG_CMD + " -i " + tmp + " -vn -y -acodec copy " + target;
    int ret = system(cmd.c_str());
    if(0 == ret){
        if(0!=std::remove(tmp.c_str())){
            log_error("\033[1;31mremove file:%s,ret:%d\n\033[0m",tmp.c_str(),ret);     
        }
    }else{
        log_debug("[%s]cmd;%s ,ret:%d\n", __FUNCTION__, cmd.c_str(), ret);
    }
    return ret;
}

int Ptz_hk::stop_preview_record(LONG handle)
{
    int ret = NET_DVR_StopSaveRealData(handle);
    usleep(500);
    log_info("[%s] stop save real data ret:%d,%d\n",__FUNCTION__,ret,NET_DVR_GetLastError());
    ret = NET_DVR_StopRealPlay(handle);
    usleep(500);
    log_info("[%s]stop real play ret:%d,%d\n",__FUNCTION__,ret,NET_DVR_GetLastError());
    return 0;
}

int Ptz_hk::stop_save_realdata_record(std::string& visible_light_video_url, std::string& infrared_video_url)
{
    log_debug("stop_save_realdata_record....");
    stop_preview_record(irealplayhandle_visible);
    stop_preview_record(irealplayhandle_ir);
    log_info("video visible:%s,ir:%s",file_record_video.c_str(),file_record_video_ir.c_str());
    std::string target;
    std::string target_ir;
    if (0!=convert_to_mp4(file_record_video, target))
    {
        log_error("visible convert failed....");
        return -1;
    }
    usleep(50);
    if (0!=convert_to_mp4(file_record_video_ir, target_ir))
    {
        log_debug("ir convert failed....");
        return -2;
    }

    if(-1 == upload_file(target,visible_light_video_url,1)){
        ;
    }else{
        log_info("visible url:%s",visible_light_video_url.c_str());
    }
    if(-1 == upload_file(target_ir,infrared_video_url,2)){
        ;
    }else{
        log_info("ir video url:%s\n",infrared_video_url.c_str());
    }
    return 0;
}

int Ptz_hk::stop_save_audio_record(std::string &audio_url)
{
    stop_preview_record(irealplayhandle_audio);
    std::string target;
    std::string target_audio;
    if (0==convert_to_mp4(file_record_audio, target))
    {
        log_debug("%s convert_to_mp4 success....", file_record_video.c_str());
    }
    if (0==convert_to_mp3(target, target_audio))
    {
        log_debug("%s:convert to standard success", file_record_video_ir.c_str());
    }
    upload_file(target_audio,audio_url,5);
    log_info("\033[1;32m----audio_url:%s----\033[0m", audio_url.c_str());
    return 0;
}

int Ptz_hk::cancel_save_realdata_record()
{
    m_save_video_data = false;
    record_uploading_ = false;
    stop_preview_record(irealplayhandle_visible);
    stop_preview_record(irealplayhandle_ir);
    return 0;
}

int Ptz_hk::cancel_save_audio()
{
    stop_preview_record(irealplayhandle_audio);
}

int Ptz_hk::stop_save_realdata_record_rp()
{
    stop_preview_record(irealplayhandle_visible_rp);
    stop_preview_record(irealplayhandle_ir_rp);

    log_info("video visible:%s,ir:%s",file_record_video_rp.c_str(),file_record_video_rp_ir.c_str());
    std::string target;
    std::string target_ir;
    if (0!=convert_to_mp4(file_record_video_rp, target))
    {
        log_error("visible convert failed....");
        return -1;
    }
    usleep(50);
    if (0!=convert_to_mp4(file_record_video_rp_ir, target_ir))
    {
        log_debug("ir convert failed....");
        return -2;
    }
    std::string visible_light_video_url_rp,infrared_video_url_rp;
    if(-1 == upload_file(target,visible_light_video_url_rp,1)){
        ;
    }else{
        log_info("visible rp url:%s",visible_light_video_url_rp.c_str());
    }
    if(-1 == upload_file(target_ir,infrared_video_url_rp,2)){
        ;
    }else{
        log_info("ir video rp url:%s\n",infrared_video_url_rp.c_str());
    }




    return 0;
}

void Ptz_hk::ptz_move_control(int direction)
{
    switch(direction) {
        case 0:
            ptz_up();
            break;
        case 1:
           ptz_down();
            break;
         case 2:
            ptz_left();
            break;
        case 3:
            ptz_right();
            break; 
        case 4:
            ptz_stop();
            break;
        default:
            break;          
    }
    log_debug("%s:direction:%d\n",__PRETTY_FUNCTION__,direction);
}

bool Ptz_hk::ptz_stop()
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, 1, TILT_DOWN, 1);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
            is_login = false;

        return false;
    }
    return true;
}

bool Ptz_hk::ptz_up()
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, 1, TILT_UP, 0);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    // usleep(500000);
    // ptz_stop();
    return ret;
}

bool Ptz_hk::ptz_down()
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, 1, TILT_DOWN, 0);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    // usleep(500000);
    // ptz_stop();
    return ret;
}

bool Ptz_hk::ptz_left()
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, 1, PAN_LEFT, 0);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    // usleep(500000);
    // ptz_stop();
    return ret;
}

bool Ptz_hk::ptz_right()
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, 1, PAN_RIGHT, 0);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    // usleep(500000);
    // ptz_stop();
    return ret;
}

void Ptz_hk::ptz_zoom_control(int op)
{
    switch (op)
    {
    case 0:
        log_debug("zoom add");
        ptz_zoom_add();
        break;
    case 1:
        log_debug("zoom dec");
        ptz_zoom_dec();
        break;
    default:
        break;
    }
}

bool Ptz_hk::ptz_zoom(int value)
{
    if (!is_login)
        return false;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return -1;
    }
    if (value >= 32)
        return false;
    zoom_ = value;
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom = zoom_; //Z2???y?��?��??????2???y?��??��?????????????D???y???o??3???��???????��?��o1-32
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    log_info("pan:%f,tilt:%f,zoom:%f，hspeed:%f,h_speed_:%f", struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan, struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt, struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom,struBuiltinPTZABSOLUTEEX.fHorizontalSpeed,h_speed_);
    set_ptz_status(struBuiltinPTZABSOLUTEEX);

    get_ptz_status(struBuiltinPTZABSOLUTEEX);

    return true;

}
bool Ptz_hk::ptz_zoom_add()
{
    if (!is_login)
        return false;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return -1;
    }
    if (zoom_ >= 32)
        return false;
    zoom_ += 1;

    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom = zoom_; //Z2???y?��?��??????2???y?��??��?????????????D???y???o??3???��???????��?��o1-32
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    log_info("pan:%f,tilt:%f,zoom:%f", struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan, struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt, struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom);
    set_ptz_status(struBuiltinPTZABSOLUTEEX);
    get_ptz_status(struBuiltinPTZABSOLUTEEX);

    return true;
}

bool Ptz_hk::ptz_zoom_dec()
{
    if (!is_login)
        return false;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }
    if (zoom_ <= 1)
        return false;

    zoom_ -= 1;
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom = zoom_; //Z2???y?��?��??????2???y?��??��?????????????D???y???o??3???��???????��?��o1-32
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //????????��o???/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    set_ptz_status(struBuiltinPTZABSOLUTEEX);
    get_ptz_status(struBuiltinPTZABSOLUTEEX);
    return true;
}

void Ptz_hk::ptz_focus_control(int op)
{
    switch (op)
    {
    case 0:
        log_debug("focus add");
        ptz_focus_add();
        break;
    case 1:
        log_debug("focus dec");
        ptz_focus_dec();
        break;
    default:
        break;
    }
}

bool Ptz_hk::ptz_focus_add()
{
    if (!is_login)
        return -2;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }

    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.dwFocus += 500; //???1
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    set_ptz_status(struBuiltinPTZABSOLUTEEX);
    return true;
}

bool Ptz_hk::ptz_focus_dec()
{
    if (!is_login)
        return -2;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return -1;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.dwFocus -= 500; //???1
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    set_ptz_status(struBuiltinPTZABSOLUTEEX);
    return true;
}

bool Ptz_hk::get_current_ptz_info()
{
    if (!is_login)
        return -2;
    char szStatusBuf[ISAPI_STATUS_LEN];
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    return get_ptz_status(struBuiltinPTZABSOLUTEEX);
}

bool Ptz_hk::ptz_wiper_control(int on_off)
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, _channel, WIPER_PWRON, 1-on_off);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    return ret;
}

bool Ptz_hk::ptz_light_control(int on_off)
{
    bool ret = NET_DVR_PTZControl_Other(lUserID, _channel, LIGHT_PWRON, 1-on_off);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());

    return ret;
}


bool Ptz_hk::ptz_up_ex(float angle)
{
    if (!is_login)
        return -2;
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt = angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    ret = set_ptz_status(struBuiltinPTZABSOLUTEEX);
    log_info("%s:%d set %s\033[0m", __PRETTY_FUNCTION__, __LINE__, ret ? "\033[1;32mOK" : "\033[1;31mFail");
    return true;
}

bool Ptz_hk::ptz_left_ex(float angle)
{
    if (!is_login)
        return -2;
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return -1;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan = angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    ret = set_ptz_status(struBuiltinPTZABSOLUTEEX);
    log_info("%s:%d set %s", __PRETTY_FUNCTION__, __LINE__, ret ? "OK" : "Fail");
    return true;
}

bool Ptz_hk::ptz_right_ex(float angle)
{
    if (!is_login)
        return -2;
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan = angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    ret = set_ptz_status(struBuiltinPTZABSOLUTEEX);
    log_info("%s:%d set %s", __PRETTY_FUNCTION__, __LINE__, ret ? "OK" : "Fail");
    return true;
}

bool Ptz_hk::ptz_down_ex(float angle)
{
    if (!is_login)
        return -2;
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt = angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    ret = set_ptz_status(struBuiltinPTZABSOLUTEEX);
    log_info("%s:%d set %s", __PRETTY_FUNCTION__, __LINE__, ret ? "OK" : "Fail");
    return ret;
}

/**
 * @brief ptz_move 让摄像头\E8\BD?动绝对\E8?\92???
 *
 * @param h_angle 水平角度\EF\BC?0-360(hk)  -180~180(dh),正数逆时针方向转，负数顺时针方向\E8\BD?
 * @param v_angle 垂直角度\EF\BC?-90-90(hk, 负数向下\E8\BD?，\E6?\A3数向上\E8\BD?) -180~180(dh, 正数向下\E8\BD?，负数向上转)
 * @param zoom -128~128 正数放大，负数缩\E5\B0?
 *
 * @return
 */
bool Ptz_hk::ptz_move(float h_angle, float v_angle, float zoom, int idleMicSec)
{
    if (!is_login)
        return false;
    NET_DVR_PTZABSOLUTEEX_CFG struBuiltinPTZABSOLUTEEX;
    memset(&struBuiltinPTZABSOLUTEEX, 0, sizeof(struBuiltinPTZABSOLUTEEX));
    bool ret = get_ptz_status(struBuiltinPTZABSOLUTEEX);
    if (!ret)
    {
        return false;
    }
    struBuiltinPTZABSOLUTEEX.dwSize = sizeof(struBuiltinPTZABSOLUTEEX);
    if (0 != h_angle)
    {
        struBuiltinPTZABSOLUTEEX.struPTZCtrl.fPan = h_angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    }
    if (0 != v_angle)
    {
        struBuiltinPTZABSOLUTEEX.struPTZCtrl.fTilt = v_angle; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    }
    struBuiltinPTZABSOLUTEEX.struPTZCtrl.fZoom = zoom; //T2?šºy¡ê¡§¡ä1?¡À2?šºy¡ê?¡ê???šš¡\E2\82?ŠÌ?D?šºyŠÌ?oš®3??¡ê?¡\E2\82???¡ì¡êo-90.000 ~ 89.000
    struBuiltinPTZABSOLUTEEX.dwFocalLen = 100;
    struBuiltinPTZABSOLUTEEX.fHorizontalSpeed =15; //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.fVerticalSpeed = v_speed_;   //ŠÌ£\E2\82???¡êo?šš/S, max 15
    struBuiltinPTZABSOLUTEEX.byZoomType = 0;
    ret = set_ptz_status(struBuiltinPTZABSOLUTEEX);
    log_info("%s:%d set %s", __PRETTY_FUNCTION__, __LINE__, ret ? "OK" : "Fail");
    return true;
}

bool Ptz_hk::ptz_move_point(unsigned int point)
{
    if (!ptz_state)
    {
        log_warn("ptz not init");
        return false;
    }
    bool ret = false;
    ret = NET_DVR_PTZPreset_Other(lUserID, _channel, GOTO_PRESET, (DWORD)point);
    log_debug("%s:%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[31merror"), NET_DVR_GetLastError());
    if (!ret)
    {
        if (NET_DVR_NETWORK_FAIL_CONNECT == NET_DVR_GetLastError())
            is_login = false;

        return false;
    }
    return true;
}

bool Ptz_hk::ptz_capture2(std::string& visible_light_url, std::string& infrared_url, std::string &imagebuf) 
{
    if (!is_login)
        return false;

    char *pBuffThermometry = new char[2*1024*1024];
    DWORD retSize=0;

    boost::lock_guard<boost::mutex> lock(ptz_register_mutex_);
    ros::Time now = ros::Time::now();
    std::stringstream uid;
    uid << ((uint64_t)(now.toSec() * 1000000000ull));
    std::string file = "/userdata/tmp/" + uid.str() + ".jpg";
    std::string file_ir = "/userdata/tmp/"+uid.str()+"_ir.jpg";

    FILE *file_ir_handle = fopen(file_ir.c_str(),"a+");
    if(NULL == file_ir_handle){
        log_info("%s%d\n",__PRETTY_FUNCTION__,file_ir_handle);
    }

    NET_DVR_JPEGPARA strPicPara = {0};
    strPicPara.wPicQuality = 2;
    strPicPara.wPicSize = 0;
    bool ret = NET_DVR_CaptureJPEGPicture(lUserID, _channel, &strPicPara, const_cast<char *>(file.c_str()));
    log_debug("%sNET_DVR_CaptureJPEGPicture %s !Last code:%d\033[0m",ret?"\033[1;32m":"\033[1;31m",ret?"success":"fail",  NET_DVR_GetLastError());
    
    std::ifstream image(file, std::ifstream::binary);
    if (!image.is_open())
    {
        log_debug("file open failed");
    } else {
        image.seekg(0, image.end);
        int len = image.tellg();
        image.seekg(0, image.beg);
        char *pbuf = new char[len];
        log_debug("image length = %d",len);
        image.read(pbuf, len);
        std::string imagefile(pbuf, len);
        imagebuf = imagefile;
        log_debug("imageBuf size = %d",imagebuf.size());
        if (pbuf) {
            delete[] pbuf;
            pbuf = NULL;
        }   
    }


    NET_DVR_JPEGPICTURE_WITH_APPENDDATA strulpJpegWithAppend = {0};
    char szLen[256] ={0};
    if(strulpJpegWithAppend.pJpegPicBuff ==NULL){
        strulpJpegWithAppend.pJpegPicBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pJpegPicBuff,0,2*1024*1024);
    }
    if(strulpJpegWithAppend.pP2PDataBuff ==NULL){
        strulpJpegWithAppend.pP2PDataBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pP2PDataBuff,0,2*1024*1024);
    }
    if(strulpJpegWithAppend.pVisiblePicBuff ==NULL){
        strulpJpegWithAppend.pVisiblePicBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pVisiblePicBuff,0,2*1024*1024);
    }
    ret = NET_DVR_SetCapturePictureMode(0);

    ret = NET_DVR_CaptureJPEGPicture_WithAppendData(lUserID, 2, &strulpJpegWithAppend);
    if(!ret){
        log_error("\033[31m error code:%d\033[0m",NET_DVR_GetLastError());
    }else{
        log_info("\033[32m h*w:%d,%d\033[0m",strulpJpegWithAppend.dwJpegPicHeight,strulpJpegWithAppend.dwJpegPicWidth);
        int ret_value = fwrite(strulpJpegWithAppend.pJpegPicBuff,strulpJpegWithAppend.dwJpegPicLen,1,file_ir_handle);
         if(ret_value<0){
            log_error("\033[31m save picture failure :%d:%d\033[0m",ret_value);
         }else{
            log_info("\033[32m save picture success :%d:%d\033[0m",strulpJpegWithAppend.dwJpegPicLen);
         }
        fclose(file_ir_handle);
    }


    if(0 ==upload_file(file,visible_light_url,3)){
           ;//response["visible_light_url"] = visible_light_url; 
    }else{
          ;// response["result"] = "fail_upload_capture";
    }
    if(0 == upload_file(file_ir,infrared_url,4)){
          ;//response["infrared_url"] = infrared_url;  
    }else{
        ;//response["result"] = "fail_upload_capture";
    }
    ;//response["result"] = "success";
    return true;
}

bool Ptz_hk::ptz_capture(const atris_msgs::SignalMessage &msg)
{
    if (!is_login)
        return false;
    char *pBuffThermometry = new char[2*1024*1024];
    DWORD retSize=0;
    boost::lock_guard<boost::mutex> lock(ptz_register_mutex_);
    ros::Time now = ros::Time::now();
    std::stringstream uid;
    uid << ((uint64_t)(now.toSec() * 1000000000ull));
    std::string file = "/userdata/tmp/" + uid.str() + ".jpg";
    std::string file_ir = "/userdata/tmp/"+uid.str()+"_ir.jpg";

    FILE *file_ir_handle = fopen(file_ir.c_str(),"a+");
    if(NULL == file_ir_handle){
        log_info("%s%d\n",__PRETTY_FUNCTION__,file_ir_handle);
    }
 
    NET_DVR_JPEGPARA strPicPara = {0};
    strPicPara.wPicQuality = 2;
    strPicPara.wPicSize = 0;
    bool ret = NET_DVR_CaptureJPEGPicture(lUserID, _channel, &strPicPara, const_cast<char *>(file.c_str()));
    log_debug("%sNET_DVR_CaptureJPEGPicture %s !Last code:%d\033[0m", ret ? "\033[1;32m" : "\033[1;31m", ret ? "success" : "fail", NET_DVR_GetLastError());
    NET_DVR_JPEGPICTURE_WITH_APPENDDATA strulpJpegWithAppend = {0};
    char szLen[256] ={0};
    if(strulpJpegWithAppend.pJpegPicBuff ==NULL){
        strulpJpegWithAppend.pJpegPicBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pJpegPicBuff,0,2*1024*1024);
    }
    if(strulpJpegWithAppend.pP2PDataBuff ==NULL){
        strulpJpegWithAppend.pP2PDataBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pP2PDataBuff,0,2*1024*1024);
    }
    if(strulpJpegWithAppend.pVisiblePicBuff ==NULL){
        strulpJpegWithAppend.pVisiblePicBuff = new char[2*1024*1024];
        memset(strulpJpegWithAppend.pVisiblePicBuff,0,2*1024*1024);
    }
    ret = NET_DVR_SetCapturePictureMode(0);

    ret = NET_DVR_CaptureJPEGPicture_WithAppendData(lUserID, 2, &strulpJpegWithAppend);
    if(!ret){
        log_error("\033[31m error code:%d\033[0m",NET_DVR_GetLastError());
    }else{
        log_info("\033[32m h*w:%d,%d\033[0m",strulpJpegWithAppend.dwJpegPicHeight,strulpJpegWithAppend.dwJpegPicWidth);
        int ret_value = fwrite(strulpJpegWithAppend.pJpegPicBuff,strulpJpegWithAppend.dwJpegPicLen,1,file_ir_handle);
         if(ret_value<0){
            log_error("\033[31m save picture failure :%d:%d\033[0m",ret_value);
         }else{
            log_info("\033[32m save picture success :%d:%d\033[0m",strulpJpegWithAppend.dwJpegPicLen);
         }
        fclose(file_ir_handle);
    }


    Json::Reader reader;
    Json::Value root, response;
    reader.parse(msg.msg, root);
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
       std::string file_url,file_ir_url;
    if(0 ==upload_file(file,file_url,3)){
           response["visible_light_url"] = file_url; 
    }else{
           response["result"] = "fail_upload_capture";
    }
    if(0 == upload_file(file_ir,file_ir_url,4)){
          response["infrared_url"] = file_ir_url;  
    }else{
        response["result"] = "fail_upload_capture";
    }
    response["result"] = "success";
    Utils::get_instance()->responseResult(msg, response, "response_ptz_capture");
}

int Ptz_hk::set_temperature_compatibility(int mode)
{
    DWORD dwChannel = 2;
    //娴嬫俯鍩烘湰鍙傛暟閰嶇疆
    NET_DVR_STD_CONFIG struStdConfig = {0};
    struStdConfig.lpCondBuffer = &dwChannel;
    struStdConfig.dwCondSize = sizeof(dwChannel);
    struStdConfig.lpInBuffer = NULL;
    struStdConfig.dwInSize = 0;

    NET_DVR_STD_ABILITY struStdAbility = {0};

    char *m_pOutBuf = new char[ISAPI_OUT_LEN];
    memset(m_pOutBuf, 0, ISAPI_OUT_LEN);
    char *m_pStatusBuf = new char[ISAPI_STATUS_LEN];
    memset(m_pStatusBuf, 0, ISAPI_STATUS_LEN);
    struStdAbility.lpCondBuffer = &dwChannel;
    struStdAbility.dwCondSize = sizeof(DWORD);
    struStdConfig.lpInBuffer = NULL;
    struStdConfig.dwInSize = 0;

    struStdAbility.lpOutBuffer = m_pOutBuf;
    struStdAbility.dwOutSize = ISAPI_OUT_LEN;
    struStdAbility.lpStatusBuffer = m_pStatusBuf;
    struStdAbility.dwStatusSize = ISAPI_STATUS_LEN;

    bool ret = NET_DVR_GetSTDAbility(lUserID, NET_DVR_GET_THERMAL_CAPABILITIES, &struStdAbility);
    log_debug("%s get thermometry capabilities  mode %s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
    // {
    //     log_debug("NET_DVR_GET_THERMAL_CAPABILITIES failed, error code: %d\n", NET_DVR_GetLastError());
    // }
    // else
    // {
    //     log_debug("NET_DVR_GET_THERMAL_CAPABILITIES is successful!");
    // }

    NET_DVR_THERMOMETRY_BASICPARAM struThermBasicParam = {0};
    struStdConfig.lpOutBuffer = (LPVOID)&struThermBasicParam;
    struStdConfig.dwOutSize = sizeof(struThermBasicParam);

    struStdConfig.lpStatusBuffer = m_pStatusBuf;
    struStdConfig.dwStatusSize = ISAPI_STATUS_LEN;

    DWORD dwReturned = 0;
     ret = NET_DVR_GetSTDConfig(lUserID, NET_DVR_GET_THERMOMETRY_BASICPARAM, &struStdConfig);
    log_debug("%s get thermometry  mode %s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
    // {
    //     log_debug("\033[31mNET_DVR_GET_THERMOMETRY failed, error code: %u\033[0m\n", NET_DVR_GetLastError());
    // }
    // else
    // {
    //     log_debug("\033[32mNET_DVR_GET_THERMOMETRY is successful!\033[0m");
    // }

    struThermBasicParam.byEnabled = 1;          //浣胯兘锟??0- 涓嶅惎鐢紝1- 鍚???
    struThermBasicParam.byStreamOverlay = 1;    //鐮佹祦鏄惁鍙犲姞娓╁害淇℃伅锟??0- 鍚︼???1- ?????
    struThermBasicParam.byThermometryRange = 1; //娴嬫俯鑼冨洿锛堣繖閲屼互鎽勬皬搴︿负鍗曚綅璁＄畻锛屽叾浠栧崟浣嶇敱涓婂眰鑷杞崲锛夛???0- 榛樿鍊硷紝1- (-20~150)?????2- (0~550)
    struThermBasicParam.byThermometryUnit = 1;  //娴嬫俯鍗曚綅锛堟墍鏈夋祴娓╅厤缃姛鑳戒腑娓╁害鍙傛暟瀵瑰簲鐨勫崟浣嶏???: 0- 鎽勬皬搴︼紙鈩冿級锟??1- 鍗庢皬搴︼紙鈩夛級锟??2- 寮€灏旀???(K)
    struStdConfig.lpInBuffer = (LPVOID)&struThermBasicParam;
    struStdConfig.dwInSize = sizeof(struThermBasicParam);

    ret = NET_DVR_SetSTDConfig(lUserID, NET_DVR_SET_THERMOMETRY_BASICPARAM, &struStdConfig);
    log_debug("%s set thermometry  basic param %s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
}

void Ptz_hk::set_thermometry_mode()
{

    NET_DVR_STD_ABILITY struStdAbility = {0};
    DWORD dwChannel = 2;
    char *m_pOutBuf = new char[ISAPI_OUT_LEN];
    memset(m_pOutBuf, 0, ISAPI_OUT_LEN);
    char *m_pStatusBuf = new char[ISAPI_STATUS_LEN];
    memset(m_pStatusBuf, 0, ISAPI_STATUS_LEN);
    struStdAbility.lpCondBuffer = &dwChannel;
    struStdAbility.dwCondSize = sizeof(DWORD);

    struStdAbility.lpOutBuffer = m_pOutBuf;
    struStdAbility.dwOutSize = ISAPI_OUT_LEN;
    struStdAbility.lpStatusBuffer = m_pStatusBuf;
    struStdAbility.dwStatusSize = ISAPI_STATUS_LEN;

    if (!NET_DVR_GetSTDAbility(lUserID, NET_DVR_GET_THERMAL_CAPABILITIES, &struStdAbility))
    {
        log_debug("NET_DVR_GET_THERMAL_CAPABILITIES failed, error code: %u\n", NET_DVR_GetLastError());
    }
    else
    {
        log_debug("get thermal capabilities  successful!");
        log_debug("ability:%s \n status:%s\n", m_pOutBuf, m_pStatusBuf);
    }
    //娴嬫俯鍩烘湰鍙傛暟閰嶇疆
    NET_DVR_STD_CONFIG struStdConfig = {0};
    struStdConfig.lpCondBuffer = &dwChannel;
    struStdConfig.dwCondSize = sizeof(dwChannel);
    struStdConfig.lpInBuffer = NULL;
    struStdConfig.dwInSize = 0;

    NET_DVR_THERMOMETRY_MODE struThermMode = {0};
    struStdConfig.lpOutBuffer = (LPVOID)&struThermMode;
    struStdConfig.dwOutSize = sizeof(struThermMode);
    bool ret =NET_DVR_GetSTDConfig(lUserID, NET_DVR_GET_THERMOMETRY_MODE, &struStdConfig);
    log_debug("%s%s get thermometry  mode %s, %d\n\033[0m",(ret ? "\033[1;32m" : "\033[1;31m"),  __PRETTY_FUNCTION__, (ret ? "success" : "error"), NET_DVR_GetLastError());
    struThermMode.byMode = 1;
    struThermMode.byThermometryROIEnabled = 2;
    struStdConfig.lpInBuffer = (LPVOID)&struThermMode;
    struStdConfig.dwInSize = sizeof(struThermMode);

    ret = NET_DVR_SetSTDConfig(lUserID, NET_DVR_SET_THERMOMETRY_MODE, &struStdConfig);
    log_debug("%s%s set thermometry  mode %s, %d\n\033[0m",(ret ? "\033[1;32m" : "\033[1;31m"),  __PRETTY_FUNCTION__, (ret ? "success" : "error"), NET_DVR_GetLastError());
}

void Ptz_hk::set_preset_tempthermometry(float x1, float y1, float x2, float y2)
{
    //娴嬫俯鍩烘湰鍙傛暟閰嶇疆
    DWORD dwChannel = 2;
  
    char m_szStatusBuf[ISAPI_STATUS_LEN];
    memset(m_szStatusBuf, 0, ISAPI_STATUS_LEN);
     NET_DVR_THERMOMETRY_PRESETINFO_PARAM struPreSetInfoConfig = {0};

     NET_DVR_STD_CONFIG struStdConfig = {0};

    struPreSetInfoConfig.byEnabled = 1;
    struPreSetInfoConfig.byRuleID = 1;
    struPreSetInfoConfig.byReflectiveEnabled = 1;
    struPreSetInfoConfig.byRuleCalibType = 1;
    struPreSetInfoConfig.struRegion.dwPointNum = 4;
    struPreSetInfoConfig.struRegion.struPos[0].fX = x1;
    struPreSetInfoConfig.struRegion.struPos[0].fY = y1;

    struPreSetInfoConfig.struRegion.struPos[1].fX = x2;
    struPreSetInfoConfig.struRegion.struPos[1].fY = y1;

    struPreSetInfoConfig.struRegion.struPos[2].fX = x2;
    struPreSetInfoConfig.struRegion.struPos[2].fY = y2;

    struPreSetInfoConfig.struRegion.struPos[3].fX = x1;
    struPreSetInfoConfig.struRegion.struPos[3].fY = y2;
    
    NET_DVR_THERMOMETRY_PRESETINFO struPreSetInfo1={0};
    struPreSetInfo1.dwSize=sizeof(NET_DVR_THERMOMETRY_PRESETINFO);
    struPreSetInfo1.wPresetNo=0;
    struPreSetInfo1.struPresetInfo[0] = struPreSetInfoConfig;

    struStdConfig.lpInBuffer = (LPVOID)&struPreSetInfo1;
    struStdConfig.dwInSize = sizeof(struPreSetInfo1);
    
    memset(m_szStatusBuf, 0, ISAPI_STATUS_LEN);
    struStdConfig.lpStatusBuffer = m_szStatusBuf;
    struStdConfig.dwStatusSize = ISAPI_STATUS_LEN;
    bool ret = NET_DVR_SetSTDConfig(lUserID, NET_DVR_SET_THERMOMETRY_PRESETINFO, &struStdConfig);
    log_debug("%s%s set thermometry presetinfo %s, %d\n\033[0m",(ret ? "\033[1;32m" : "\033[1;31m"), __PRETTY_FUNCTION__,ret?"success":"error", NET_DVR_GetLastError());

}

int Ptz_hk::get_preset_alarm(int check_boundary)
{

    DWORD dwChannel = 2;
    NET_DVR_STD_CONFIG struStdConfig = {0};
    struStdConfig.lpCondBuffer = &dwChannel;
    struStdConfig.dwCondSize = sizeof(dwChannel);
    struStdConfig.lpInBuffer = NULL;
    struStdConfig.dwInSize = 0;

    NET_DVR_THERMOMETRY_ALARMRULE_PARAM stru_theremotry_alarmrule = {0};
    struStdConfig.lpOutBuffer = (LPVOID)&stru_theremotry_alarmrule;
    struStdConfig.dwInSize = sizeof(stru_theremotry_alarmrule);

    if (!NET_DVR_GetSTDConfig(lUserID, NET_DVR_GET_THERMOMETRY_ALARMRULE, &struStdConfig))
    {
        log_error("\033[31m error code:%d\033[0m",NET_DVR_GetLastError()) ;
    }
    else
    {
        log_debug("\033[32menable:%d,rulename:%s\n\033[0m", stru_theremotry_alarmrule.byEnable, stru_theremotry_alarmrule.szRuleName);
    }

    stru_theremotry_alarmrule.byEnable = 1;
    stru_theremotry_alarmrule.byRule = check_boundary;
    stru_theremotry_alarmrule.fAlert = 25.0;            //棰勮娓╁害
    stru_theremotry_alarmrule.fAlarm = 25.0;            //鎶ヨ娓╁害
    stru_theremotry_alarmrule.fThreshold = 25.0;        //闂ㄩ檺娓╁害
    stru_theremotry_alarmrule.dwAlertFilteringTime = 0; //娓╁害棰勮绛夊緟鏃堕棿,鍗曚綅绉?
    stru_theremotry_alarmrule.dwAlarmFilteringTime = 0; //娓╁害鎶ヨ绛夊緟鏃堕棿,鍗曚綅绉?
    struStdConfig.lpInBuffer = (LPVOID)&stru_theremotry_alarmrule;
    struStdConfig.dwInSize = sizeof(stru_theremotry_alarmrule);
    bool ret = NET_DVR_SetSTDConfig(lUserID, NET_DVR_SET_THERMOMETRY_ALARMRULE, &struStdConfig);
    log_debug("%s preset temperature  alarm rule%s, %d\n\033[0m", __PRETTY_FUNCTION__, (ret ? "\033[1;32msuccess" : "\033[1;31merror"), NET_DVR_GetLastError());
    return 0;
}

bool Ptz_hk::start_remote_config()
{
    //启动实时温度检???
    NET_DVR_REALTIME_THERMOMETRY_COND struThermCond = {0};
    struThermCond.dwSize = sizeof(struThermCond);
    struThermCond.byRuleID = 1;       //规则ID???0代表获取全部规则，具体规则ID???1开???
    struThermCond.dwChan = 2; //???1开始，0xffffffff代表获取全部通道

    LONG lHandle = NET_DVR_StartRemoteConfig(lUserID, NET_DVR_GET_REALTIME_THERMOMETRY, &struThermCond, sizeof(struThermCond), GetThermInfoCallback, NULL);
    log_info("get realtime thermometry %s:code:%d\n",lHandle<0?"failure":"success", NET_DVR_GetLastError());
    sleep(5);  //等待一段时间，接收实时测温结果
    //关闭长连接配置接口所创建的句柄，释放资源
    bool ret =NET_DVR_StopRemoteConfig(lHandle);
    log_debug("%s%s stop remote config %s, %d\n\033[0m", (ret ? "\033[1;32m" : "\033[1;31m"), __PRETTY_FUNCTION__, (ret ? "success" : "merror"), NET_DVR_GetLastError());
}
bool Ptz_hk::stop_disconnect_remote_config()
{
    //关闭长连接配置接口所创建的句柄，释放资源
    if (!NET_DVR_StopRemoteConfig(lHandle))
    {
        log_debug("NET_DVR_StopRemoteConfig failed, error code: %d\n", NET_DVR_GetLastError());
    }
}
/**
 * @brief wait_for_ptz_idle 等待云台进入空闲状态，再执行下一条指\E4\BB?
 *
 * @param timeout 最大的等待时间
 *
 * @return
 */
bool Ptz_hk::wait_for_ptz_idle(int timeout)
{
    return false;
}

#endif
