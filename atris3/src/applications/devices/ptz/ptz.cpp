#include <boost/thread.hpp>
#include <json/json.h>
#include "dhnetsdk.h"
#include "ptz.h"
#include "transferfile/transferfile.h"

static LLONG login_handle;
#define PTZ_DEFAULT_SPEED   5 //1~8
//#define PTZ_LOGIN_TIMEOUT   90//seconds
#define QINIU_BUCKET_NAME "http://video.ubtrobot.com/"
class UploadCaptureFile: public FileObject {
  virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
    this->state = state;
  }

  public:
    UploadCaptureFile(): state(TRANSFER_FILE_STARTED) { }
    TransferStates state;
};

#ifdef PTZ_KEYBOARD_CONTROL

#define KEYCODE_R 0x52
#define KEYCODE_L 0x4C
#define KEYCODE_U 0x55
#define KEYCODE_D 0x44
#define KEYCODE_O 0x4F
#define KEYCODE_Q 0x81


int h = 90;
int v = 15;
void Ptz::ptz_keyboard_input(void)
{
    char c;
    struct termios cooked, raw;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("ptz Reading from keyboard");
    puts("---------------------------");
    while(kb_run){
        if(read(kfd, &c, 1) < 0)  {
            log_error("read():");
            exit(-1);
        }
        switch(c)  {
        case KEYCODE_L:
            log_info("ptz left:%d", h);
            //ptz_left_ex(h);
            //ptz_move_hori(h);
            ptz_rotate(h, v, 0, 1);
            break;
        case KEYCODE_R:
            log_info("ptz right:%d", v);
            //ptz_right_ex(h);
            //ptz_move(-h, 0, -64);
            //ptz_rotate(h, -v, 0);
            break;
        case KEYCODE_U:
            log_info("ptz up");
            //ptz_move(0, v, 0);
            break;
        case KEYCODE_D:
            log_info("ptz down");
            //ptz_down();
            //ptz_down_ex(10);
            //ptz_move_point(PTZ_POINT_DEFAULT);
            //ptz_move(0, -v, 0);
            break;
        case '0'://stop
            ptz_move_point(PTZ_POINT_DEFAULT);
            break;
        case KEYCODE_Q:
            log_warn("quit keyboard input thread");
            close(kfd);
            return;
        default:
            break;
        }
    }
}
#endif

void Ptz::ptz_rotate_async()
{
    Json::Reader reader;
    Json::Value value;

    boost::unique_lock<boost::mutex> lock(ptz_mutex_, boost::defer_lock);
    while(true) {
        lock.lock();
        if ((!ptz_rotate_data_) || (!movePtzData_)) {
            ptz_cond_.wait(lock);
        }

        if (movePtzData_) {
            reader.parse(movePtzData_->msg, value);
            int vertical = value["content"]["verticalangel"].asInt();
            int horizon  = value["content"]["horizontalangel"].asInt();
            is_rotate_async = true;
            ptz_move(vertical, horizon, 0, 300);
            is_rotate_async = false;
            delete movePtzData_;
            movePtzData_ = NULL;
        }

        is_rotate_async = false;

        if (ptz_rotate_data_) {
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
        int vel[3] = {-vel_angle, vel_angle*2, -vel_angle};
        while(is_rotate_async) {
            if (rotate_reset) {
                rotate_reset = false;
                ptz_move_point(default_point);
            }

            for (size_t i=0; i < 3 && is_rotate_async; i++) {
                now = ros::Time::now().toSec() * 1000;
                ptz_move(0, vel[i], 0);
                lock.lock();
                if (is_rotate_async) {
                    last = ros::Time::now().toSec() * 1000;
                    diff = last - now;
                    if (diff > 0 && diff < interval) {
                        ptz_cond_.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(interval - diff));
                    }
                }
                lock.unlock();
            }

            if (is_rotate_async) {
                now = ros::Time::now().toSec() * 1000;
                ptz_move(hor_angle, 0, 0);
                lock.lock();
                if (is_rotate_async) {
                    last = ros::Time::now().toSec() * 1000;
                    diff = last - now;
                    if (diff > 0 && diff < interval) {
                        ptz_cond_.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(interval - diff));
                    }
                }
                lock.unlock();
            }
        }

        ptz_stop();
    }
}

bool Ptz::ptz_rotate_stop()
{
    if(!is_login){
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

bool Ptz::ptz_is_idle()
{
    return (ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP);
}

bool Ptz::ptz_rotate(int h_angle, int v_angle, int zoom, int def_point)
{
    if(!is_login){
        log_info("ptz not login");
        return false;
    }

    if(is_rotate){
        ptz_rotate_stop();
    }
    while(ptz_rotate_state_ != PTZ_ROTATE_STOP && ptz_rotate_state_ != PTZ_ROTATE_IDLE){
        usleep(100*1000);
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

bool Ptz::ptz_rotate_pause(int onoff)
{
    if(ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP){
        return false;
    }
    log_info("ptz pause:%d, state=%d", onoff, ptz_rotate_state_);
    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    if(onoff == 1){
        ptz_rotate_pause_flag = true;
        if(ptz_rotate_state_ == PTZ_ROTATE_PAUSED){
            return true;
        }
    }else{
        ptz_rotate_pause_flag = false;
        if(ptz_rotate_state_ != PTZ_ROTATE_PAUSED){
            return true;
        }
        ptz_pause_con_.notify_all();
    }
    return true;
}

bool Ptz::ptz_rotate_pause()
{
    if(ptz_rotate_state_ == PTZ_ROTATE_IDLE || ptz_rotate_state_ == PTZ_ROTATE_STOP){
        return false;
    }
    log_info("ptz pause state=%d", ptz_rotate_state_);
    boost::unique_lock<boost::mutex> lock(ptz_mutex_);
    if(ptz_rotate_state_ == PTZ_ROTATE_PAUSED){
        ptz_rotate_pause_flag = false;
        ptz_pause_con_.notify_all();
    }else if(ptz_rotate_state_ == PTZ_ROTATE_PREPARE || ptz_rotate_state_ == PTZ_ROTATE_RUNING){
        ptz_rotate_pause_flag = true;
    }
    return true;
}

bool Ptz::ptz_rotate_run()
{
    boost::unique_lock<boost::mutex> rotate_lock(ptz_mutex_, boost::defer_lock);
    boost::unique_lock<boost::mutex> pause_lock(ptz_wait_mutex_, boost::defer_lock);

    while(1){
        rotate_lock.lock();
        ptz_rotate_con_.wait(rotate_lock);

        ptz_rotate_state_ = PTZ_ROTATE_RUNING;
        is_rotate = true;
        pause_lock.lock();
        if(ptz_rotate_pause_flag){
            ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
            ptz_pause_con_.wait(pause_lock);
        }
        pause_lock.unlock();
        ptz_move_point(def_point_);
        int v_count = 0;
        int vel[3] = {-v_angle_, v_angle_*2, -v_angle_};

        int times = 0;
        if(h_angle_ > 0 && h_angle_ <= 180){
            times = 360/h_angle_;
        }else if(h_angle_ > 180 && h_angle_ < 360){
            times = 1;
            h_angle_ = h_angle_ - 360;
        }else if(h_angle_ == 0 || h_angle_ == 360){
            times = 0;
        }
        rotate_lock.unlock();

        v_count = 0;
        while(v_count < 3 && is_rotate){
            if(v_angle_ == 0)
                break;
            log_info("ptz v_angle=%d", v_angle_);
            ptz_move(0, vel[v_count], zoom_);
            v_count ++;
            rotate_lock.lock();
            if(is_rotate && wait_ms_ > 0){
                ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
            }
            rotate_lock.unlock();
            pause_lock.lock();
            if(ptz_rotate_pause_flag){
                ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                ptz_pause_con_.wait(pause_lock);
                ptz_rotate_state_ = PTZ_ROTATE_RUNING;
            }
            pause_lock.unlock();
        }

        log_info("ptz times=%d, pauseflag=%d", times, ptz_rotate_pause_flag);
        while(is_rotate && times != 0){
            if(h_angle_ != 0){
                log_info("ptz h_angle=%d", h_angle_);
                ptz_move(h_angle_, 0, zoom_);
                times--;
                rotate_lock.lock();
                if(is_rotate && wait_ms_ > 0){
                    ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
                }
                rotate_lock.unlock();
                pause_lock.lock();
                if(ptz_rotate_pause_flag){
                    ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                    ptz_pause_con_.wait(pause_lock);
                    ptz_rotate_state_ = PTZ_ROTATE_RUNING;
                }
                pause_lock.unlock();
            }else{
                break;
            }

            v_count = 0;
            while(v_count < 3 && is_rotate){
                if(v_angle_ == 0)
                    break;
                log_info("ptz v_angle=%d", v_angle_);
                ptz_move(0, vel[v_count], zoom_);
                v_count ++;
                rotate_lock.lock();
                if(is_rotate && wait_ms_ > 0){
                    ptz_pause_con_.timed_wait(rotate_lock, boost::get_system_time() + boost::posix_time::milliseconds(wait_ms_));
                }
                rotate_lock.unlock();
                pause_lock.lock();
                if(ptz_rotate_pause_flag){
                    ptz_rotate_state_ = PTZ_ROTATE_PAUSED;
                    ptz_pause_con_.wait(pause_lock);
                    ptz_rotate_state_ = PTZ_ROTATE_RUNING;
                }
                pause_lock.unlock();
           }
        }
        pause_lock.lock();
        if(ptz_rotate_pause_flag){
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

void ptz_disconnect(LLONG lLoginID, char *pchDVRIP, LONG nDVRPort, LDWORD dwUser)
{
    Ptz* ptz = Ptz::get_instance();
    log_info("ptz_disConnect.");
    if(!ptz->is_reconnect)
        return;
#ifdef PTZ_KEYBOARD_CONTROL
    ptz->kb_run = false;
    if(ptz->kfd > 0)
        close(ptz->kfd);
#endif
    sleep(1);

    ptz->init();
}

Ptz::Ptz() {
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Ptz::on_recv_ptz_ctrl, this);
    get_ptz_camera_srv_ = nh_.advertiseService(SRV_GET_PTZ_STATUS, &Ptz::doGetPtzStatus, this);
    ptz_control_srv_ = nh_.advertiseService(SRV_PTZ_CONTROL, &Ptz::on_recv_ptz_ctrl, this);
    CLIENT_Init(ptz_disconnect, 0);
    cfg = Config::get_instance();
    utils = Utils::get_instance();
    ptz_state = false;
    is_login = false;
    login_thread = NULL;
    rotate_thread = NULL;
    wait_ms_ = 5000; //default 2s
    is_rotate = false;
    is_rotate_async = false;
    ptz_rotate_state_ = PTZ_ROTATE_IDLE;
    ptz_rotate_data_ = NULL;
    movePtzData_ = NULL;
}

bool Ptz::doGetPtzStatus(atris_msgs::GetPtzStatus::Request& req,
  atris_msgs::GetPtzStatus::Response& res) {
  if (get_state()) {
    if (is_rotate_async)
      res.status = atris_msgs::GetPtzStatus::Response::PTZ_ROTATE_STATUS;
    else 
      res.status = atris_msgs::GetPtzStatus::Response::PTZ_IDLE_STATUS;
  } else {
    res.status = atris_msgs::GetPtzStatus::Response::PTZ_NOT_LOGIN_STATUS;
  }
  return true;
}

bool Ptz::on_recv_ptz_ctrl(atris_msgs::PtzControl::Request& req,
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
    res.result = ptz_move_control(req.move_op);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_UP) {
    res.result = ptz_move_control(0);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_DOWN) {
    res.result = ptz_move_control(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_LEFT) {
    res.result = ptz_move_control(2);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE_RIGHT) {
    res.result = ptz_move_control(3);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_CAPTURE) {
    //TODO
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_RECORD_CONTROL) {
    //TODO
    res.result = true;
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ZOOM_CONTROL) {
    res.result = ptz_zoom_control(req.zoom_op);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ZOOM_ADD) {
    res.result = ptz_zoom_control(0);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_ZOOM_DEC) {
    res.result = ptz_zoom_control(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_FOCUS_CONTROL) {
    res.result = ptz_focus_control(req.focus_op);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_FOCUS_ADD) {
    res.result = ptz_focus_control(0);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_FOCUS_DEC) {
    res.result = ptz_focus_control(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_WIPER_CONTROL) {
    res.result = ptz_wiper_control(req.wiper_op);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_WIPER_ON) {
    res.result = ptz_wiper_control(1);
  } else if (req.cmd == atris_msgs::PtzControl::Request::PTZ_WIPER_OFF) {
    res.result = ptz_wiper_control(0);
  }

  return true;
}

void Ptz::on_recv_ptz_ctrl(const atris_msgs::SignalMessage  &msg)
{
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
        log_info("ptz pause @");
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
        ptz_move_point(PTZ_POINT_DEFAULT);
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
    } else if (msg.title == "request_ptz_move_control") {
        resp_name = "response_ptz_move_control";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["direction"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
        /*
        TODO 
        */
        resp["result"] = "success";
        }
        ptz_move_control(req["content"]["direction"].asInt());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_ptz_capture") {
        resp_name = "response_ptz_capture";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        } else {
            /*
            TODO 
            */
            resp["visible_light_url"] = "";
            resp["infrared_url"] = "";
            resp["result"] = "success"; 
            if (capture_uploading_) {
                resp["result"] = "fail_capture_already_upload";
                Utils::get_instance()->responseResult(msg, resp, resp_name);
            } else {
                if (capture_thread_) {
                    delete capture_thread_;
                    capture_thread_ = NULL;
                }
                capture_uploading_ = true;
                capture_thread_ = new boost::thread(boost::bind(&Ptz::ptz_capture, this, msg));
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
            /*
            TODO 
            */
            resp["result"] = "success";
        }
        ptz_zoom_control(req["content"]["operation"].asInt());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if (msg.title == "request_ptz_record") {
        resp_name = "response_ptz_record";
        reader.parse(msg.msg, req);
        if (!is_login) {
            resp["result"] = "fail_no_login";
            Utils::get_instance()->responseResult(msg, resp, resp_name);
            return;
        }
        if (req["content"]["switch"].isNull()) {
            resp["result"] = "fail_invalid_data";
        } else {
            /*
            TODO 
            */
            resp["result"] = "";
            resp["result"] = "success";
        }
        Utils::get_instance()->responseResult(msg, resp, resp_name);
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
            /*
            TODO 
            */
            resp["result"] = "success";
        }
        ptz_wiper_control(req["content"]["switch"].asInt());
        Utils::get_instance()->responseResult(msg, resp, resp_name);
    } else if(msg.title == "request_set_ptz_register_info") {
    resp_name="response_set_ptz_register_info";
    reader.parse(msg.msg,req);
    if (!is_login) {
      resp["result"] = "fail_no_login";
      Utils::get_instance()->responseResult(msg, resp, resp_name);
      return;
    }
    if (req["content"]["server_addr"].isNull() ||
        req["content"]["server_port"].isNull() ||
        req["content"]["device_id"].isNull()   ||
        req["content"]["enable"].isNull()      ) {
        resp["result"] = "fail_invalid_data";
    } else {
        if (setPtzRegisterInfo(req["content"]["server_addr"].asString(),
                               req["content"]["server_port"].asInt(),
                               req["content"]["device_id"].asString(),
                               req["content"]["enable"].asInt())) {
            resp["result"] = "success";
        } else {
            resp["result"] = "fail_set_ptz_regiser_info";
        }
    }
    Utils::get_instance()->responseResult(msg, resp, resp_name);
  } else if(msg.title == "request_get_ptz_register_info") {
    resp_name="response_get_ptz_register_info";
    reader.parse(msg.msg,req);
    if (!is_login) {
      resp["result"] = "fail_no_login";
      Utils::get_instance()->responseResult(msg, resp, resp_name);
      return;
    }
    std::string server_addr, device_id;
    int server_port;
    bool enable;
    if (getPtzRegisterInfo(server_addr, server_port, device_id, enable)) {
        resp["server_addr"] = server_addr;
        resp["server_port"] = server_port;
        resp["device_id"] = device_id;
        resp["enable"] = enable ? 1 : 0;           
        resp["result"] = "success";
    } else {
        resp["result"] = "fail_get_ptz_register_info";
    }
    Utils::get_instance()->responseResult(msg, resp, resp_name);
  } else {

  }
}

bool Ptz::check_state()
{
    bool ret = false;
    NET_DEVICEINFO stDevInfo = {0};
    LLONG handle;
    is_reconnect = false;

    log_info("check ptz state.");
    if (!cfg->ptz_ip.empty() && !cfg->ptz_user.empty() && !cfg->ptz_psw.empty()) {
        handle = CLIENT_Login(cfg->ptz_ip.c_str(), 37777,
                              cfg->ptz_user.c_str(), cfg->ptz_psw.c_str(), &stDevInfo);
        if(handle == 0){
            log_error("ptz not work");
            ret = false;
        }
        else{
            log_debug("ptz work fine.");
            ret = true;
        }
        CLIENT_Logout(handle);

    } else {
        log_error("%s ptz config error.", __FUNCTION__);
    }

    ptz_state = ret;

    log_info("check ptz state end.");

    return ret;
}

bool Ptz::get_state()
{
    return ptz_state;
}

int Ptz::init()
{
    log_info("ptz init");
    is_login = false;
    if(login_thread != NULL){
        log_info("delete login thread before.");
        login_retry = -1;
        sleep(2);
        delete login_thread;
        login_thread = NULL;
    }
    login_thread = new boost::thread(boost::bind(&Ptz::ptz_login, this));
#if 0
    CLIENT_Init(ptz_disconnect, 0);
    NET_DEVICEINFO stDevInfo = {0};
    int retry = cfg->ptz_login_timeout;
    while(retry > 0){
        login_handle = CLIENT_Login(cfg->ptz_ip.c_str(), 37777, cfg->ptz_user.c_str(), cfg->ptz_psw.c_str(), &stDevInfo);
        if(login_handle == 0){
            log_error("login ptz fail, retry");
            sleep(1);
            retry --;
            continue;
        }
        else{
            log_info("login to ptz successfully");
            is_login = true;
            break;
        }
    }

    if(retry == 0){
        log_error("can't login to ptz.");
        CLIENT_Cleanup();
        return -1;
    }
    log_info("ptz connect successfully!");
#ifdef PTZ_KEYBOARD_CONTROL
    new boost::thread(boost::bind(&Ptz::ptz_keyboard_input, this));
    kb_run = true;
#endif

#endif

    return 0;
}

int Ptz::set_horiz(int degree)
{

    return 0;
}

int Ptz::set_vertical(int degree)
{
    return 0;
}
#if 1

bool Ptz::ptz_move_control(int direction)
{
    bool result = false;
    switch(direction) {
        case 0:
            log_debug("ptz up");
            //ptz_up();
            result = ptz_move(0, -10, 0, 600);
            break;
        case 1:
            log_debug("ptz down");
            //ptz_down();
            result = ptz_move(0, 10, 0, 600);
            break;
         case 2:
            log_debug("ptz left");
            //ptz_left();
            result = ptz_move(10, 0, 0, 600);
            break;
        case 3:
            log_debug("ptz right");
            //ptz_right();
            result = ptz_move(-10, 0, 0, 600);
            break; 
        default:
            break;          
    }
    return result;
}
bool Ptz::ptz_up()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_UP_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_down()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_DOWN_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_left()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_LEFT_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_right()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_RIGHT_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_zoom_control(int op)
{
    return (op) ? ptz_move(0, 0, -5, 600) : ptz_move(0, 0, 5, 600);  
}

bool Ptz::ptz_zoom_add()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_ZOOM_ADD_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_zoom_dec()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_ZOOM_DEC_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_focus_control(int op)
{
    return (op) ? ptz_focus_dec() : ptz_focus_add();
}

bool Ptz::ptz_focus_add()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_FOCUS_ADD_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_focus_dec()
{
    return CLIENT_PTZControl(login_handle, 0, DH_PTZ_FOCUS_DEC_CONTROL, PTZ_DEFAULT_SPEED, FALSE);
}

bool Ptz::ptz_wiper_control(int on_off)
{
    return (on_off) ? CLIENT_DHPTZControlEx2(login_handle, 0, DH_PTZ_LAMP_CONTROL, 1, 0, 0, FALSE):
                      CLIENT_DHPTZControlEx2(login_handle, 0, DH_PTZ_LAMP_CONTROL, 0, 0, 0, FALSE);
}

bool Ptz::ptz_capture(const atris_msgs::SignalMessage &msg)
{
    boost::lock_guard<boost::mutex> lock(ptz_register_mutex_);

    NET_IN_SNAP_PIC_TO_FILE_PARAM stuInParam = {sizeof(stuInParam)};
    NET_OUT_SNAP_PIC_TO_FILE_PARAM stuOutParam = {sizeof(stuOutParam)};
    SNAP_PARAMS stuSnapParams = {0};
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    std::string file = "/userdata/tmp/" + uid.str() + ".jpg";
    stuSnapParams.Channel = 0;
    stuInParam.stuParam = stuSnapParams;
    strcpy(stuInParam.szFilePath, file.c_str());
    // 以第一个通道为例
    int nBufferLen = 2*1024*1024;
    char* pBuffer = new char[nBufferLen]; // 图片缓存 
    memset(pBuffer, 0, nBufferLen);
    stuOutParam.szPicBuf = pBuffer;
    stuOutParam.dwPicBufLen = nBufferLen;
    if (FALSE == CLIENT_SnapPictureToFile(login_handle, &stuInParam, &stuOutParam, 2000))
    {
        log_error("“CLIENT_SnapPictureToFile Failed !Last Error[%x]", CLIENT_GetLastError());
    }

    if (pBuffer) {
        delete[] pBuffer;
        pBuffer = NULL;
    }

    Json::Reader reader;
    Json::Value root, response;
    reader.parse(msg.msg, root);
    response["id"] = root["content"]["id"];
    response["timestamp"] = root["content"]["timestamp"];
    response["visible_light_url"] = "";
    response["infrared_url"] = "";
    response["result"] = "success"; 

 do {
    FILE *fp = NULL;

    boost::shared_ptr<UploadCaptureFile> upload_file = boost::shared_ptr<UploadCaptureFile> (new UploadCaptureFile());
    upload_file->local_path = file;
    if(Config::get_instance()->hfs_type == "qiniu"){
        upload_file->remote_path = QINIU_BUCKET_NAME + uid.str();
    }else{
        if (*(Config::get_instance()->hfs_url.end()-1) != '/') {
            upload_file->remote_path = Config::get_instance()->hfs_url + "/" + uid.str() + ".jpg";
        }else{
            upload_file->remote_path = Config::get_instance()->hfs_url + uid.str() + ".jpg";
        }
    }

    upload_file->deleteDays = 7;
    upload_file->state = TRANSFER_FILE_STARTED;
    TransferFile::upload(upload_file);
    
    while(true) {
      if(upload_file->state == TRANSFER_FILE_COMPLETED) {
        if(Config::get_instance()->hfs_url.find("upload") != std::string::npos){
            log_debug("use go-fastdfs...");
            upload_file->remote_path = Utils::get_instance()->get_fileurl();
        } else if (Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos) {
            log_debug("use udfs-tracer...");
            upload_file->remote_path = Utils::get_instance()->get_fileurl();
        }

        response["visible_light_url"] = upload_file->remote_path;
        log_debug("visible_light_url : %s.", upload_file->remote_path.c_str() );
        std::string rm = "rm -rf " + file;

        if((fp = popen(rm.c_str(), "r"))) {
          pclose(fp);
        }

        break;
      } else if (upload_file->state == TRANSFER_FILE_ERROR){
        log_error("upload capture files fail.");
        response["result"] = "fail_upload_capture";
        break;
      }
      usleep(500*1000);
    }
  }while(0);

  Utils::get_instance()->responseResult(msg, response, "response_ptz_capture");

  capture_uploading_ = false;

}

bool Ptz::ptz_up_ex(int angle)
{
    bool ret = CLIENT_DHPTZControl(login_handle, 0, DH_EXTPTZ_EXACTGOTO, angle* 10, angle*10, 2, 0);
    // bool ret = CLIENT_DHPTZControl(login_handle, 0, DH_EXTPTZ_FASTGOTO, angle* 10, angle*10, 2, 0);
    if(ret)
        log_info("set OK");
    else
        log_info("set fail");

    return true;
}

bool Ptz::ptz_down_ex(int angle)
{
    bool ret = CLIENT_DHPTZControl(login_handle, 0, DH_PTZ_DOWN_CONTROL, 0, 5, 0, 0);
    sleep(1);
    ret = CLIENT_DHPTZControl(login_handle, 0, DH_PTZ_DOWN_CONTROL, 0, 5, 0, 1);
    if(ret)
        log_info("set OK");
    else
        log_info("set fail");

    return true;
}

/**
 * @brief ptz_move 让摄像头转动绝对角度
 *
 * @param h_angle 水平角度，-180~180,正数逆时针方向转，负数顺时针方向转
 * @param v_angle 垂直角度，-180~180, 正数向下转，负数向上转
 * @param zoom -128~128 正数放大，负数缩小
 *
 * @return
 */
bool Ptz::ptz_move(int h_angle, int v_angle, int zoom, int idleMicSec)
{
    if(!ptz_state){
        log_warn("ptz not init");
        return false;
    }
    //log_info("h angle:%d v angle:%d zoom:%d", h_angle, v_angle, zoom);

    NET_IN_MOVERELATIVELY_INFO stuInParam = {sizeof(stuInParam)};
    stuInParam.stuSpeed.fPositionX = h_angle ? 0.5 : 0.0;
    stuInParam.stuSpeed.fPositionY = v_angle ? 0.5 : 0.0;
    stuInParam.stuSpeed.fZoom = 0;

    stuInParam.stuTranslation.fPositionX = h_angle/180.0f;//-1~1
    stuInParam.stuTranslation.fPositionY = v_angle/180.0f;//-1~1
    stuInParam.stuTranslation.fZoom = zoom/128.0f;//-1~1

    //log_info("x=%f y=%f zome=%f", stuInParam.stuTranslation.fPositionX,
    //stuInParam.stuTranslation.fPositionY, stuInParam.stuTranslation.fZoom);
    wait_for_ptz_idle(idleMicSec);
    log_info("ptz 222---");
    BOOL ret = CLIENT_DHPTZControlEx2(login_handle, 0, DH_EXTPTZ_MOVE_RELATIVELY, 0, 0, 0,FALSE, &stuInParam);
    log_info("ptz 333---");
#if 0
    if(!ret)
        log_info("set ptz move OK");
    else
        log_info("set ptz move fail");
#endif

    return true;
}

bool Ptz::ptz_move_point(int point)
{
    if(!ptz_state){
        log_warn("ptz not init");
        return false;
    }

    bool ret = CLIENT_DHPTZControlEx(login_handle, 0, DH_PTZ_POINT_MOVE_CONTROL, 0, point, 0, 0);
    if(!ret)
        log_info("ptz exec fail");

    return ret;
}

bool Ptz::ptz_stop() {
    return CLIENT_DHPTZControlEx2(login_handle, 0, DH_EXTPTZ_MOVE_RELATIVELY, 0, 0, 0,true);
}

/**
 * @brief wait_for_ptz_idle 等待云台进入空闲状态，再执行下一条指令
 *
 * @param timeout 最大的等待时间
 *
 * @return
 */
bool Ptz::wait_for_ptz_idle(int timeout)
{
    DH_PTZ_LOCATION_INFO ptz_locat_info;
    const int step = 100;//ms
    int times = timeout/step;

    for(int i = 0; i < times; i ++){
        int val_len = 0;
        int ret = CLIENT_QueryDevState(login_handle, DH_DEVSTATE_PTZ_LOCATION,
                                       (char *)&ptz_locat_info, sizeof(DH_PTZ_LOCATION_INFO), &val_len, 300);
#if 1
        if(ret){
            //log_info("check state ok");
        }
        else
            log_info("check state fail.");
#endif

        if(ptz_locat_info.bState == 2){
            //log_info("ptz state ready");
            return true;
        }
        usleep(step * 1000);
    }

    return false;
}

void Ptz::ptz_login()
{
    is_reconnect = true;
    NET_DEVICEINFO stDevInfo = {0};
    login_retry = cfg->ptz_login_timeout;
    while(login_retry > 0){
        if(!utils->check_network_state(cfg->router_ip.c_str())){
            log_error("ptz network unreachable.");
            sleep(2);
            login_retry --;
            continue;
        }

        login_handle = CLIENT_Login(cfg->ptz_ip.c_str(), 37777, cfg->ptz_user.c_str(), cfg->ptz_psw.c_str(), &stDevInfo);
        if(login_handle == 0){
            log_error("login ptz fail, retry, %d", login_retry);
            sleep(1);
            login_retry --;
            continue;
        }
        else{
            log_info("login to ptz successfully");
            is_login = true;
            ptz_state = true;
            break;
        }
    }

    if(login_retry < 0 ){
        log_warn("exit login to ptz.");
        return ;
    }

    if(login_retry == 0){
        log_error("login ptz fail, please check ptz devices.");
        CLIENT_Cleanup();
        return ;
    }
    ptz_move_point(PTZ_POINT_DEFAULT);
    log_info("ptz connect successfully!");
#ifdef PTZ_KEYBOARD_CONTROL
    kb_run = true;
    new boost::thread(boost::bind(&Ptz::ptz_keyboard_input, this));
#endif

    ptz_thread_ = new boost::thread(boost::bind(&Ptz::ptz_rotate_async, this));

    rotate_thread = new boost::thread(boost::bind(&Ptz::ptz_rotate_run, this));
}

bool Ptz::getPtzRegisterInfo(std::string &serverip, int &port, std::string &registerId, bool &enable)
{
    boost::lock_guard<boost::mutex> lock(ptz_register_mutex_);

    DHDEV_REGISTER_SERVER data;
    DWORD dwRetLen = 0;

    bool bok = CLIENT_GetDevConfig(login_handle, DH_DEV_REGISTER_CFG, 0, &data, sizeof(data), &dwRetLen);

    if (!bok) {
        log_error("fail get ptz register info.");
        return bok;
    }

    log_error("%s, data.szDeviceID: %s, data.bServerNum: %d, dwRetLen: %d, bok: %d",
            __FUNCTION__, data.szDeviceID, data.bServerNum, dwRetLen, bok);

    serverip = data.lstServer[0].szServerIp;
    port = data.lstServer[0].nServerPort;
    registerId = data.szDeviceID; 
    enable = data.bEnable;

    return bok;
}

bool Ptz::setPtzRegisterInfo(std::string serverip, int port, std::string registerId, bool enable)
{
    boost::lock_guard<boost::mutex> lock(ptz_register_mutex_);

    DHDEV_REGISTER_SERVER data;
    data.bEnable = enable;
    memset(data.lstServer[0].szServerIp, 0, 32);
    memcpy(data.lstServer[0].szServerIp, serverip.data(), serverip.size());
    data.lstServer[0].nServerPort = port;
    data.bServerNum = 1;
    memset(data.szDeviceID, 0, sizeof(data.szDeviceID));
    memcpy(data.szDeviceID, registerId.data(), registerId.size());
    data.dwSize = sizeof(data);

    bool bok = CLIENT_SetDevConfig(login_handle, DH_DEV_REGISTER_CFG, 0, &data, data.dwSize);
         
    log_error("xxx%s: serverip: %s, port: %d, registerId: %s, enbale: %d, book: %d.",
               __FUNCTION__, serverip.c_str(), port, registerId.c_str(), enable, bok);

    return bok;
}
#endif
