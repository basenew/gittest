#include "ultrasound.h"
#include "imemory/atris_imemory_api.h"
#include <json/json.h>
#include "atris_msgs/RobotInfo.h"
#include "utils/utils.h"

Ultrasound::Ultrasound()
  : ultrasound_flag_(false)
  , ultrasound_interval_(500) {
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Ultrasound::on_recv_signal, this);
    diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
}

int Ultrasound::init(CanService *can_service)
{
    this->diag_status = 0;
    this->can_service = can_service;
    memset(timeout, 0 ,sizeof(timeout));
    memset(ultra_data, 0xFFFF, sizeof(ultra_data));

    DevFilter filter;
    filter.filter = boost::bind(&Ultrasound::ultrasonic_filter, this, _1);
    can_service->add_filter(filter);

    boost::thread ultrasound_thread(boost::bind(&Ultrasound::notify_ultraSound, this));
    ultrasound_thread.detach();

    return 0;
}

int Ultrasound::ultrasonic_filter(void *data)
{
    CanPkg *pkg = (CanPkg*)data;
    if(pkg->head.id.channel == CH_ULTRASOUND_1 || pkg->head.id.channel == CH_ULTRASOUND_2) {
        check_state(*pkg);
        return 0;
    }
    return -1;
}

void Ultrasound::check_state(CanPkg &pkg)
{
    if(CH_ULTRASOUND_1 == pkg.head.id.channel){
        for(int i = 0; i < 8; i += 2) {
            uint16_t val = pkg.data[i] | (pkg.data[i+1] & 0x00FF) << 8;
            int idx = i/2;
            ultra_data[idx] = val;
            if (ultra_data[idx] == (uint16_t)0xFFFF) {
                diag_status |= (0x01 << idx);
            } else {
                diag_status &= ~(0x01 << idx);
            }
        }
    }
    else if(CH_ULTRASOUND_2 == pkg.head.id.channel){
        for(int i = 0; i < 8; i += 2) {
            uint16_t val = pkg.data[i] | (pkg.data[i+1] & 0x00FF) << 8;
            int idx = i/2 + 4;
            ultra_data[idx] = val;
            if (ultra_data[idx] == (uint16_t)0xFFFF) {
                diag_status |= (0x01 << idx);
            } else {
                diag_status &= ~(0x01 << idx);
            }
        }
    }

    if (CH_ULTRASOUND_1 == pkg.head.id.channel || CH_ULTRASOUND_2 == pkg.head.id.channel) {
        shm::UltraSound ultroso;
        memcpy(ultroso.data, ultra_data, sizeof(ultra_data));
        shm::iMemory_write_UltraSound(&ultroso);
        
        this->send_diag(diag_status);
    }
}

void Ultrasound::send_diag(int diag) {
    Json::FastWriter fw;
    Json::Value rbtInfoValue;
    atris_msgs::RobotInfo rbtInfo;

    rbtInfoValue["robot_info"]["ultrasound"]["error"] = diag;
    for (int i=0; i<ULTRASOUND_NUM; i++) {
        rbtInfoValue["robot_info"]["ultrasound"]["data"].append(ultra_data[i]);
    }
    rbtInfo.json = fw.write(rbtInfoValue);
    diag_info_pub_.publish(rbtInfo);
}


void Ultrasound::notify_ultraSound() {
    boost::unique_lock<boost::mutex> lock(ultrasound_mutex_, boost::defer_lock);
    while(true) {
      lock.lock();
      if (!ultrasound_flag_) {
          ultrasound_cond_.wait(lock);
      }
      lock.unlock();

      if (ultrasound_flag_) {
          shm::Robot shmrbt;
          shm::iMemory_read_Robot(&shmrbt);
          atris_msgs::SignalMessage origin;
          origin.account = shmrbt.robot.receiver;
          origin.type ="local_mqtt";

          ros::Time now = ros::Time::now();
          std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
          Json::Value resp;
          resp["id"] = uid.str();
          for (int i=0; i<ULTRASOUND_NUM; i++) {
              resp["ultrasound"].append(Ultrasound::get_instance()->ultra_data[i]);
          }
          Utils::get_instance()->responseResult(origin, resp, "notify_ultrasound_data");
      }
      usleep(ultrasound_interval_ * 1000);
    }
}

void Ultrasound::on_recv_signal(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value req, resp;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if (msg.title == "request_subscribe_ultrasound") {
        reader.parse(msg.msg, req);
        if (!req["content"]["switch"].isNull() && !req["content"]["interval"].isNull()) {
            boost::unique_lock<boost::mutex> lock(ultrasound_mutex_);
            int interval = req["content"]["interval"].asInt();
            ultrasound_interval_ = interval > 0 ? interval : 500;
            ultrasound_flag_ = req["content"]["switch"].asInt() ? true : false;
            ultrasound_cond_.notify_all();
        } else {
            resp["result"] = "fail_invalid_data";
        }
        Utils::get_instance()->responseResult(msg, resp, "response_subscribe_ultrasound");
    }
}
