#include "can_service.h"

#include "json/json.h"
#include "ultrasound/ultrasound.h"
#include "utils/utils.h"


CanService::CanService() {
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &CanService::on_recv_can_service, this);
    can_data_sub_ = nh_.subscribe(TOPIC_CAN_CMD_MESSAGE, 100, &CanService::on_can_data_cb, this);
    can_dev = new CanDevice(DEV_NAME);
    int ret = can_dev->can_open();
    if(ret < 0){
        log_error("open can device fail!!!!!!!!!!!!!!!!!!!");
    }

    new boost::thread(boost::bind(&CanService::proc_channel_data, this));
    usleep(100);//wait for read data thread start

    seq = 0;
}

void CanService::on_can_data_cb(const atris_msgs::CanData& msg) 
{
    CanPkg pkg = {0};
    pkg.head.id.channel = msg.channel;
    for (uint32_t i=0; i < msg.size; i++) {
        pkg.data[i] = msg.data[i];
    }
    send(pkg, msg.size);
}

int CanService::add_filter(DevFilter &filter)
{
    boost::unique_lock<boost::mutex> lock(filter_mutex);
    filter_id ++;
    filter.id = filter_id;
    filter_list.push_back(filter);

    return filter_id;
}

int CanService::del_filter(int id)
{
    boost::unique_lock<boost::mutex> lock(filter_mutex);
    std::list<DevFilter>::iterator it;
    for(it = filter_list.begin(); it != filter_list.end(); it ++){
        if(it->id == id){
            filter_list.erase(it);
            return 0;
        }
    }

    return -1;
}


int CanService::send(void *data, int len)
{
    CanPkg *pkg = (CanPkg *)data;

    return can_dev->can_write(pkg, len);
}

int CanService::send(CanPkg &pkg, int len)
{
    return can_dev->can_write(&pkg, len);
}

void CanService::proc_channel_data()
{
    CanPkg pkg = {0};
    while(1){
        memset(&pkg, 0, sizeof(CanPkg));
        if ((can_dev->can_read(&pkg, 2000)) > 0) {
            if(!ack_filter(pkg)) {
                continue;
            }
            do_filter(&pkg);
        } else {
            Ultrasound::get_instance()->send_diag(-1);
            can_dev->can_close();
            can_dev->can_open();
        }
    }
}

/**
 * @brief ack_filter 过滤非应答包
 *
 * @param pkg
 *
 * @return 如果数据已经被过滤处理，则返回0，否则返回负数
 */
int CanService::ack_filter(CanPkg &pkg)
{
#ifdef BATTERY_TEST
    if((pkg.head.id.channel & 0xF0) == 0xF0){
        return -1;
    }
#endif

    if(pkg.head.id.channel == CH_ULTRASOUND_1
            || pkg.head.id.channel == CH_ULTRASOUND_2){//不处理超声数据
        return -1;
    }
    #if 0
    if(pkg.data[0] == FILTER_BATTERY_CMD){//电池信息
        return -1;
    }
    #endif
    
    boost::unique_lock<boost::mutex> concur_lock(concur_mutex);
    for (std::list<AckCanData>::iterator it = concurrent_list.begin(); it != concurrent_list.end(); ++it){
        if(it->id == (pkg.data[0])){
            memcpy(it->pkg, &pkg, sizeof(CanPkg));
            if(it->cv != NULL){
                boost::unique_lock<boost::mutex> wlock(*it->mutex);
                it->cv->notify_one();
            }

            concurrent_list.erase(it);
            break;
        }
    }

    return 0;
}

void CanService::do_filter(void *data)
{
    boost::unique_lock<boost::mutex> lock(filter_mutex);
    int ret = -1;
    std::list<DevFilter>::iterator it;
    for(it = filter_list.begin(); it != filter_list.end(); it ++){
        ret = it->filter(data);
        if(ret == 0){//指令已经处理，则不需要继续送到下一个过滤器进行处理,
            //否则送到下一个过滤器继续处理
            return ;
        }
    }
}

/**
 * @brief send_for_ack 发送并等待回复，等待超时时间 1 秒
 *
 * @param pkg 发送的数据，并存储收到回复的数据
 * @param len 发送can数据的长度
 *
 * @return
 */
int CanService::send_for_ack(CanPkg *pkg, int len)
{
    AckCanData cd;

    cd.seq = seq ++;
    cd.id = pkg->data[0] + 1;//ack 请求命令+1
    cd.cv = new boost::condition_variable_any();
    cd.mutex = new boost::mutex();
    cd.pkg = pkg;

    int ret;
    {
        boost::unique_lock<boost::mutex> concur_lock(concur_mutex, boost::defer_lock);
        concur_lock.lock();
        
        concurrent_list.push_back(cd);
        
        boost::unique_lock<boost::mutex> wlock(*cd.mutex);
        ret = send(*pkg, len);
        if(ret < 0){
            //send data fail, delete data
            for (std::list<AckCanData>::iterator it = concurrent_list.begin();
                    it != concurrent_list.end(); ++it){
                if(it->seq == cd.seq){
                    //log_info("delete seq:%d", cd.seq);
                    concurrent_list.erase(it);
                    break;
                }
            }
            log_error("send data fail.");
            wlock.unlock();
            concur_lock.unlock();
            delete cd.cv;
            delete cd.mutex;
            return -1;
        }
        
        concur_lock.unlock();

        //log_info("start to wait for ack");
        if(cd.cv->timed_wait(wlock, boost::get_system_time() + boost::posix_time::seconds(ACK_MAX_TIMEOUT))){
            //log_info("recv ack");
            ret = 0;
        } else {
            //log_error("no ack");
            ret = -2;
        }
        wlock.unlock();

        concur_lock.lock();
        for (std::list<AckCanData>::iterator it = concurrent_list.begin(); it != concurrent_list.end(); ++it){
            if(it->seq == cd.seq){
                //log_info("delete seq:%d", cd.seq);
                concurrent_list.erase(it);
                break;
            }
        }
        concur_lock.unlock();
    }

    delete cd.cv;
    delete cd.mutex;

    return ret;
}

void CanService::on_recv_can_service(const atris_msgs::SignalMessage &msg)
{
    Json::Reader reader;
    Json::Value req, resp;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if (msg.title == "request_can_command") {
        reader.parse(msg.msg, req);
        if (!req["content"]["channel"].isNull() && !req["content"]["size"].isNull()
            && !req["content"]["data"].isNull() && req["content"]["data"].isArray()) {
            int size = req["content"]["size"].asInt();
            int size_array = req["content"]["data"].size();
            if (size >= 1 && size <= 8 && size == size_array) {
                CanPkg pkg = {0};
                pkg.head.id.channel = req["content"]["channel"].asInt();
                for(int i=0; i<size; i++) {
                  pkg.data[i] = (uint8_t)(req["content"]["data"][i].asInt() & 0xFF);
                }
                send_for_ack(&pkg, size);
                resp["size"] = size;
                    resp["channel"] = pkg.head.id.channel;
                for(int i=0; i<size; i++){
                    resp["data"][i] = pkg.data[i];
                }
            } else {
                resp["result"] = "fail_invalid_data";
                log_error("%s Invalid data size: %d, arraySize: %d", __FUNCTION__, size, size_array);
            }
        } else {
            resp["result"] = "fail_invalid_data";
        }
        Utils::get_instance()->responseResult(msg, resp, "response_can_command");
    }
}
