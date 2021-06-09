#include "telecom.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <json/json.h>

#include "utils/utils.h"
#include "log/log.h"
#include "base64/base64.h"
#include "router/router.h"


Telecom::Telecom()
{
    register_thread_ = new boost::thread(boost::bind(&Telecom::RegisterThread, this));
}

Telecom::~Telecom() 
{
    if (register_thread_) {
        register_thread_->interrupt();
        register_thread_->join();
        delete register_thread_;
    }
}

void Telecom::RegisterThread()
{
    int register_interval = 0;

    const std::string url = "http://zzhc.vnet.cn:9998/";
    std::string content;
    std::vector<std::string> http_header;
    http_header.push_back("Content-Type:application/encrypted-json");
    std::string resp;
    Json::Reader reader;
    Json::Value root;
    int ret = -1;
    int result = -1;
    int times = 10;
    bool get_content_success = false;

    while(1) {
        while (times > 0) {
            get_content_success = false;
            if (Router::get_instance()->IsMobileWebConnected()) {
                if (GetRegisterCotent(content) == 0) {
                    get_content_success = true;
                    times--;
                    ret = Utils::get_instance()->http_post(url, content, resp, http_header, 60);
                    if (ret == 0) {
                        if (reader.parse(resp.c_str(), root)) {
                            std::string code = root["resultCode"].isNull() ? "" : root["resultCode"].asString();
                            std::string msg  = root["resultDesc"].isNull() ? "" : root["resultDesc"].asString();
                            if ((code == "0") && (msg == "Success")) {
                                result =  0;
                                log_info("self register success");
                            }
                        } else {
                            result = -1;
                        }
                    } else {
                        result = -2;
                    }
                }
            }

            if (get_content_success) {
                if (result == 0) {
                    break;
                } else {
                    sleep(3600);
                }
            } else {
                sleep(30);
            }
        }
        if (result != 0) {
            break;
        }
        sleep(12*3600);
        times = 10;
    }
}

int Telecom::GetRegisterCotent(std::string &content)
{
    std::string sim_iccid = "0";
    std::string sim_imsi = "0";
    std::string macid = "0";
    std::string imei = "0";
    std::string sim_cellid = "0";
    ros::Time now = ros::Time::now();
    std::string time = timestampToDate(now.toSec());

    MobileWebInfo info;
    if (Router::get_instance()->GetMobileWebInfo(&info) == 0) {
        sim_iccid = info.iccid;
        sim_imsi = info.imsi;
        macid = info.mac;
        imei = info.imei;
        sim_cellid = info.cellid;
    } else {
        return -1;
    }
 

    char buffer[1024] = {0};
    snprintf(buffer, sizeof(buffer), "{\"REGVER\":\"7.0\","
                                      "\"MODEL\":\"SYB-03H19003\","
                                      "\"SWVER\":\"Atris1W-R-1.03\","
                                      "\"UETYPE\":29,"
                                      "\"SIM1ICCID\":\"%s\","
                                      "\"SIM1LTEIMSI\":\"%s\","
                                      "\"MACID\":\"%s\","
                                      "\"OSVER\":\"Linux imx6ull14x14evk 4.1.15\","
                                      "\"IMEI1\":\"%s\","
                                      "\"SIM1CELLID\":\"%s\","
                                      "\"REGDATE\":\"%s\""
                                      "}",
                                      sim_iccid.c_str(),
                                      sim_imsi.c_str(),
                                      macid.c_str(),
                                      imei.c_str(),
                                      sim_cellid.c_str(),
                                      time.c_str());

    log_info("telecome Register json_string is:");
    log_info("%s", buffer);
    content = base64_encode((unsigned char*)buffer, strlen(buffer)+1);

    return 0;
}

std::string  Telecom::timestampToDate(time_t time)
{
        struct tm *t = localtime(&time);
        char dateBuf[128] = "0";
        snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,
                t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

        std::string date(dateBuf);
        return date;
}