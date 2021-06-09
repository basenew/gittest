#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <stdlib.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <netinet/in.h>
#include "config/config.h"
#include "rtc/rtc.h"
#include <json/json.h>
#include <ros/ros.h>
#include "atris_msgs/SignalMessage.h"
#include "atris_defines.h"
#define setbit(x,pos) (x|=(1<<pos))
#define getbit(x,pos) ((x>>pos)&1)
#define clrbit(x,pos) (x&=~(1<<pos))
#define STR_LEN 10

class MyPing
{
    private:
        int rawsock;
        struct sockaddr_in dest_addr;

        unsigned int pid;
        unsigned short icmp_cksum(unsigned char *data);
        void icmp_pack(struct icmp *icmph);
        int icmp_unpack(char *buf, int len);

    public:
        MyPing(int pid);
        ~MyPing();
        int icmp_send(const char *ip);
        int icmp_recv();
};

enum EVENT{
    EVT_LAMP = 1,
    EVT_FLASHING,
    EVT_LEAKING,
    EVT_CHARGE,
    EVT_SHUTDOWN,
    EVT_IMPACTED,
    EVT_AVOIDING,
    EVT_EMERGENCY,
    EVT_LEAVE_PILE_PATROL
};

enum EVENT_VALUE{
    EVT_OPENED      = 1,
    EVT_CLOSED      = 0,
    EVT_TRIGGERED   = 1,
    EVT_UNTRIGGERED = 0
};

class Utils
{
    private:
        Utils() { }
        Config *cfg;
        bool http_debug;
        boost::function<int ()> time_cb;
        boost::mutex md5_mutex;
        MyPing *ping;
        std::string fileurl;
	      RtcDevice * rtc_dev;
        static bool response_handle_init;
        static bool notify_handle_init;
    public:
        static bool time_sync_state;
        static int createDir(std::string dirname);
        int init();
        bool gen_md5(const char *file, char *md5);
        void  getRandStr(char *str);
        int init_core_dump();
        int check_net_interface_state(const char* if_name);
        bool check_network_state(const char* ip);
        void sync_sys_time();
        int http_post(const std::string &url, const std::string &json, std::string &resp, const std::vector<std::string> &header = std::vector<std::string>(1, "Content-Type:application/json"), int timeout = 60);
        int http_post(const std::string &url, const std::string &json, std::string &resp, const std::vector<std::string> &header, long & res_code, int timeout);
		int http_post_with_header(const std::string &url, const std::string &json, std::string &resp, int timeout = 60);
        std::string url_encoder(const char * str, int size);
        int http_post_file(const std::string &url, const std::string &file_path, std::string &resp);
        std::string get_fileurl();
        std::string set_fileurl(const std::string &url);
        std::string parse_http_resp(std::string &resp);
        void parse_http_resp(std::string &resp, std::string & file_url);
        int http_local_post_file(std::string &url, const std::string &file_local_path, std::string &resp);
        int http_get(const std::string &url, std::string &resp, const std::vector<std::string> &header = std::vector<std::string>(), int timeout = 60);
        int http_get_file(const std::string &url, std::string &path);
        void responseResult(const atris_msgs::SignalMessage& origin,
          const Json::Value& content, std::string title);
        void NotifyRobotStatus(const std::string &title, const Json::Value& content, std::string type = "local_mqtt");
        void GetHttpHeader(std::vector<std::string> &http_header);
        void GetHttpHeaderSign(std::string &sign);
        void GetRandString(char *str);
		std::string GenFullApiPath(std::string host, std::string port, bool https, std::string sub_api_path){
			std::string full_api;
			full_api.append(https ? "https://":"http://");
			full_api.append(host);
			if (port != "80")full_api.append(":" + port);
			full_api.append(sub_api_path);
			return full_api;
		}
 
		int getSign(unsigned num);
		int getExp(unsigned num);
		int float2int(float ft);
        inline void replace_all(std::string& str, const char* old, const char* newstr)
        {
            std::string::size_type pos(0);

            while(true){
                pos=str.find(old, pos);
                if (pos!=(std::string::npos))
                    str.replace(pos, strlen(old), newstr);
                else
                    break;
            }
        }
        static Utils* get_instance(){
            static Utils singleton;
            return &singleton;
        }
        static void publish_event(EVENT evt, EVENT_VALUE val);
};

#endif
