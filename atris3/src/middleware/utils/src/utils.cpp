#include <stdlib.h>
#include <iostream>
#include <vector>
#include <json/json.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
#include <net/route.h>
#include <ifaddrs.h>
#include <libgen.h>
#include <netinet/ip_icmp.h>
#include <unistd.h>
#include <signal.h>
#include <sys/stat.h>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <sys/resource.h>
#include "md5/md5.h"
#include "utils.h"
#include "curl/curl.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"
#include "log/log.h"

#define MAX_BUF_LEN 2048
#define ROUTER_CONN_TIMEOUT 5//second
#define TIMEOUT 3
#define JAN_1970 0x83AA7E80
#define ROUTER_UPDATE_TIME  30
bool Utils::response_handle_init = false;
bool Utils::notify_handle_init = false;
bool Utils::time_sync_state = false;

int Utils::init()
{
    cfg = Config::get_instance();
    rtc_dev = RtcDevice::get_instance();
    //rtc_dev->rtc_open();
    http_debug = cfg->http_debug_enable;
    int ret = -1;
    ret = check_net_interface_state("eth0");
    if(ret < 0){

    }

    int pid = getpid();
    ping = new MyPing(pid);

    boost::thread time_thread(boost::bind(&Utils::sync_sys_time, this));

    return ret;
}
// 如果ntp时间和系统时间相差过多，则同步系统时间
// 相差不多，不用同步时间
// 需要同步时间return 1, 不需要返回0
#if 1
bool need_sync_time(struct tm *time_tm)
{
	if(time_tm == NULL)
	{
		log_error("input parameter error!!!\r\n");
		std::cout<<"input parameter error"<<std::endl;
		return 0;
	}
	
    struct timeval time_tv;
    time_t timeCur;
    time_t timeSys;
    long currentSysSec;

    Config *cfg = Config::get_instance();
    time_tm->tm_hour += cfg->timezone; //加上时区
    timeCur = mktime(time_tm);

	gettimeofday(&time_tv, NULL); // 获取系统时间(带时区)
	//time_t t_sys = time(NULL);
    //tm* local_tm = localtime(&t_sys);
	currentSysSec = time_tv.tv_sec + time_tv.tv_usec/1000000;
	timeSys = (time_t)currentSysSec;

	//if((time_sys - timep > 10) || (timep - time_sys > 10))
    if(abs(timeSys-timeCur) > 10)
	{
		log_warn("need to sync time from ntp to sys, current sys time = %ld sec, ntp ts = %ld sec\r\n",timeSys,timeCur);
		//log_warn("time diff = %ld sec\r\n",abs(time_sys - timep));
		return true;
	}

	log_info("do not need to sync time from ntp to sys, current sys time = %ld, ntp ts = %ld\r\n",timeSys,timeCur);
	//log_info("time diff = %ld\r\n",abs(time_sys - timep));
    
    return false;

}
#endif
static void construct_ntp_packet(char content[])
{
    long timer;
    memset(content, 0, 48);
    content[0] = 0x1b;          // LI = 0 ; VN = 3 ; Mode = 3 (client);
    time((time_t *)&timer);
    timer = htonl(timer + JAN_1970);
    memcpy(content + 40, &timer, sizeof(timer));  //trans_timastamp
}

static int get_ntp_time(int sockfd, struct sockaddr_in *server_addr, struct tm *net_tm)
{
    char content[256];
    time_t timet;
    long temp;
    int addr_len = 16;
    struct timeval block_time;
    fd_set sockfd_set;
    FD_ZERO(&sockfd_set);
    FD_SET(sockfd, &sockfd_set);
    block_time.tv_sec = TIMEOUT;      //time out
    block_time.tv_usec = 0;

    construct_ntp_packet(content);

    if (sendto(sockfd, content, 48, 0, (struct sockaddr *)server_addr, addr_len) < 0) {
        return (-1);
    }

    if (select(sockfd + 1, &sockfd_set, NULL, NULL, &block_time ) > 0) {
        if (recvfrom(sockfd, content, 256, 0, (struct sockaddr *)server_addr, (socklen_t *)&addr_len) < 0) {
            return (-1);
        } else {
            memcpy(&temp, content + 40, 4);
            temp = (time_t)(ntohl(temp) - JAN_1970 );
            timet = (time_t)temp;
            memcpy(net_tm, gmtime(&timet), sizeof(struct tm));
        }
    } else {
        return (-1);
    }

    return (0);
}

static int set_system_time(struct tm *time_tm)
{
    struct timeval time_tv;
    time_t timep;
	// comment out by jinzhongxi , since we compare time in need to sync function , we do not need to add the time zone for settimeofday function
#if 0
    Config *cfg = Config::get_instance();
    time_tm->tm_hour += cfg->timezone; //加上时区
#endif
    timep = mktime(time_tm);
    time_tv.tv_sec  = timep;
    time_tv.tv_usec = 0;

    int ret = settimeofday(&time_tv, NULL);
    if(ret < 0){
        log_error("set time fail.%s", strerror(errno));
        return -1;
    }

    return 0;
}
#if 1
// 设置时间到rtc时钟
int set_ntp_time_to_rtc(struct tm *time_tm)
{
	int iRet;
	//struct rtc_time cur_rtc_tm;
	Rtc_Time rtc_utc_ts;
	time_t timep;
	
	struct tm * time_tm_rtc;
	
	if(time_tm == NULL)
	{
		log_error("function set_ntp_time_to_rtc : input parameter invalid\r\n");
		//std::cout <<"function set_ntp_time_to_rtc : input parameter invalid"<<std::endl;
		return -1;
	}

	timep = mktime(time_tm); // this function transfer to UTC time *****
	
	RtcDevice * pRtcDev = RtcDevice::get_instance();
	//Config *cfg = Config::get_instance();
    //time_tm->tm_hour -= cfg->timezone;
	//timep -= cfg->timezone*3600; // -8个小时
	time_tm_rtc = gmtime(&timep);
	#if 0
	// 打开rtc设备
	iRet = p_rtc_dev->rtc_open();
	if(0 > iRet)
	{
		log_error("function : set ntp time to rtc , open rtc device failed!!!\r\n");
		std::cout << "function : set ntp time to rtc , open rtc deivice failed , iRet = " << iRet << std::endl;
		return -1;
	}
	
	// 写入时间
	//iRet = p_rtc_dev->write_rtc(time_tm_rtc->tm_year, time_tm_rtc->tm_mon, time_tm_rtc->tm_mday, time_tm_rtc->tm_hour, time_tm_rtc->tm_min, time_tm_rtc->tm_sec);
	rtc_utc_ts.iYear = time_tm_rtc->tm_year;
	rtc_utc_ts.iMonth = time_tm_rtc->tm_mon;
	rtc_utc_ts.iDay = time_tm_rtc->tm_mday;
	rtc_utc_ts.iHour = time_tm_rtc->tm_hour;
	rtc_utc_ts.iMin = time_tm_rtc->tm_min;
	rtc_utc_ts.iSecond = time_tm_rtc->tm_sec;

	iRet = p_rtc_dev->write_rtc(rtc_utc_ts);
	if(0 > iRet)
	{
		log_error("function : set ntp time to rtc , write rtc device failed!!!\r\n");
		std::cout << "function : set ntp time to rtc , write rtc deivice failed iRet = "<< iRet << std::endl;
		return -1;
	}

	// 验证rtc时间写入
	// read current rtc
	iRet = p_rtc_dev->read_rtc(&cur_rtc_tm);
	if(0 > iRet)
	{
		log_error("function set ntp time to rtc , read current rtc failed iRet = %d\r\n", iRet);
		std::cout << "function set ntp time to rtc , read current rtc time failed!!! iRet = " << iRet << std::endl;
		return -1;
	}

	// 关闭rtc设备
	p_rtc_dev->rtc_close();
	//std::cout<<"close rtc dev"<<std::endl;
	#endif
	rtc_utc_ts.iYear = time_tm_rtc->tm_year;
	rtc_utc_ts.iMonth = time_tm_rtc->tm_mon;
	rtc_utc_ts.iDay = time_tm_rtc->tm_mday;
	rtc_utc_ts.iHour = time_tm_rtc->tm_hour;
	rtc_utc_ts.iMin = time_tm_rtc->tm_min;
	rtc_utc_ts.iSecond = time_tm_rtc->tm_sec;

	pRtcDev->setRtcTime(rtc_utc_ts);
	
	return 0;
}
#endif
static int try_get_time(char *ip)
{
    int sockfd;
    struct tm net_tm;
    struct sockaddr_in addr;
    char date_buf[50];
    int ret = 0;
	int iRet;
	//RtcDevice * p_rtc_dev = RtcDevice::get_instance();

    memset(&addr, 0, sizeof(addr));
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(123);

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        return (-1);
    }

    if (get_ntp_time(sockfd, &addr, &net_tm) == 0) 
	{
#if 1
		//iRet = need_sync_time(&net_tm);
		if(need_sync_time(&net_tm))
		{
			// set current ntp time to system
        	set_system_time(&net_tm);
			// also set utc time to correct current rtc
			set_ntp_time_to_rtc(&net_tm);
        	strftime(date_buf, sizeof(date_buf), "\"%F %T\"", &net_tm);
        	log_info("got ntp time:%s", date_buf);
			std::cout<<"got ntp time"<<date_buf;
			log_info("need to sync time to system\r\n");
			//std::cout<<"need to sync time to system"<<std::endl;
		}
		else
		{
			log_info("do not need to sync time to sys\r\n");
			//std::cout<<"do not need to sync time to system\r\n"<<std::endl;
		}
#endif

		// return 0 means that we got the ntp time from server success
        ret = 0;
    }  else
        ret = -1;

    close(sockfd);

    return ret;
}

// sync the ntp server time to the system
static int sync_ntp_time()
{
    struct hostent *he;
    char **pptr;
    char str[32] = {0};
	// added by jinzhongxi 2019/09/23
	// initialize value should not be zero
	// if ret = 0, we can not connect to internet , get host by name is null and continue
	// current wrong system time will be sync to hard ware
    int ret = -1;
    Config *cfg = Config::get_instance();
    int ntp_server_num = cfg->ntp_server_list.size();

    for (int i = 0; i < ntp_server_num; i ++) {
        he = gethostbyname(cfg->ntp_server_list.at(i).c_str());
        if (he == NULL)
            continue;

        switch (he->h_addrtype) {
            case AF_INET:
            case AF_INET6:
                pptr = he->h_addr_list;

                for (; *pptr != NULL; pptr++) {
                    inet_ntop(he->h_addrtype, *pptr, str, sizeof(str));

                    if (try_get_time(str) >= 0) {
                        ret = 0;
                        return ret;
                    }
                }

                ret = -1;
                break;

            default:
                break;
        }
    }

    return ret;
}

int set_local_time_to_rtc(void)
{
	struct timeval time_tv;
	struct tm * pstrTimeVal = NULL;
	long currentSysSec;
	time_t timeSys;
	int iRet;	

	gettimeofday(&time_tv,NULL);
	currentSysSec = time_tv.tv_sec + time_tv.tv_usec/1000000;
	timeSys = (time_t)currentSysSec;
	pstrTimeVal = localtime(&timeSys);
	#if 0
	printf("year:%d-month:%d-day:%d-hour:%d-minute:%d-second:%d\r\n", \
		1900+pstrTimeVal->tm_year, \
		1+pstrTimeVal->tm_mon, \
		pstrTimeVal->tm_mday, \
		pstrTimeVal->tm_hour, \
		pstrTimeVal->tm_min, \
		pstrTimeVal->tm_sec \
		);
	#endif
	iRet = set_ntp_time_to_rtc(pstrTimeVal);
	if(iRet < 0)
	{
		log_error("%s set ntp time to rtc failed!!! iRet = %d\r\n",__FUNCTION__,iRet);
		return -1;
	}
	else
	{
		log_info("%s set ntp time to rtc success...\r\n",__FUNCTION__);
	}

	return 0;
}

// if one of the ntp server is synced ok , we dont need to sync the system time again
static int ntpdate_time()
{
	int ret = -1;
	bool sync_ok_flg = false;
    Config *cfg = Config::get_instance();
    int ntp_server_num = cfg->ntp_server_list.size();
	//printf("ntp server num = %d\r\n",ntp_server_num);
	for(int i = 0; i < ntp_server_num;i++)
	{
		std::string ntp_cmd = "";
		FILE * fp = NULL;
		char buffer[256] = {0};
		ntp_cmd = "ntpdate ";
		ntp_cmd += cfg->ntp_server_list.at(i);
		//std::cout<<i<<"th , "<<"ntp cmd string : " << ntp_cmd <<std::endl;
		if((fp = popen(ntp_cmd.c_str(),"r")))
		{
			while(fgets(buffer,sizeof(buffer)-1,fp))
			{
				//log_info("ntp return buf = %s\r\n",buffer);
				if(strstr(buffer,"no server suitable")!=NULL)
				{
					// sync time failed!!!
					log_warn("sync time failed!!!\r\n");
					sync_ok_flg = false;
				}
				else if(strstr(buffer,"step time server")!= NULL || strstr(buffer,"adjust time server")!= NULL)
				{
					// sync time success...
					log_info("sync time success...\r\n");
					sync_ok_flg = true;
				}
				else
				{
					log_error("sync time return unrecognized string! %s\r\n",buffer);
					sync_ok_flg = false;
				}
			}
			
		}

		pclose(fp);
		fp = NULL;

		if(sync_ok_flg)
		{
			log_info("sync time ok on ntp server %s",cfg->ntp_server_list.at(i).c_str());
			ret = 0;
			break;
		}
	}

	return ret;
}

void Utils::sync_sys_time() {
    //log_info("sync system time"); 
    int retry_times = 3;
    Utils::time_sync_state = false;
    int ret;
	// keep syncing time using ntp
    while(true)
	{
		#if 0
        ret = sync_ntp_time();
        if (ret == 0)
        {
            log_info("sync system time ok");

			// added by jzx , here we do not write the ntp time into the hardware
			// however we need to substract the system time by 8 hours because rtc is using the utc time
			/*
            // set the hardware clock to the current system time
            FILE *fp = NULL;
            if((fp = popen("hwclock -w", "r"))) {
                pclose(fp);
            }
            */
            
            Utils::time_sync_state = true;
            retry_times = 3;
        }
        else
        {
            log_info("sync system time fail"); 
            if (--retry_times == 0)
            {
                log_info("sync state set false"); 
                Utils::time_sync_state = false;
            }
        }
		#endif

		// if sync time ok , we are sync again after 30 mins, if failed , we resync the time after 10seconds
        // sleep(ret == 0 ? 1800:10);
		ret = ntpdate_time();
		if(ret != 0)
		{
			//printf("sync time failed!!! ret = %d\r\n",ret);
		}
		else
		{
			//printf("sync time ok , right system time to rtc...\r\n");
			set_local_time_to_rtc();
		}

		//printf("sync time using ntpdate every 60 seconds\r\n");
		sleep(ret == 0 ? 1800:60);
	// for test
	//sleep(ret == 0 ? 10:10);
    }
}

int Utils::check_net_interface_state(const char *if_name)
{
    struct ifaddrs *ifa = NULL, *if_list;

    if (getifaddrs(&if_list) < 0){
        return -1;
    }

    for(ifa = if_list; ifa != NULL; ifa = ifa->ifa_next){
        if (ifa->ifa_addr == NULL)
            continue;
        if(ifa->ifa_addr->sa_family == AF_INET ||  ifa->ifa_addr->sa_family == AF_INET6){
            if(strcmp(ifa->ifa_name, if_name) == 0){
                if(!(ifa->ifa_flags & IFF_UP)){
                    log_error("%s:DEVICE_DOWN", if_name);
                    freeifaddrs(if_list);
                    return -1;
                }

                if(!(ifa->ifa_flags & IFF_RUNNING)){
                    log_error("%s:DEVICE_UNPLUGGED", if_name);
                    freeifaddrs(if_list);
                    return -1;
                }

                log_info("%s:DEVICE_LINKED", if_name);
                freeifaddrs(if_list);
                return 0;
            }
        }
    }

    log_info("%s:DEVICE_NONE", if_name);
    freeifaddrs(if_list);

    return -1;
}


static int on_debug(CURL *, curl_infotype itype, char * pData, size_t size, void *)
{
    if(itype == CURLINFO_TEXT)
        log_debug("[TEXT]\n%s", pData);
    //else if(itype == CURLINFO_HEADER_IN)
    //	log_debug("[HEADER_IN]\n%s", pData);
    else if(itype == CURLINFO_HEADER_OUT)
        log_debug("[HEADER_OUT]\n%s", pData);
    else if(itype == CURLINFO_DATA_IN)
        log_debug("[DATA_IN]\n%s", pData);
    else if(itype == CURLINFO_DATA_OUT){
        log_debug("[DATA_OUT]\n%s", pData);
    }

    return 0;
}

static int on_debug_file(CURL *, curl_infotype itype, char * pData, size_t size, void *)
{
    if(itype == CURLINFO_TEXT)
        log_debug("[TEXT]\n%s", pData);
    //else if(itype == CURLINFO_HEADER_IN)
    //	log_debug("[HEADER_IN]\n%s", pData);
    else if(itype == CURLINFO_HEADER_OUT)
        log_debug("[HEADER_OUT]\n%s", pData);
    else if(itype == CURLINFO_DATA_IN)
        log_debug("[DATA_IN]\n%s", pData);
    else if(itype == CURLINFO_DATA_OUT){
        //log_debug("[DATA_OUT]\n%02x", *pData);
    }

    return 0;
}


static size_t on_write_data(void* buffer, size_t size, size_t nmemb, void* userdata)
{
    std::string* resp = dynamic_cast<std::string*>((std::string *)userdata);
    if( NULL == resp || NULL == buffer )
    {
        log_error("resp null");
        return -1;
    }

    char* data = (char*)buffer;
    resp->append(data, size * nmemb);

    //log_debug("resp:%s", data);
    return nmemb*size;
}

#if 0
std::string Utils::url_decode(const std::string& str)
{
    std::string strTemp = "";
    size_t length = str.length();
    for (size_t i = 0; i < length; i++)
    {
        if (str[i] == '+') strTemp += ' ';
        else if (str[i] == '%')
        {
            assert(i + 2 < length);
            unsigned char high = FromHex((unsigned char)str[++i]);
            unsigned char low = FromHex((unsigned char)str[++i]);
            strTemp += high*16 + low;
        }
        else strTemp += str[i];
    }
    return strTemp;
}
#endif

std::string Utils::url_encoder(const char * str, int size)
{
    std::string result = "";
    int i;
    char ch;

    if ((str == NULL) || (size <= 0)) {
        return result;
    }

    for ( i=0; i<size; ++i){
        ch = str[i];

        if (((ch>='A') && (ch<='Z')) ||
                ((ch>='a') && (ch<='z')) ||
                ((ch>='0') && (ch<='9'))){
            result += ch;
        }

        else if (ch == ' '){
            result += '+';
        }

        else if (ch == '.' || ch == '-' || ch == '_' || ch == '*'){
            result += ch;
        }
        else{
            char temp[3]="";
            sprintf(temp, "%%%02X", (unsigned char)ch);
            result += temp;
        }
    }

    return result;
}

int Utils::http_get(const std::string &url, std::string &resp, const std::vector<std::string>& header, int timeout)
{
    CURLcode res;
    CURL* curl = curl_easy_init();
    if(NULL == curl)
    {
        log_error("curl easy init fail.");
        return -1;
    }

    if(http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug);
    }

    struct curl_slist* headerList = NULL;
    for(unsigned int i = 0; i < header.size(); i++) {
        if (!header.empty()) {
            headerList = curl_slist_append(headerList, header[i].c_str());
        }
    }

    if (headerList) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerList);
    }
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        log_error("%s error(%d): %s", __FUNCTION__, res, curl_easy_strerror(static_cast<CURLcode>(res)));
    }
    curl_easy_cleanup(curl);
    curl_slist_free_all(headerList);

    return res;
}

void Utils::responseResult(const atris_msgs::SignalMessage& origin,
  const Json::Value& content, std::string title) {
    ros::Time now = ros::Time::now();
    Json::FastWriter jwriter;
    Json::Value root;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = Json::Value(title);
    root["accid"] = shmrbt.robot.sn;
    root["content"] = content;
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
    if (root["content"]["id"].isNull()) {
      std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
      root["content"]["id"] = uid.str();
    }

    atris_msgs::SignalMessage resp;
    resp.type = origin.type;
    resp.account = origin.account.empty() ? "unknow" : origin.account;
    resp.msgID = root["content"]["id"].asString();
    resp.msg = jwriter.write(root);
    resp.title = title;

    static ros::NodeHandle resp_nh;
    static ros::Publisher resp_pub = resp_nh.advertise <atris_msgs::SignalMessage> (TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    if (!Utils::response_handle_init) {
      int retry_count = 200;
      do {
        usleep(30*1000);
      } while(!resp_pub.getNumSubscribers() && (--retry_count) > 0);

      Utils::response_handle_init = true;
    }
    resp_pub.publish(resp);
}

void Utils::NotifyRobotStatus(const std::string &title, const Json::Value& content, std::string type)
{
    ros::Time now = ros::Time::now();
    Json::FastWriter jwriter;
    Json::Value root;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    
    root["title"] = Json::Value(title);
    root["accid"] = shmrbt.robot.sn;
    root["content"] = content;
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
    if (root["content"]["id"].isNull()) {
      std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
      root["content"]["id"] = uid.str();
    }

    std::string receiver = shmrbt.robot.receiver;
    atris_msgs::SignalMessage resp;
    resp.type = type;
    resp.account = receiver.empty() ? "unknow" : receiver;
    resp.msgID = root["content"]["id"].asString();
    resp.title = title;
    resp.msg = jwriter.write(root);

    static ros::NodeHandle notify_nh;
    static ros::Publisher notify_pub = notify_nh.advertise <atris_msgs::SignalMessage> (TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    if (!Utils::notify_handle_init) {
      int retry_count = 200;
      do {
        usleep(30*1000);
      } while(!notify_pub.getNumSubscribers() && (--retry_count) > 0);

      Utils::notify_handle_init = true;
    }
    notify_pub.publish(resp);
}


int file_size = 0;
static size_t on_write_file_data(void* buffer, size_t size, size_t nmemb, void* userdata)
{
    FILE* fp = static_cast<FILE *>(userdata);

    file_size += size*nmemb;
    size_t length = fwrite(buffer, size, nmemb, fp);
    if (length != nmemb)
    {
        return length;
    }

    return nmemb*size;
}

int Utils::http_get_file(const std::string &url, std::string &path)
{
    CURLcode res;

    FILE *fp = fopen(path.c_str(), "ab+");
    if(fp == NULL){
        log_error("open file error:%s", path.c_str());
        return -1;
    }

    CURL* curl = curl_easy_init();
    if(NULL == curl)
    {
        log_error("curl easy init fail.");
        fclose(fp);
        return -1;
    }

    if(http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug);
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_file_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)fp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    fclose(fp);

    return res;
}

int Utils::http_post(const std::string &url, const std::string &json, std::string &resp, const std::vector<std::string> &header, long & res_code, int timeout)
{
    log_info("%s http post with stauts code\r\n",__FUNCTION__);
    CURLcode res;
    CURL* curl = curl_easy_init();

    if (NULL == curl)
    {
        log_error("curl easy init fail.");
        return -1;
    }

    if (http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug);
    }

    struct curl_slist* headerList = NULL;
    for (unsigned int i = 0; i < header.size(); i++) {
        if (!header.empty()) {
            headerList = curl_slist_append(headerList, header[i].c_str());
        }
    }

    if (headerList) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerList);
    }
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
    //curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json.size());
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, timeout);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);

    log_info("%s , set CURLOPT_SSL_VERIFYPEER CURLOPT_SSL_VERIFYHOST \r\n",__FUNCTION__);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0); //... temp
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
    res = curl_easy_perform(curl);
    if(res == CURLE_OK)
    {
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &res_code);
    }
    
    curl_easy_cleanup(curl);

    curl_slist_free_all(headerList);
    log_info("res:%d resp:%s res_code:%d\r\n", res, resp.c_str(),res_code);
    return res;
}

int Utils::http_post_with_header(const std::string &url, const std::string &json, std::string &resp, int timeout){
    log_info("%s url:%s json:%s", __FUNCTION__, url.c_str(), json.c_str());
    CURLcode res;
    CURL* curl = curl_easy_init();

    if (NULL == curl){
        log_error("curl easy init fail.");
        return -1;
    }

    if (http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug);
    }

    struct curl_slist* headerList = NULL;
    std::vector<std::string> header;
	header.push_back("Content-Type:application/json");
	header.push_back("Accept: application/json");
    GetHttpHeader(header);
    for (size_t i = 0; i < header.size(); i++) {
        if (!header.empty()) {
            headerList = curl_slist_append(headerList, header[i].c_str());
        }
    }

    if (headerList) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerList);
    }
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
    //curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json.size());
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        log_error("%s error(%d): %s", __FUNCTION__, res, curl_easy_strerror(static_cast<CURLcode>(res)));
    }
    curl_easy_cleanup(curl);

    curl_slist_free_all(headerList);
    log_info("%s res:%d resp:%s",__FUNCTION__, res, resp.c_str());
    return res;
}

int Utils::http_post(const std::string &url, const std::string &json, std::string &resp, const std::vector<std::string> &header, int timeout)
{
    log_info("line:%d:%s",__LINE__,json.c_str());
    CURLcode res;
    CURL* curl = curl_easy_init();

    if (NULL == curl)
    {
        log_error("curl easy init fail.");
        return -1;
    }

    if (http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug);
    }

    struct curl_slist* headerList = NULL;
    for (unsigned int i = 0; i < header.size(); i++) {
        if (!header.empty()) {
            headerList = curl_slist_append(headerList, header[i].c_str());
        }
    }

    if (headerList) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerList);
    }
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
    //curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json.size());
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        log_error("%s error(%d): %s", __FUNCTION__, res, curl_easy_strerror(static_cast<CURLcode>(res)));
    }
    curl_easy_cleanup(curl);

    curl_slist_free_all(headerList);
    log_info("line:%d,res:%d resp:%s",__LINE__, res, resp.c_str());
    return res;
}

std::string Utils::get_fileurl()
{
    return fileurl;
}
std::string Utils::set_fileurl(const std::string &url)
{
    fileurl = url;
    return fileurl;
}

std::string Utils::parse_http_resp(std::string &resp)
{
    Json::Reader reader;
    Json::Value root;
    std::string url = "";

    if(resp.empty()){
        log_error("http resp empty");
        return url;
    }

    if(!reader.parse(resp, root)){
        log_error("parse http resp fail.");
        return url;
    }
     
    if(!root.isMember("url"))
    {
        log_error(" http resp no url member.");
        return url;
    }

    if(!root["url"].isNull()){
        url = root["url"].asString();
    }else if((!root["data"].isNull()) && !root["data"]["url"].isNull()){
        url = root["data"]["url"].asString();
    }else{
        log_error("http resp no url format is invalid:%s.", resp.c_str());
    }

    return url;
}

void Utils::parse_http_resp(std::string &resp, std::string & file_url)
{
    Json::Reader reader;
    Json::Value root;

    if (resp.empty()) {
        log_error("http resp empty");
        return;
    }

    if(!reader.parse(resp, root)){
        log_error("parse http resp fail.");
        return;
    }
    if (!root["msg"].isNull()) { 
        if (root["msg"].asString() == "success") {
        	if(root["data"].isArray()){
        	    int sz = root["data"].size();
        	    for (int i = 0; i < sz; i++){
					Json::Value it = root["data"][i];
        			if (it["url"].isString() && it["url"].asString().find(".zip") != std::string::npos){
        	            file_url = it["url"].asString();
        	            log_info("idx:%d file url:%s", i, file_url.c_str());
						break;
					}
        	    }
            }else if (!root["data"].isNull() && !root["data"]["url"].isNull()) {
                file_url = root["data"]["url"].asString();
                log_debug("after parse http resp, url : %s ", file_url.c_str());
        	}else{
        	    log_info("can't find url in resp");
        	}
		}else{
        	log_info("resp not success");
        }
    }
}

int Utils::http_local_post_file(std::string &url, const std::string &file_local_path, std::string &resp)
{
    CURLcode res;
    struct curl_httppost* post = NULL;
    struct curl_httppost* last = NULL;
    CURL* curl = curl_easy_init();
    std::vector<std::string> header;
    if(NULL == curl)
    {
        log_error("curl easy init fail.");
        return -1;
    }

    if(http_debug){
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
        curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug_file);
    }

    struct curl_slist* headers = NULL;

    if(url.find("udfs-tracer") != std::string::npos) {
        header.clear();
        GetHttpHeader(header);
        for (unsigned int i = 0; i < header.size(); i++) {
            if (!header.empty()) {
                headers = curl_slist_append(headers, header[i].c_str());
            }
        }

		if (file_local_path.find(".zip") != std::string::npos){
			log_info("upload zip file to set unzip and needsubpath");
    	    curl_formadd(&post, &last, CURLFORM_COPYNAME, "unzip", CURLFORM_COPYCONTENTS, "true", CURLFORM_END);
    	    curl_formadd(&post, &last, CURLFORM_COPYNAME, "needSubPath", CURLFORM_COPYCONTENTS, "true", CURLFORM_END);
		}
    }
    
    headers = curl_slist_append(headers, "Content-Type:multipart/form-data");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_formadd(&post, &last, CURLFORM_COPYNAME, "file",
        CURLFORM_FILE, file_local_path.c_str(), CURLFORM_CONTENTTYPE, "multipart/form-data", CURLFORM_END);

    bool use_fastdfs = false;
    if(url.find("upload") != std::string::npos){

        use_fastdfs = true;
        if(*(url.end()-1) == '/'){
            url = url.substr(0, url.length() - 1);
        }
        std::string output = "json";
        std::string path = "/atrisfiles/map/";
        std::string scene = "default";

        curl_formadd(&post, &last, CURLFORM_COPYNAME, "output",
        CURLFORM_COPYCONTENTS, output.c_str(),
        CURLFORM_END);

        curl_formadd(&post, &last, CURLFORM_COPYNAME, "path",
        CURLFORM_COPYCONTENTS, path.c_str(),
        CURLFORM_END);

        curl_formadd(&post, &last, CURLFORM_COPYNAME, "scene",
        CURLFORM_COPYCONTENTS, scene.c_str(),
        CURLFORM_END);
    
    }

    log_info("utils function: %s url:  %s , file local path: %s ",__FUNCTION__, url.c_str(), file_local_path.c_str() );

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, post);


    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

    res = curl_easy_perform(curl);
    curl_formfree(post);
    curl_easy_cleanup(curl);

    curl_slist_free_all(headers);

    log_info("res:%d resp:%s", res, resp.c_str());
    std::string file_url = "";
    if (use_fastdfs) {
        file_url = parse_http_resp(resp);
        log_info("file url : %s. ", file_url.c_str());
        set_fileurl(file_url);
    }
    else if (url.find("udfs-tracer") != std::string::npos) {
        parse_http_resp(resp, file_url);
        log_info("file url : %s. ", file_url.c_str());
        set_fileurl(file_url);
    }

    return res;  

}

int Utils::http_post_file(const std::string &url, const std::string &file_path, std::string &resp)
{
    CURLcode res;
    CURL* curl = curl_easy_init();

    if(NULL == curl)
    {
        log_error("curl easy init fail.");
        return -1;
    }

    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type:application/octet-stream");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    curl_easy_setopt(curl, CURLOPT_POST, 1L);

    FILE *fp= fopen(file_path.c_str(), "rb");
    if (fp == NULL) {
        log_error("open file %s error", file_path.c_str());
        return -1;
    }

    struct stat fstat;
    if (stat(file_path.c_str(), &fstat) != 0) {
        fclose(fp);
        log_error("statfile %s error", file_path.c_str());
        return -1;
    }

    curl_easy_setopt(curl, CURLOPT_READDATA, fp);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (curl_off_t)fstat.st_size);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&resp);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    curl_slist_free_all(headers);

    fclose(fp);
    log_info("res:%d resp:%s", res, resp.c_str());
    return res;
}


MyPing::MyPing(int pid)
{
    this->pid = pid;
    rawsock = -1;

    struct protoent *protocol;
    if ((protocol = getprotobyname("icmp")) == NULL)
    {
        log_error("getprotobyname");
        return ;
    }

    rawsock = socket(AF_INET, SOCK_RAW, protocol->p_proto/*IPPROTO_ICMP*/);
    if(rawsock < 0){
        log_error("crate raw socket fail.");
        return ;
    }

}

MyPing::~MyPing()
{
    if(rawsock >= 0)
        close(rawsock);
    rawsock = -1;
}

unsigned short MyPing::icmp_cksum(unsigned char *data)
{
    int sum = 0;
    int len = sizeof(struct icmp);
    int odd = len & 0x01;
    while(len & 0xfffe){
        sum += *(unsigned short*)data;
        data += 2;
        len -= 2;
    }
    /*判断是否为奇数个数据,若ICMP报头为奇数个字节,会剩下最后一个字节*/
    if(odd){
        unsigned short tmp = ((*data)<<8)&0xff00;
        sum += tmp;
    }
    sum = (sum >> 16) + (sum & 0xffff);
    sum += (sum >> 16);

    return ~sum;
}

void MyPing::icmp_pack(struct icmp *icmph)
{
    icmph->icmp_type = ICMP_ECHO; //ICMP回显请求
    icmph->icmp_code = 0; //code的值为0
    icmph->icmp_cksum = 0;//先将cksum的值填为0，便于以后的cksum计算
    icmph->icmp_seq = 1;//本报的序列号
    icmph->icmp_id = pid & 0xffff;//填写PID
    icmph->icmp_cksum = icmp_cksum((unsigned char*)icmph);
}

int MyPing::icmp_unpack(char *buf, int len)
{
    int iphdrlen;
    struct ip *ip = NULL;
    struct icmp *icmp = NULL;

    ip = (struct ip *)buf;//IP报头
    iphdrlen = ip->ip_hl << 2; //IP头部长度
    icmp = (struct icmp *)(buf+iphdrlen);//ICMP段的地址
    len -= iphdrlen;
    //判断长度是否为ICMP包
    if(len < 8){
        log_warn("not icmp data");
        return -1;
    }

    if((icmp->icmp_type == ICMP_ECHOREPLY) && (icmp->icmp_id == pid)){
        return 0;
    }
    return -1;
}

int MyPing::icmp_send(const char *ip)
{
    int size = 0;
    struct icmp packet;

    memset(&dest_addr, 0, sizeof(dest_addr));

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(ip);

    icmp_pack(&packet);
    size = sendto(rawsock, &packet, sizeof(struct icmp),0,
            (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if(size < 0){
        log_error("sendto error");
        return -1;
    }

    return 0;
}

int MyPing::icmp_recv()
{
    struct timeval tv;
    fd_set readfd;
    int retry = 2;

    while(retry > 0){
        tv.tv_usec = 0;
        tv.tv_sec = 1;
        int ret = 0;
        FD_ZERO(&readfd);
        FD_SET(rawsock,&readfd);
        ret = select(rawsock+1,&readfd,NULL,NULL,&tv);
        retry --;
        switch(ret)
        {
            case -1:
                //错误发生
                break;
            case 0:
                //超时
                break;
            default :
                {
                    unsigned int fromlen = sizeof(struct sockaddr_in);
                    struct sockaddr from;
                    char pkg[64] = {0};
                    int size = ::recvfrom(rawsock, &pkg,sizeof(pkg),0, &from, &fromlen);
                    if(errno == EINTR){
                        log_error("recvfrom error");
                        continue;
                    }
                    ret = icmp_unpack((char *)&pkg,size);
                    if(ret < 0){
                        //log_error("not icmp data.");
                        continue;
                    }
                    else
                        return 0;
                }
                break;
        }
    }
    return -1;
}

bool Utils::check_network_state(const char *ip)
{
    int pid = getpid();
    MyPing *ping = new MyPing(pid);

    int ret = ping->icmp_send(ip);
    if(ret < 0){
        log_info("check network state fail.");
        delete ping;
        return false;
    }

    ret = ping->icmp_recv();
    if(ret == 0){
        delete ping;
        return true;
    }

    delete ping;
    return false;
}

#define CORE_SIZE   1024 * 1024 * 10   //10m
int Utils::init_core_dump()
{
    struct rlimit rlmt;

    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }

    rlmt.rlim_cur = (rlim_t)CORE_SIZE;
    rlmt.rlim_max  = (rlim_t)CORE_SIZE;

    if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }

    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }

    printf("rlimit CORE dump current is:%d, max is:%d\n", (int)rlmt.rlim_cur, (int)rlmt.rlim_max);

    return 0;
}

void Utils::getRandStr(char *str)
{
    if (str == NULL) {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    int rand_num =  tv.tv_sec + tv.tv_usec;
    srand(rand_num);

    int len = 10;
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

bool Utils::gen_md5(const char *file, char *md5)
{
    boost::lock_guard<boost::mutex> lock(md5_mutex);
    std::string cmd = "/usr/bin/md5sum  " ;//+ std::to_string(file) + " > /tmp/uploaded.md5 ";
    cmd = cmd + file;
    cmd = cmd + " > /tmp/uploaded.md5";

    //log_info("cmd:%s", cmd.c_str());
    int r = system(cmd.c_str());
    int t = 10* 5;
    while(t > 0){
        if(access("/tmp/uploaded.md5", 0) != 0)
            return false;
        else{
            usleep(200*1000);
            t --;
        }
    }

    FILE *f;
    f = fopen("/tmp/uploaded.md5", "r");
    if(f == NULL){
        log_error("open md5 file fail.");
        return false;
    }

    char buf[128] = {0};
    while(!feof(f)){
        if(fgets(buf, 128, f) != NULL){
            char *p = strtok(buf, " ");
            strcpy(md5, p);
            //memcpy(md5, p, strlen(p));
            break;
        }
    }

    //delete file
    int r1 = system("rm /tmp/uploaded.md5");

    return true;
}

int Utils::createDir(std::string dirname) {
    if(access(dirname.c_str(), F_OK) == 0 ) {
      return 0;
    }
    
    char* dir = strdup(dirname.c_str());
    if (!dir) {
        log_error("%s no memory error", __FUNCTION__);
        return -1;
    }

    int i, ret = -1, len = strlen(dir);

    log_info("%s mkdir: %s", __FUNCTION__, dir);

    for(i=1; i < len; i++) {
        if(dir[i]=='/') {
            dir[i] = 0;
            if(access((const char*)dir, F_OK) != 0 ) {
                if(mkdir(dir, 0777) == -1) {
                    log_error("%s mkdir: %s", __FUNCTION__, dir);
                    goto beach;
				}else{
					log_info("%s %s success", __FUNCTION__, dir);
				}
            }else{
				log_info("%s %s already exist", __FUNCTION__, dir);
			}
            dir[i] = '/';
        }
    }

    if(access((const char*)dir, F_OK) != 0 ) {
        if(mkdir(dir, 0777) == -1) {
            log_error("%s mkdir: %s", __FUNCTION__, dir);
            goto beach;
        }
    }else{
		log_info("%s %s create success", __FUNCTION__, dir);
	}

    ret = 0;

beach:
    if (!dir) {
        free(dir);
    }
    return ret;
}

void Utils::publish_event(EVENT evt, EVENT_VALUE val){
    Json::Value root;
    Json::Value js_evt;
    switch(evt){
        case EVT_LAMP:     js_evt["lamp"]     = val;break;
        case EVT_FLASHING: js_evt["flashing"] = val;break;
        case EVT_LEAKING:  js_evt["leaking"]  = val;break;
        case EVT_CHARGE:   js_evt["charge"]   = val;break;
        case EVT_SHUTDOWN: js_evt["shutdown"] = val;break;
        case EVT_IMPACTED: js_evt["impacted"] = val;break;
        case EVT_AVOIDING: js_evt["avoiding"] = val;break;
        case EVT_EMERGENCY:js_evt["emergency"]= val;break;
        case EVT_LEAVE_PILE_PATROL: js_evt["leave_pile_patrol"] = val;break;
        default:return;
    }

    root["result"] = "success";
    root["result_code"] = 0;
    root["events"].append(js_evt);

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    atris_msgs::SignalMessage msg;
    msg.account = shmrbt.robot.receiver;
    msg.type = "";
    log_info("publish event:%d val:%d", evt, val);
    Utils::get_instance()->responseResult(msg, root, "notify_event");
}

void Utils::GetHttpHeader(std::vector<std::string> &http_header)
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

void Utils::GetHttpHeaderSign(std::string &sign)
{
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string device_id  = shmrbt.robot.sn;
    std::string key = Config::get_instance()->abi_key;
    std::string version_num = Config::get_instance()->abi_version;
    std::string bank_space = " ";

    ros::Time now_time = ros::Time::now();
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


void Utils::GetRandString(char *str)
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

int Utils::getSign(unsigned num)
{
    int sign = num & (1<<31);
    return sign == 0 ? 1 : -1; 
}
 
int Utils::getExp(unsigned num)
{
    int exp = 0;
    for(int i = 23; i < 31; ++i)
        exp |= (num & (1<<i));
    exp = (exp>>23) - 127;
    return exp;
}
 
int Utils::float2int(float ft)
{
    unsigned num;
    memcpy(&num, &ft, sizeof(float));
 
    int exp = getExp(num);
    if(exp < 0)
    {
        return 0;
    }
    else
    {
        int res = num & ((1<<23)-1);
        res |= 1<<23;
        res >>= (23-exp);
        return res*getSign(num);
    }
}

