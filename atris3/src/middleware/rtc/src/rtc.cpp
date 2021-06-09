//#include <poll.h>
//#include <signal.h>
//#include <fcntl.h>
#include <errno.h>
#include "rtc.h"
#include <time.h>
#include <iostream>
#include "log/log.h"
boost::mutex RtcDevice::operationMutex_;

RtcDevice::RtcDevice()
:fd(-1)
{
    memset(rtc_dev_name, 0, RTC_DEV_NAME_LEN);
	memcpy(rtc_dev_name, "/dev/rtc", RTC_DEV_NAME_LEN);
}
// get the device name
RtcDevice::RtcDevice(const char* dev_name = "/dev/rtc")
:fd(-1)
{
    if(dev_name == NULL){
        log_error("no rtc device name");
        return;
    }
	
    memset(rtc_dev_name, 0, RTC_DEV_NAME_LEN);
    memcpy(this->rtc_dev_name, dev_name, RTC_DEV_NAME_LEN);
}

// 析构函数
// deliberately empty
RtcDevice::~RtcDevice()
{
	// 关闭rtc设备
	if(fd > 0)
	{
		rtc_close();
	}
}

// 返回单例类实例
/*
RtcDevice* RtcDevice::get_instance() {
	static RtcDevice ins;
	return &ins;
}
*/

// get the rtc device name
const char* RtcDevice::rtc_name()
{
    return this->rtc_dev_name;
}

// open the rtc dev
int RtcDevice::rtc_open()
{
    if(strlen(rtc_dev_name) == 0){
        log_error("null device");
        return -1;
    }

	// since rtc can only be open one time
	#if 0
	if(fd > 0)
	{
		std::cout << "rtc device already opened , fd = " << fd << std::endl;
		return -1;
	}
	#endif
	// open device for reading and writing 
	fd = ::open(RTC_DEV_NAME, O_RDWR);
    if (fd < 0) {
		std::cout << "function rtc open : open rtc device failed!!!\r\n" << std::endl;
        perror("open /dev/rtc");
        //exit(errno);
        log_error("open rtc device failed!!!\r\n");
        return -1;
    }

	std:: cout << "function rtc open : open rtc device success... fd = "<< fd << std:: endl;
	log_info("open rtc device ok...\r\n");
	
    return 0;
}
// 获取rtc时间, 读rtc设备时间
// 目前不用加锁，只有一个线程读写rtc设备
int RtcDevice::read_rtc(struct rtc_time * p_rtc_ts)
{
	int iRet;
	time_t timep;
	struct tm * p;
	
	if(p_rtc_ts == NULL)
	{
		log_warn("function read_rtc : input parameter invalid!!!");
		std::cout << "function read_rtc : input parameter invalid!!!" << std::endl;
		return -1;
	}
	#if 0
	if(fd < 0)
	{
		log_warn("function read_rtc : fd less than 0!!! fd = %d\r\n",fd);
		std::cout << "function read_rtc : fd less than 0!!! fd = " << fd << std::endl;
		return -1;
	}
	#endif

	// 读取rtc时间
	iRet = ioctl(fd, RTC_RD_TIME, p_rtc_ts);
	if(0 > iRet)
	{
		std::cout << "read current RTC time failed , iRet = " << iRet <<std::endl; 
		perror("ioctl read");
		return -1;
	}

	fprintf(stderr, "RTC date/time : %04d/%02d/%02d %02d:%02d:%02d\r\n", \
		p_rtc_ts->tm_year+1900, p_rtc_ts->tm_mon+1, p_rtc_ts->tm_mday, \
		p_rtc_ts->tm_hour, p_rtc_ts->tm_min, p_rtc_ts->tm_sec);


	// os date/time , current UTC
	time(&timep);
	p = gmtime(&timep);
	fprintf(stderr, "OS date/ time (UTC) : %04d/%02d/%02d %02d:%02d:%02d\r\n", \
		p->tm_year+1900,p->tm_mon +1, p->tm_mday, \
		p->tm_hour, p->tm_min, p->tm_sec);

	// local time
	p = localtime(&timep);
	fprintf(stderr, "OS date/time(Local) : %04d/%02d/%02d %02d:%02d:%02d\r\n", \
		p->tm_year+1900,p->tm_mon +1, p->tm_mday, \
		p->tm_hour, p->tm_min, p->tm_sec);

	return 0;
}


int RtcDevice::write_rtc(const Rtc_Time & rtc_time_stamp)
{
	int iRet;
	struct rtc_time rtc_tm;
	if(rtc_time_stamp.iYear <= 0 || rtc_time_stamp.iMonth < 0 || rtc_time_stamp.iMonth > 12 || rtc_time_stamp.iDay < 1 || rtc_time_stamp.iDay > 31 || rtc_time_stamp.iHour < 0 || rtc_time_stamp.iHour > 23 || rtc_time_stamp.iMin < 0 || rtc_time_stamp.iMin > 59 || rtc_time_stamp.iSecond < 0 || rtc_time_stamp.iSecond > 59)
	{
		log_warn("function write RTC interface 4 input invalid : input parameter invalid!!!");
		//std::cout<<"function write RTC : input parameter invalid!!!" << std::endl;
		return -1;
	}
	#if 0
	if(fd < 0)
	{
		log_error("function write RTC : fd is less than 0 fd = %d",fd);
		//std::cout<<"function write RTC : fd is less than 0 fd = " << fd <<std::endl;
		return -1;
	}
	#endif
	
	rtc_tm.tm_year = rtc_time_stamp.iYear;
	rtc_tm.tm_mon = rtc_time_stamp.iMonth;
	rtc_tm.tm_mday = rtc_time_stamp.iDay;
	rtc_tm.tm_hour = rtc_time_stamp.iHour;
	rtc_tm.tm_min = rtc_time_stamp.iMin;
	rtc_tm.tm_sec = rtc_time_stamp.iSecond;

	iRet = ioctl(fd, RTC_SET_TIME, &rtc_tm);
	if(0 > iRet)
	{
		perror("write rtc : ");
		std::cout<<"function write RTC : set rtc time failed iRet = " << iRet<<std::endl;
		//rtc_close();
		//fd = -1;
		return -1;
	}
	else
	{
		std::cout<<"function write RTC : set rtc time success... iRet : "<<iRet<<std::endl;
	}
	
	return 0;
}

// 设置rtc时间
// 接口1，提供年月日时分秒
// 需提供准确UTC 时间
int RtcDevice::write_rtc(const int &iYear, const int &iMonth, const int &iDay, const int &iHour, const int &iMin, const int &iSecond)
{
	int iRet;
	struct rtc_time rtc_tm;
	if(iYear <= 0 || iMonth < 0 || iMonth > 12 || iDay < 1 || iDay > 31 || iHour < 0 || iHour > 23 || iMin < 0 || iMin > 59 || iSecond < 0 || iSecond > 59)
	{
		log_warn("function write RTC : input parameter invalid!!!");
		std::cout<<"function write RTC : input parameter invalid!!!" << std::endl;
		return -1;
	}
	#if 0
	if(fd < 0)
	{
		std::cout<<"function write RTC : fd is less than 0 fd = " << fd <<std::endl;
		return -1;
	}
	#endif
	
	rtc_tm.tm_year = iYear;
	rtc_tm.tm_mon = iMonth;
	rtc_tm.tm_mday = iDay;
	rtc_tm.tm_hour = iHour;
	rtc_tm.tm_min = iMin;
	rtc_tm.tm_sec = iSecond;

	iRet = ioctl(fd, RTC_SET_TIME, &rtc_tm);
	if(0 > iRet)
	{
		perror("write rtc : ");
		log_error("function write RTC : set rtc time failed iRet = %d",iRet);
		//rtc_close();
		//fd = -1;
		return -1;
	}
	else
	{
		//std::cout<<"function write RTC : set rtc time success... iRet : "<<iRet<<std::endl;
	}
	
	return 0;
}

// 直接传入rtc time接口 , 接口2
int RtcDevice::write_rtc(const struct rtc_time & rtc_ts)
{
	
	int iRet;
	struct rtc_time rtc_tm;
	#if 0
	if(fd < 0)
	{
		std::cout<<"function write RTC : fd is less than 0 fd = " << fd <<std::endl;
		return -1;
	}
	#endif
	
	rtc_tm.tm_year = rtc_ts.tm_year;
	rtc_tm.tm_mon = rtc_ts.tm_mon;
	rtc_tm.tm_mday = rtc_ts.tm_mday;
	rtc_tm.tm_hour = rtc_ts.tm_hour;
	rtc_tm.tm_min = rtc_ts.tm_min;
	rtc_tm.tm_sec = rtc_ts.tm_sec;

	iRet = ioctl(fd, RTC_SET_TIME, &rtc_tm);
	if(0 > iRet)
	{
		perror("write rtc interface 2: ");
		std::cout<<"function write RTC interface 2 : set rtc time failed iRet = " << iRet<<std::endl;
		//rtc_close();
		//fd = -1;
		return -1;
	}
	else
	{
		std::cout<<"function write RTC interface 2 : set rtc time success... iRet : "<<iRet<<std::endl;
	}
	
	return 0;
}

// 直接传入struct tm接口 , 接口3
int RtcDevice::write_rtc(const struct tm & s_time_stamp)
{

	int iRet;
	struct rtc_time rtc_tm;
	
	rtc_tm.tm_year = s_time_stamp.tm_year;
	rtc_tm.tm_mon = s_time_stamp.tm_mon;
	rtc_tm.tm_mday = s_time_stamp.tm_mday;
	rtc_tm.tm_hour = s_time_stamp.tm_hour;
	rtc_tm.tm_min = s_time_stamp.tm_min;
	rtc_tm.tm_sec = s_time_stamp.tm_sec;

	iRet = ioctl(fd, RTC_SET_TIME, &rtc_tm);
	if(0 > iRet)
	{
		perror("write rtc interface 2: ");
		std::cout<<"function write RTC interface 2 : set rtc time failed iRet = " << iRet<<std::endl;
		//rtc_close();
		//fd = -1;
		return -1;
	}
	else
	{
		std::cout<<"function write RTC interface 2 : set rtc time success... iRet : "<<iRet<<std::endl;
	}

	return 0;
}

// 关闭rtc设备
int RtcDevice::rtc_close()
{
    log_warn("rtc close:%s\r\n", rtc_dev_name);
	if(fd > 0)
	{
		::close(fd);
		fd = -1;
	}
	
    return 0;
}

int RtcDevice::setRtcTime(const Rtc_Time & rtc_ts)
{
	int iRet;
	struct rtc_time cur_rtc_tm;
	
	boost::unique_lock<boost::mutex> lock(operationMutex_);
	iRet = rtc_open();
	if(0 > iRet)
	{
		log_error("function : set ntp time to rtc , open rtc device failed!!!\r\n");
		std::cout << "function : set ntp time to rtc , open rtc deivice failed , iRet = " << iRet << std::endl;
		return -1;
	}
	
	iRet = write_rtc(rtc_ts);
	if(0 > iRet)
	{
		log_error("function : set ntp time to rtc , write rtc device failed!!!\r\n");
		std::cout << "function : set ntp time to rtc , write rtc deivice failed iRet = "<< iRet << std::endl;
		rtc_close();
		return -1;
	}

	// 验证rtc时间写入
	// read current rtc
	iRet = read_rtc(&cur_rtc_tm);
	if(0 > iRet)
	{
		log_error("function set ntp time to rtc , read current rtc failed iRet = %d\r\n", iRet);
		std::cout << "function set ntp time to rtc , read current rtc time failed!!! iRet = " << iRet << std::endl;
		rtc_close();
		return -1;
	}

	// 关闭rtc设备
	rtc_close();
	
	return 0;
}

