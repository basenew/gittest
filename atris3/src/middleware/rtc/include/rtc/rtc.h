#ifndef __RTC_H__
#define __RTC_H__

// created by jinzhongxi
// 设置imx6板子的rtc时间
// 传入的时间(设置的时间应该是utc时间)
// 这样系统
#include <string.h>
#include <stdlib.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <boost/thread.hpp>

#define RTC_DEV_NAME    "/dev/rtc"
#define RTC_DEV_NAME_LEN    32

// rtc时间结构体
typedef struct __Rtc_Time_Stru
{
	int iYear;
	int iMonth;
	int iDay;
	int iHour;
	int iMin;
	int iSecond;
}Rtc_Time;
// 提供 读写rtc时间的接口
class RtcDevice
{
	public:
        RtcDevice();
        RtcDevice(const char* dev_name);
		~RtcDevice();
        const char* rtc_name();  //rtc设备名称
        int rtc_open();
        //int read_rtc();
        // 读取rtc时间
        int read_rtc(struct rtc_time * p_rtc_ts);
        //int write_rtc();
        // 写rtc时间，接口1，年月日时分秒
		int write_rtc(const int &iYear, const int &iMonth, const int &iDay, const int &iHour, const int &iMin, const int &iSecond);
		// 写rtc时间，接口2, rtc_time
		int write_rtc(const struct rtc_time & rtc_ts);
		// 写入rtc时间, 接口3 , tm
		int write_rtc(const struct tm & s_time_stamp);
		int write_rtc(const Rtc_Time & rtc_time_stamp);
		int setRtcTime(const Rtc_Time & rtc_ts);
		int rtc_close();

static RtcDevice* get_instance() {
	static RtcDevice ins;
	return &ins;
}

    private:
        int fd; // 文件描述符
        char rtc_dev_name[RTC_DEV_NAME_LEN]; // RTC设备名称
        static boost::mutex operationMutex_;
        // currently do not need this
        //static boost::mutex mutex_;
    	//static RtcDevice * RtcDev_;

};



#endif
