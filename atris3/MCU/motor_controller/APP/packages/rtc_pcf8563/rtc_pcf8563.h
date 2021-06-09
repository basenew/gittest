#ifndef ____LCOALPACKAGES_RTC_PCF8563_H__
#define ____LCOALPACKAGES_RTC_PCF8563_H__


#define RTC_ALARM_REG_DISABLE_MASK  (1<<7)
#define RTC_ALARM_REG_ENABLE_MASK   (~RTC_ALARM_REG_DISABLE_MASK)


int rt_hw_rtc_init(void);



#endif


