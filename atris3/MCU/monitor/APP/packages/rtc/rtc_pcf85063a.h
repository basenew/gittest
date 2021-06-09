#ifndef ____LCOALPACKAGES_RTC_PCF85063A_H__
#define ____LCOALPACKAGES_RTC_PCF85063A_H__



int rt_hw_rtc_init(void);

rt_err_t hw_rtc_check_os(void);
rt_err_t hw_rtc_shutdown_clkout(void);

#endif


