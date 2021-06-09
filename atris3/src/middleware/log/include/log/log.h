#ifndef _LOG_H_
#define _LOG_H_
#include "ros/ros.h"

namespace atrislog
{
void mtrace(int level, const char *chfr, ...);

#define log_info(format, ...) atrislog::mtrace(ros::console::levels::Info, format, ##__VA_ARGS__)
#define log_warn(format, ...) atrislog::mtrace(ros::console::levels::Warn, format, ##__VA_ARGS__)
#define log_error(format, ...) atrislog::mtrace(ros::console::levels::Error, format, ##__VA_ARGS__)
#define log_debug(format, ...) atrislog::mtrace(ros::console::levels::Debug, format, ##__VA_ARGS__)
#define log_diag(format, ...) atrislog::mtrace(-1, format, ##__VA_ARGS__)


#define LOG_ONCE_TIME_THROTTLE  1
#define log_once_info(format, ...) { \
  static double __once_time = -1.0f; \
  if(__once_time < 0.0f) \
    __once_time = ros::Time::now().toSec(); \
  double __once_tem_sec =(ros::Time::now().toSec() - __once_time); \
  if((__once_tem_sec > LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = ros::Time::now().toSec(); \
    atrislog::mtrace(ros::console::levels::Info, format, ##__VA_ARGS__); \
  } \
}
#define log_once_warn(format, ...) { \
  static double __once_time = -1.0f; \
  if(__once_time < 0.0f) \
    __once_time = ros::Time::now().toSec(); \
  double __once_tem_sec =(ros::Time::now().toSec() - __once_time); \
  if((__once_tem_sec > LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = ros::Time::now().toSec(); \
    atrislog::mtrace(ros::console::levels::Warn, format, ##__VA_ARGS__); \
  } \
}
#define log_once_error(format, ...) { \
  static double __once_time = -1.0f; \
  if(__once_time < 0.0f) \
    __once_time = ros::Time::now().toSec(); \
  double __once_tem_sec =(ros::Time::now().toSec() - __once_time); \
  if((__once_tem_sec > LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = ros::Time::now().toSec(); \
    atrislog::mtrace(ros::console::levels::Error, format, ##__VA_ARGS__); \
  } \
}
#define log_once_debug(format, ...) { \
  static double __once_time = -1.0f; \
  if(__once_time < 0.0f) \
    __once_time = ros::Time::now().toSec(); \
  double __once_tem_sec =(ros::Time::now().toSec() - __once_time); \
  if((__once_tem_sec > LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = ros::Time::now().toSec(); \
    atrislog::mtrace(ros::console::levels::Debug, format, ##__VA_ARGS__); \
  } \
}
}  // namespace atrislog

#endif


