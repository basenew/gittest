#include <stdarg.h>
#include <limits.h>
#include <stdio.h>
#include <string>
#include "log.h"
#include "tiny_ros/ros.h"

#define LOG_MAX_SIZE 5120

namespace atrislog {
void mtrace(int level, const char *chfr, ...) {
  char buffer[LOG_MAX_SIZE] = {0};
  va_list ap;
  va_start(ap, chfr);
  vsnprintf(buffer, sizeof(buffer) - 1, chfr, ap);
  va_end(ap);
  if(level == ros::console::levels::Debug) tinyros::logdebug(buffer);
  else if(level == ros::console::levels::Info) tinyros::loginfo(buffer);
  else if(level == ros::console::levels::Warn) tinyros::logwarn(buffer);
  else if(level == ros::console::levels::Error) tinyros::logerror(buffer);
  else if(level == -1) tinyros::logdiag(buffer);
}
}

