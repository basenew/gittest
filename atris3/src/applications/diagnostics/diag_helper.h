#ifndef DIAG_HELPER_H__
#define DIAG_HELPER_H__
#include "diagnostics.h"
#include "log/log.h"
#include <iostream>
int get_warn_level_by_content(std::string event_id);
int get_warn_type_by_content(std::string event_id);
std::string get_event_id_by_content(std::string event_content);
std::string get_event_content_by_var_index(int var_index);
int get_alarm_type_by_content(std::string event_id);
#endif