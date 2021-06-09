#pragma once

#include "date.h"
#include <iostream>
#include <list>

namespace sched
{

    class Event_Instance
    {
    public:
        int id, event_id;
        std::string name, map_name, operation_type, job_id;
        Date dtstart, dtend;
        int status;
        bool is_enabled;
    };
} // namespace sched