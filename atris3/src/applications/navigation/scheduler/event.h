#pragma once

#include "date.h"
#include "event_instance.h"
#include <list>
#include <iostream>
#include <stdio.h>
#include <string.h>

#define EVENT_YEARS_MAX 1

namespace sched
{
    typedef enum
    {
        TODO = 0,
        FINISH = 1,
        OVERDUE = 2,
        TERMINATION = 3,
        PAUSE = 4,
        RUNNING = 5
    } EventStatus;

    typedef std::list<Date> LDATE;

    inline TimeUnit ConvertFrequency(std::string Name)
    {
        if (Name == "SECONDLY")
            return SECOND;
        if (Name == "MINUTELY")
            return MINUTE;
        if (Name == "HOURLY")
            return HOUR;
        if (Name == "DAILY")
            return DAY;
        if (Name == "WEEKLY")
            return WEEK;
        if (Name == "MONTHLY")
            return MONTH;
        return YEAR;
    }

    struct Recurrence
    {

        Recurrence() : freq(YEAR), interval(0), count(0),
                       bymonth(0), byweekday(0), bymonthday(0),
                       byhour(-1), byminute(-1), bysecond(-1) {}
        operator std::string() const;
        bool isEmpty() const { return (interval == 0); }
        void clear()
        {
            freq = YEAR;
            interval = 0;
            count = 0;
            interval = 0;
            bymonth = -1;
            byweekday = -1;
            bymonthday = -1;
            byhour = -1;
            byminute = -1;
            bysecond = -1;
        }

        TimeUnit freq;
        unsigned short interval, count;
        short bymonth, byweekday, bymonthday, byhour, byminute, bysecond;

        Date until;
    };

    class Event
    {
    public:
        Event() : is_enabled(true), is_recurring(false), base_event(this){};
        Event(const Event &Base) : event_id(Base.event_id),
                                   name(Base.name),
                                   map_name(Base.map_name),
                                   operation_type(Base.operation_type),
                                   dtstart(Base.dtstart),
                                   dtend(Base.dtend),
                                   rrule(Base.rrule),
                                   is_recurring(Base.is_recurring),
                                   is_enabled(Base.is_enabled),
                                   job_id(Base.job_id)
        {
            base_event = Base.base_event == (Event *)&Base ? (Event *)&Base : Base.base_event;
        }
        ~Event(){};
        operator std::string() const;
        int event_id;
        std::string name, map_name, operation_type, job_id;
        Date dtstart, dtend;
        Recurrence rrule;
        bool is_recurring, is_enabled;
        Event *base_event;
        Event_Instance ev;

        LDATE get_all();
    };



} // namespace sched