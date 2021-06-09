#pragma once

#include "event.h"

namespace sched
{

    std::ostream &operator<<(std::ostream &stream, const Recurrence &RRule)
    {
        stream << RRule.operator std::string();
        return stream;
    }

    Recurrence::operator std::string() const
    {
        char Temp[5];
        std::string Text = "FREQ=";
        switch (freq)
        {
            case YEAR: Text += "YEARLY"; break;
            case MONTH: Text += "MONTHLY"; break;
            case DAY: Text += "DAILY"; break;
            case HOUR: Text += "HOURLY"; break;
            case MINUTE: Text += "MINUTELY"; break;
            case SECOND: Text += "SECONDLY"; break;
            case WEEK: Text += "WEEKLY"; break;
        }

        if (count > 0)
        {
            Text += ";COUNT="; sprintf(Temp, "%d", count); Text += Temp;
            memset(Temp, 0, 5);
        }

        if (byweekday > 0)
        {
            Text += ";BYDAY="; sprintf(Temp, "%d", byweekday); Text += Temp;
            memset(Temp, 0, 5);
        }

        if (bymonth > 0)
        {
            Text += ";BYMONTHDAY="; sprintf(Temp, "%d", byweekday); Text += Temp;
            memset(Temp, 0, 5);
        }

        if (byhour > 0)
        {
            Text += ";BYHOUR="; sprintf(Temp, "%d", byhour); Text += Temp;
            memset(Temp, 0, 5);
        }

        if (byminute > 0)
        {
            Text += ";BYMINUTE="; sprintf(Temp, "%d", byminute); Text += Temp;
            memset(Temp, 0, 5);
        }

        if (bysecond > 0)
        {
            Text += ";BYSECOND="; sprintf(Temp, "%d", bysecond); Text += Temp;
            memset(Temp, 0, 5);
        }

        Text += ";INTERVAL=";
        sprintf(Temp, "%d", interval);
        Text += Temp;
        memset(Temp, 0, 5);

        if (!until.IsEmpty())
        {
            Text += ";UNTIL=";
            Text += until;
            Text += "Z";
        }

        return Text;
    }

    LDATE Event::get_all()
    {
        LDATE datelist;
        Date start_time = dtstart;
        Date endtime;
        unsigned int interval = rrule.interval + 1;
/*
        if (start_time.IsEmpty())
        {
            start_time.SetToNow();
        }
*/
        if (rrule.byhour >= 0 || rrule.byminute >= 0 || rrule.bysecond >= 0)
        {

            char Temp[10];
            short hour = rrule.byhour < 0 ? 0 : rrule.byhour;
            short minute = rrule.byminute < 0 ? 0 : rrule.byminute;
            short second = rrule.bysecond < 0 ? 0 : rrule.bysecond;

            sprintf(Temp, "T%.2d%.2d%.2d", hour, minute, second);
            start_time.WithTime = false;
            std::string tmptime = std::string(start_time) + std::string(Temp);
            start_time.WithTime = true;
            Date exec_time;
            exec_time = tmptime;

            if (exec_time >= start_time)
            {
                start_time = exec_time;
            }
            else
            {
                exec_time[DAY] += 1;
                start_time = exec_time;
            }
        }

        //consider not recurring event
        if (!is_recurring)
        {
            datelist.push_back(start_time);
            return datelist;
        }

        if (rrule.until.IsEmpty() || rrule.until < start_time)
        {
            Date temp = start_time;
            temp[YEAR] += EVENT_YEARS_MAX;
            endtime = dtend.IsEmpty() ? temp : dtend;
        }
        else
        {
            endtime = rrule.until;
        }

        if (rrule.freq == WEEK && rrule.byweekday > 0)
        {
            if (start_time.GetWeekDay() < rrule.byweekday)
            {
                start_time[DAY] += rrule.byweekday - start_time.GetWeekDay();
            }
            else if (start_time.GetWeekDay() > rrule.byweekday)
            {
                start_time[DAY] += (7 - start_time.GetWeekDay() + rrule.byweekday);
            }
        }

        if (rrule.freq == MONTH && rrule.bymonthday > 0)
        {
            if (start_time.DateInDay() <= rrule.bymonthday)
            {
                start_time[DAY] += (rrule.bymonthday - start_time.DateInDay());
                if (start_time.DateInDay() > start_time.DaysInMonth())
                {
                    start_time[DAY] = start_time.DaysInMonth();
                }
            }
            else
            {
                start_time[MONTH] += 1;
            }
        }

        int count = 0;
        while (start_time <= endtime)
        {
            count++;
            datelist.push_back(start_time);
            start_time[rrule.freq] += interval;

            if (rrule.freq == MONTH && rrule.bymonthday > 0)
            {
                start_time[DAY] = rrule.bymonthday;
                if (start_time.DateInDay() > start_time.DaysInMonth())
                {
                    start_time[DAY] = start_time.DaysInMonth();
                }
            }

            if (rrule.count > 0)
            {
                if (count >= rrule.count)
                    break;
            }
        }

        std::cout << "[ datalist szie :] " << datelist.size() << std::endl;
        return datelist;
    }

} // namespace sched