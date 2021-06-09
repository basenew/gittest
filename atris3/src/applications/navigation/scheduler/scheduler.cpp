#include <iostream>
#include <string>
#include <boost/date_time/gregorian/gregorian.hpp>

#include "scheduler.h"
#include "date.h"
#include "database/sqliteengine.h"
#include "log/log.h"


#define JOB_TABLE_COLUMN 4
#define JOB_TABLE "CREATE TABLE IF NOT EXISTS [job] ("     \
                  "[job_id] TEXT NOT NULL PRIMARY KEY," \
                  "[name] TEXT,"                           \
                  "[type] TEXT,"                        \
                  "[content] TEXT NOT NULL)"

#define EVENT_INSTANCE_TABLE_COLUMN 10
#define EVENT_INSTANCE_TABLE "CREATE TABLE IF NOT EXISTS [event_instance] ("    \
                             "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
                             "[event_id] INTEGER,"                              \
                             "[name] TEXT,"                                     \
                             "[map_name] TEXT,"                                 \
                             "[operation_type],"                                \
                             "[dtstart] TEXT NOT NULL,"                         \
                             "[dtend] TEXT,"                                    \
                             "[status] INTEGER,"                                \
                             "[is_enabled] INTEGER,"                            \
                             "[job_id] TEXT NOT NULL)"

#define EVENT_TABLE_COLUMN 10
#define EVENT_TABLE "CREATE TABLE IF NOT EXISTS [event] ("                   \
                    "[event_id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
                    "[name] TEXT,"                                           \
                    "[map_name] TEXT,"                                       \
                    "[operation_type],"                                \
                    "[dtstart] TEXT NOT NULL,"                               \
                    "[dtend] TEXT,"                                          \
                    "[rrule] TEXT,"                                          \
                    "[is_recurring] INTEGER,"                                \
                    "[is_enabled] INTEGER,"                                  \
                    "[job_id] TEXT NOT NULL)"

#define EVENT_YEARS_MAX 1

namespace sched
{

    typedef std::list<Date> LDATE;
    std::multimap<LDATE, std::shared_ptr<Task>> tasks;

    int Scheduler::init()
    {
        SqliteEngine::execSQL(EVENT_TABLE);
        SqliteEngine::execSQL(EVENT_INSTANCE_TABLE);
        SqliteEngine::execSQL(JOB_TABLE);
        return 0;
    }

    void Scheduler::add_task(const Clock::time_point time, std::shared_ptr<Task> t)
    {
        std::lock_guard<std::mutex> l(lock);
        tasks.emplace(time, std::move(t));
        sleeper.interrupt();
    }

    void Scheduler::manage_tasks()
    {
        std::lock_guard<std::mutex> l(lock);

        auto end_of_tasks_to_run = tasks.upper_bound(Clock::now());

        // if there are any tasks to be run and removed
        if (end_of_tasks_to_run != tasks.begin())
        {
            // keep track of tasks that will be re-added
            decltype(tasks) recurred_tasks;

            // for all tasks that have been triggered
            for (auto i = tasks.begin(); i != end_of_tasks_to_run; ++i)
            {

                auto &task = (*i).second;

                if (task->interval)
                {
                    // if it's an interval task, only add the task back after f() is completed
                    threads.push([this, task](int) {
                        task->f();
                        // no risk of race-condition,
                        // add_task() will wait for manage_tasks() to release lock
                        add_task(task->get_new_time(), task);
                    });
                }
                else
                {
                    threads.push([task](int) {
                        task->f();
                    });
                    // calculate time of next run and add the new task to the tasks to be recurred
                    if (task->recur)
                        recurred_tasks.emplace(task->get_new_time(), std::move(task));
                }
            }

            // remove the completed tasks
            tasks.erase(tasks.begin(), end_of_tasks_to_run);

            // re-add the tasks that are recurring
            for (auto &task : recurred_tasks)
                tasks.emplace(task.first, std::move(task.second));
        }
    }

    int Scheduler::add_event(Event &event)
    {
        int ret = 0;
        std::string name = event.name;
        std::string map_name = event.map_name;
        std::string operation_type = event.operation_type;

        Date datetime_start = event.dtstart;
        std::string dtstart = datetime_start;

        Date datetime_end = event.dtend;
        std::string dtend = datetime_end.Format();

        Recurrence myrrule = event.rrule;
        std::string rrule = myrrule;

        std::string is_recurring = std::to_string(event.is_recurring ? 1 : 0);
        std::string is_enabled = std::to_string(event.is_enabled ? 1 : 0);

        std::string job_id = event.job_id;

        SqliteEngine::execSQL("INSERT INTO event(name, map_name, operation_type, dtstart, dtend, \
                            rrule, is_recurring, is_enabled, job_id) VALUES('"
                            + name + "', '"
                            + map_name + "', '"
                            + operation_type + "', '"
                            + dtstart + "', '"
                            + dtend + "', '"
                            + rrule + "', "
                            + is_recurring + ","
                            + is_enabled + ", '"
                            + job_id + "')");

        char **result = nullptr;
        int row = 0;
        int column = 0;
        int sqlret = SqliteEngine::query("select seq from sqlite_sequence where name = 'event'",
                                         &result, &row, &column);

        if (sqlret != SQLITE_OK)
        {
            log_error("sqlite query err!");
            SqliteEngine::freeQuery(result);
            return -1;
        }

        int last_insert_id = atoi(result[row * column]);
        SqliteEngine::freeQuery(result);

        // store event_instance
        LDATE datelist = event.get_all();
        for (auto &date : datelist)
        {
            Event_Instance event_instance;
            event_instance.event_id = last_insert_id;
            event_instance.name = event.name;
            event_instance.map_name = map_name;
            event_instance.operation_type = operation_type;
            event_instance.status = TODO;
            event_instance.dtstart = date;
            event_instance.dtend = event.dtend;
            event_instance.is_enabled = event.is_enabled;
            event_instance.job_id = event.job_id;
            ret = add_event_instance(event_instance);
            if (ret != 0)
            {
                return ret;
            }
        }
        return 0;
    }

    int Scheduler::del_event(int event_id)
    {
        if (event_id < 0) {
            log_error( "param err!");
            return -1;
        }

        std::stringstream ss;
        ss << event_id;
        int sqlret = SqliteEngine::execSQL("DELETE FROM event WHERE event_id='"+ss.str()+"'");

        if (sqlret != SQLITE_OK) {
            return -2;
        }

        return 0;
    }

    int Scheduler::del_event(std::string event_name)
    {
        if (event_name.empty()) {
            log_error( "param err!");
            return -1;
        }

        int sqlret = SqliteEngine::execSQL("DELETE FROM event WHERE name='"+event_name+"'");

        if (sqlret != SQLITE_OK) {
            return -2;
        }

        return 0;
    }

    //TODO
    int Scheduler::query_event(int event_id, Event &event)
    {
        return 0;
    }

    //TODO
    int Scheduler::update_event(Event &event)
    {
        return 0;
    }

    int Scheduler::add_event_instance(const Event_Instance &event_instance)
    {
        SqliteEngine::execSQL("INSERT INTO event_instance(event_id, name, map_name, operation_type,\
                                dtstart, dtend, status, is_enabled, job_id) VALUES("
                    +std::to_string(event_instance.event_id)+", '"
                    +event_instance.name+"', '"
                    +event_instance.map_name+"', '"
                    +event_instance.operation_type+"', '"
                    +std::string(event_instance.dtstart)+"', '"
                    +std::string(event_instance.dtend)+"', "
                    +std::to_string(event_instance.status)+", "
                    +std::to_string(event_instance.is_enabled)+", '"
                    +event_instance.job_id+"')");
        return 0;
    }

    int Scheduler::del_event_instance(int id)
    {
        if (id < 0) {
            log_error( "param err!");
            return -1;
        }

        std::stringstream ss;
        ss << id;
        int sqlret = SqliteEngine::execSQL("DELETE FROM event_instance WHERE id='"+ss.str()+"'");

        if (sqlret != SQLITE_OK) {
            return -2;
        }

        return 0;
    }

    int Scheduler::del_event_instance(std::string name)
    {
        if (name.empty()) {
            log_error( "param err!");
            return -1;
        }

        int sqlret = SqliteEngine::execSQL("DELETE FROM event_instance WHERE name='"+name+"'");

        if (sqlret != SQLITE_OK) {
            return -2;
        }

        return 0;
    }

    int Scheduler::query_event_instance_base(const std::string query_conditions,
                                             std::list<Event_Instance> &lev)
    {
        char **result = nullptr;
        int row = 0;
        int column = 0;

        int sqlret = SqliteEngine::query(query_conditions, &result, &row, &column);

        if (sqlret != SQLITE_OK)
        {
            log_error("sqlite query err!");
            return -1;
        }

        if (row <= 0)
        {
            SqliteEngine::freeQuery(result);
            log_error("query db is empty");
            return -2;
        }

        Event_Instance event_instance;

        int index = column;
        for (int i = 1; i <= row; i++)
        {
            event_instance.id = atoi(result[i * column]);
            event_instance.event_id = atoi(result[i * column + 1]);
            event_instance.name = result[i * column + 2];
            event_instance.map_name = result[i * column + 3];
            event_instance.operation_type = result[i * column + 4];
            event_instance.dtstart = result[i * column + 5];
            event_instance.dtend = result[i * column + 6];
            event_instance.status = atoi(result[i * column + 7]);
            event_instance.is_enabled = atoi(result[i * column + 8]);
            event_instance.job_id = result[i * column + 9];
            lev.push_back(event_instance);
        }

        SqliteEngine::freeQuery(result);
        return 0;
    }

    int Scheduler::query_event_instance(Date startdate, Date enddate,
                                        std::list<Event_Instance> &lev)
    {
        if (startdate.IsEmpty() || enddate.IsEmpty())
        {
            log_error("startdate or endate is empty");
            return -1;
        }

        if (enddate < startdate)
        {
            log_error( " endate < startdate ");
            return -2;
        }

        char **result = nullptr;
        int row = 0;
        int column = 0;
        std::string dtstart = startdate, dtend = enddate;
        std::string query_conditions = "SELECT * FROM event_instance WHERE dtstart >='" \
                                        + dtstart + "' AND dtstart <='" + dtend + "' ";

        int ret = query_event_instance_base(query_conditions, lev);
        return ret;
    }

    int Scheduler::query_event_instance_before(Date beforedate,
                                               std::list<Event_Instance> &lev)
    {
        if (beforedate.IsEmpty())
        {
            log_error("beforedate is empty");
            return -1;
        }

        char **result = nullptr;
        int row = 0;
        int column = 0;
        std::string dtstart = beforedate;
        std::string query_conditions = "SELECT * FROM event_instance WHERE dtstart <='" + dtstart + "'";

        int ret = query_event_instance_base(query_conditions, lev);
        return ret;
    }

    int Scheduler::query_event_instance_after(Date afterdate,
                                              std::list<Event_Instance> &lev)
    {
        if (afterdate.IsEmpty())
        {
            log_error("afterdate is empty");
            return -1;
        }

        char **result = nullptr;
        int row = 0;
        int column = 0;
        std::string dtstart = afterdate;
        std::string query_conditions = "SELECT * FROM event_instance WHERE dtstart >='" + dtstart + "'";

        int ret = query_event_instance_base(query_conditions, lev);
        return ret;
    }

    int Scheduler::update_event_instance(int id, std::string column_name, std::string value)
    {
        if (id < 0 || column_name.empty() || value.empty())
        {
            log_error("param err!");
            return -1;
        }

        std::stringstream ss;
        ss << id;
        SqliteEngine::execSQL("UPDATE event_instance SET '" \
                                + column_name + "' = '" + value + "' WHERE id == " + ss.str());
    }

    int Scheduler::update_event_instance(Date startdate, std::string column_name, std::string value)
    {
        log_info("%s", __FUNCTION__);
        if (startdate.IsEmpty() || column_name.empty() || value.empty()) {
            log_error("param err!");
            return -1;
        }

        std::string dtstart = startdate;
        log_info("%s, %s", __FUNCTION__, dtstart.c_str());
        SqliteEngine::execSQL("UPDATE event_instance SET '" \
                                + column_name + "' = '" + value + "' WHERE dtstart == '" + dtstart + "'");
        return 0;
    }

    int Scheduler::add_job(Job &job)
    {
        SqliteEngine::execSQL("INSERT INTO job(job_id, name, type, content) VALUES('"
                                + job.job_id + "', '"
                                + job.job_name + "', '"
                                + job.type + "','"
                                + job.content + "')");
        return 0;
    }

    int Scheduler::del_job(std::string job_id)
    {
        if (job_id.empty()) {
            log_error( "param err!");
            return -1;
        }

        int sqlret = SqliteEngine::execSQL("DELETE FROM job WHERE job_id='" + job_id + "'");

        if (sqlret != SQLITE_OK) {
            return -2;
        }
       
        return 0;
    }
    
    int Scheduler::query_job(std::string job_id, Job &job)
    {
        log_info("%s", __FUNCTION__);
        char **result = nullptr;
        int row = 0;
        int column = 0;
        std::string query_conditions = "SELECT * FROM job WHERE job_id =='" + job_id + "'";
        int sqlret = SqliteEngine::query(query_conditions, &result, &row, &column);

        if (sqlret != SQLITE_OK)
        {
            log_error("sqlite query err!");
            return -1;
        }

        if (row <= 0)
        {
            SqliteEngine::freeQuery(result);
            log_error("query db is empty");
            return -2;
        }

        int index = column;
        job.job_id = result[column];
        job.job_name = result[column + 1];
        job.type = result[column + 2];
        job.content = result[column + 3];

        SqliteEngine::freeQuery(result);
        return 0;
    }

    //TODO
    int Scheduler::update_job(Job &job)
    {
        return 0;
    }

} // namespace sched