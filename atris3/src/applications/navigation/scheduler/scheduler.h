
#pragma once

#include <string>
#include <list>
#include <iomanip>
#include <map>
#include <functional>
#include <thread>

#include "ros/ros.h"
#include "log/log.h"

#include "date.h"
#include "ctpl_stl.h"
#include "InterruptableSleep.h"
#include "Cron.h"

#include "event.h"
#include "job.h"

namespace sched
{

	class Task
	{
	public:
		explicit Task(std::function<void()> &&f, bool recur = false, bool interval = false) : f(std::move(f)), recur(recur), interval(interval) {}

		virtual Clock::time_point get_new_time() const = 0;

		std::function<void()> f;

		bool recur;
		bool interval;
	};

	class InTask : public Task
	{
	public:
		explicit InTask(std::function<void()> &&f) : Task(std::move(f)) {}

		// dummy time_point because it's not used
		Clock::time_point get_new_time() const override { return Clock::time_point(Clock::duration(0)); }
	};

	class EveryTask : public Task
	{
	public:
		EveryTask(Clock::duration time, std::function<void()> &&f, bool interval = false) : Task(std::move(f), true, interval), time(time) {}

		Clock::time_point get_new_time() const override
		{
			return Clock::now() + time;
		};
		Clock::duration time;
	};

	class CronTask : public Task
	{
	public:
		CronTask(const std::string &expression, std::function<void()> &&f) : Task(std::move(f), true),
																			 cron(expression) {}

		Clock::time_point get_new_time() const override
		{
			return cron.cron_to_next();
		};
		Cron cron;
	};

	inline bool try_parse(std::tm &tm, const std::string &expression, const std::string &format)
	{
		std::stringstream ss(expression);
		return !(ss >> std::get_time(&tm, format.c_str())).fail();
	}

	class Scheduler
	{
	public:
		explicit Scheduler(unsigned int max_n_tasks = 200) : threads(max_n_tasks + 1)
		{
			init();
			threads.push([this](int) {
				while (1)
				{
					if (tasks.empty())
					{
						sleeper.sleep();
					}
					else
					{
						auto time_of_first_task = (*tasks.begin()).first;
						sleeper.sleep_until(time_of_first_task);
					}
					manage_tasks();
				}
			});
		}
		~Scheduler()
		{
			sleeper.interrupt();
		}

		int init();

		Scheduler(const Scheduler &) = delete;
		Scheduler(Scheduler &&) noexcept = delete;
		Scheduler &operator=(const Scheduler &) = delete;
		Scheduler &operator=(Scheduler &&) noexcept = delete;

		template <typename _Callable, typename... _Args>
		void in(const Clock::time_point time, _Callable &&f, _Args &&... args)
		{
			std::shared_ptr<Task> t = std::make_shared<InTask>(
				std::bind(std::forward<_Callable>(f), std::forward<_Args>(args)...));
			add_task(time, std::move(t));
		}

		template <typename _Callable, typename... _Args>
		void in(const Clock::duration time, _Callable &&f, _Args &&... args)
		{
			in(Clock::now() + time, std::forward<_Callable>(f), std::forward<_Args>(args)...);
		}

		template <typename _Callable, typename... _Args>
		int at(const std::string &time, _Callable &&f, _Args &&... args)
		{
			// get current time as a tm object
			auto time_now = Clock::to_time_t(Clock::now());
			std::tm tm = *std::localtime(&time_now);

			// our final time as a time_point
			Clock::time_point tp;

			if (try_parse(tm, time, "%H:%M:%S"))
			{
				// convert tm back to time_t, then to a time_point and assign to final
				tp = Clock::from_time_t(std::mktime(&tm));
			}
			else if (try_parse(tm, time, "%Y-%m-%d %H:%M:%S"))
			{
				tp = Clock::from_time_t(std::mktime(&tm));
			}
			else if (try_parse(tm, time, "%Y/%m/%d %H:%M:%S"))
			{
				tp = Clock::from_time_t(std::mktime(&tm));
			}
			else
			{
				// could not parse time
				log_error("cannot parse time");
				return -1;
			}

			if (Clock::now() >= tp)
			{
				log_error("already passed this time");
				return -2;
			}
			in(tp, std::forward<_Callable>(f), std::forward<_Args>(args)...);
		}

		void stop(bool isWait = false)
		{
			threads.stop(isWait);
		}

		int add_event(Event &event);
		int del_event(int event_id);
		int del_event(std::string event_name);

		int query_event(int event_id, Event &event);
		int update_event(Event &event);

		int add_event_instance(const Event_Instance &event_instance);
		int del_event_instance(int id);
		int del_event_instance(std::string name);
		int query_event_instance_base(const std::string query_conditions,
									  std::list<Event_Instance> &lev);
		int query_event_instance(Date startdate, Date enddate,
								 std::list<Event_Instance> &lev);
		int query_event_instance_before(Date beforedate,
										std::list<Event_Instance> &lev);
		int query_event_instance_after(Date afterdate,
									   std::list<Event_Instance> &lev);
		int update_event_instance(int id, std::string column_name, std::string value);
        int update_event_instance(Date startdate, std::string column_name, std::string value);

		int add_job(Job &job);
		int del_job(std::string job_id);
		int query_job(std::string job_id, Job &job);
		int update_job(Job &job);

	private:
		InterruptableSleep sleeper;

		std::multimap<Clock::time_point, std::shared_ptr<Task>> tasks;
		std::mutex lock;
		thread_pool threads;
		void add_task(const Clock::time_point time, std::shared_ptr<Task> t);
		void manage_tasks();
	};

} // namespace sched
