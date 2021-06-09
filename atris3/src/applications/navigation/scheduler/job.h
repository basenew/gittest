#pragma once

#include <string>
#include <list>

namespace sched
{
    class Job
    {
    public:
        std::string job_name, content, type, job_id;

        Job &operator=(const Job &rhs)
        {
            if (this == &rhs)
            {
                return *this;
            }
            job_id = rhs.job_id;
            job_name = rhs.job_name;
            content = rhs.content;
            type = rhs.type;
            return *this;
        }
    };
} // namespace sched
