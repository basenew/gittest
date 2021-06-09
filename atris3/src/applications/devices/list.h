#ifndef __LIST_H__
#define __LIST_H__

#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

template <typename T>

class MsgList {
    public:
        void add(const T& msg)
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            queue.push_back(msg);
            cond.notify_one();
        }
        T get()
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            while (queue.empty()){
                cond.wait(lock);
            }
            T msg = queue.front();
            queue.pop_front();

            return msg;
        }
        bool get_noblock(T& msg)
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            if (!queue.empty())
            {
                msg = queue.front();
                queue.pop_front();
                return true;
            }
            else
            {
                return false;
            }
        }
        int size()
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            return queue.size();
        }
        bool is_empty()
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            return queue.empty();
        }
        void clear()
        {
            boost::unique_lock<boost::mutex> lock(mutex);

            queue.clear();
        }

        T at(int n)
        {
            return queue.at(n);
        }
    private:
        std::deque<T> queue;
        boost::mutex mutex;
        boost::condition_variable cond;
};









#endif
