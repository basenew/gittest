#ifndef ATRIS_DEVICE_TELECOM_H_
#define ATRIS_DEVICE_TELECOM_H_

#include "string"
#include <boost/thread.hpp>

class Telecom {
public:
    virtual ~Telecom();
    static Telecom* get_instance()
    {
        static Telecom singleton;
        return &singleton;
    }
    
private:
    Telecom();
    void RegisterThread();
    int  GetRegisterCotent(std::string &content);
    std::string timestampToDate(time_t time);

private:
    boost::thread* register_thread_;
};



#endif