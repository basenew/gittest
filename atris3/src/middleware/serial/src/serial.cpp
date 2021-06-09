#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include "serial.h"

//! Macro for throwing an exception with a message, passing args
#define SEREAL_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in serial::SerialPort::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

serial::SerialPort::SerialPort() : fd_(-1)
{
    stream_thread_ = NULL;
}

serial::SerialPort::~SerialPort()
{
    if(port_open()) close();
}

void serial::SerialPort::open(const char * port_name, int baud_rate)
{
    if(port_open()) close();

    // Make IO non blocking. This way there are no race conditions that
    // cause blocking when a badly behaving process does a read at the same
    // time as us. Will need to switch to blocking for writes or errors
    // occur just after a replug event.
    fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);

    if(fd_ == -1)
    {
        const char *extra_msg = "";
        switch(errno)
        {
            case EACCES:
                extra_msg = "You probably don't have premission to open the port for reading and writing.";
                break;
            case ENOENT:
                extra_msg = "The requested port does not exist. Is the hokuyo connected? Was the port name misspelled?";
                break;
        }
        SEREAL_EXCEPT(serial::Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
    }

    try
    {
        struct flock fl;
        fl.l_type = F_WRLCK;
        fl.l_whence = SEEK_SET;
        fl.l_start = 0;
        fl.l_len = 0;
        fl.l_pid = getpid();

        if(fcntl(fd_, F_SETLK, &fl) != 0)
            SEREAL_EXCEPT(serial::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

        // Settings for USB?
        struct termios newtio;
        tcgetattr(fd_, &newtio);
        memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
        newtio.c_cflag = CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        newtio.c_cflag &= ~PARENB;
        newtio.c_iflag &= ~INPCK;

        cfsetspeed(&newtio, baud_rate);
        baud_ = baud_rate;

        tcflush(fd_, TCIFLUSH);
        if(tcsetattr(fd_, TCSANOW, &newtio) < 0)
            SEREAL_EXCEPT(serial::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
        usleep (200000);
    }
    catch(serial::Exception& e)
    {
        // These exceptions mean something failed on open and we should close
        if(fd_ != -1) ::close(fd_);
        fd_ = -1;
        throw e;
    }
}

void serial::SerialPort::close()
{
    int retval = 0;

    retval = ::close(fd_);

    fd_ = -1;

    if(retval != 0)
        SEREAL_EXCEPT(serial::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}

int serial::SerialPort::write(const char * data, int length)
{
    int len = length==-1 ? strlen(data) : length;

    // IO is currently non-blocking. This is what we want for the more serialon read case.
    int origflags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK); // TODO: @todo can we make this all work in non-blocking?
    int retval = ::write(fd_, data, len);
    fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);

    if(retval == len) return retval;
    else
        return -1;
        //SEREAL_EXCEPT(serial::Exception, "write failed");
}

int serial::SerialPort::read(char * buffer, int max_length, int timeout)
{
    int ret;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    if((retval = poll(ufd, 1, timeout)) < 0) SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

    if(retval == 0) SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

    if(ufd[0].revents & POLLERR)
    {
        SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
        return -1;
    }

    ret = ::read(fd_, buffer, max_length);

    if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SEREAL_EXCEPT(serial::Exception, "read failed");

    return ret;
}

int serial::SerialPort::read_bytes(char * buffer, int length, int timeout)
{
    int ret;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    if((retval = poll(ufd, 1, timeout)) < 0)
        SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

    if(retval == 0)
        SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

    if(ufd[0].revents & POLLERR){
        SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
        return -1;
    }

    ret = ::read(fd_, buffer, length);
    if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
        SEREAL_EXCEPT(serial::Exception, "read failed");

    return ret;
}

int serial::SerialPort::read_line(char * buffer, int length, int timeout)
{
    int ret;
    int current = 0;
    std::string buf;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    while(current < length-1)
    {
        if(current > 0)
            if(buffer[current-1] == '\n' || buffer[current-1] == '\r')
            {
                buffer[current] = 0;
                return current;
            }

        if((retval = poll(ufd, 1, timeout)) < 0)
            SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

        if(retval == 0)
            SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

        if(ufd[0].revents & POLLERR)
        {
            SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
            return -1;
        }

        ret = ::read(fd_, &buffer[current], length-current);

        if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
            SEREAL_EXCEPT(serial::Exception, "read failed");

        current += ret;
    }
    SEREAL_EXCEPT(serial::Exception, "buffer filled without end of line being found");
}

bool serial::SerialPort::read_line(std::string * buffer, int timeout)
{
    //int ret;
    std::string::size_type pos;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    buffer->clear();
    while(buffer->size() < buffer->max_size()/2)
    {
        // Look for the end char
        pos = buffer->find_first_of('\r'); //以回车符号作为结束符
        if(pos != std::string::npos)
        {
            // If it is there clear everything after it and return
            buffer->erase(pos +1, buffer->size()-pos -1);
            return true;
        }

        if((retval = poll(ufd, 1, timeout)) < 0) SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

        if(retval == 0) SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

        if(ufd[0].revents & POLLERR)
        {
            SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
            return false;
        }

        char temp_buffer[128];
        int ret = ::read(fd_, temp_buffer, 128);

        if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SEREAL_EXCEPT(serial::Exception, "read failed");

        // Append the new data to the buffer
        try{
            buffer->append(temp_buffer, ret);
        }
        catch(std::length_error& le)
        {
            SEREAL_EXCEPT(serial::Exception, "buffer filled without reaching end of data stream");
        }
    }
    SEREAL_EXCEPT(serial::Exception, "buffer filled without end of line being found");
}

bool serial::SerialPort::read_between_length(std::string * buffer, char start, char end, int length, int timeout)
{
    int ret;

    struct pollfd ufd[1];
    static std::string erased;
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    // Clear the buffer before we start
    buffer->clear();
    while(buffer->size() < buffer->max_size()/2)
    {
        if((retval = poll(ufd, 1, timeout)) < 0) SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

        if(retval == 0) SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

        if(ufd[0].revents & POLLERR)
        {
            SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
            return false;
        }

        // Append erased characters in last iteration
        if(!erased.empty())
        {
            try
            {
                buffer->append(erased);
                erased.clear();
            }
            catch(std::length_error& le)
            {
                SEREAL_EXCEPT(serial::Exception, "failed to append erased to buffer");
            }
        }

        char temp_buffer[3];
        ret = ::read(fd_, temp_buffer, 3);

        if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SEREAL_EXCEPT(serial::Exception, "read failed");

        // Append the new data to the buffer
        try{
            buffer->append(temp_buffer, ret);
        }
        catch(std::length_error& le)
        {
            SEREAL_EXCEPT(serial::Exception, "buffer filled without reaching end of data stream");
        }

        // Look for the start char
        ret = buffer->find_first_of(start);
        // If it is not on the buffer, clear it
        if(ret == -1) buffer->clear();
        // If it is there, but not on the first position clear everything behind it
        else if(ret > 0) buffer->erase(0, ret);

        // Look for the end char
        ret = buffer->find_first_of(end);
        //log_warn("ret:%d limit len:%d\r\n", ret, length);
        while(ret < length -1 && ret > 0){
            ret = buffer->find_first_of(end, ret+1);
            //   log_warn("continue find end:%d", ret);
        }
        if(ret == length -1)
        {
            // If it is there clear everything after it and return
            erased = buffer->substr(ret+1, buffer->size()-ret-1);
            //std::cout << "sobra |" << erased << "|\n";
            buffer->erase(ret+1, buffer->size()-ret-1);
            return true;
        }
    }
    SEREAL_EXCEPT(serial::Exception, "buffer filled without reaching end of data stream");
}


bool serial::SerialPort::read_between(std::string * buffer, char start, char end, int timeout)
{
    int ret;

    struct pollfd ufd[1];
    static std::string erased;
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

    // Clear the buffer before we start
    buffer->clear();
    while(buffer->size() < buffer->max_size()/2)
    {
        if((retval = poll(ufd, 1, timeout)) < 0) SEREAL_EXCEPT(serial::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

        if(retval == 0) SEREAL_EXCEPT(serial::TimeoutException, "timeout reached");

        if(ufd[0].revents & POLLERR)
        {
            SEREAL_EXCEPT(serial::Exception, "error on socket, possibly unplugged");
            return false;
        }

        // Append erased characters in last iteration
        if(!erased.empty())
        {
            try
            {
                buffer->append(erased);
                erased.clear();
            }
            catch(std::length_error& le)
            {
                SEREAL_EXCEPT(serial::Exception, "failed to append erased to buffer");
            }
        }

        char temp_buffer[3];
        ret = ::read(fd_, temp_buffer, 3);

        if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SEREAL_EXCEPT(serial::Exception, "read failed");

        // Append the new data to the buffer
        try{
            buffer->append(temp_buffer, ret);
        }
        catch(std::length_error& le)
        {
            SEREAL_EXCEPT(serial::Exception, "buffer filled without reaching end of data stream");
        }

        // Look for the start char
        ret = buffer->find_first_of(start);
        // If it is not on the buffer, clear it
        if(ret == -1) buffer->clear();
        // If it is there, but not on the first position clear everything behind it
        else if(ret > 0) buffer->erase(0, ret);

        // Look for the end char
        ret = buffer->find_first_of(end);
        if(ret > 0)
        {
            // If it is there clear everything after it and return
            erased = buffer->substr(ret+1, buffer->size()-ret-1);
            //std::cout << "sobra |" << erased << "|\n";
            buffer->erase(ret+1, buffer->size()-ret-1);
            return true;
        }
    }
    SEREAL_EXCEPT(serial::Exception, "buffer filled without reaching end of data stream");
}

int serial::SerialPort::flush()
{
    int retval = tcflush(fd_, TCIOFLUSH);
    if(retval != 0) SEREAL_EXCEPT(serial::Exception, "tcflush failed");

    return retval;
}

bool serial::SerialPort::start_read_stream(boost::function<void(char*, int)> f)
{
    if(stream_thread_ != NULL) return false;

    stream_stopped_ = false;
    stream_paused_ = false;

    read_callback = f;

    stream_thread_ = new boost::thread(boost::bind(&serial::SerialPort::read_thread, this));
    return true;
}

void serial::SerialPort::read_thread()
{
    char data[MAX_LENGTH];
    int ret;

    struct pollfd ufd[1];
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;

    while(!stream_stopped_)
    {
        if(!stream_paused_)
        {
            if(poll(ufd, 1, 10) > 0)
            {
                if(!(ufd[0].revents & POLLERR))
                {
                    ret = ::read(fd_, data, MAX_LENGTH);
                    if(ret>0)
                    {
                        read_callback(data, ret);
                    }
                }
            }
        }
    }
}

bool serial::SerialPort::start_read_line_stream(boost::function<void(std::string*)> f)
{
    if(stream_thread_ != NULL) return false;

    stream_stopped_ = false;
    stream_paused_ = false;

    read_line_callback = f;

    stream_thread_ = new boost::thread(boost::bind(&serial::SerialPort::read_line_thread, this));
    return true;
}

void serial::SerialPort::read_line_thread()
{
    std::string data;
    bool error = false;

    while(!stream_stopped_)
    {
        if(!stream_paused_)
        {
            error = false;
            try{ read_line(&data, 100); }
            catch(serial::Exception& e)
            {
                error = true;
            }

            if(!error && data.size()>0) read_line_callback(&data);
        }
    }
}

bool serial::SerialPort::start_read_between_stream(boost::function<void(std::string*)> f, char start, char end)
{
    if(stream_thread_ != NULL) return false;

    stream_stopped_ = false;
    stream_paused_ = false;

    read_between_callback = f;

    stream_thread_ = new boost::thread(boost::bind(&serial::SerialPort::read_between_thread, this, start, end));
    return true;
}

void serial::SerialPort::read_between_thread(char start, char end)
{
    std::string data;
    bool error = false;

    while(!stream_stopped_)
    {
        if(!stream_paused_)
        {
            error = false;
            try{ read_between(&data, start, end, 100); }
            catch(serial::Exception& e)
            {
                error = true;
            }

            if(!error && data.size()>0) read_between_callback(&data);
        }
    }
}

void serial::SerialPort::stop_stream()
{
    stream_stopped_ = true;
    stream_thread_->join();

    delete stream_thread_;
    stream_thread_ = NULL;
}

void serial::SerialPort::pause_stream()
{
    stream_paused_ = true;
}

void serial::SerialPort::resume_stream()
{
    stream_paused_ = false;
}

// EOF
