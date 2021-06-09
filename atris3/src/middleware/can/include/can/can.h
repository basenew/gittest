#ifndef __CAN_H__
#define __CAN_H__

#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_pkg.h"

#define DEV_NAME    "can1"
#define CAN_DEV_NAME_LEN    32

class CanDevice
{
    private:
        int dev;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct can_frame frame;
        char dev_name[CAN_DEV_NAME_LEN];
    public:
        CanDevice(const char* dev_name);
        const char* can_name();
        int can_open();
        int can_read(CanPkg *pkg);
        int can_read(CanPkg *pkg, int timeout);
        int can_write(CanPkg* pkg, int size);
        int can_close();
        int set_filter(struct can_filter *filters, int len);
};



#endif
