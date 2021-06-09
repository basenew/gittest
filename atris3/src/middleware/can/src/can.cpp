#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <string>

#include "can.h"

CanDevice::CanDevice(const char* dev_name)
{
    if(dev_name == NULL){
        log_error("null can device");
        return ;
    }
    memcpy(this->dev_name, dev_name, CAN_DEV_NAME_LEN);
}

const char* CanDevice::can_name()
{
    return this->dev_name;
}

int CanDevice::can_open()
{
    if(strlen(dev_name) == 0){
        log_error("null device");
        return -1;
    }

    dev = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(dev < 0){
        log_error("create can dev fail.");
        return -1;
    }

    strcpy(ifr.ifr_name, dev_name);
    ioctl(dev, SIOCGIFINDEX, &ifr); /* assign can device */
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(dev, (struct sockaddr *)&addr, sizeof(addr)) < 0){ /* bind can device */
        log_error("bind can device fail");
        close(dev);
        return -1;
    }

    int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
    setsockopt(dev, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    log_warn("open can device:%s", dev_name);
    int ro = 0; // 0 表示关闭( 默认), 1 表示开启
    setsockopt(dev, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &ro, sizeof(ro));

    //set can tx queue buffer length;
    char path[128] = {0};
    sprintf(path, "/sys/class/net/%s/tx_queue_len", dev_name);

    int fd = open(path, O_RDWR);
    char val[64] = {0};
    sprintf(val, "%s", "1000");
    write(fd, val, strlen(val));
    close(fd);

    return 0;
}


int CanDevice::can_read(CanPkg *pkg)
{
    memset(pkg, 0, sizeof(CanPkg));
    int i;
    int byte = read(dev, &frame, sizeof(frame));
    if(byte > 0){
        pkg->head.h32 = 0;
        pkg->head.h32 = frame.can_id;
        for(i = 0; i < frame.can_dlc; i ++){
            pkg->data[i] = frame.data[i];
        }

        pkg->head.id.size = frame.can_dlc-1;
        //dump_can_pkg(pkg);
    }

    return byte;
}

int CanDevice::can_read(CanPkg* pkg, int timeout)
{
    memset(pkg, 0, sizeof(CanPkg));
    int i;

    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = dev;
    ufd[0].events = POLLIN;

    if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
    if((retval = poll(ufd, 1, timeout)) < 0){
        log_error("poll failed:%s",strerror(errno));
    }
    if(retval == 0){
        log_warn("read timeout reached.");
        return 0;
    }

    if(ufd[0].revents & POLLERR)
    {
        log_error("error on socket, possibly unplugged");
        return -1;
    }

    int byte = ::read(dev, &frame, sizeof(frame));
    if(byte > 0){
        pkg->head.h32 = 0;
        pkg->head.h32 = frame.can_id;
        for(i = 0; i < frame.can_dlc; i ++){
            pkg->data[i] = frame.data[i];
        }

        pkg->head.id.size = frame.can_dlc-1;
        //dump_can_pkg(pkg);
    }

    return byte;
}

int CanDevice::can_write(CanPkg* pkg, int size)
{
    struct can_frame frame;
    int retry = 3;

    //dump_can_pkg(pkg);

    frame.can_id = pkg->head.id.channel;
    frame.can_dlc = size;

    int ret;
    memcpy(frame.data, pkg->data, size);
    while(retry > 0){
        ret = write(dev, &frame, sizeof(frame));
        if(ret < 0){
            log_once_error("write can data fail. ch:%02x. reason:%s retry:%d", frame.can_id, strerror(errno), retry);
            retry --;
            usleep(100);
            continue;
        }
        else
            break;
    }

    return ret;
}

int CanDevice::set_filter(struct can_filter *filters, int len)
{
    int ret = setsockopt(dev, SOL_CAN_RAW, CAN_RAW_FILTER, filters, len);

    return ret;
}

int CanDevice::can_close()
{
    log_warn("can close:%s", dev_name);
    return close(dev);
}
