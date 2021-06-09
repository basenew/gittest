#ifndef __IMU_H__
#define __IMU_H__

#include "packet.h"

#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/RobotImu.h"

#include <stdint.h>
#include <stdbool.h>

class Imu
{
public:
    Imu();
    int init();

private:
    ~Imu() {}
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    Packet packet;
    void imu_data_proc();
    int open_port(char *port_device);

    int imu_data_decode_init(void);
    int get_raw_acc(int16_t *a);
    int get_raw_gyo(int16_t *g);
    int get_raw_mag(int16_t *m);
    int get_id(uint8_t *user_id);
    int get_eular(float *e);
    int get_quat(float *q);

    static void OnDataReceived(Packet_t *pkt);

public:
    static Imu *get_instance()
    {
        static Imu singleton;
        return &singleton;
    }
};

#endif