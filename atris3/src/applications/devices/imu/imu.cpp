#include <boost/thread/thread.hpp>
#include <thread>
#include <chrono>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

#include "imu.h"

static Packet_t _RxPkt;
static int16_t _acc[3];
static int16_t _gyo[3];
static int16_t _mag[3];
static float _eular[3];
static float _quat[4];
static uint8_t _id;

Imu::Imu() {
    log_info("%s", __FUNCTION__);
    imu_pub_ = nh_.advertise<atris_msgs::RobotImu>(TOPIC_IMU, 100);
    new boost::thread(boost::bind(&Imu::imu_data_proc, this));
}

int Imu::init()
{
    log_info("Imu init");
    return 0;
}

void Imu::imu_data_proc()
{
    int fd = open_port((char *)"/dev/ttyS2");

    //tcgetattr(fd, &options);

    //Enable the receiver and set local mode...
    /* options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    //No parity
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
	options.c_cc[VTIME] = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST; */

    imu_data_decode_init();

    uint8_t ID = 0;
    int16_t Acc[3] = {0};
    int16_t Gyo[3] = {0};
    int16_t Mag[3] = {0};
    float Eular[3] = {0};
    float Quat[4] = {0};
    int32_t Pressure = 0;

    int i;
    uint8_t buf[1024];
    while (1)
    {
        ssize_t n = read(fd, buf, sizeof(buf));
        for (i = 0; i < n; i++)
        {
            packet.Packet_Decode(buf[i]);
        }

        get_raw_acc(Acc);
        get_raw_gyo(Gyo);
        get_raw_mag(Mag);
        get_eular(Eular);
        get_id(&ID);
        /*
		log_debug("Acc:%d %d %d\r\n",Acc[0], Acc[1], Acc[2]);
		log_debug("Gyo:%d %d %d\r\n",Gyo[0], Gyo[1], Gyo[2]);
		log_debug("Mag:%d %d %d\r\n",Mag[0], Mag[1], Mag[2]);
		log_debug("Eular(P R Y):%0.2f %0.2f %0.2f\r\n",Eular[0], Eular[1], Eular[2]);
        */
        atris_msgs::RobotImu imu_info;
        imu_info.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
        /*
		imu_info.imu.header.stamp =
        imu_info.imu.header.frame_id =
        imu_info.imu.orientation_covariance[0] =
        imu_info.imu.orientation_covariance[1] =
        imu_info.imu.orientation_covariance[2] =
        imu_info.imu.orientation_covariance[3] =
        imu_info.imu.orientation_covariance[4] =
        imu_info.imu.orientation_covariance[5] =
        imu_info.imu.orientation_covariance[6] =
        imu_info.imu.orientation_covariance[7] =
        imu_info.imu.orientation_covariance[8] =
        */
        imu_info.imu.angular_velocity.x = (double)(Gyo[0] * 3.14 / (180));
        imu_info.imu.angular_velocity.y = (double)(Gyo[1] * 3.14 / (180));
        imu_info.imu.angular_velocity.z = (double)(Gyo[2] * 3.14 / (180));

        imu_info.imu.linear_acceleration.x = (double)(((Acc[0]) * 9.80665 / 1000.f));
        imu_info.imu.linear_acceleration.y = (double)(((Acc[1]) * 9.80665 / 1000.f));
        imu_info.imu.linear_acceleration.z = (double)(((Acc[2]) * 9.80665 / 1000.f));

        imu_info.imu.orientation.w = Quat[0];
        imu_info.imu.orientation.x = Quat[1];
        imu_info.imu.orientation.y = Quat[2];
        imu_info.imu.orientation.z = Quat[3];

        imu_info.euler_angles.x = Eular[1]; //roll
        imu_info.euler_angles.y = Eular[0]; //pitch
        imu_info.euler_angles.z = Eular[2]; //raw

        imu_pub_.publish(imu_info);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(fd);
}

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */
int Imu::open_port(char *port_device)
{
    struct termios options;

    int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    tcgetattr(fd, &options);

    if (fd == -1)
    {
        log_debug("open_port: Unable to open SerialPort");
        return (-1);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        log_debug("fcntl failed\n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        log_debug("standard input is not a terminal device\n");
    }
    else
    {
        log_debug("isatty success!\n");
    }

    bzero(&options, sizeof(options));

    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return (fd);
}

int Imu::get_raw_acc(int16_t *a)
{
    memcpy(a, _acc, sizeof(_acc));
    return 0;
}

int Imu::get_raw_gyo(int16_t *g)
{
    memcpy(g, _gyo, sizeof(_gyo));
    return 0;
}

int Imu::get_raw_mag(int16_t *m)
{
    memcpy(m, _mag, sizeof(_mag));
    return 0;
}

int Imu::get_eular(float *e)
{
    memcpy(e, _eular, sizeof(_eular));
    return 0;
}

int Imu::get_quat(float *q)
{
    memcpy(q, _quat, sizeof(_quat));
    return 0;
}

int Imu::get_id(uint8_t *user_id)
{
    *user_id = _id;
    return 0;
}

/*  callback function of  when recv a data frame successfully */
void Imu::OnDataReceived(Packet_t *pkt)
{

    if (pkt->type != 0xA5)
    {
        return;
    }

    int offset = 0;
    uint8_t *p = pkt->buf;
    while (offset < pkt->payload_len)
    {
        switch (p[offset])
        {
        case kItemID:
            _id = p[1];
            offset += 2;
            break;
        case kItemAccRaw:
        case kItemAccCalibrated:
        case kItemAccFiltered:
        case kItemAccLinear:
            memcpy(_acc, p + offset + 1, sizeof(_acc));
            //log_debug("_acc:%d %d %d\r\n",_acc[0], _acc[1], _acc[2]);
            offset += 7;
            break;
        case kItemGyoRaw:
        case kItemGyoCalibrated:
        case kItemGyoFiltered:
            memcpy(_gyo, p + offset + 1, sizeof(_gyo));
            //log_debug("_gyo:%d %d %d\r\n", _gyo[0], _gyo[1], _gyo[2]);
            offset += 7;
            break;
        case kItemMagRaw:
        case kItemMagCalibrated:
        case kItemMagFiltered:
            memcpy(_mag, p + offset + 1, sizeof(_mag));
            offset += 7;
            break;
        case kItemRotationEular:
            _eular[0] = ((float)(int16_t)(p[offset + 1] + (p[offset + 2] << 8))) / 100;
            _eular[1] = ((float)(int16_t)(p[offset + 3] + (p[offset + 4] << 8))) / 100;
            _eular[2] = ((float)(int16_t)(p[offset + 5] + (p[offset + 6] << 8))) / 10;
            //log_debug("_eular(P R Y):%0.2f %0.2f %0.2f\r\n",_eular[0], _eular[1], _eular[2]);
            offset += 7;
            break;
        case kItemRotationEular2:
            memcpy(_eular, p + offset + 1, sizeof(_eular));
            offset += 13;
            break;
        case kItemRotationQuat:
            memcpy(_quat, p + offset + 1, sizeof(_quat));
            offset += 17;
            break;
        case kItemPressure:
            offset += 5;
            break;
        case kItemTemperature:
            offset += 5;
            break;
        default:
            log_debug("data decode wrong\r\n");
            return;
            break;
        }
    }
}

int Imu::imu_data_decode_init(void)
{
    packet.Packet_DecodeInit(&_RxPkt, OnDataReceived);
    return 0;
}