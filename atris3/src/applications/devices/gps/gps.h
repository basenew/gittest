#ifndef GPS_H__
#define GPS_H__

#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/RobotInfo.h"
#include <json/json.h>

#include <stdint.h>
#include <stdbool.h>

enum parse_status
{
    PS_HEAD,
    PS_DATA,
    PS_CS,
    PS_TAIL
};

class Gps
{
public:
    Gps();
    int init();

private:
    ~Gps() {}
    ros::NodeHandle nh_;
    ros::Publisher gps_pub_;
    void gps_data_proc();
    int open_port(const char *port_device);
    int send_gps_ctrl_cmd(char * cmd);
    int isGpsFrameValid(char * frame);
    uint8_t AsciiToHex(char * asc);
    char * get_next_comma_field(char * data_ptr);
    void gps_handle_frame(char * frame);
    void parse_gps_rmc_frame(char * frame);
    int num_asc_to_int(char * asc, int len);
    int get_gps_longitude_and_latitude(char * gps_data);
    unsigned short get_gps_speed_direction(char * gps_direction);
    void printGpsContent(char * p_buf, int len);
    int parseLat(char * buf);
    unsigned int parseLatDeg(char * buf);
    int parseLon(char * buf);
    unsigned int parseLonDeg(char * buf);
    void sendGpsData(int position_status);

    ros::Publisher diag_info_pub_;
    int uart_fd_;
    int last_gps_valid_flag_;
    unsigned short speed_direction_;
    unsigned int longitude_;
    int lat_deg_;
    int lat_min_;
    int lat_sec_;
    int lon_deg_;
    int lon_min_;
    int lon_sec_;
    unsigned int latitude_;
    unsigned int latitude_NS_;
    unsigned int longitude_EW_;

public:
    static Gps *get_instance()
    {
        static Gps singleton;
        return &singleton;
    }
};


#endif
