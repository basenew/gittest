#ifndef __GAUSSIAN_H__
#define __GAUSSIAN_H__

#include <stdio.h>
#include "config/config.h"
#include "gs_api.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/VelCmd.h"
#include "atris_msgs/NavReset.h"
#include "atris_msgs/RobotImu.h"
#include "tiny_ros/ros.h"
#include "tiny_ros/atris_msgs/RobotPose.h"
#include "atris_msgs/PowerChargeCmd.h"

//从高仙通信协议中拷贝

enum GS_CMD{
    GS_CMD_MOVE = 0x01,
    GS_CMD_MOVE_ACKMAN = 0x02,
    GS_CMD_MOVE_4WD = 0X03 // Four wheel drive chassis
};

enum ELECTRODE_STATE{
    ELECTRODE_IDEL = 0X00,
    ELECTRODE_WORK = 0X01,
};

enum CHARGE_STATE {
    CHARGE_IDEL = 0X00,
    CHARGE_WORK = 0X01,
};

#pragma pack(1)
typedef struct gs_head_
{
    char head[2]; //0x0B 0x0D
    char len;
}gs_head;

typedef struct gs_move_
{
    gs_head head;
    char cmd;
    signed short spd_l;/* 本体当前线速度*/
    signed short spd_a;/* 本体当前角速度*/
    char checksum;
}gs_move;

typedef struct gs_move_ackman_
{
    gs_head head;
    char cmd;
    signed int spd_l;/* 本体当前线速度*/
    signed int spd_a;/* 本体当前角速度*/
    char checksum;
}gs_move_ackman;

typedef struct gs_move_4wd_
{
    gs_head head;
    char cmd;
    short spd_x; /* 本体当前x轴方向线速度*/
    short spd_y; /* 本体当前y轴方向线速度*/ 
    short spd_a; /* 本体当前角速度*/
    char checksum;
}gs_move_4wd;

typedef struct gs_dist_
{
    gs_head head;
    char cmd;
    int left_dist;
    int right_dist;
    char checksum;
}gs_dist;

typedef struct gs_angle_
{
    gs_head head;
    char cmd;
    int angle;
    char checksum;
}gs_angle;

typedef struct gs_speed_
{
    gs_head head;
    char cmd;
    int speed_x;
    int speed_y;
    char checksum;
}gs_speed;

typedef struct gs_odom_
{
    gs_head head;
    char cmd;
    int pos_x;
    int pos_y;
    signed short pos_a;
    signed short speed_x;
    signed short speed_y;
    signed short speed_a;
    char checksum;
}gs_odom;

typedef struct gs_imu_
{
    gs_head head;
    char cmd;
    signed short roll;
    signed short pitch;
    signed short yaw;

    signed short gyro_x;
    signed short gyro_y;
    signed short gyro_z;

    signed short accel_x;
    signed short accel_y;
    signed short accel_z;

    char checksum;
}gs_imu;

typedef struct gs_charge_
{
    gs_head head;
    char cmd;
    char electrode_state;
    char charge_state;
    char checksum;
}gs_charge;

typedef struct robot_pose_
{
    double pos_x;
    double pos_y;
    double pos_a;
    double speed_x;
    double speed_y;
    double speed_a;
}robot_pose;

typedef struct robot_imu_
{
    double roll;
    double pitch;
    double yaw;

    double gyro_x;
    double gyro_y;
    double gyro_z;

    double accel_x;
    double accel_y;
    double accel_z;
}robot_imu;


typedef struct gs_normal_sensor_
{
    gs_head head; /*8个超声波数据，单位mm，0—7分别是：左，左前，前，右前，右，右后，后，左后*/
    char cmd;
    unsigned char bumper; //碰撞传感器
    unsigned char drop;  //跌落传感器
    unsigned short ultra_data[8]; //当数据时0XFFFF时，表示对应的超声波异常了。
    char checksum;
}gs_normal_sensor;
#pragma pack()

class Gaussian{
    private:
        Gaussian();

        int fd;//server fd
        int gs_fd;
        float old_angle, old_line;
        Config *cfg;
        boost::thread *gs_thread;

        bool state;
        bool gs_server_run_flag_;
        
        robot_pose _robot_pose = {0, 0, 0, 0, 0, 0};
        robot_imu_ _robot_imu = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        char _electrode_state;
        char _charge_state;
        bool _is_leaving_pile;
        bool _is_charging;
        boost::mutex mutex_ch;
        ros::NodeHandle nh_;
        ros::Publisher set_vel_cmd_pub_;
        ros::Subscriber nav_reset_sub_;
        ros::Subscriber robot_imu_sub_;
        ros::Subscriber charge_sub_;
        tinyros::Subscriber<tinyros::atris_msgs::RobotPose, Gaussian> robot_pose_sub_;

        void on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg);
        void on_receive_imu(const atris_msgs::RobotImu& msg);
        void on_receive_odom(const tinyros::atris_msgs::RobotPose& msg);
        void on_nav_reset(const atris_msgs::NavReset& msg);
        void odom_update_proc();
        void gs_server_run(int fd);
        int recv_gs_data(int fd, unsigned char* gs_data);
        int send_recv(void *snd_data, int snd_len, void* rcv_data, int rcv_len);
        int send_to_gs(void *data , int len);
        void proc_gs_cmd(void *data);
        bool is_move(int speed, int angle);
        bool is_move(short speed_x, short speed_y, short angle);
        void on_move(gs_move *move);
        void on_move_ackman(gs_move_ackman *move);
        void on_move_4wd(gs_move_4wd *move);
        char gs_checksum(const unsigned char *buffer, unsigned char len);
        void create_server(int port);
        void build_gs_speed(gs_speed &speed, int speed_x);
        void build_gs_dist(gs_dist &dist, int left_dist, int right_dist);
        void build_gs_angle(gs_angle &gs_angle, int angle);
        void build_gs_normal_sensor(gs_normal_sensor &sensor,
                unsigned char bumper, unsigned char drop,  unsigned short ultra_data[]);
        void build_gs_odom(gs_odom &odom, int pos_x, int pos_y, short pos_a,
                short speed_x, short speed_y, short speed_a);
        void build_gs_imu(gs_imu &imu, short roll, short pitch ,
                short yaw, short gyro_x, short gyro_y, short gyro_z,
                short accel_x, short accel_y, short accel_z);
        void build_gs_charge(gs_charge &charge, char electrode_state, char charge_state);
        inline void dump_data(char* data, int len){
            printf("data:");
            for(int i = 0; i < len; i ++){
                printf(":%02x", data[i]);
            }
            printf("\r\n");
        }
    public:
        ~Gaussian();
        int init();
        void move(short spd, short ang);
        void set_charge_state_idel();
        void update_odom();
        static Gaussian* get_instance(){
            static Gaussian singleton;
            return &singleton;
        }

};

#endif //__GAUSSIAN_H__
