#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <poll.h>
#include <math.h>

#include <boost/thread/thread.hpp>
#include "gaussian.h"
#include "gs_api.h"
#include "gs_nav.h"
#include "platform/chassis/ChassisDef.h"
#include "imemory/atris_imemory_api.h"
#include "navigationv2/navigation.h"
#include "navigationv2/scheme_manager.h"

#define ODOM_UPDATE_FREQ    50//HZ

short gs_convert_value_short(char* data)
{
    short val = 0;
    val = (unsigned char)data[0] | ((unsigned char)(data[1] & 0xFF) << 8);

    return val;
}

int gs_convert_value_int(char* data)
{
    int val = 0;
    val = data[0] | ((data[1] & 0xFF) << 8) |
        ((data[2] & 0xFF) << 16) | ((data[3] & 0xFF) << 24);

    return val;
}

Gaussian::Gaussian()
  : robot_pose_sub_(TOPIC_ODOM, &Gaussian::on_receive_odom, this),
   _electrode_state(ELECTRODE_IDEL),
   _charge_state(CHARGE_IDEL),
   _is_leaving_pile(false),
   _is_charging(0)
   {
    nav_reset_sub_ = nh_.subscribe(TOPIC_NAV_RESET, 100, &Gaussian::on_nav_reset, this);
    set_vel_cmd_pub_ = nh_.advertise<atris_msgs::VelCmd>(TOPIC_SET_VEL_TO_CHASSIS, 100);
    robot_imu_sub_ = nh_.subscribe(TOPIC_IMU, 100, &Gaussian::on_receive_imu, this);
    charge_sub_ = nh_.subscribe(TOPIC_POWER_CHARGE_CMD_RESP_MESSAGE, 100, &Gaussian::on_receive_charge_state, this);
    tinyros::nh()->subscribe(robot_pose_sub_);
    gs_fd = -1;
    cfg = Config::get_instance();
    gs_thread = NULL;
    state = false;
    gs_server_run_flag_ = false;
    GsNav::get_instance()->init();
    new boost::thread(boost::bind(&Gaussian::create_server, this, cfg->gs_srv_port));
    //new boost::thread(boost::bind(&Gaussian::odom_update_proc, this));
}

Gaussian::~Gaussian()
{
    close(gs_fd);
}

int Gaussian::init()
{
    return 0;//remove wait for gs device
    while(!state){
        log_info("wait for gs device.");
        sleep(1);
    }

    return 0;
}

void Gaussian::on_receive_imu(const atris_msgs::RobotImu& msg) {
    // log_once_info("receive imu");
    _robot_imu.roll = msg.euler_angles.x;
    _robot_imu.pitch = msg.euler_angles.y;
    _robot_imu.yaw = msg.euler_angles.z;
    _robot_imu.accel_x = msg.imu.linear_acceleration.x;
    _robot_imu.accel_y = msg.imu.linear_acceleration.y;
    _robot_imu.accel_z = msg.imu.linear_acceleration.z;
    _robot_imu.gyro_x = msg.imu.angular_velocity.x;
    _robot_imu.gyro_y = msg.imu.angular_velocity.y;
    _robot_imu.gyro_z = msg.imu.angular_velocity.z;
}

void Gaussian::on_receive_odom(const tinyros::atris_msgs::RobotPose& msg) {
    // log_once_info("receive odom");
    _robot_pose.pos_x = msg.p_x;
    _robot_pose.pos_y = msg.p_y;
    _robot_pose.pos_a = msg.p_z;
    _robot_pose.speed_x = msg.v_x;
    _robot_pose.speed_y = msg.v_y;
    _robot_pose.speed_a = msg.v_z;
    if(fabs(msg.v_x) > Config::get_instance()->forward_max 
    || fabs(msg.v_y) > Config::get_instance()->forward_max
    || fabs(msg.v_z) > Config::get_instance()->angular_max) {
       Lwarn("odom data unnormal:[vx: %lf, vy: %lf, vz: %lf]", msg.v_x, msg.v_y, msg.v_z);
    }
}

void Gaussian::set_charge_state_idel(){
    _electrode_state = ELECTRODE_IDEL;
    _charge_state = CHARGE_IDEL;
}

void Gaussian::on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg) {
    Linfo("Gaussian on_receive_charge");
    if (msg.charge_msg_type == atris_msgs::PowerChargeCmd::START_LEAVE_PILE) {
        _is_leaving_pile = true;
        _is_charging = false;
        return;
    }

    if(msg.charge_msg_type == atris_msgs::PowerChargeCmd::CHASSIS_NOTIFY_CHARGE_STATUS
        && msg.charge_status == 0x01) {
            if(_is_leaving_pile || _is_charging) {
                _electrode_state = ELECTRODE_IDEL;
                _charge_state = CHARGE_IDEL;
                _is_leaving_pile = false;
                return;
            } else {
                _is_charging = true;
                _electrode_state = ELECTRODE_WORK;
                _charge_state = CHARGE_WORK;
            }

    }
}

void Gaussian::on_nav_reset(const atris_msgs::NavReset& msg) {
  if (msg.reset == atris_msgs::NavReset::NAV_POWER_OFF) {
    GsApi::get_instance()->power_off();
  } else if (msg.reset == atris_msgs::NavReset::NAV_STOP) {
    nav::Navigation::get_instance().stop();
    nav::SchemeManager::get_instance().stop();
  } else if (msg.reset == atris_msgs::NavReset::NAV_RESET) {
    GsApi::get_instance()->set_time();
    nav::Navigation::get_instance().stop();
    nav::SchemeManager::get_instance().stop();
    GsApi::get_instance()->reset_setting();
    GsApi::get_instance()->reboot();
  } else if (msg.reset == atris_msgs::NavReset::NAV_RESET_CLEAR) {
    GsApi::get_instance()->set_time();
    nav::Navigation::get_instance().stop();
    nav::SchemeManager::get_instance().stop();
    GsApi::get_instance()->reset_setting();
    GsApi::get_instance()->map_all_del();
    GsApi::get_instance()->reboot();
  }
}

/**
 * @brief guild_gs_get_ultrasonic 构建超声数据
 *
 * @param ultra 超声返回值头部
 * @param ultra_data[] 超声数据，最长支持8组数据
 */
void Gaussian::build_gs_normal_sensor(gs_normal_sensor &sensor,
        unsigned char bumper, unsigned char drop,  unsigned short ultra_data[])
{
    sensor.head.head[0] = 0x0B;
    sensor.head.head[1] = 0x0D;

    sensor.head.len = sizeof(sensor) - sizeof(gs_head) - 1;
    sensor.cmd = 0x02;

    sensor.bumper = bumper;
    sensor.drop = drop;

    memcpy(sensor.ultra_data, ultra_data, sizeof(sensor.ultra_data));

    unsigned char *checksum_data = ((unsigned char*)&sensor) + 2;
    sensor.checksum = gs_checksum(checksum_data, sensor.head.len+1);
}

bool Gaussian::is_move(int speed, int angle) {
    static int reset_count = 50;
    bool can_move = true;
    if (!speed && !angle) {
        can_move = reset_count > 0;
        reset_count = reset_count > 0 ? (reset_count - 1) : 0;
    } else {
        reset_count = 50;
    }
    return can_move;
}

bool Gaussian::is_move(short speed_x, short speed_y, short angle) {
    static int reset_count = 50;
    bool can_move = true;
    if (!speed_x && !speed_y && !angle) {
        can_move = reset_count > 0;
        reset_count = reset_count > 0 ? (reset_count - 1) : 0;
    } else {
        reset_count = 50;
    }
    return can_move;
}

void Gaussian::on_move(gs_move *move)
{
    short speed = gs_convert_value_short((char*)&move->spd_l);
    short angle = gs_convert_value_short((char*)&move->spd_a);

    gs_angle gsangle;

    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    double cur_angle = odom_info.speed_z;
    
    //log_info("on move line:%d angle:%d cur angle:%d", speed, angle, cur_angle);
    build_gs_angle(gsangle, (int)(cur_angle*1000));
    int ret = send_to_gs(&gsangle, sizeof(gsangle));
    if(ret < 0){
        log_info("resp angle fail.");
    }

    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = GS;
    vel_cmd.priority = *((int32_t*)&owner);
#ifndef _CHASSIS_MARSHELL_
    vel_cmd.x = speed/1000.0;
    vel_cmd.z = angle/1000.0;
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    if (is_move(speed, angle)) {
        set_vel_cmd_pub_.publish(vel_cmd);
    }
#else
  	if ((angle >= 300 || angle <= -300) && speed == 0)
  	{
  		speed = 100;
  	}
    
    vel_cmd.x = speed/1000.0;
    vel_cmd.z = angle/1000.0;
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    if (is_move(speed, angle)) {
        set_vel_cmd_pub_.publish(vel_cmd);
    }

    gs_dist dist;
    shm::iMemory_read_OdomInfo( &odom_info );
    int distance = odom_info.dist_center*1000;
    //log_info("on move dist:%d", distance);
    build_gs_dist(dist, distance, ~distance+1);
    ret = send_to_gs(&dist, sizeof(dist));
    if(ret < 0){
        //log_info("on move send dist fail.");
    }

    gs_speed speed_t;
    int speed_x = odom_info.speed_x*1000;
    build_gs_speed(speed_t, speed_x);
    ret = send_to_gs(&speed_t, sizeof(speed_t));
    if(ret < 0){
        log_info("on move ackman send speed fail.");
    }
#endif
}

void Gaussian::odom_update_proc()
{
    while(true){
        update_odom();

        usleep(1.0/ODOM_UPDATE_FREQ* 1000 * 1000);
    }
}

void Gaussian::update_odom()
{
	gs_dist dist;
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
#ifdef _CHASSIS_MARSHELL_
	int distance = odom_info.dist_center*1000;
	log_info("update_odom dist:%d", distance);
	build_gs_dist(dist, distance, ~distance+1);
#else
	double dist_l = odom_info.dist_left;
    double dist_r = odom_info.dist_right;
	int d_r = ~((int)(dist_r * 1000)) + 1;
	int d_l = dist_l * 1000;
	log_info("update_odom dl:%d dr:%d", d_l, d_r);
	build_gs_dist(dist, d_l,d_r);
#endif
	int ret = send_to_gs(&dist, sizeof(dist));
	if(ret < 0){
		//log_error("on move send dist fail.");
	}
}

void Gaussian::on_move_ackman(gs_move_ackman *move)
{
    int speed = gs_convert_value_int((char*)&move->spd_l);
    int angle = gs_convert_value_int((char*)&move->spd_a);
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = GS;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.x = speed/1000.0;
    vel_cmd.z = angle/1000.0;
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    if (is_move(speed, angle)) {
        set_vel_cmd_pub_.publish(vel_cmd);
    }

    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
#ifdef _CHASSIS_MARSHELL_
    gs_angle gsangle;
    double cur_angle = odom_info.speed_z;
    static int count = 0;
    if (count++ == ODOM_UPDATE_FREQ)
    {
        count = 0;
        log_info("on move line:%d angle:%d cur angle:%f", speed, angle, cur_angle);
    }
    build_gs_angle(gsangle, (int)(cur_angle*1000));
    int ret = send_to_gs(&gsangle, sizeof(gsangle));
    if(ret < 0){
        log_info("resp angle fail.");
    }

#else

    gs_angle gsangle;
    angle = odom_info.speed_z;

    if(angle > 45)
        angle = 45;
    else if(angle < -45)
        angle = -45;

    build_gs_angle(gsangle, angle);
    int ret = send_to_gs(&gsangle, sizeof(gsangle));
    if(ret < 0){
        log_info("send angle fail.");
    }
#endif
    gs_dist dist;
    int distance = odom_info.dist_center*1000;
    build_gs_dist(dist, distance, ~distance+1);
    //log_info("dist:%d", distance);
    ret = send_to_gs(&dist, sizeof(dist));
    if(ret < 0){
        log_info("on move ackman send dist fail.");
    }

    gs_speed speed_t;
    int speed_x = odom_info.speed_x*1000;
    build_gs_speed(speed_t, speed_x);
    ret = send_to_gs(&speed_t, sizeof(speed_t));
    if(ret < 0){
        log_info("on move ackman send speed fail.");
    }
}

void Gaussian::on_move_4wd(gs_move_4wd *move) {

    short speed_x = gs_convert_value_short((char*)&move->spd_x);
    short speed_y = gs_convert_value_short((char*)&move->spd_y);
    short speed_a = gs_convert_value_short((char*)&move->spd_a);
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = GS;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.x = speed_x / 1000.0;
    vel_cmd.y = speed_y / 1000.0;
    vel_cmd.z = speed_a / 1000.0;
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    if (is_move(speed_x, speed_y, speed_a)) {
        set_vel_cmd_pub_.publish(vel_cmd);
    }

    gs_odom odom_data;
    int p_x = _robot_pose.pos_x * 1000;
    int p_y = _robot_pose.pos_y * 1000;
    short p_a = _robot_pose.pos_a * 1000;
    short v_x = _robot_pose.speed_x * 1000;
    short v_y = _robot_pose.speed_y * 1000;
    short v_a = _robot_pose.speed_a * 1000;

    build_gs_odom(odom_data, p_x, p_y, p_a, v_x, v_y, v_a);
    int ret = send_to_gs(&odom_data, sizeof(odom_data));
    if(ret < 0){
        log_info("on move 4wd send odom fail.");
    }

    gs_imu imu_data;
    short roll = _robot_imu.roll * 10;
    short pitch = _robot_imu.pitch * 10;
    short yaw = _robot_imu.yaw * 10;
    short gyro_x = _robot_imu.gyro_x * 10 * (180 / 3.14);
    short gyro_y = _robot_imu.gyro_y * 10 * (180 / 3.14);
    short gyro_z = _robot_imu.gyro_z * 10 * (180 / 3.14);
    short accel_x = _robot_imu.accel_x * (1000 / 9.80665);
    short accel_y = _robot_imu.accel_y * (1000 / 9.80665);
    short accel_z = _robot_imu.accel_z * (1000 / 9.80665);

//    log_debug("imu data : gyro_x: %hd ,gyro_y: %hd ,gyro_z : %hd ,accel_x: %hd ,accel_y: %hd ,accel_z: %hd, roll: %hd,pitch: %hd, yaw: %hd ."
//         ,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z, roll, pitch, yaw);
    build_gs_imu(imu_data, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
    ret = send_to_gs(&imu_data, sizeof(imu_data));
    if(ret < 0){
        log_info("on move 4wd send imu fail.");
    }

    gs_charge charge;
    build_gs_charge(charge, _electrode_state, _charge_state);
    ret = send_to_gs(&charge, sizeof(charge));
    if(ret < 0){
        log_info("on move 4wd send charge info fail.");
    }
}

/**
 * @brief build_gs_angle
 *
 * @param gs_angle
 * @param angle unit: angle
 */
void Gaussian::build_gs_angle(gs_angle &gsangle, int angle)
{
    gsangle.head.head[0] = 0x0B;
    gsangle.head.head[1] = 0x0D;

    gsangle.head.len =sizeof(gs_angle) - sizeof(gs_head) -1;
    gsangle.cmd = 0x0D;

    //float a = angle *1000.0f / 180.0f * M_PI;
    gsangle.angle = angle;

    gsangle.checksum = gs_checksum((unsigned char*)(&gsangle) +2, gsangle.head.len + 1);
	unsigned char* pkg = (unsigned char*)(&gsangle);
    //log_error("ANGL:%02X %02X %02X %02X A:%d", pkg[0], pkg[1], pkg[2], pkg[3], angle);
}

void Gaussian::build_gs_dist(gs_dist &dist, int left_dist, int right_dist)
{
    dist.head.head[0] = 0x0B;
    dist.head.head[1] = 0x0D;

    dist.head.len = sizeof(dist) - sizeof(gs_head) - 1;
    dist.cmd = 0x0A;

    dist.left_dist = left_dist;
    dist.right_dist = right_dist;

    unsigned char *checksum_data = ((unsigned char*)&dist) + 2;
    dist.checksum = gs_checksum(checksum_data, dist.head.len+1);
	  unsigned char* pkg = (unsigned char*)(&dist);
    //log_error("DIST:%02X %02X %02X %02X %02X L:%d R:%d", pkg[0], pkg[1], pkg[2], pkg[3], left_dist, right_dist);
}

void Gaussian::build_gs_speed(gs_speed &speed, int speed_x)
{
    speed.head.head[0] = 0x0B;
    speed.head.head[1] = 0x0D;

    speed.head.len = sizeof(speed) - sizeof(gs_head) - 1;
    speed.cmd = 0x0E;//todo 

    speed.speed_x = speed_x;
    speed.speed_y = speed_x;

    unsigned char *checksum_data = ((unsigned char*)&speed) + 2;
    speed.checksum = gs_checksum(checksum_data, speed.head.len+1);
	unsigned char* pkg = (unsigned char*)(&speed);
    log_debug("SPEED:%02X %02X %02X %02X X:%d", pkg[0], pkg[1], pkg[2], pkg[3], speed_x);
}

void Gaussian::build_gs_imu(gs_imu &imu, short roll, short pitch , short yaw,
                short gyro_x, short gyro_y, short gyro_z,
                short accel_x, short accel_y, short accel_z)
{
    imu.head.head[0] = 0x0B;
    imu.head.head[1] = 0x0D;

    imu.head.len = sizeof(imu) - sizeof(gs_head) - 1;
    imu.cmd = 0x09;

    imu.roll = roll;
    imu.pitch = pitch;
    imu.yaw = yaw;

    imu.gyro_x = gyro_x;
    imu.gyro_y = gyro_y;
    imu.gyro_z = gyro_z;

    imu.accel_x = accel_x;
    imu.accel_y = accel_y;
    imu.accel_z = accel_z;

    unsigned char *checksum_data = ((unsigned char*)&imu) + 2;
    imu.checksum = gs_checksum(checksum_data, imu.head.len+1);
    unsigned char* pkg = (unsigned char*)(&imu);
    // log_debug("IMU:%02X %02X %02X %02X %02X %02x", pkg[0], pkg[1], pkg[2], pkg[2], pkg[3], pkg[4], pkg[5]);
}

void Gaussian::build_gs_odom(gs_odom &odom, int pos_x, int pos_y, short pos_a,
                            short speed_x, short speed_y, short speed_a)
{
    odom.head.head[0] = 0x0B;
    odom.head.head[1] = 0x0D;

    odom.head.len = sizeof(odom) - sizeof(gs_head) - 1;
    odom.cmd = 0x0F;

    odom.pos_x = pos_x;
    odom.pos_y = pos_y;
    odom.pos_a = pos_a;
    odom.speed_x = speed_x;
    odom.speed_y = speed_y;
    odom.speed_a = speed_a;

    unsigned char *checksum_data = ((unsigned char*)&odom) + 2;
    odom.checksum = gs_checksum(checksum_data, odom.head.len+1);
	unsigned char* pkg = (unsigned char*)(&odom);
    // log_debug("ODOM:%02X %02X %02X %02X %02X %02x", pkg[0], pkg[1], pkg[2], pkg[2], pkg[3], pkg[4], pkg[5]);
}

void Gaussian::build_gs_charge(gs_charge &charge, char electrode_state, char charge_state)
{
    charge.head.head[0] = 0x0B;
    charge.head.head[1] = 0x0D;

    charge.head.len = sizeof(charge) - sizeof(gs_head) - 1;
    charge.cmd = 0x11;

    charge.electrode_state = _electrode_state;
    charge.charge_state = _charge_state;

    unsigned char *checksum_data = ((unsigned char*)&charge) + 2;
    charge.checksum = gs_checksum(checksum_data, charge.head.len+1);
	unsigned char* pkg = (unsigned char*)(&charge);
    //log_debug("Charge:%02X %02X %02X %02X %02X %02x %02x", pkg[0], pkg[1], pkg[2], pkg[2], pkg[3], pkg[4], pkg[5], pkg[6]);
}

/**
 * @brief gs_checksum 生成校验码
 *
 * @param buffer 整个需要生成校验码的数据
 * @param len payload 的长度
 *
 * @return
 */
char Gaussian::gs_checksum(const unsigned char *buffer, unsigned char len)
{
    char val = 0;
    int l = len;

    while(l > 0){
        val = (*buffer++)^val;
        l --;
    }

    return val;
}

int Gaussian::send_to_gs(void *data, int snd_len)
{
    assert(data != NULL && snd_len > 0);
    int len, sent_len = 0;

    if(gs_fd < 0)
        return -1;

    while(sent_len < snd_len){
        len = ::send(gs_fd, (char*)data + sent_len, snd_len - sent_len, 0);
        if(len <= 0){
            log_once_error("%s send data error", __FUNCTION__);
            return -1;
        }
        sent_len += len;
    }

    return sent_len;
}

int Gaussian::send_recv(void *snd_data, int snd_len, void* rcv_data, int rcv_len)
{
    int len = snd_len, sent_len= 0;
    struct pollfd fds;
    const int timeout = 1*1000;//ms

    if(gs_fd < 0)
        return -1;

    fds.fd = gs_fd;
    fds.events = POLLIN | POLLPRI;

    while(sent_len < snd_len){
        len = ::send(gs_fd, (char*)snd_data + len, snd_len - sent_len, 0);
        if(len <= 0){
            log_once_error("%s send data error", __FUNCTION__);
            return -1;
        }
        sent_len += len;
    }

    int ret = poll(&fds, 1, timeout);
    if(ret == 0){
        log_error("read data timeout");
        return -2;
    }
    else{
        ret = ::recv(gs_fd, rcv_data, rcv_len, 0);
        if(ret == 0){
            log_error("peer connection  close");
            return -3;
        }
        else if(ret < 0){
            log_error("recv error:%s", strerror(errno));
            return -4;
        }
    }

    return ret;
}

inline char get_checksum(char *data)
{
    gs_head *h = (gs_head*)data;

    int len  = sizeof(gs_head) + h->len + 1;

    return data[len];
}

int Gaussian::recv_gs_data(int fd, unsigned char *gs_data)
{
    int readed = 0, read_len;
    int head_size = sizeof(gs_head);
    unsigned char* data = gs_data;
    unsigned char* buf = NULL;

    //read gs header
    while(readed < head_size){
        read_len = recv(fd, &data[readed], head_size - readed, 0);
        if(read_len == 0){
            log_error("read head tcp zero, peer close:%s", strerror(errno));
            return -2;
        }
        else if(read_len < 0){
            log_error("read server data fail:%s", strerror(errno));
            return -1;
        }

        readed += read_len;
    }

    gs_head* h = (gs_head*)data;

    if((data[0] != 0x0B) && (data[1] != 0x0D)){
        log_error("not gs frame data");
        return -1;
    }

    //read left data
    int left_size = h->len + 1;
    buf = &data[head_size];
    readed = 0;

    while(readed < left_size){
        read_len = recv(fd, &buf[readed], left_size, 0);
        if(read_len == 0){
            log_error("read data tcp zero, peer close:%s", strerror(errno));
            return -2;
        }
        else if(read_len < 0){
            log_error("read server data fail:%s", strerror(errno));
            return -1;
        }
        readed += read_len;
    }

    gs_head *head = (gs_head*)data;
    unsigned char checksum = gs_checksum((unsigned char*)(data + 2), head->len+1);
    int pos = sizeof(gs_head) + h->len + 1;
    if(data[pos-1] != checksum){
       log_error("gs checksum error!!");
        return -3;
    }

    //dump_data((char*)data, pos);

    return 0;
}

void Gaussian::proc_gs_cmd(void *data)
{
    unsigned char* payload = (unsigned char*)((unsigned char*)data + sizeof(gs_head));

    {
        shm::UltraSound ultroso;
        shm::iMemory_read_UltraSound(&ultroso);
        
        gs_normal_sensor s;
        build_gs_normal_sensor(s, 0, 0, ultroso.data);
        int ret = send_to_gs(&s, sizeof(gs_normal_sensor));
        if(ret < 0){
            log_error("resp ultra fail.");
        }
        //log_info("send ultra data");
    }
    
    if(payload[0] == GS_CMD_MOVE){
        on_move((gs_move*)data);
    }
    else if(payload[0] == GS_CMD_MOVE_ACKMAN){
        on_move_ackman((gs_move_ackman*)data);
    }
    else if(payload[0] == GS_CMD_MOVE_4WD){
        on_move_4wd((gs_move_4wd*)data);
    }
}

void Gaussian::create_server(int port)
{
    struct sockaddr_in addr;
    struct sockaddr_in peer;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0){
        log_error("gaussian create socket fail");
        return ;
    }

    int on = 1;
    int ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if(ret < 0){
        log_error("setsockopt error:%s", strerror(errno));
        close(fd);
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(cfg->local_ip.c_str());//inet_addr(INADDR_ANY);
    if(bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log_error("bind error:%s", strerror(errno));
        close(fd);
        return;
    }
    int listen_sock = listen(fd, 5);
    if(listen_sock < 0){
        log_error("listen fail");
        close(fd);
        return;
    }

    socklen_t peer_len = sizeof(peer);
    log_warn("listen for connect.......");
    while(true){
        gs_fd = accept(fd, (struct sockaddr*)&peer, &peer_len);
        if(gs_fd < 0){
            log_error("server accept fail");
        }
        else{
            char buf[1024];
            log_warn("gs fd:%d client connected with ip: %s  and port: %d ",
                    gs_fd, inet_ntop(AF_INET,&peer.sin_addr, buf, 1024), ntohs(peer.sin_port));

            if(gs_thread != NULL){
                gs_server_run_flag_ = false;
                gs_thread->interrupt();
                gs_thread->timed_join(boost::posix_time::milliseconds(500));
                delete gs_thread;
                gs_thread = NULL;
            }

            state = true;
            gs_thread = new boost::thread(boost::bind(&Gaussian::gs_server_run, this, gs_fd));
        }
    }
}

void Gaussian::gs_server_run(int fd)
{
    int ret = 0;
    unsigned char gs_data[1024];

    log_info("create gs thread, fd=%d!", fd);
    GsApi::get_instance()->set_time();
    gs_server_run_flag_ = true;

    int on = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    if(ret < 0){
        log_error("setsockopt error:%s", strerror(errno));
        close(fd);
        return;
    }

    while(gs_server_run_flag_){
        ret = recv_gs_data(fd, gs_data);
        if(ret == 0){
            proc_gs_cmd(gs_data);
        }
        else if(ret == -2){
            log_error("peer connection close, exit this thread.");
            close(fd);
            break;
        }
        else if(ret < 0){
            log_error("recv gs data fail");
            sleep(1);
            close(fd);
            break;
        }
    }
}

/**
 * @brief move
 *
 * @param spd *1000
 * @param ang *1000
 */
void Gaussian::move(short spd, short ang)
{
    short speed = spd;
    short angle = ang;

    gs_angle gsangle;

    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo( &odom_info );
    float cur_angle = odom_info.speed_z;
    build_gs_angle(gsangle, cur_angle);
    int ret = send_to_gs(&gsangle, sizeof(gsangle));
    if(ret < 0){
        log_error("resp angle fail.");
    }

    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = GS;
    vel_cmd.x = speed/1000.0;
    vel_cmd.z = angle/1000.0;
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    if (is_move(speed, angle)) {
        set_vel_cmd_pub_.publish(vel_cmd);
    }
}
