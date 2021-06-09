/*
    Udock by xiangbin.huang
*/
#include "udock_planning.h"
#include "udock_manager.h"
#include "unistd.h"
#include "udock_common.h"
#include <math.h>
#include "unistd.h"
#include <string.h>
#include "udock_data.h"
#include "imemory/atris_imemory_api.h"
#include "platform/chassis/ChassisDef.h"

#ifdef _CHASSIS_MARSHELL_

#define     MOVE_ANGLE_RADIUS   32.5f
#define     DEST_PLUS           0.05f

moveControl::moveControl()
: set_vel_cmd_pub_(TOPIC_SET_VEL_TO_CHASSIS, new atris_msgs::VelCmd())
{
  set_vel_cmd_pub_ = nh_.advertise<atris_msgs::VelCmd>(TOPIC_SET_VEL_TO_CHASSIS, 100);
}

void moveControl::init(void)
{
    udock_manager = UdockTankManager::get_instance();
    udock_wheel_obstacle = UdockWheelObstacleDetect::get_instance();
    udock_data = UdockData::get_instance();
//    new boost::thread(boost::bind(&moveControl::chassis_test_thread, this));
}

moveControl::~moveControl()
{

}
/*-------------------------------------------
 * 轮式运动测试线程
 *
---------------------------------------------*/
void moveControl::chassis_test_thread(void)
{
    log_info("[udock move test] run------------!");
    while(1)
    {
        const char *dir_path = "/home/atris/testmove";
        const char *dir_path1 = "/home/atris/testmoveback";
        const char *dir_path2 = "/home/atris/testmovefront";
        if(access(dir_path, F_OK) == 0)
        {
            move_info.step_one_steering =STEERING_LEFT;
            move_info.step_one_dist =500; //mm
            move_info.step_two_dist =500;
            move_info.step_three_steering =STEERING_RIGHT;
            move_info.step_three_dist =500;
            if(move_three_step()){
                log_info("[udock move test] success!");
            }
            else{
                log_info("[udock move test] fail!");
            }
        }
        if(access(dir_path1, F_OK) == 0){
            if(wheel_move_back_dist(1000.0f, 10.0f, 0)){
                log_info("[udock move test] success!");
            }
            else{
                log_info("[udock move test] fail!");
            }
        }
        if(access(dir_path2, F_OK) == 0){
            if(wheel_move_front_dist(1000.0f, 10.0f, 0)){
                log_info("[udock move test] success!");
            }
            else{
                log_info("[udock move test] fail!");
            }
        }
        usleep(200 *1000);
    }
}
/*-------------------------------------------
 * 轮式运动距离和转向控制
 *
---------------------------------------------*/
bool moveControl::wheel_move_angle(const float angle, const int tim)//degree second
{
    shm::OdomInfo odom_info;
    int         try_count   =1;
    timespec    time_begain;
    timespec    time_now;
    double      angle_rad   =angle /180 *M_PI;
    clock_gettime(CLOCK_REALTIME, &time_begain); 
    
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    
    while(1)
    {
        vel_cmd.x = 0.0;
        vel_cmd.z = angle;
        vel_cmd.priority = *((int32_t*)&owner);
        vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
        set_vel_cmd_pub_.publish(vel_cmd);
            
        shm::iMemory_read_OdomInfo(&odom_info);
        if(fabs(odom_info.speed_z - angle_rad) <0.035f){
            break;
        }
        usleep(1000 *100);
        clock_gettime(CLOCK_REALTIME, &time_now);
        if(time_now.tv_sec - time_begain.tv_sec >tim){
            shm::iMemory_read_OdomInfo(&odom_info);           
            log_info("[udock]%s timeout",__FUNCTION__);
            log_info("[udock]current steering angle:%f", odom_info.speed_z *180.0 /M_PI);
            if (try_count < 3)
            {
                log_info("[udock] retry turning steering ......");
                try_count++;
                for (int i = 0; i < 30; i++)
                {
                    vel_cmd.x = 0.0;
                    vel_cmd.z = 0.0;
                    vel_cmd.priority = *((int32_t*)&owner);
                    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
                    set_vel_cmd_pub_.publish(vel_cmd);
                    usleep(1000 *100);
                }
                clock_gettime(CLOCK_REALTIME, &time_begain); 
                continue;
            }
            return false;
        }
    }
    usleep(1000 *500);
    shm::iMemory_read_OdomInfo(&odom_info);
    double now_angle = odom_info.speed_z;
    log_info("[udock][erro degree] = dest: %f, cur: %f, erro: %f (degree)", angle_rad*180/M_PI, now_angle*180/M_PI, (now_angle - angle_rad)*180/M_PI);
    return true;
}

bool moveControl::wheel_move_back_dist(const float dist, const float speed, const float angle)//mm cm/s degree second
{
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    bool res = true;
    float spd = speed/100.0f; //---m/s
    
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo(&odom_info);
    
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = -fabs(dist/1000.0f) + odo_start;
    double angle_rad = angle /180 *M_PI;
    timespec time_begain;
    timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_begain);

    int time_out = (int)(fabs((dist/1000.0f) / (spd)));
    time_out +=15;
    int time_out_count = 0;
    log_info("[udock][start odom] %.2f", odo_cur*100);

    while(odo_cur > (dest +0.006f))
    {
        //time out detect
        clock_gettime(CLOCK_REALTIME, &time_now);
        if(time_now.tv_sec - time_begain.tv_sec >time_out){
            log_info("[udock]%s timeout fail",__FUNCTION__);
            return false;
        }
        //obstacle check
        if(!udock_wheel_obstacle->need_back_move_stop(angle_rad)){
            vel_cmd.x = -fabs(spd);
            vel_cmd.z = angle_rad;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }
        else{
            vel_cmd.x = 0.0;
            vel_cmd.z = angle_rad;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
            log_info("[udock state] need to stop because of abnormal lidar data or obstacle!");
            time_out +=1;
            time_out_count +=1;
            if(time_out_count >300){
                log_info("[udock]%s abnomal state timeout fail",__FUNCTION__);
                return false;
            }
            sleep(1);
        }

        if(!udock_manager->get_move_state()){
            res = false;
            log_info("[udock state]ancel");
            break;
        }
        usleep(30*1000);//33HZ
        shm::iMemory_read_OdomInfo(&odom_info);
        odo_cur = odom_info.dist_center;
        //        log_info("[udock] distance %.2f", odo_cur);
    }
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    shm::iMemory_read_OdomInfo(&odom_info);
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >max(5.0f*speed/DOCK_LINEAR_SPEED, 5.0f)){
        res = false;
    }
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    return res;
}

bool moveControl::wheel_move_back_dist_ignore_ob(const float dist, const float speed, const float angle)//mm cm/s degree second
{
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    bool res = true;
    float spd = speed/100.0f; //---m/s
    
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo(&odom_info);
    
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = -fabs(dist/1000.0f) + odo_start;
    double angle_rad = angle /180 *M_PI;
    timespec time_begain;
    timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_begain);

    int time_out = (int)(fabs((dist/1000.0f) / (spd)));
    time_out +=15;
    int time_out_count = 0;
    log_info("[udock][start odom] %.2f", odo_cur*100);

    while(odo_cur > (dest +0.006f))
    {
        //time out detect
        clock_gettime(CLOCK_REALTIME, &time_now);
        if(time_now.tv_sec - time_begain.tv_sec >time_out){
            log_info("[udock]%s timeout fail",__FUNCTION__);
            return false;
        }
        //obstacle check
        if(1){
            vel_cmd.x = -fabs(spd);
            vel_cmd.z = angle_rad;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }

        if(!udock_manager->get_move_state()){
            res = false;
            log_info("[udock state]ancel");
            break;
        }
        usleep(30*1000);//33HZ
        shm::iMemory_read_OdomInfo(&odom_info);
        odo_cur = odom_info.dist_center;
        //        log_info("[udock] distance %.2f", odo_cur);
    }
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
    usleep(500*1000);
    shm::iMemory_read_OdomInfo(&odom_info);
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >max(5.0f*speed/DOCK_LINEAR_SPEED, 5.0f)){
        res = false;
    }
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    return res;
}



bool moveControl::wheel_move_front_dist(const float dist, const float speed, const float angle)//mm cm/s degree second
{
    atris_msgs::VelCmd vel_cmd;
    Control_Owner owner = RECHARGE;
    bool res = true;
    float spd = speed/100.0f;
    
    shm::OdomInfo odom_info;
    shm::iMemory_read_OdomInfo(&odom_info);
    
    double odo_start = odom_info.dist_center;
    double odo_cur = odom_info.dist_center;
    double dest = fabs(dist/1000.0f) + odo_start;
    double angle_rad = angle /180 *M_PI;
    timespec time_begain;
    timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_begain);

    int time_out = (int)(fabs((dist/1000.0f) / (spd)));
    time_out +=15;
    int time_out_count = 0;
    log_info("[udock][start odom] %.2f", odo_cur*100);

    while(odo_cur < (dest -0.006f))
    {
        clock_gettime(CLOCK_REALTIME, &time_now);
        if(time_now.tv_sec - time_begain.tv_sec >time_out){
            log_info("%s timeout fail",__FUNCTION__);
            return false;
        }

        if(!udock_wheel_obstacle->need_front_move_stop(angle_rad)){
            vel_cmd.x = fabs(spd);
            vel_cmd.z = angle_rad;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
        }
        else{
            vel_cmd.x = 0.0;
            vel_cmd.z = angle_rad;
            vel_cmd.priority = *((int32_t*)&owner);
            vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
            set_vel_cmd_pub_.publish(vel_cmd);
            
            log_info("[udock state] need to stop because of abnormal lidar data or obstacle!");
            time_out +=1;
            time_out_count +=1;
            if(time_out_count >300){
                log_info("[udock]%s abnomal state timeout fail",__FUNCTION__);
                return false;
            }
            sleep(1);
        }
        if(!udock_manager->get_move_state()){
            res = false;
            log_info("[state]ancel");
            break;
        }
        usleep(30*1000);//33HZ
        shm::iMemory_read_OdomInfo(&odom_info);
        odo_cur = odom_info.dist_center;
        //        log_info("[udock] distance %.2f", odo_cur);
    }
    
    vel_cmd.x = 0.0;
    vel_cmd.z = 0.0;
    vel_cmd.priority = *((int32_t*)&owner);
    vel_cmd.timestamp = (int64_t)(ros::Time::now().toSec() * 1000);
    set_vel_cmd_pub_.publish(vel_cmd);
    set_vel_cmd_pub_.publish(vel_cmd);
            
    usleep(500*1000);
    shm::iMemory_read_OdomInfo(&odom_info);
    odo_cur = odom_info.dist_center;
    float erro = fabs((odo_cur - dest)*100.0f);
    if(erro >max(5.0f*speed/DOCK_LINEAR_SPEED, 5.0f)){
        res = false;
    }
    log_info("[udock][erro distance cm] = dest: %f, cur: %f, erro: %f (cm)", dest*100.0f, odo_cur*100.0f, (odo_cur - dest)*100.0f);
    return res;
}
/*-------------------------------------------
 * 运动规划
 *
---------------------------------------------*/
bool moveControl::get_the_path(robotPose local, robotPose dest)
{
    bool res =false;
    float x1 =dest.x,           y1 =dest.y;
    float x2 =local.x,          y2 =local.y;
    float r1 =1.3342f,          r2 =1.3342f;
    float ang = local.theta;
    memset(&move_info, 0, sizeof(move_info));
    dockPathPlanning dock_path(x1, y1, r1, x2, y2, r2, ang);
    if(dock_path.is_have_a_solution()){
        if(dock_path.find_a_right_path(&move_info)){
            if((move_info.step_one_dist >0 && move_info.step_two_dist >0) && (move_info.step_three_dist >0)){
                res =true;
                log_info("[udock] find right solution");
            }
            else{
                log_info("[udock] find erro solution");
            }
        }
    }
    return res;
}

bool moveControl::leave_from_dock(const float dist, const int speed) //use
{
    int time_t =0;
    LaserRaw data;
    while(true)
    {
        if(udock_data->get_lidar_data_assistant(data)){
            break;
        }
        else{
            sleep(2);
            time_t +=2;
            if(time_t >= 200){
                log_info("[udock] %s leave from dock fail!",__FUNCTION__);
                return false;
            }
            log_info("[udock] %s leave wait for lidar data!",__FUNCTION__);
            continue;
        }
    }
    log_info("[udock] %s leave from dock!",__FUNCTION__);
    if(!wheel_move_angle(0, 10)){
        return false;
    }
    if(!wheel_move_front_dist(dist*10.0f, speed, 0.0f)){
        return false;
    }
    return true;
}
/*-------------------------------------------
 * 向前运动到开始位置
 *
---------------------------------------------*/
bool moveControl::move_back_to_start_point(void)
{
    log_info("[udock] %s!",__FUNCTION__);
    if(!wheel_move_angle(0, 10)){
        return false;
    }
    if(!wheel_move_front_dist(900.0f, DOCK_LINEAR_SPEED*2, 0.0f)){//50 seconds
        return false;
    }
    return true;
}
/*-------------------------------------------
 * 三步运动
 *
---------------------------------------------*/
bool moveControl::move_three_step(void)
{
    log_info("[udock] %s!",__FUNCTION__);
    float keep_angle;

    //rotate and move 1
    log_info("[udock] movestep1");
    if(move_info.step_one_dist >=20.0f){
        if(move_info.step_one_steering == STEERING_LEFT){
            keep_angle = MOVE_ANGLE_RADIUS;
            if(!wheel_move_angle(MOVE_ANGLE_RADIUS, 10)){
                return false;
            }
        }
        else{
            keep_angle = -MOVE_ANGLE_RADIUS;
            if(!wheel_move_angle(-MOVE_ANGLE_RADIUS, 10)){
                return false;
            }
        }

        if(!wheel_move_back_dist(move_info.step_one_dist, DOCK_LINEAR_SPEED, keep_angle)){
            return false;
        }
    }
    else{
        log_info("[udock] movestep1 ignor small move");
    }

    //rotate and move 2
    log_info("[udock] movestep2");
    if(!wheel_move_angle(0, 10)){
        return false;
    }
    if(move_info.step_two_dist >0){
        if(!wheel_move_back_dist(move_info.step_two_dist, DOCK_LINEAR_SPEED, 0)){
            return false;
        }
    }
    else{
        return false;
    }

    //rotate and move 3
    log_info("[udock] movestep3");
    if(move_info.step_three_dist >=10.0f){
        if(move_info.step_three_steering == STEERING_LEFT){
            keep_angle = MOVE_ANGLE_RADIUS;
            if(!wheel_move_angle(MOVE_ANGLE_RADIUS, 10)){
                return false;
            }
        }
        else{
            keep_angle = -MOVE_ANGLE_RADIUS;
            if(!wheel_move_angle(-MOVE_ANGLE_RADIUS, 10)){
                return false;
            }
        }
        if(!wheel_move_back_dist(move_info.step_three_dist, DOCK_LINEAR_SPEED, keep_angle)){
//            udock_manager->power_manager->sendTtstext(TTSStrings::TTS_KEY_MOVE_FAIL);
            return false;
        }
    }
    return true;
}
/*-------------------------------------------
 * 姿态摆正
 *
---------------------------------------------*/
bool moveControl::goto_right_direction(void)
{
    log_info("[udock] %s!",__FUNCTION__);
    robotPose local, dest;
    dest.x = 0;
    if (Config::get_instance()->dock_1_0 == 1)
    {
        dest.y = POINT3_DISTANCE - DOCK_OFFSET;
    }
    else
    {
        dest.y = POINT3_DISTANCE;
    }
    dest.theta = 0;
    if(!udock_manager->localizition(&local, 30, 5)){
        return false;
    }
    robotPose temp_dest = dest;
    temp_dest.x = local.x;
    temp_dest.theta = 0;

    uint16_t i =0;
    while(1)
    {
        if(get_the_path(local, temp_dest)){
            break;
        }
        else{
            if(local.theta >0){
                temp_dest.x += DEST_PLUS;
            }
            else{
                temp_dest.x -= DEST_PLUS;
            }
        }
        i++;
        if(i >8){
            return false;
        }
    }
    //have solution begain move
    if(!move_three_step()){
        return false;
    }
    //move to start point
    if(!move_back_to_start_point()){
        return false;
    }
    log_info("[udock] inderect solution!");
    return true;
}
/*-------------------------------------------
 * 运动到中间位置
 *
---------------------------------------------*/
bool moveControl::goto_temp_destination(robotPose local, robotPose dest)
{
    log_info("[udock] %s!",__FUNCTION__);
    robotPose temp_dest = dest;
    //get now pose
    uint16_t i =0;
    while(1)
    {
        if(get_the_path(local, temp_dest)){
            break;
        }
        else{
            if(local.x >0){
                temp_dest.x += DEST_PLUS;
            }
            else{
                temp_dest.x -= DEST_PLUS;
            }
        }
        i++;
        if(i >10){
            return false;
        }
    }
    log_info("[udock] try move! x = %f", temp_dest.x);
    //have solution begain move
    if(!move_three_step()){
        return false;
    }

    return true;
}

bool moveControl::goto_point2(void)
{
    robotPose local, dest;
    dest.x = 0;
    if (Config::get_instance()->dock_1_0 == 1)
    {
        dest.y = POINT3_DISTANCE - DOCK_OFFSET;
    }
    else
    {
        dest.y = POINT3_DISTANCE;
    }
    

    dest.theta = 0;
    log_info("[udock] %s!",__FUNCTION__);
    //尝试运动到目标位姿
    if(!goto_right_direction()){
        return false;
    }

    int try_count = 0;
    while(1)
    {
        try_count++;
        if(try_count >8){
            return false;
        }
        if(!udock_manager->localizition(&local, 40, 5)){
            continue;
        }

        if(!goto_temp_destination(local, dest)){
            return false;
        }
        //localizition
        if(udock_manager->localizition(&local, 60, 15)){
            if((fabs(local.x) < 0.05f) && fabs(local.theta) < 2.5f && fabs(local.y - dest.y) <10.0f){
                break;
            }
            if(!move_back_to_start_point()){
                return false;
            }
        }
        else{
            return false;
        }
    }
    return true;
}
/*-------------------------------------------
 * 对桩
 *
---------------------------------------------*/
bool moveControl::goto_last_point(void)
{
    float dist;
    robotPose local;
    log_info("[udock] %s!",__FUNCTION__);
    if(!udock_manager->localizition(&local, 60, 15)){
        return false;
    }
    if(Config::get_instance()->dock_1_0 == 1){
        dist = (local.y - LAST_MOVE_DISTANCE + 0.6)*1000.0f; 
    }
    else{
        dist = (local.y - LAST_MOVE_DISTANCE)*1000.0f; //mm
    }

    return wheel_move_back_dist_ignore_ob(dist, 3, 0);
}


#endif
