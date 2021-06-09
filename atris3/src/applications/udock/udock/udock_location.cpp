/*
    Udock algrithm by xiangbin.huang
*/

#include "udock_location.h"
#include <string>
#include <math.h>
#include "Eigen/Dense"
#include "log/log.h"



using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

UdockTank::UdockTank()
{
    config = Config::get_instance();
    if(200 <= config->dock_wall_width && config->dock_wall_width <=300){
        wall_width_ = config->dock_wall_width /100.0f;

    }
    else{
        wall_width_ = DEFAULT_LINE_DISTANCE;
        log_info("[udock wall width use default]  = %f", wall_width_);
    }
    wall_width_confirmed_ = false;
    log_info("[udock wall width is]  = %f", wall_width_);

    // udock  1.1
    location_1_ = UdockLocation::get_instance();

}

UdockTank::~UdockTank()
{

}

float UdockTank::get_lidar_wall_distance(void)
{
    return distance_lidar_wall_;
}

float UdockTank::get_distance(void)
{
    return point2_dis_;
}

float UdockTank::get_angle(void)
{
    return point2_angle_;
}

float UdockTank::get_yposition(void)
{
    return trans_y_;
}

float UdockTank::get_xposition(void)
{
    return trans_x_;
}

float UdockTank::get_line_angle(void)
{
    return line_angle_;
}

void UdockTank::get_dest_point(float *x, float *y)
{
    *x = point2_x_;
    *y = point2_y_;
}

void UdockTank::get_center_point(float *x, float *y)
{
    *x = shadow_point_mid_x;
    *y = shadow_point_mid_y;
}

void UdockTank::get_pose(float *x, float *y)
{
    *x = trans_x_;
    *y = trans_y_;
}

float UdockTank::line_angle_to_axisx(float line_angle)
{
    if((90< line_angle) && (line_angle <=180)){         //Beta Quadrant
        line_angle = -180 + line_angle;
    }
    else if((-180< line_angle) && (line_angle <=-90)){  //third quadrant
        line_angle =  180 + line_angle;
    }
    return line_angle;
}

float UdockTank::line_to_line_angle(float line_angle1, float line_angle2)
{
    float line_angle;
    line_angle = fabs(line_angle1 - line_angle2);
    if(line_angle > 90.0){
        line_angle = 180 - line_angle;
    }
    return line_angle;
}

float UdockTank::v1_trans_v2_angle(float v1_x, float v1_y, float v2_x, float v2_y)
{
    float d1 = sqrt(v1_x * v1_x + v1_y * v1_y);
    float d2 = sqrt(v2_x * v2_x + v2_y * v2_y);
    v1_x = v1_x/d1;
    v1_y = v1_y/d1;
    v2_x = v2_x/d2;
    v2_y = v2_y/d2;
    float tmp = v2_x * v2_x + v2_y * v2_y;
    float sin_theta = (v2_y * v1_x - v1_y * v2_x)/tmp;
    float cos_theta = (v1_x * v2_x + v1_y * v2_y)/tmp;
    return atan2(sin_theta, cos_theta);
}

//投影点
void UdockTank::point_shadowin_line(float x0, float y0, float a, float b, float *s_x, float *s_y, float *dis)
{
    float x, y;
    *s_x = (x0 + a * y0 - a * b )/(1 + a * a);
    *s_y = (a * x0 + a *a * y0 + b )/(1 + a * a);
    x = *s_x;
    y = *s_y;
    *dis = sqrt((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y));
}

//雷达点坐标转换
bool UdockTank::axis_transform(LaserRaw *data)
{
    double angle_start = data->angle_min + LIDAR_OFFSET*M_PI/180.0f;
    point_num_ = data->range_size;

    //udock 1.1
    if (config->dock_1_0 == 1)
        point_xy_.point_size = point_num_;

//    log_debug("angle min [%f]!\r\n", data->angle_min);
//    log_debug("angle max [%f]!\r\n", data->angle_max);
//    log_debug("angle inc [%f]!\r\n", data->angle_increment);
//    log_debug("lidar point [%d]!", data->range_size);
    if(data->range_size != LIDAR_TOTAL_NUM_VELODYNE){
        log_error("[ udock ]!!!!!!erro lidar point number! receive num[%d] right num[%d]", data->range_size ,LIDAR_TOTAL_NUM_VELODYNE);
        return false;
    }
    point_xy_.angle_increase = data->angle_increment;
    for(unsigned int i =0; i < data->range_size; i++){

        if(data->laser_range[i] <10.0){
            point_x[i] = data->laser_range[i] * sin(angle_start);
            point_y[i] =  -(data->laser_range[i] * cos(angle_start));
        }
        else{
            point_x[i] = 0;
            point_x[i] = 0;
        }

    //udock 1.1
    if (config->dock_1_0 == 1){
        point_xy_.x[i] =point_x[i];
        point_xy_.y[i] =point_y[i];
    }
        angle_start += data->angle_increment;
//        log_info("[point] = %f %f", point_x[i], point_y[i]);
    }
    log_info("[ udock ] %s", __FUNCTION__);
    return true;
}


//对准误差
float UdockTank::caculate_last_angle(void) //degree
{
    float ans =0.0f;
//    float c =0.0f, a =0.0f;

//    a = fabs(trans_y_);
//    c = fabs(line_b_) + LIDAR_INSTALL_POSITION;
//    if(a <=c){
//        ans = acos(a/c) *180.0f /M_PI;
//        if(trans_parameter_.parameter1[2] > trans_x_){
//            ans = -ans;
//        }
//    }
//    else{
//        log_info("[udock last angle erro caculate a > c]");
//    }
//    log_info("[udock last angle a c angle]  =%f, %f, %f", a, c, ans);

    ans = (atan(line_k_)) *180.0f /M_PI;
    return ans;
}

//数据有效性
bool UdockTank::is_data_valid(const float *x, const float *y, uint16_t len)
{
    for(uint16_t i =0; i< len-1; i++){
        float dis = sqrt((x[i+1]-x[i])*(x[i+1]-x[i]) + (y[i+1]-y[i])*(y[i+1]-y[i]));
        if(dis >1.0f){ //1m
            log_info("[udock] %s.",__FUNCTION__);
            log_info("[udock --erro--lidar--data-----]");
            for(uint16_t j =0; j <len; j++){
                log_info("[udock normal lidar data] = %f %f", x[i], y[i]);
            }
            return false;
        }
    }
    return true;
}

//
void UdockTank::least_squre_method(float *x, float *y, int size, float *k, float *b)
{
    MatrixXd Y_MAT(size, 1);
    MatrixXd W_MAT(size, 2);
    MatrixXd WT_MAT(2, size);
    MatrixXd R_MAT(2, 1);
    MatrixXd WTW_MAT(2, 2);
    MatrixXd WTW_INV_MAT(2, 2);

    for(int i =0; i <size; i++){
        Y_MAT(i, 0) = y[i];
        W_MAT(i, 0) = x[i];
        W_MAT(i, 1) = 1;
    }
    WT_MAT = W_MAT.transpose();
    WTW_MAT = WT_MAT * W_MAT;
    WTW_INV_MAT = WTW_MAT.inverse();
    R_MAT = WTW_INV_MAT * WT_MAT * Y_MAT;
    *k = R_MAT(0, 0);
    *b = R_MAT(1, 0);
}

//
bool UdockTank::find_wall_angle(float *angle, int point_num)
{
    float line_k, line_b;
    int lidar_normal_start, lidar_normal_end;
    float wall_angle;
    lidar_normal_start = LIDAR_NORMAL_MID - point_num/2;
    lidar_normal_end = LIDAR_NORMAL_MID + point_num/2;

    if(!is_data_valid(&point_x[lidar_normal_start], &point_y[lidar_normal_start], lidar_normal_end-lidar_normal_start)){
        return false;
    }

//    for(int i =lidar_normal_start; i <lidar_normal_end; i++){
//        log_info("[udock lidar data] = %f %f", point_x[i], point_y[i]);
//    }

    least_squre_method(&point_x[lidar_normal_start], &point_y[lidar_normal_start],
        (lidar_normal_end - lidar_normal_start), &line_k, &line_b);
    wall_angle = atan2(line_k, 1) / M_PI *180.0; //-90 -> +90

    *angle = wall_angle;
    log_info("[udock normal angle]  = %f", *angle);
    if(-30.0f < wall_angle && wall_angle < 30.0f){
        return true;
    }
    return false;
}

void UdockTank::transform_to_dock(float inx, float iny, float *outx, float *outy)
{
    *outx = trans_parameter_.parameter1[0] *inx +trans_parameter_.parameter1[1] *iny +trans_parameter_.parameter1[2];
    *outy = trans_parameter_.parameter2[0] *inx +trans_parameter_.parameter2[1] *iny +trans_parameter_.parameter2[2];
}


void UdockTank::find_point_by_angle(void)
{
    float line_angle, dy, dx, line_angle_temp;
    float erro;
    int i;
    int d_point = 2;

    for(i =LIDAR_NORMAL_START;  i >0; i -=d_point){
        dy = point_y[i] - point_y[LIDAR_NORMAL_MID];
        dx = point_x[i] - point_x[LIDAR_NORMAL_MID];
        line_angle_temp = atan2(dy, dx) * 180.0 /M_PI;           //-180 -> 180
        line_angle = line_angle_to_axisx(line_angle_temp);       //-90 -> 90
//        log_info("[start] = %f %f", line_angle_temp, line_angle);
        erro = line_to_line_angle(line_angle, normal_angle_);

        if(fabs(erro) > SIMPLE_ERRO_MAX){
            break;
        }
    }
    simple_point_start = i + d_point;


    for(i =LIDAR_NORMAL_END;  i <point_num_; i +=d_point){
        dy = point_y[i] - point_y[LIDAR_NORMAL_MID];
        dx = point_x[i] - point_x[LIDAR_NORMAL_MID];
        line_angle_temp = atan2(dy, dx) * 180.0 /M_PI;
        line_angle = line_angle_to_axisx(line_angle_temp);
//        log_info("[end] = %f %f", line_angle_temp, line_angle);
        erro = line_to_line_angle(line_angle, normal_angle_);

        if(fabs(erro) > SIMPLE_ERRO_MAX){
            break;
        }
    }
    simple_point_end = i - d_point;
//    log_info("[udock normal point] = %d %d", simple_point_start, simple_point_end);
//    for(int i =simple_point_start; i <simple_point_end; i++){
//        log_debug("[udock normal lidar data] = %f %f", point_x[i], point_y[i]);
//    }

}

//
void UdockTank::find_point_precise(void)
{
    int i; int d_point = 5;
    float line_angle, dy, dx;
    float erro;
    for(i =simple_point_start; i <LIDAR_NORMAL_START; i+=d_point){
        dy = point_y[i] - point_y[i-d_point];
        dx = point_x[i] - point_x[i-d_point];
        line_angle = atan2(dy, dx) * 180.0 /M_PI;

        line_angle = line_angle_to_axisx(line_angle);
        erro = line_to_line_angle(line_angle, normal_angle_);

        if(fabs(erro) < PRECISE_ERRO_MAX){
            break;
        }
    }
    precise_point_start = i;

    for(i =simple_point_end; i >LIDAR_NORMAL_END; i-=d_point){
        dy = point_y[i] - point_y[i-d_point];
        dx = point_x[i] - point_x[i-d_point];
        line_angle = atan2(dy, dx) * 180.0 /M_PI;

        line_angle = line_angle_to_axisx(line_angle);

        erro = line_to_line_angle(line_angle, normal_angle_);

        if(fabs(erro) < PRECISE_ERRO_MAX){
            break;
        }
    }
    precise_point_end = i;
//    log_info("[udock precise point] = %d %d", precise_point_start, precise_point_end);
//    for(int i =precise_point_start; i <precise_point_end; i++){
//        log_debug("[udock precise lidar data] = %f %f", point_x[i], point_y[i]);
//    }

}

//
void UdockTank::lines_angle(void)
{
//修改坐标系
//    float dy = shadow_point_mid_y - point2_y_;
//    float dx = shadow_point_mid_x - point2_x_;

    float dx = -(shadow_point_mid_y - point2_y_);
    float dy = shadow_point_mid_x - point2_x_;

    float angle1 = atan2(dy, dx) * 180 / M_PI;
    float angle2 = point2_angle_;
    if(angle1 < 0){
        angle1 = 180 + angle1;
    }
    if(angle2 < 0){
        angle2 = 180 + angle2;
    }
    float erro = fabs(angle1 - angle2);
    if(erro > 90.0){
        erro = 180 - erro;
    }
    line_angle_ = erro;
}

//
void UdockTank::point_distance(float point2_d)
{
    float x0 = shadow_point_mid_x;
    float y0 = shadow_point_mid_y;
    float k = line_k_;
    float b = line_b_;
    float point2_x1, point2_y1, point2_x2, point2_y2;
    float dis1, dis2;
    log_info("[ udock ][prameter] = k:%f d:%f x0:%f y0:%f", k, point2_d, x0, y0);

    point2_x1 = x0 - point2_d * k * sqrt(1/(1 + k * k));
    point2_y1 = y0 + point2_d * sqrt(1/(1 + k * k));

    point2_x2 = x0 + point2_d * k * sqrt(1/(1 + k * k));
    point2_y2 = y0 - point2_d * sqrt(1/(1 + k * k));

    dis1 = point2_x1 * point2_x1 + point2_y1 * point2_y1;
    dis2 = point2_x2 * point2_x2 + point2_y2 * point2_y2;

    if(dis1 < dis2){
        point2_x_ = point2_x1;
        point2_y_ = point2_y1;
    }
    else{
        point2_x_ = point2_x2;
        point2_y_ = point2_y2;
    }
//    log_info("[dest point] = %f %f", point2_x_, point2_y_);
    point2_dis_ = sqrt((point2_x_ * point2_x_) + (point2_y_ - LIDAR_POSITION) * (point2_y_ - LIDAR_POSITION));

    float d_point2_x = -(point2_y_ - LIDAR_POSITION);
    float d_point2_y =  point2_x_;
    point2_angle_ = atan2(d_point2_y, d_point2_x) * 180.0 / M_PI;
    //center point
    transform_to_dock(0, LIDAR_POSITION, &trans_x_, &trans_y_);
    log_info("[ udock ][location_x, location_y] = %f %f", trans_x_, trans_y_);
    //move point
    transform_to_dock(point2_x_, point2_y_, &point2_trans_x_, &point2_trans_y_);
    log_info("[ udock ][destination_x, destination_y] = %f %f", point2_trans_x_, point2_trans_y_);
    //normal laser point
    log_info("[normal point] = %d %d", simple_point_start, simple_point_end);
    log_info("[ udock ]-------------------------------------");
    for(int i =simple_point_start; i <simple_point_end; i++){
        float x, y;
        transform_to_dock(point_x[i], point_y[i], &x, &y);
        log_debug("[normal lidar data] = %f %f", x, y);
    }
    //precise laser point
    log_info("[ udock ]-------------------------------------");
    log_info("[precise point] = %d %d", precise_point_start, precise_point_end);
    for(int i =precise_point_start; i <precise_point_end; i++){
        float x, y;
        transform_to_dock(point_x[i], point_y[i], &x, &y);
        log_debug("[precise lidar data] = %f %f", x, y);
    }
    log_info("[ udock ]-------------------------------------");
}


//
bool UdockTank::caculate_distance(int num)
{
    float line_k, line_b, distance;
    float x0, y0;
    int size = precise_point_end - precise_point_start;
    if(size < 30){
        log_info("size < 30 erro!");
        return false;
    }

    least_squre_method(&point_x[precise_point_start], &point_y[precise_point_start],
    size, &line_k, &line_b);
    log_info("[ udock ][wall equation] = %f %f", line_k, line_b);

    shadow_point_start_x = 0;
    shadow_point_start_y = 0;
    for(int i =0; i< num; i++){
        x0 = point_x[simple_point_start + i];
        y0 = point_y[simple_point_start + i];
        float x_sha, y_sha;
        point_shadowin_line(x0, y0, line_k, line_b, &x_sha, &y_sha, &distance);
        shadow_point_start_x += x_sha;
        shadow_point_start_y += y_sha;
    }
    shadow_point_start_x = shadow_point_start_x/num;
    shadow_point_start_y = shadow_point_start_y/num;
    log_info("[ udock ][down location] = %f %f", shadow_point_start_x, shadow_point_start_y);

    shadow_point_end_x = 0;
    shadow_point_end_y = 0;
    for(int i =0; i <num; i++){
        x0 = point_x[simple_point_end -i];
        y0 = point_y[simple_point_end -i];
        float x_sha, y_sha;
        point_shadowin_line(x0, y0, line_k, line_b, &x_sha, &y_sha, &distance);
        shadow_point_end_x += x_sha;
        shadow_point_end_y += y_sha;
    }
    shadow_point_end_x = shadow_point_end_x/num;
    shadow_point_end_y = shadow_point_end_y/num;
    log_info("[ udock ][up location] = %f %f", shadow_point_end_x, shadow_point_end_y);


    //计算投影
    shadow_point_mid_x = (shadow_point_start_x + shadow_point_end_x)/2;
    shadow_point_mid_y = (shadow_point_start_y + shadow_point_end_y)/2;
    log_info("[ udock ][center location] = %f %f", shadow_point_mid_x, shadow_point_mid_y);

    line_k_ = line_k;
    line_b_ = line_b;

    //计算坐标变换矩阵
    trans_angle_ = atan2((shadow_point_end_y - shadow_point_mid_y), (shadow_point_end_x - shadow_point_mid_x));
    float sin_a = sin(trans_angle_);
    float cos_a = cos(trans_angle_);
    trans_parameter_.parameter1[0] =  cos_a;
    trans_parameter_.parameter1[1] =  sin_a;
    trans_parameter_.parameter1[2] = -shadow_point_mid_x *cos_a -shadow_point_mid_y *sin_a;
    trans_parameter_.parameter2[0] = -sin_a;
    trans_parameter_.parameter2[1] =  cos_a;
    trans_parameter_.parameter2[2] =  shadow_point_mid_x *sin_a -shadow_point_mid_y *cos_a;

    //计算墙的宽度
    distance =  sqrt((shadow_point_end_y - shadow_point_start_y)*(shadow_point_end_y - shadow_point_start_y) 
    + (shadow_point_end_x - shadow_point_start_x)*(shadow_point_end_x - shadow_point_start_x));

    log_info("[ udock ][wall width] = %f", distance);
    if (wall_width_confirmed_)
    {
        if((wall_width_ - ERRO_DISTANCE < distance) && (distance < wall_width_ + ERRO_DISTANCE)){
            distance_wall_ = distance;
            distance_lidar_wall_ = sqrt(shadow_point_mid_x * shadow_point_mid_x + shadow_point_mid_y * shadow_point_mid_y);
            log_info("[ udock ][lidar to wall distance] = %f", distance_lidar_wall_);
            return true;
        }
    }
    else
    {
        if ((wall_width_ - ERRO_DISTANCE_FIRST_STAGE < distance) && (distance < wall_width_ + ERRO_DISTANCE_FIRST_STAGE))
        {
            distance_wall_ = distance;
            wall_width_ = distance;
            wall_width_confirmed_ = true;
            log_info("[ udock ] [wall width setting] now changed to %f", wall_width_);
            distance_lidar_wall_ = sqrt(shadow_point_mid_x * shadow_point_mid_x + shadow_point_mid_y * shadow_point_mid_y);
            log_info("[ udock ][lidar to wall distance] = %f", distance_lidar_wall_);
            return true;
        }
    }
    return false;
}


bool UdockTank::caculate_distance(void)
{
    if(!location_1_->find_dock(point_xy_))
    {
        return false;
    }

    line_k_ = location_1_->k_;
    line_b_ = location_1_->b_;

    shadow_point_start_x = location_1_->cluster_average_shadow[0].x;
    shadow_point_start_y = location_1_->cluster_average_shadow[0].y;   

    shadow_point_end_x = location_1_->cluster_average_shadow[2].x;
    shadow_point_end_y = location_1_->cluster_average_shadow[2].y;

    shadow_point_mid_x = location_1_->cluster_average_shadow[1].x;
    shadow_point_mid_y = location_1_->cluster_average_shadow[1].y;

    shadow_point_mid_x =  (shadow_point_start_x + shadow_point_end_x + shadow_point_mid_x * 2.0) / 4.0;
    shadow_point_mid_y =  (shadow_point_start_y + shadow_point_end_y + shadow_point_mid_y * 2.0) / 4.0;

    //计算坐标变换矩阵
    trans_angle_ = atan2((shadow_point_end_y - shadow_point_mid_y), (shadow_point_end_x - shadow_point_mid_x));
    float sin_a = sin(trans_angle_);
    float cos_a = cos(trans_angle_);
    trans_parameter_.parameter1[0] =  cos_a;
    trans_parameter_.parameter1[1] =  sin_a;
    trans_parameter_.parameter1[2] = -shadow_point_mid_x *cos_a -shadow_point_mid_y *sin_a;
    trans_parameter_.parameter2[0] = -sin_a;
    trans_parameter_.parameter2[1] =  cos_a;
    trans_parameter_.parameter2[2] =  shadow_point_mid_x *sin_a -shadow_point_mid_y *cos_a;

    distance_lidar_wall_ = sqrt(shadow_point_mid_x * shadow_point_mid_x + shadow_point_mid_y * shadow_point_mid_y);

    return true;
}


//
bool UdockTank::udock_caculate(LaserRaw *data, int point_num, int showd_num)
{
    if (axis_transform(data))
    {
        //udock 1.1
        if (config->dock_1_0 == 1)
        {
            if (caculate_distance())
            {
                return true;
            }
        }

        else
        {
            if (find_wall_angle(&normal_angle_, point_num))
            {
                find_point_by_angle();
                find_point_precise();
                if (caculate_distance(showd_num))
                {
                    return true;
                }
            }
            log_info("[udock normal point] = %d %d", simple_point_start, simple_point_end);
            log_info("[udock precis point] = %d %d", precise_point_start, precise_point_end);
        }
        //print erro lidar data
        log_info("[udock erro lidar data--------------------]");
        for(int i =0; i <point_num_; i++){
            log_info("[udock lidar data] = %f %f", point_x[i], point_y[i]);
        }
    }
    return false;
}

void UdockTank::reset_wall_width()
{
    if (!wall_width_confirmed_) return;
    if(200 <= config->dock_wall_width && config->dock_wall_width <=300){
        wall_width_ = config->dock_wall_width /100.0f;
    }
    else{
        wall_width_ = DEFAULT_LINE_DISTANCE;
    }
    wall_width_confirmed_ = false;
}


/*
    END file Udock algrithm by xiangbin.huang
*/
