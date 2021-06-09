/*
    Udock by xiangbin.huang
*/
#ifndef __UDOCK_PLANNING__
#define __UDOCK_PLANNING__

#include <iostream>
#include <stdio.h>
#include <math.h>
#include "Eigen/Dense"
#include <vector>
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/VelCmd.h"
#include "udock_common.h"

using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

typedef Matrix<int, Dynamic, Dynamic> MatrixXdInt;
typedef Matrix<int, Dynamic, 1> MatrixXEInt;
typedef Matrix<int, 1, Dynamic> MatrixXFInt;
//-------------------------------------------
typedef struct robotPose_{
    float x; //m
    float y; //m
    float theta;//degree
    float lidar_to_wall;
}robotPose;

typedef enum directionDef_{
    STEERING_LEFT = 0,
    STEERING_RIGHT =1
}directionDef;

typedef struct threeStepMove_{
    directionDef    step_one_steering;
    float   step_one_dist;
    float   step_two_dist;
    directionDef    step_three_steering;
    float   step_three_dist;
}threeStepMove;


//-------------------------------------------
class findPath{

public:
     findPath(){}
     ~findPath(){}
    int find_all_Path(MatrixXdInt &matr, int start_enm, int end_num)
    {
        if(start_enm !=0){
            matrix_transform(matr, start_enm);
            find_all_Path(matris_adjacency_, end_num);
            for(size_t i=0; i<path_all_.size(); i++){
                vector<int> &point = path_all_[i];
                for(size_t j =1; j<point.size(); j++){
                    if(point[j] == start_enm){
                        point[j] = 0;
                    }
                }
                point[0] = start_enm;
            }
        }
        else{
            find_all_Path(matr, end_num);
        }
        return path_all_.size();
    }

    vector< vector<int> > path_all_;
    vector<int> path_dis_;

private:
    void matrix_transform(MatrixXdInt &matr, int start_num)
    {
        matris_adjacency_ = matr;

        MatrixXFInt row0 = matris_adjacency_.row(0);
        MatrixXFInt rown = matris_adjacency_.row(start_num);
        matris_adjacency_.row(0) = rown;
        matris_adjacency_.row(start_num) = row0;

        MatrixXEInt col0 = matris_adjacency_.col(0);
        MatrixXEInt coln = matris_adjacency_.col(start_num);
        matris_adjacency_.col(0) = coln;
        matris_adjacency_.col(start_num) = col0;
    }

    int find_all_Path(MatrixXdInt &matr, int end_num)
    {
        //check
        if(matr.cols() != matr.rows()){
            return 0;
        }
        book.clear();
        record.clear();
        path_all_.clear();
        path_dis_.clear();
        for(int i=0; i<matr.rows(); i++){
            book.push_back(0);
        }
        book[0] =1;
        record.push_back(0);
        find_a_Path(matr, 0, end_num);
        return path_all_.size();
    }

    void find_a_Path(MatrixXdInt &matr, int current, int end)
    {
        if(current == end){  //arrive end
            vector<int> path_tep;
            for(size_t i=0; i<record.size(); i++){
                path_tep.push_back(record[i]);
            }
            path_all_.push_back(path_tep);
            int dist = 0;
            for(size_t i=0; i<record.size()-1; i++){
                int m = record[0+i];
                int n = record[1+i];
                dist += matr(m, n);
            }
            path_dis_.push_back(dist);
            return;
        }
        for(size_t j=0; j<book.size(); j++){
            if(matr(current, j) != 0 && book[j] ==0){
                record.push_back(j);
                book[j] = 1;
                find_a_Path(matr, j, end);
                book[j] = 0;
                record.pop_back();
            }
        }
        return;
    }

    vector<int> record;
    vector<int> book;
    MatrixXdInt matris_adjacency_;
};

//-------------------------------------------
class findShortWay{

public:
    findShortWay(){
    }
    ~findShortWay(){
    }

    float power(float x)
    {
        return x * x;
    }

    void caculate_tangent(float x10, float y10, float x20, float y20, float r1, float r2, float *k, float *b)
    {
        float ex1, ex4;
//        float ex2, ex3;
        ex1 =sqrt(-power(r1) -2*r1*r2 -power(r2) +power(x10) -2*x10*x20 +power(x20) +power(y10) -2*y10*y20 + power(y20));
//        ex2 =sqrt(-power(r1) +2*r1*r2 -power(r2) +power(x10) -2*x10*x20 +power(x20) +power(y10) -2*y10*y20 + power(y20));
//        ex3 = -power(r1) +2*r1*r2 -power(r2) +power(x10) -2*x10*x20 +power(x20);
        ex4 =  power(r1) +2*r1*r2 +power(r2) -power(x10) +2*x10*x20 -power(x20);

        k[0] = -(r1*ex1 +r2*ex1 +x10*y10 -x10*y20 -x20*y10 +x20*y20)/(ex4);
        k[1] = (r1*ex1 +r2*ex1 -x10*y10 +x10*y20 +x20*y10 -x20*y20)/(ex4);
    //    k[2] = -(r1*ex2 -r2*ex2 -x10*y10 +x10*y20 +x20*y10 -x20*y20)/(ex3);
    //    k[3] = (r1*ex2 -r2*ex2 +x10*y10 -x10*y20 -x20*y10 +x20*y20)/(ex3);

//        cout << k[0] <<endl;
//        cout << k[1] <<endl;
    //    cout << k[2] <<endl;
    //    cout << k[3] <<endl;

        b[0] = (power(r2)*y10 + power(r1)*y20 -power(x10)*y20 -power(x20)*y10 +r2*x10*ex1 +r1*x20*ex1 +r1*r2*y10 +r1*r2*y20 +x10*x20*y10 +x10*x20*y20)/(ex4);
        b[1] = (power(r2)*y10 + power(r1)*y20 -power(x10)*y20 -power(x20)*y10 -r2*x10*ex1 -r1*x20*ex1 +r1*r2*y10 +r1*r2*y20 +x10*x20*y10 +x10*x20*y20)/(ex4);
    //    b[2] = -(power(r2)*y10 + power(r1)*y20 -power(x10)*y20 -power(x20)*y10 +r2*x10*ex2 -r1*x20*ex2 -r1*r2*y10 -r1*r2*y20 +x10*x20*y10 +x10*x20*y20)/(ex3);
    //    b[3] = -(power(r2)*y10 + power(r1)*y20 -power(x10)*y20 -power(x20)*y10 -r2*x10*ex2 +r1*x20*ex2 -r1*r2*y10 -r1*r2*y20 +x10*x20*y10 +x10*x20*y20)/(ex3);
//        cout << b[0] <<endl;
//        cout << b[1] <<endl;
    //    cout << b[2] <<endl;
    //    cout << b[3] <<endl;
    }

    void point_shadowin_line(float x0, float y0, float a, float b, float *s_x, float *s_y, float *dis)
    {
        float x, y;
        *s_x = (x0 + a * y0 - a * b )/(1 + a * a);
        *s_y = (a * x0 + a *a * y0 + b )/(1 + a * a);
        x = *s_x;
        y = *s_y;
        *dis = sqrt((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y));
    }

    bool is_point_in_line(float x, float y, float k, float b)
    {
        float y1 = k *x +b;
        if(fabs(y1 -y) <0.001f){
            return true;
        }
        return false;
    }

    bool is_in_same_line(float x1, float y1, float x2, float y2, float *d, float k1, float b1, float k2, float b2)
    {
        if(is_point_in_line(x1, y1, k1, b1) && is_point_in_line(x2, y2, k1, b1)){
            *d = sqrt(power(x1-x2) +  power(y1-y2));
            return true;
        }

        if(is_point_in_line(x1, y1, k2, b2) && is_point_in_line(x2, y2, k2, b2)){
            *d = sqrt(power(x1-x2) +  power(y1-y2));
            return true;
        }
        return false;
    }

    bool is_in_same_cycle(float x1, float y1, float x2, float y2, float *d, float x0, float y0, float r)
    {
        float d1 = sqrt(power(x1-x0) +  power(y1-y0));
        float d2 = sqrt(power(x2-x0) +  power(y2-y0));
        if(fabs(d1-d2) <0.001f){
            if(fabs(d1-r) <0.001f){
                float d12 = (sqrt(power(x2-x1) +  power(y2-y1)))*0.5;
                float ang = asin(d12/r);
                *d = 2 *r *ang;
                return true;
            }
        }
        return false;

    }

};

//-------------------------------------------
class dockPathPlanning
{
public:
    dockPathPlanning(float x_1, float y_1, float r_1, float x_2, float y_2, float r_2, float ang_d){
        x1 = x_1;
        y1 = y_1;
        r1 = r_1;
        x2 = x_2;
        y2 = y_2;
        r2 = r_2;
        ang = ang_d;
    }
    ~dockPathPlanning(){}
    findShortWay find_alg;

    float x1,    y1;
    float x2,    y2;
    float r1,    r2;
    float ang;
    float x10, y10, x20, y20;//两个圆的圆心
    float k[4], b[4];   //两个圆的四个切线方程
    //six point list
    //ciycle 1 point1
    //ciycle 1 point2
    //ciycle 2 point1
    //ciycle 2 point2
    //point start
    //point end
    float point_x[6], point_y[6];//六个点
    float move_dis[3];//三次运动距离
    vector<int> min_path;//最短路径
    int moving_clock_direction;//圆周运动方向

    bool is_have_a_solution(){
        float cosa = cos(ang *M_PI/180);
        float sina = sin(ang *M_PI/180);
        if(x2 >x1){
            x20 = x2 -r2 *cosa;
            y20 = y2 -r2 *sina;
            x10 = x1 +r1;
            y10 = y1;
            moving_clock_direction = 1;
        }
        else{
            x20 = x2 +r2*cosa;
            y20 = y2 +r2*sina;
            x10 = x1 -r1;
            y10 = y1;
            moving_clock_direction = -1;
        }
        float dis = find_alg.power((x10 -x20)) +find_alg.power(y10 -y20);\
        if(sqrt(dis) >(r1 +r2)){
            find_alg.caculate_tangent(x10, y10, x20, y20, r1, r2, k, b);
            float d;
            find_alg.point_shadowin_line(x10, y10, k[0], b[0], &point_x[0], &point_y[0], &d);
//            cout << point_x[0] <<" "  << point_y[0] <<endl;
            find_alg.point_shadowin_line(x10, y10, k[1], b[1], &point_x[1], &point_y[1], &d);
//            cout << point_x[1] <<" "  << point_y[1] <<endl;

            find_alg.point_shadowin_line(x20, y20, k[0], b[0], &point_x[2], &point_y[2], &d);
//            cout << point_x[2] <<" "  << point_y[2] <<endl;
            find_alg.point_shadowin_line(x20, y20, k[1], b[1], &point_x[3], &point_y[3], &d);
//            cout << point_x[3] <<" "  << point_y[3] <<endl;

            point_x[4] =x2; point_y[4] = y2;
            point_x[5] =x1; point_y[5] = y1;
            return true;
        }
        return false;
    }

    bool find_a_right_path(threeStepMove *move_info)
    {
        //---邻接矩阵
        MatrixXdInt LN_MAT(6, 6);
        LN_MAT <<
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0;
        float d = 0.0f;
        //01
        if(find_alg.is_in_same_cycle(point_x[0], point_y[0], point_x[1], point_y[1], &d, x10, y10, r1)){
            LN_MAT(0,1) = (int)(d *1000);
            LN_MAT(1,0) = (int)(d *1000);
        }
        //02
        if(find_alg.is_in_same_line(point_x[0], point_y[0], point_x[2], point_y[2], &d, k[0], b[0], k[1], b[1])){
            LN_MAT(0,2) = (int)(d *1000);
            LN_MAT(2,0) = (int)(d *1000);
        }
        //03
        if(find_alg.is_in_same_line(point_x[0], point_y[0], point_x[3], point_y[3], &d, k[0], b[0], k[1], b[1])){
            LN_MAT(0,3) = (int)(d *1000);
            LN_MAT(3,0) = (int)(d *1000);
        }
        //04
        if(find_alg.is_in_same_cycle(point_x[0], point_y[0], x2, y2, &d, x10, y10, r1)){
            LN_MAT(0,4) = (int)(d *1000);
            LN_MAT(4,0) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[0], point_y[0], x2, y2, &d, x20, y20, r2)){
            LN_MAT(0,4) = (int)(d *1000);
            LN_MAT(4,0) = (int)(d *1000);
        }
        //05
        if(find_alg.is_in_same_cycle(point_x[0], point_y[0], x1, y1, &d, x10, y10, r1)){
            LN_MAT(0,5) = (int)(d *1000);
            LN_MAT(5,0) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[0], point_y[0], x1, y1, &d, x20, y20, r2)){
            LN_MAT(0,5) = (int)(d *1000);
            LN_MAT(5,0) = (int)(d *1000);
        }
        //12
        if(find_alg.is_in_same_line(point_x[1], point_y[1], point_x[2], point_y[2], &d, k[0], b[0], k[1], b[1])){
            LN_MAT(1,2) = (int)(d *1000);
            LN_MAT(2,1) = (int)(d *1000);
        }
        //13
        if(find_alg.is_in_same_line(point_x[1], point_y[1], point_x[3], point_y[3], &d, k[0], b[0], k[1], b[1])){
            LN_MAT(1,3) = (int)(d *1000);
            LN_MAT(3,1) = (int)(d *1000);
        }
        //14
        if(find_alg.is_in_same_cycle(point_x[1], point_y[1], x2, y2, &d, x10, y10, r1)){
            LN_MAT(1,4) = (int)(d *1000);
            LN_MAT(4,1) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[1], point_y[1], x2, y2, &d, x20, y20, r2)){
            LN_MAT(1,4) = (int)(d *1000);
            LN_MAT(4,1) = (int)(d *1000);
        }
        //15
        if(find_alg.is_in_same_cycle(point_x[1], point_y[1], x1, y1, &d, x10, y10, r1)){
            LN_MAT(1,5) = (int)(d *1000);
            LN_MAT(5,1) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[1], point_y[1], x1, y1, &d, x20, y20, r2)){
            LN_MAT(1,5) = (int)(d *1000);
            LN_MAT(5,1) = (int)(d *1000);
        }
        //23
        if(find_alg.is_in_same_cycle(point_x[2], point_y[2], point_x[3], point_y[3], &d, x20, y20, r1)){
            LN_MAT(2,3) = (int)(d *1000);
            LN_MAT(3,2) = (int)(d *1000);
        }
        //24
        if(find_alg.is_in_same_cycle(point_x[2], point_y[2], x2, y2, &d, x10, y10, r1)){
            LN_MAT(2,4) = (int)(d *1000);
            LN_MAT(4,2) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[2], point_y[2], x2, y2, &d, x20, y20, r2)){
            LN_MAT(2,4) = (int)(d *1000);
            LN_MAT(4,2) = (int)(d *1000);
        }
        //25
        if(find_alg.is_in_same_cycle(point_x[2], point_y[2], x1, y1, &d, x10, y10, r1)){
            LN_MAT(2,5) = (int)(d *1000);
            LN_MAT(5,2) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[2], point_y[2], x1, y1, &d, x20, y20, r2)){
            LN_MAT(2,5) = (int)(d *1000);
            LN_MAT(5,2) = (int)(d *1000);
        }
        //34
        if(find_alg.is_in_same_cycle(point_x[3], point_y[3], x2, y2, &d, x10, y10, r1)){
            LN_MAT(3,4) = (int)(d *1000);
            LN_MAT(4,3) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[3], point_y[3], x2, y2, &d, x20, y20, r2)){
            LN_MAT(3,4) = (int)(d *1000);
            LN_MAT(4,3) = (int)(d *1000);
        }
        //35
        if(find_alg.is_in_same_cycle(point_x[3], point_y[3], x1, y1, &d, x10, y10, r1)){
            LN_MAT(3,5) = (int)(d *1000);
            LN_MAT(5,3) = (int)(d *1000);
        }
        else if(find_alg.is_in_same_cycle(point_x[3], point_y[3], x1, y1, &d, x20, y20, r2)){
            LN_MAT(3,5) = (int)(d *1000);
            LN_MAT(5,3) = (int)(d *1000);
        }
//        cout << "---------------------" <<endl;
//        cout << LN_MAT <<endl;

        findPath al_findpath;
        al_findpath.find_all_Path(LN_MAT, 4, 5);

        log_info("[udock] w find result: totoal path num: %d", al_findpath.path_all_.size());

        int min_dis =100000;
        for(size_t i=0; i <al_findpath.path_all_.size(); i++){
            log_info("[udock] w path: %d", i+1);
            log_info("[udock] w path dis: %d", al_findpath.path_dis_[i]);
            log_info("[udock] w point num:");
            vector<int> path = al_findpath.path_all_[i];
            for(size_t j=0; j<path.size(); j++){
                log_info("%d", path[j]);
            }
            if(min_dis > al_findpath.path_dis_[i]){
                min_dis = al_findpath.path_dis_[i];
                min_path = al_findpath.path_all_[i];
            }
        }

        if(min_path.size() != 4){
            log_info("[udock] w erro path point");
            return false;
        }
        move_dis[0] = LN_MAT(min_path[0], min_path[1]);
        move_dis[1] = LN_MAT(min_path[1], min_path[2]);
        move_dis[2] = LN_MAT(min_path[2], min_path[3]);

        if(moving_clock_direction ==1){
            log_info("[udock] [FIRST]left Steering wheel");
            move_info->step_one_steering = STEERING_LEFT;
        }
        else{
            log_info("[udock] [FIRST]right Steering wheel");
            move_info->step_one_steering = STEERING_RIGHT;
        }
        if(point_y[min_path[0]] > point_y[min_path[1]]){
            log_info("[udock] move circle front d= %f", move_dis[0]);
            move_info->step_one_dist = move_dis[0];
        }
        else{
            log_info("[udock] move circle back d= %f", -move_dis[0]);
            move_info->step_one_dist = -move_dis[0];
        }

        log_info("[udock] [SECOND]linear moving");
        if(point_y[min_path[1]] > point_y[min_path[2]]){
            log_info("[udock] move line front d= %f", move_dis[1]);
            move_info->step_two_dist = move_dis[1];
        }
        else{
            log_info("[udock] move line back d= %f", -move_dis[1]);
            move_info->step_two_dist = -move_dis[1];
        }

        if(moving_clock_direction ==-1){
            log_info("[udock] [THRID]left Steering wheel");
            move_info->step_three_steering = STEERING_LEFT;
        }
        else{
            log_info("[udock] [THRID]right Steering wheel");
            move_info->step_three_steering = STEERING_RIGHT;
        }
        if(point_y[min_path[2]] > point_y[min_path[3]]){
            log_info("[udock] move circle front d= %f", move_dis[2]);
            move_info->step_three_dist = move_dis[2];
        }
        else{
            log_info("[udock] move circle back d= %f", move_dis[2]);
            move_info->step_three_dist = -move_dis[2];
        }
        return true;
    }


};

#ifdef _CHASSIS_MARSHELL_

class UdockTankManager;
#include "udockwheel_obstacle_detect.h"

class moveControl{

public:
    ~moveControl();
    void init(void);
    UdockTankManager    *udock_manager;
    UdockWheelObstacleDetect *udock_wheel_obstacle;
    UdockData *udock_data;

    bool wheel_move_angle(const float angle, const int tim);
    bool wheel_move_back_dist(const float dist, const float speed, const float angle);//cm
    bool wheel_move_back_dist_ignore_ob(const float dist, const float speed, const float angle);//mm cm/s degree second
    bool wheel_move_front_dist(const float dist, const float speed, const float angle);//cm

    bool move_three_step(void);

    bool get_the_path(robotPose local, robotPose dest);
    bool move_back_to_start_point(void);
    bool leave_from_dock(const float dist, const int speed);
    bool goto_right_direction(void);
    bool goto_temp_destination(robotPose local, robotPose dest);
    bool goto_point2(void);
    bool goto_last_point(void);
public:
    static moveControl *get_instance(void)
    {
        static moveControl move_control_;
        return &move_control_;
    }
private:
    moveControl();
    void chassis_test_thread(void);

    threeStepMove move_info;
    ros::NodeHandle nh_;
    ros::Publisher set_vel_cmd_pub_;
};

#endif

#endif

