#include "udock_obstacle_avoid_under_given_control.h"
#include <cmath>
#include <iostream>
#include "imemory/atris_imemory_api.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

ObstacleAvoidUnderGivenControl::ObstacleAvoidUnderGivenControl() {
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    gs_udp = UdockData::get_instance();

    CarSizeParam        car_size;
    UltroSoPosition_t   ultroso_position;
    msg_.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;

    flag_locked_after_front_slanted_laser_detected_sth_ = false;

#ifndef _CHASSIS_MARSHELL_
#ifdef _POLICE_
    car_size = CarSizeParam(0.98, 1.316, 0.734, 0.98 / 2); //履带公安版尺寸

    ultroso_position << -0.303, 0.358, 0.680, 0.680, 0.358, -0.303, -0.537, -0.537,
                        0.471, 0.417, 0.170, -0.170, -0.417, -0.471, -0.341, 0.341,
                        0.489, 0.425, 0.400, 0.400, 0.425, 0.489, 0.426, 0.426;
#else
    car_size = CarSizeParam(0.961, 1.300, 0.711, 0.961 / 2); //履带标准版尺寸

    ultroso_position << -0.257, 0.404, 0.680, 0.680, 0.404, -0.257, -0.523, -0.523,
                        0.396, 0.396, 0.280, -0.280, -0.396, -0.396, -0.350, 0.350,
                        0.460, 0.460, 0.368, 0.368, 0.460, 0.460, 0.353, 0.353;
#endif
#else

#endif
    init_params(car_size, ultroso_position);

}

bool ObstacleAvoidUnderGivenControl::scan_data_preprocess()
{
    LaserRaw scan;
    vector<dataPoint>   ob_data;
    ob_check_points_.clear();
    ob_check_points_slanted_.clear();

    // 处理多线雷达数据
    if (!gs_udp->get_lidar_data(scan))
    {
        if (flag_print_) log_info("[ udock ] [ obstacle avoid ]gs_udp get scan data failed!");
        // if (flag_play_tts_)
        // {
        //     msg_.text = TTSStrings::text(TTSStrings::TTS_KEY_RC_MULTI_LIDAR_FAILURE);
        //     aisound_tts_pub_.publish(msg_);
        // } 
        // play_tts(TTSStrings::TTS_KEY_RC_MULTI_LIDAR_FAILURE);
        return false;
    }
    
    // check data is valid or not
    if (moving_direction_ != MoveDirection::BACKWARDS && !ob_det_.is_lidar_data_valid(scan, 3.0f * M_PI / 5.0, -3.0f * M_PI / 5.0))
     {
         return false;
     }

    ob_det_.lidar_data_filter(2.5f, scan, ob_data);
    
    if (!gs_udp->get_lidar_data_bottom_line(scan))
    {
        if (flag_print_) log_info("[ udock ] [ obstacle avoid ]gs_udp get bottom_line scan data failed!");
        // return false;
    }
    if (is_in_udocking_)
        ob_det_.lidar_data_filter_bottom_line(h_bottom_line_ - 0.5, scan, ob_data); //正在回充过程，避免充电桩导致的避障
    else
        ob_det_.lidar_data_filter_bottom_line(h_bottom_line_, scan, ob_data);

    int num =  ob_data.size();
    // if (flag_print_) log_info("[ udock ] [ obstacle avoid ]num of points velodyne laser find:  %d ", num);
    ob_check_points_.resize(num);
    for (int i = 0; i < num; i++)
    {
        ob_check_points_[i] = Point_t(ob_data[i].x + dis_multilidar_to_center_, ob_data[i].y);//以机器人旋转中心为坐标系，修正坐标
    }

    // 前进时处理前置倾斜小激光数据
    #ifndef _Atris1CDvt2_
    if (moving_direction_ == MoveDirection::FORWARD)
    {
        if (!gs_udp->get_lidar_data_front_slanted(scan))
        {
            if (flag_print_) log_info("[ udock ] [ obstacle avoid ]gs_udp get front slanted scan data failed!");
            // if (flag_play_tts_)
            // {
            //     msg_.text = TTSStrings::text(TTSStrings::TTS_KEY_RC_FRONT_SINGLE_LIDAR_FAILURE);
            //     aisound_tts_pub_.publish(msg_);
            // }
            // play_tts(TTSStrings::TTS_KEY_RC_FRONT_SINGLE_LIDAR_FAILURE);
            return false;
        }

        float h_ob = 0;
        if (is_in_udocking_) 
            h_ob = h_slanted_lidar_ - 0.3;
        else
            h_ob = h_slanted_lidar_;
        ob_det_.lidar_data_filter_front_slanted(h_ob, h_cliff_, scan, ob_data, flag_cliff_);
        num =  ob_data.size();
        ob_check_points_slanted_.resize(num);
        // if (flag_print_) log_info("[ udock ] [ obstacle avoid ]num of points front slanted laser find:  %d ", num);
        for (int i = 0; i < num; i++)
        {
            ob_check_points_slanted_[i] = Point_t(ob_data[i].x + dis_singlelidar_to_center_ , ob_data[i].y);//以机器人旋转中心为坐标系，修正坐标
        }          
    }
    #endif
    return true;
}

bool ObstacleAvoidUnderGivenControl::ultroso_data_preprocess()
{
    uint16_t ultroso_measurement;

    if (moving_direction_ != MoveDirection::BACKWARDS)   //向前运动检测前面超声
    {
        shm::UltraSound ultroso;
        memset(ultroso.data, 0, sizeof(ultroso.data));
        shm::iMemory_read_UltraSound(&ultroso);
        for (int i = 2; i < 4; i++)
        {
            ultroso_measurement = ultroso.data[i];
            if (ultroso_measurement == 0xFFFF)
            {
                if (flag_print_)
                    log_info("[ udock ] [ obstacle avoid ]ultroso measurement invalid...");
                return false;                
            }
            if (ultroso_measurement < 400) // 取低于400测量值
            {
                ob_check_points_.push_back(Point_t(ultroso_position_(0, i) + ultroso_measurement / 1000.0, ultroso_position_(1, i)));
                // if (flag_print_)
                //     log_info("[ udock ] [ obstacle avoid ]ultroso(%d) measurement  %f m", i+1, ultroso_measurement / 1000.0);          
            }
        }
    }
    if (moving_direction_ != MoveDirection::FORWARD)//向后运动检测后面超声
    {
        shm::UltraSound ultroso;
        memset(ultroso.data, 0, sizeof(ultroso.data));
        shm::iMemory_read_UltraSound(&ultroso);
        for (int i = 6; i < 8; i++)
        {
            ultroso_measurement = ultroso.data[i];
            if (ultroso_measurement == 0xFFFF)
            {
                if (flag_print_)
                    log_info("[ udock ] [ obstacle avoid ]ultroso measurement invalid...");
                return false;                
            }
            if (ultroso_measurement < 400) // 取低于400测量值
            {
                ob_check_points_.push_back(Point_t(ultroso_position_(0, i) - ultroso_measurement / 1000.0, ultroso_position_(1, i)));
                // if (flag_print_)
                //     log_info("[ udock ] [ obstacle avoid ]ultroso(%d) measurement  %f m", i+1, ultroso_measurement / 1000.0);          
            }
        }        
    }
    return true;
}

bool ObstacleAvoidUnderGivenControl::exist_obstacle_in_detect_polygan(std::vector<Point_t>& check_points, Ring_t& obstacle_detect_polygan)
{
    for (auto point : check_points)
    {
        if (bg::within(point, obstacle_detect_polygan))
        {
            return true;
        }
    }
    return false;
}

bool ObstacleAvoidUnderGivenControl::exist_obstacle_in_detect_polygan(std::vector<Point_t>& check_points, Ring_t& obstacle_detect_polygan, Point_t& obstacle_point)
{
    for (auto point : check_points)
    {
        if (bg::within(point, obstacle_detect_polygan))
        {
            obstacle_point = point;
            return true;
        }
    }
    return false;
}

void ObstacleAvoidUnderGivenControl::determine_safe_control(const ControlVariable &in, ControlVariable &out, bool is_in_udocking)
{
    static int count_for_print = 1;
    flag_print_ = false;
    if (count_for_print++ > 5)
    {
        count_for_print = 1;
        flag_print_ = true;
    }
    out.v_x = 0;
    out.v_theta = 0;

    is_in_udocking_ = is_in_udocking;

    if (in.v_x > 0.0001) 
        moving_direction_ = MoveDirection::FORWARD;//前进
    else if (in.v_x < -0.0001)
    {
        moving_direction_ = MoveDirection::BACKWARDS; //后退
        if(flag_locked_after_front_slanted_laser_detected_sth_)
        {
            flag_locked_after_front_slanted_laser_detected_sth_ = false; //向后倒清除标志 
            log_info("[ udock ] [ obstacle avoid ]move backward, chassis release lock..."); 
        } 
    }
    else
    {
        moving_direction_ = MoveDirection::NONE;
    }
    

    if (moving_direction_ != MoveDirection::BACKWARDS && flag_locked_after_front_slanted_laser_detected_sth_)
    {
        if (flag_print_) log_info("[ udock ] [ obstacle avoid ] chassis locked ...");
        // play_tts(TTSStrings::TTS_KEY_RC_AFTER_CHASSIS_LOCKED_BY_LIDAR);        
        return;
    }

    //!ultroso_data_preprocess()
    if (!scan_data_preprocess() || !ultroso_data_preprocess()){
        return;
    }
    // if (moving_direction_ != MoveDirection::BACKWARDS && !flag_locked_after_front_slanted_laser_detected_sth_ && flag_cliff_) //悬崖检测
    // {
    //     flag_locked_after_front_slanted_laser_detected_sth_ = true;
    //     log_info("[ udock ] [ obstacle avoid ]chassis locked because cliff detected...");
    //     // play_tts(TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR);
    //     // play_tts(TTSStrings::TTS_KEY_RC_AFTER_CHASSIS_LOCKED_BY_LIDAR, true);  
    //     return;        
    // }


    out = in;
    out.reduce_to_resonable_value(); //速度截断至合理值
    deceleration_affect(out.v_x);

    if (ob_check_points_.size() == 0 && ob_check_points_slanted_.size() == 0)
    {
        if (flag_print_) log_info("[ udock ] [ obstacle avoid ] no obstacle check points");
        if (flag_print_) log_info("[ udock ] [ obstacle avoid ] output speed [%f, %f]", out.v_x, out.v_theta);
        return;
    }
    calc_obstacle_detect_polygan(out, obstacle_detect_polygan_);

    // 多线雷达检测到障碍物
    if (exist_obstacle_in_detect_polygan(ob_check_points_, obstacle_detect_polygan_))
    {
        log_info("[ udock ] [ obstacle avoid ]obstacle detected, slow down...");
        //逐级减速
        do
        {
            out = out * 0.6;
            deceleration_record(out.v_x);
            if (fabs(out.v_x) < 0.01 && fabs(out.v_theta) < 0.01) //1cm/s 1deg
            {
                out.v_x = 0;
                out.v_theta = 0;
                break;
            }
            calc_obstacle_detect_polygan(out, obstacle_detect_polygan_);
        } while (exist_obstacle_in_detect_polygan(ob_check_points_, obstacle_detect_polygan_));
    }

    // 多线雷达未检测到障碍物
    // 正在回充，前方小激光检测到障碍物不锁定底盘
    else if (is_in_udocking_)
    {
        if (out.v_x > 0.02 ) //忽略线速度为负的情况
        {
            if (exist_obstacle_in_detect_polygan(ob_check_points_slanted_, obstacle_detect_polygan_))
            {
                out.v_x = 0;
                out.v_theta = 0; 
            }
        }
    }
    // 非回充状态，前方小激光检测到障碍物锁定底盘
    else
    {
        // 倾斜小雷达检测到障碍物，锁定底盘
        if (out.v_x > 0.02 && !flag_locked_after_front_slanted_laser_detected_sth_) //忽略线速度为负的情况
        {
            Point_t obstacle_point;
            if (exist_obstacle_in_detect_polygan(ob_check_points_slanted_, obstacle_detect_polygan_slanted_, obstacle_point))
            {
                log_info("[ udock ] [ obstacle avoid ] slanted laser find obstacle [%f, %f]...", 
                            bg::get<0>(obstacle_point) - car_vertex_A_(0), bg::get<1>(obstacle_point));
                if(!exist_obstacle_in_detect_polygan(ob_check_points_, obstacle_detect_polygan_slanted_))
                {
                    flag_locked_after_front_slanted_laser_detected_sth_ = true;
                    out.v_x = 0;
                    out.v_theta = 0;
                    log_info("[ udock ] [ obstacle avoid ] chassis locked because short obstacle deteted...");
                    // play_tts(TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR);
                    // play_tts(TTSStrings::TTS_KEY_RC_AFTER_CHASSIS_LOCKED_BY_LIDAR, true);            
                }
                else
                {
                    if (flag_print_) log_info("[ udock ] [ obstacle avoid ] do NOT lock chassis because velodyne detect the obstacle too...");
                }    
            }          
        }
    }
    if (flag_print_) log_info("[ udock ] [ obstacle avoid ] output speed [%f, %f]", out.v_x, out.v_theta);
    return;
}

void ObstacleAvoidUnderGivenControl::init_params(CarSizeParam& car_size, UltroSoPosition_t ultroso_position)
{
    car_size_ = car_size;

    float safe_distance_x = 0.1;
    float safe_distance_y = 0.04;

    car_vertex_A_ << car_size.A_to_center_x + safe_distance_x, car_size.A_to_center_y + safe_distance_y;
    car_vertex_B_ << car_size.A_to_center_x + safe_distance_x, -(car_size.robot_width - car_size.A_to_center_y) - safe_distance_y;
    car_vertex_C_ << -(car_size.robot_length - car_size.A_to_center_x) - safe_distance_x, -(car_size.robot_width - car_size.A_to_center_y) - safe_distance_y ;
    car_vertex_D_ << -(car_size.robot_length - car_size.A_to_center_x) - safe_distance_x, car_size.A_to_center_y + safe_distance_y;

    car_border_middle_point_E_ << 0.0, car_vertex_A_(1);
    car_border_middle_point_F_ << 0.0, car_vertex_B_(1);

    // 将前方小激光测距范围设置成一固定值
    Matrix2Xf polygan_mat_for_slanted_laser(2, 5);
    Vector2f temp_A(car_vertex_A_(0) + 0.6, car_vertex_A_(1));
    Vector2f temp_B(car_vertex_B_(0) + 0.6, car_vertex_B_(1));
    polygan_mat_for_slanted_laser << car_vertex_A_, temp_A, temp_B, car_vertex_B_, car_vertex_A_;
    matrix_2_ring(polygan_mat_for_slanted_laser, obstacle_detect_polygan_slanted_);

    ultroso_position_ = ultroso_position;
}

Matrix2Xf ObstacleAvoidUnderGivenControl::calc_obstacle_polygan_for_small_vel()
{
    Matrix2Xf polygan_mat(2, 5);

    polygan_mat << car_vertex_A_, car_vertex_B_, car_vertex_C_, car_vertex_D_, car_vertex_A_;

    return polygan_mat;
}

Matrix2Xf ObstacleAvoidUnderGivenControl::calc_obstacle_polygan_for_short_radius(float turning_radius, float rotation_angle)
{
    //旋转中心
    Vector2f turning_center = Vector2f(0, turning_radius);

    //
    Vector2f car_border_a_point_G, car_border_a_point_H;
    car_border_a_point_G << car_vertex_A_(0), turning_radius;
    car_border_a_point_H << car_vertex_C_(0), turning_radius;
    //Matrix2Xf转中心至车身定点向量
    Vector2f turning_center_to_A = car_vertex_A_ - turning_center;
    Vector2f turning_center_to_B = car_vertex_B_ - turning_center;
    Vector2f turning_center_to_C = car_vertex_C_ - turning_center;
    Vector2f turning_center_to_D = car_vertex_D_ - turning_center;
    Vector2f turning_center_to_E = car_border_middle_point_E_ - turning_center;
    Vector2f turning_center_to_F = car_border_middle_point_F_ - turning_center;
    Vector2f turning_center_to_G = car_border_a_point_G - turning_center;
    Vector2f turning_center_to_H = car_border_a_point_H - turning_center;

    //根据角速度将车身旋转一定角度
    Rotation2D<float> rot_complete(rotation_angle);
    Rotation2D<float> rot_half(rotation_angle / 2.0);

    //预测运动后各参考点位置
    Vector2f car_vertex_A_predicted_final = turning_center + rot_complete * turning_center_to_A;
    Vector2f car_vertex_B_predicted_final = turning_center + rot_complete * turning_center_to_B;
    Vector2f car_vertex_C_predicted_final = turning_center + rot_complete * turning_center_to_C;
    Vector2f car_vertex_D_predicted_final = turning_center + rot_complete * turning_center_to_D;
    Vector2f car_vertex_A_predicted_halfway = turning_center + rot_half * turning_center_to_A;
    Vector2f car_vertex_B_predicted_halfway = turning_center + rot_half * turning_center_to_B;
    Vector2f car_vertex_C_predicted_halfway = turning_center + rot_half * turning_center_to_C;
    Vector2f car_vertex_D_predicted_halfway = turning_center + rot_half * turning_center_to_D;
    Vector2f car_point_E_predicted_halfway = turning_center + rot_half * turning_center_to_E;
    Vector2f car_point_F_predicted_halfway = turning_center + rot_half * turning_center_to_F;
    Vector2f car_point_G_predicted_halfway = turning_center + rot_half * turning_center_to_G;
    Vector2f car_point_H_predicted_halfway = turning_center + rot_half * turning_center_to_H;

    //设置的另一些参考点
    Vector2f point_AB_cross = turning_center + (car_point_G_predicted_halfway - turning_center) / cos(rotation_angle / 2.0);
    Vector2f point_BC_cross = turning_center + (car_point_F_predicted_halfway - turning_center) / cos(rotation_angle / 2.0);
    Vector2f point_CD_cross = turning_center + (car_point_H_predicted_halfway - turning_center) / cos(rotation_angle / 2.0);
    Vector2f point_DA_cross = turning_center + (car_point_E_predicted_halfway - turning_center) / cos(rotation_angle / 2.0);

    Matrix2Xf polygan_mat(2, 17);
    if (car_vertex_A_(1) - point_AB_cross(1) > 0.05)
    {
        polygan_mat << car_vertex_A_predicted_final, car_vertex_A_predicted_halfway, car_vertex_A_, point_AB_cross,
            car_vertex_B_predicted_final, car_vertex_B_predicted_halfway, car_vertex_B_, point_BC_cross,
            car_vertex_C_predicted_final, car_vertex_C_predicted_halfway, car_vertex_C_, point_CD_cross,
            car_vertex_D_predicted_final, car_vertex_D_predicted_halfway, car_vertex_D_, point_DA_cross,
            car_vertex_A_predicted_final;
    }
    else
    {
        polygan_mat.resize(2, 15);
        polygan_mat << car_vertex_A_predicted_final, car_vertex_A_predicted_halfway, car_vertex_A_,
            car_vertex_B_predicted_final, car_vertex_B_predicted_halfway, car_vertex_B_, point_BC_cross,
            car_vertex_C_predicted_final, car_vertex_C_predicted_halfway, car_vertex_C_,
            car_vertex_D_predicted_final, car_vertex_D_predicted_halfway, car_vertex_D_, point_DA_cross,
            car_vertex_A_predicted_final;
    }

    return polygan_mat;
}

Matrix2Xf ObstacleAvoidUnderGivenControl::calc_obstacle_polygan_for_medium_radius(float turning_radius, float rotation_angle)
{
    //旋转中心
    Vector2f turning_center = Vector2f(0, turning_radius);

    //Matrix2Xf转中心至车身定点向量
    Vector2f turning_center_to_A = car_vertex_A_ - turning_center;
    Vector2f turning_center_to_B = car_vertex_B_ - turning_center;
    Vector2f turning_center_to_C = car_vertex_C_ - turning_center;
    Vector2f turning_center_to_E = car_border_middle_point_E_ - turning_center;
    Vector2f turning_center_to_F = car_border_middle_point_F_ - turning_center;

    //根据角速度将车身旋转一定角度
    Rotation2D<float> rot_complete(rotation_angle);
    Rotation2D<float> rot_half(rotation_angle / 2.0);

    //预测运动后各参考点位置
    Vector2f car_vertex_A_predicted_final = turning_center + rot_complete * turning_center_to_A;
    Vector2f car_vertex_B_predicted_final = turning_center + rot_complete * turning_center_to_B;
    Vector2f car_vertex_B_predicted_halfway = turning_center + rot_half * turning_center_to_B;
    Vector2f car_vertex_C_predicted_final = turning_center + rot_complete * turning_center_to_C;
    Vector2f car_vertex_C_predicted_halfway = turning_center + rot_half * turning_center_to_C;
    Vector2f car_point_E_predicted_halfway = turning_center + rot_half * turning_center_to_E;
    Vector2f car_point_F_predicted_halfway = turning_center + rot_half * turning_center_to_F;

    //设置的另一些参考点
    Vector2f point_BC_cross = turning_center + (car_point_F_predicted_halfway - turning_center) / cos(rotation_angle / 2.0);

    // 计算障碍物检测矩形
    Vector2f E_to_E_halfway = car_point_E_predicted_halfway - car_border_middle_point_E_;
    Matrix2Xf polygan_mat(2, 13);
    if (E_to_E_halfway.norm() > 0.08)
    {
        Vector2f car_point_E_predicted_final = turning_center + rot_complete * turning_center_to_E;
        polygan_mat << car_vertex_A_predicted_final, car_vertex_B_predicted_final, car_vertex_B_predicted_halfway,
            car_vertex_B_, point_BC_cross, car_vertex_C_predicted_final, car_vertex_C_predicted_halfway,
            car_vertex_C_, car_vertex_D_, car_border_middle_point_E_, car_point_E_predicted_halfway,
            car_point_E_predicted_final, car_vertex_A_predicted_final;

        if (point_BC_cross(1) < car_vertex_C_predicted_final(1) + 0.03)
        {
            Matrix2Xf polygan_mat(2, 12);
            polygan_mat << car_vertex_A_predicted_final, car_vertex_B_predicted_final, car_vertex_B_predicted_halfway,
                car_vertex_B_, point_BC_cross, car_vertex_C_predicted_halfway,
                car_vertex_C_, car_vertex_D_, car_border_middle_point_E_, car_point_E_predicted_halfway,
                car_point_E_predicted_final, car_vertex_A_predicted_final;
        }
    }
    else
    {
        polygan_mat.resize(2, 11);
        Vector2f point_AD_cross = car_point_E_predicted_halfway / cos(rotation_angle / 2.0);
        polygan_mat << car_vertex_A_predicted_final, car_vertex_B_predicted_final, car_vertex_B_predicted_halfway,
            car_vertex_B_, point_BC_cross, car_vertex_C_predicted_final, car_vertex_C_predicted_halfway,
            car_vertex_C_, car_vertex_D_, point_AD_cross, car_vertex_A_predicted_final;
    }

    return polygan_mat;
}

Matrix2Xf ObstacleAvoidUnderGivenControl::calc_obstacle_polygan_for_long_radius(float turning_radius, float rotation_angle)
{
    //旋转中心
    Vector2f turning_center = Vector2f(0, turning_radius);

    //旋转中心至车身定点向量
    Vector2f turning_center_to_A = car_vertex_A_ - turning_center;
    Vector2f turning_center_to_B = car_vertex_B_ - turning_center;

    //根据角速度将车身旋转一定角度
    Rotation2D<float> rot_complete(rotation_angle);

    //运动后预测A B点位置
    Vector2f car_vertex_A_predicted_final = turning_center + rot_complete * turning_center_to_A;
    Vector2f car_vertex_B_predicted_final = turning_center + rot_complete * turning_center_to_B;

    Matrix2Xf polygan_mat(2, 7);

    polygan_mat << car_vertex_A_predicted_final, car_vertex_B_predicted_final, car_vertex_B_, car_vertex_C_,
        car_vertex_D_, car_vertex_A_, car_vertex_A_predicted_final;

    return polygan_mat;
}

void ObstacleAvoidUnderGivenControl::calc_obstacle_detect_polygan(const ControlVariable& control_variable, Ring_t& obstacle_detect_polygan)
{
    Matrix2Xf polygan_mat(2, 1);
    //Matrix2Xf polygan_mat_slanted(2,1);

    float v_theta = control_variable.v_theta;
    float v_x = control_variable.v_x;

    if(fabs(v_x) < 0.02 && fabs(v_theta) < 0.02)
    {
        polygan_mat = calc_obstacle_polygan_for_small_vel();
        matrix_2_ring(polygan_mat, obstacle_detect_polygan); 
        return;
    }

    float pose_predict_time = 0.7; //预测0.7s后车身位置

    //计算转弯半径
    //避免0 theta角导致除零操作
    if (fabs(v_theta) < 0.0001)
        v_theta = (v_theta >= 0 ? 0.0001 : -0.0001);
    float turning_radius = fabs(v_x / v_theta);
    // turning_radius = v_x > 0 ? turning_radius : -turning_radius;

    if (abs(turning_radius) > 8) // 大半径  近似矩形
    {
        polygan_mat = calc_obstacle_polygan_for_long_radius(turning_radius, fabs(v_theta * pose_predict_time));
    }
    else if (abs(turning_radius) > 0.25) //中等旋转半径
    {
        polygan_mat = calc_obstacle_polygan_for_medium_radius(turning_radius, fabs(v_theta * pose_predict_time));
    }
    else //小旋转半径
    {
        polygan_mat = calc_obstacle_polygan_for_short_radius(turning_radius, fabs(v_theta * pose_predict_time));
    }

    // 角速度为负，翻转
    if (v_theta < 0)
    {
        Matrix2f flip_x;
        flip_x << 1, 0, 0, -1;
        polygan_mat = flip_x * polygan_mat;
        polygan_mat = polygan_mat.rowwise().reverse().eval();
    }

    // 线速度为负 旋转
    if (v_x < 0)
    {
        Matrix2f rot_pi;
        rot_pi << -1, 0, 0, -1;
        polygan_mat = rot_pi * polygan_mat;
    }

    matrix_2_ring(polygan_mat, obstacle_detect_polygan);

    // if (flag_print_) log_info("[ udock ] [ obstacle avoid ]control variable: %.2f   %.2f", v_x, v_theta);
    // if (flag_print_) log_info("[ udock ] [ obstacle avoid ]obstacle detect polygan vertex start -----------------");
    // for (auto point : obstacle_detect_polygan)
    //     if (flag_print_) log_info("[ udock ] [ obstacle avoid ] %.4f   %.4f", bg::get<0>(point), bg::get<1>(point));
    // if (flag_print_) log_info("[ udock ] [ obstacle avoid ]obstacle detect polygan vertex end -----------------");
}

// void ObstacleAvoidUnderGivenControl::play_tts(int key, bool force_mute)
// {
//     // 时间记录数组的长度暂定为10；，若增大需要更改
//     static time_point<steady_clock, seconds> time_record[10];
//     auto time_now = time_point_cast<seconds>(steady_clock::now());
//     uint16_t index = key - TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR;
//     if (index >= 10) return;
//     if (duration_cast<seconds>(time_now-time_record[index]).count() >= 6)//播报时间间隔6s
//     {
//         msg_.text = TTSStrings::text(key);
//         if (!force_mute) aisound_tts_pub_.publish(msg_);
//         time_record[index] = time_now;
//     }
// }

void ObstacleAvoidUnderGivenControl::deceleration_record(float speed)
{
    duration_deceleration_ = std::chrono::milliseconds(int(2 * speed * 1000));
    time_point_deceleration_start_ = std::chrono::steady_clock::now();
    speed_thresh_ = speed;
    // log_info("[ udock ] [ obstacle avoid ] record speed %f", speed_thresh_);
    // log_info("[ udock ] [ obstacle avoid ] record ms %d", int(2 * speed * 1000));
    return ;
}

void ObstacleAvoidUnderGivenControl::deceleration_affect(float& speed)
{
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_point_deceleration_start_) < duration_deceleration_)
    {
        // log_info("[ udock ] [ obstacle avoid ] deceleration affect %f", speed_thresh_);
        speed = min(speed_thresh_, speed);
    }
}