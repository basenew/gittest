#ifndef __OBSTACLE_AVOID_UNDER_GIVEN_CONTROL__
#define __OBSTACLE_AVOID_UNDER_GIVEN_CONTROL__
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>
#include "udock_obstacle_detect.h"
#include "udock_common.h"
#include "udock_data.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"

namespace bg = boost::geometry;

typedef bg::model::point<float, 2, bg::cs::cartesian> Point_t;
typedef bg::model::ring<Point_t> Ring_t;
typedef Eigen::Matrix<float, 3, 8> UltroSoPosition_t;

struct ControlVariable
{
// #ifndef _CHASSIS_MARSHELL_
    float v_x;       //线速度
    float v_theta;   //角速度

    ControlVariable(float v_x = 0, float v_tt = 0):v_x(v_x), v_theta(v_tt){}
    ControlVariable operator*(float a)
    {
        return ControlVariable(this->v_x * a, this->v_theta * a);
    }

    // 截断至合理区间
    void reduce_to_resonable_value()  
    {
        if (v_x >= 0)
            v_x = std::min(v_x, 1.0f);
        else
            v_x = std::max(v_x, -1.0f);

        if (v_theta >= 0)
            v_theta = std::min(v_theta, 0.63f);
        else
            v_theta = std::max(v_theta, -0.63f);
    }

// #else
//     float speed;     //速度
//     float steer_angle; //前轮转向角
//     ControlVariable(float speed = 0, float steer_angle = 0):speed(speed), steer_angle(steer_angle){};
// #endif
};

struct CarSizeParam
{
    //   A---G---B
    //   |   ^   |
    //   E y<    F
    //   |       |
    //   D---H---C
    float robot_width, robot_length, A_to_center_x, A_to_center_y;
    CarSizeParam(float robot_w = 0.98, float robot_l = 1.316, float A_to_c_x = 0.734, float A_to_c_y = 0.98/2):
        robot_width(robot_w), robot_length(robot_l), A_to_center_x(A_to_c_x), A_to_center_y(A_to_c_y){}
};

class ObstacleAvoidUnderGivenControl
{
public:
    enum struct MoveDirection
    {
        NONE,
        FORWARD,
        BACKWARDS
    };
    static ObstacleAvoidUnderGivenControl *get_instance()
    {
        static ObstacleAvoidUnderGivenControl obstacle_avoid;
        return &obstacle_avoid;
    }

    void determine_safe_control(const ControlVariable &in, ControlVariable &out, bool is_in_udocking = false);

protected:
    UdockData               *gs_udp;
    UdockObstacleDetect     ob_det_;
    std::vector<Point_t>    ob_check_points_;
    std::vector<Point_t>    ob_check_points_ultraso_;
    std::vector<Point_t>    ob_check_points_slanted_;
    Ring_t                  obstacle_detect_polygan_;
    Ring_t                  obstacle_detect_polygan_slanted_;
    bool                    flag_locked_after_front_slanted_laser_detected_sth_;
    bool                    flag_print_;//减速打印
    MoveDirection           moving_direction_;
    bool                    is_in_udocking_; //是否在回充过程
    float                   h_cliff_ = 1.127; //悬崖检测高度阈值
    bool                    flag_cliff_;
    // bool                    flag_play_tts_;

    CarSizeParam            car_size_;//车身尺寸
    Eigen::Vector2f         car_vertex_A_, car_vertex_B_, car_vertex_C_, car_vertex_D_, 
                            car_border_middle_point_E_, car_border_middle_point_F_; //车身顶点及边线中点
    UltroSoPosition_t       ultroso_position_;
    atris_msgs::AisoundTTS msg_;
    ros::NodeHandle nh_;
    ros::Publisher aisound_tts_pub_;
    

    ObstacleAvoidUnderGivenControl();
    void init_params(CarSizeParam& car_size, UltroSoPosition_t ultroso_position);
    bool scan_data_preprocess();
    bool ultroso_data_preprocess();

    Eigen::Matrix2Xf calc_obstacle_polygan_for_small_vel();
    Eigen::Matrix2Xf calc_obstacle_polygan_for_short_radius(float turning_radius, float rotation_angle);
    Eigen::Matrix2Xf calc_obstacle_polygan_for_medium_radius(float turning_radius, float rotation_angle);
    Eigen::Matrix2Xf calc_obstacle_polygan_for_long_radius(float turning_radius, float rotation_angle);
    //void calc_obstacle_detect_polygan(const ControlVariable& control_variable);
    void calc_obstacle_detect_polygan(const ControlVariable& control_variable, Ring_t& obstacle_detect_polygan);
    bool exist_obstacle_in_detect_polygan(std::vector<Point_t>& check_points, Ring_t& obstacle_detect_polygan);
    bool exist_obstacle_in_detect_polygan(std::vector<Point_t>& check_points, Ring_t& obstacle_detect_polygan, Point_t& obstacle_point);
    //void play_tts(int key, bool force_mute = false);

    // 延迟减速作用
    std::chrono::steady_clock::time_point       time_point_deceleration_start_;
    float                                       speed_thresh_; 
    std::chrono::milliseconds                   duration_deceleration_;
    void deceleration_record(float speed);
    void deceleration_affect(float& speed);



    // 将eigen::matrix类型转化成boost::geometry::ring 类型
    template <typename Derived>
    void matrix_2_ring(Eigen::DenseBase<Derived> &src_matrix, Ring_t &dest_ring)
    {
        int col = src_matrix.cols();
        dest_ring.resize(col);
        for (int i = 0; i < col; i++)
        {
            dest_ring[i] = Point_t(src_matrix(0, i), src_matrix(1, i));
        }
    }

 #ifdef _POLICE_
         float h_bottom_line_ = 0.977 - 0.2;    //height of bottom line detection range of police version
         float h_slanted_lidar_ = 0.649;    //height of front slanted lidardetection range of police version
         float h_cliff_definitely_ = 1.36;    //height of definitely cliff
         float h_cliff_possiblely_ = 0.937 + 0.03 + 0.1; //height of possible cliff
         float h_cliff_diff_ = 0.063; //height diff considered to be cliff 
         float dis_multilidar_to_center_ = 0.260;
         float dis_singlelidar_to_center_ = 0.235;
 #else
         float h_bottom_line_ = 0.840 - 0.2;    //height of bottom line detection range of police version
         float h_slanted_lidar_ = 0.559;    //height of front slanted lidardetection range of police version
         float h_cliff_definitely_ = 1.219;    //height of definitely cliff
         float h_cliff_possiblely_ = 0.797 + 0.03 + 0.1; //height of possible cliff
         float h_cliff_diff_ = 0.065; //height diff considered to be cliff 
         float dis_multilidar_to_center_ = 0.375;
         float dis_singlelidar_to_center_ = 0.290;
 #endif   

};

#endif
