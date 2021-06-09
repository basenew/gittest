#ifndef COLLISION_DETECT_H
#define COLLISION_DETECT_H

#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/Twist.h>



namespace collision_detection
{       
// 位姿
const static double DEGREE_PER_RAD = 57.2957795;

struct Pose2D
{
    double x;
    double y;
    double theta;

    Pose2D(double x_ = 0, double y_ = 0, double theta_ = 0) : x(x_), y(y_), theta(theta_){}
    void set(double x_ = 0, double y_ = 0, double theta_ = 0);
    Pose2D operator + (const Pose2D &a);
};

// 速度
struct Velocity
{
    Velocity(double _v_x = 0.0, double _v_y = 0.0, double _v_theta = 0.0);
    void set(double _v_x, double _v_y, double _v_theta);

    double v_x;
    double v_y;
    double v_theta;

    geometry_msgs::Twist to_twist() const
    {
        geometry_msgs::Twist twist;
        twist.linear.x = this->v_x;
        twist.linear.y = this->v_y;
        twist.angular.z = this->v_theta;
        return twist;
    }

    void from_twist(const geometry_msgs::Twist& twist)
    {
        this->v_x = twist.linear.x;
        this->v_y = twist.linear.y;
        this->v_theta = twist.angular.z;
    }

};

// 轨迹生成器
class TrajectorySampler
{
public:
    TrajectorySampler(){}
    void calc_rotate_center(const Velocity &velocity,  Eigen::Vector2f& turning_center);
    // 接口，用于生成某一形状机器人在一定速度下扫过的区域包含的地图栅格index
    virtual bool generate_trajectory(const Velocity &velocity, const std::shared_ptr<const grid_map::GridMap>& map_ptr, double deceleration_dis, double sample_delta_x, std::vector<grid_map::Index> &traj) = 0;
    // 从某一起点开始，生成该点在一定速度下生成的轨迹
    bool generate_trajectory_from_point(const Pose2D &origin, const Velocity &velocity, double deceleration_dis, double sample_delta_x, std::vector<Pose2D> &traj);

private:

};


// 矩形机器人 轨迹生成器
class TrajectorySamplerRectangularRobot : public TrajectorySampler
{
public:
    TrajectorySamplerRectangularRobot(double x, double y, double map_resolution) : robot_x_(x), robot_y_(y), map_resolution_(map_resolution) { generate_check_points(); }
    bool generate_trajectory(const Velocity &velocity, const std::shared_ptr<const grid_map::GridMap>& map_ptr, double deceleration_dis, double sample_delta_x, std::vector<grid_map::Index> &traj);
    
private:
    // 根据速度方向，选取进行测试的点集合
    void generate_check_points();
    void generate_grid_index_inside(const std::shared_ptr<const grid_map::GridMap>& map_ptr);
    void choose_check_points(const Velocity &velocity, std::vector<Pose2D> &check_points);
    double robot_x_;
    double robot_y_;
    double map_resolution_;
    std::vector<Pose2D> check_points_positive_vx_;
    std::vector<Pose2D> check_points_negtive_vx_;
    std::vector<Pose2D> check_points_zero_vx_;

    std::vector<grid_map::Index> index_inside_;
};

class CollisionDetector
{
public:
    enum CollisionCheckResult
    {
        NO_COLLISION = 0,
        NO_MAP,
        COllISION
    };

    CollisionDetector(ros::NodeHandle nh);
    // 碰撞检测，传入速度及减速（即检测）距离，传出发生碰撞的地图栅格index
    CollisionCheckResult collision_check(const Velocity &vel_in,  double  deceleration_distance, grid_map::Position& collision_position, std::string& collision_layer);
    // 输入原始速度，输出安全速度
    void collision_avoid(const Velocity &vel_in, Velocity &vel_out);
    void collision_avoid(const geometry_msgs::Twist& twist_in, geometry_msgs::Twist& twist_out);

private:
    void local_map_sub_call_back(const grid_map_msgs::GridMap &);
    void publish_map();

    ros::NodeHandle nh_;
    // /collision_detect/grid_map
    ros::Subscriber local_map_sub_;
    ros::Publisher map_publisher_;
    std::mutex local_map_mtx_;

    std::shared_ptr<grid_map::GridMap> collision_check_grid_map_ptr_;
    std::shared_ptr<TrajectorySampler> trajectory_sampler_ptr_;
    double local_map_latest_update_time_;
    std::string map_topic_name_;
    double map_resolution_;
    std::string map_out_put_topic_name_;
    double local_map_not_update_tolerance_duration_; // 容许地图不更新的时间, seconds
    std::string detect_area_layer_name_ = "detect_area";
    std::string all_sensor_layer_name_ = "all_sensor";
    int speed_rank_;
    std::vector<double> speed_level_;  // 预设的速度等级
    std::vector<double> deceleration_dis_level_; // 对应个速度等级的减速时间
    std::vector<std::string> map_layers_; // 地图的层名
    std::vector<std::string> sensor_layers_; // 各种传感器对应的层名
};
}




#endif