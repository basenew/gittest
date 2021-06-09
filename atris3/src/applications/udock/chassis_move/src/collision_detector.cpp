#include "collision_detector.h"

using namespace std;
using namespace grid_map;

namespace collision_detection
{
template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void Pose2D::set(double x_, double y_, double theta_)
{
    x = x_;
    y = y_;
    theta = theta_;
}

Pose2D Pose2D::operator + (const Pose2D &a)
{
    Pose2D res;
    res.x = this->x + a.x;
    res.y = this->y + a.y;
    res.theta = this->theta + a.theta;

    return res;
}

Velocity::Velocity(double _v_x, double _v_y, double _v_theta) : v_x(_v_x), v_y(_v_y), v_theta(_v_theta) {}
void Velocity::set(double _v_x, double _v_y, double _v_theta)
{
    v_x = _v_x;
    v_y = _v_y;
    v_theta = _v_theta;
}

void TrajectorySampler::calc_rotate_center(const Velocity &velocity, Eigen::Vector2f& turning_center)
{
    double turning_radius = fabs(sqrt(velocity.v_x * velocity.v_x + velocity.v_y * velocity.v_y) / velocity.v_theta);
    Eigen::Vector2f vel = Eigen::Vector2f(velocity.v_x, velocity.v_y);
    Eigen::Rotation2D<float> rot_right_angle(sgn(velocity.v_theta) * M_PI_2);
    turning_center = rot_right_angle * (vel / vel.norm()) * turning_radius;
}

bool TrajectorySampler::generate_trajectory_from_point(const Pose2D &origin, const Velocity &vel, double deceleration_dis, double sample_delta_x, std::vector<Pose2D> &traj)
{
    Velocity velocity = vel;
    // 按照同一速度进行采样，故先算瞬心
    if (fabs(velocity.v_theta) < 0.000001)
    {
        velocity.v_theta = 0.000001;
    }
    if (fabs(velocity.v_x) < 0.000001)
    {
        velocity.v_x = 0.000001;
    }
    Eigen::Vector2f turning_center;
    calc_rotate_center(velocity, turning_center);

    // 计算采样得到的轨迹点
    Eigen::Matrix2f rot_m;
    Eigen::Vector2f origin_vec(origin.x, origin.y);
    Eigen::Vector2f turning_center_to_origin = origin_vec - turning_center;
    // 位姿点个数
    double delta_t;
    int trajectoy_pose_num;

    if (std::fabs(velocity.v_x / velocity.v_theta) > 0.2)
    {
        delta_t = fabs(sample_delta_x / velocity.v_x);
        trajectoy_pose_num = deceleration_dis / sample_delta_x;
    }
    else
    {
        delta_t =  M_PI / 40.0 / std::fabs(velocity.v_theta); 
        trajectoy_pose_num = 2.0 / delta_t;   //预测2S
    }
    traj.resize(trajectoy_pose_num);

    Pose2D pose(0,0,0);
    // 旋转矩阵
    double theta = velocity.v_theta * delta_t;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;

    for (int i = 0; i < trajectoy_pose_num; i++)
    {
        Eigen::Vector2f point_vec = turning_center + turning_center_to_origin;
        turning_center_to_origin = rot_m * turning_center_to_origin;
        pose.set(point_vec(0), point_vec(1), i * theta);
        traj[i] = pose;
    }
    return true;
}

// TrajectorySamplerRectangularRobot
bool TrajectorySamplerRectangularRobot::generate_trajectory(const Velocity &velocity,
                                                            const std::shared_ptr<const grid_map::GridMap> &map_ptr,
                                                            double deceleration_dis, double sample_delta_x,
                                                            std::vector<grid_map::Index> &traj)
{
    traj.clear();
    std::vector<Pose2D> check_points;
    choose_check_points(velocity, check_points);
    std::vector<Pose2D> pose2D_traj;
    std::vector<grid_map::Index> index_traj_a_check_point;
    grid_map::Index index;
    for (auto &pose_check : check_points)
    {
        TrajectorySampler::generate_trajectory_from_point(pose_check, velocity, deceleration_dis, sample_delta_x, pose2D_traj);
        index_traj_a_check_point.clear();
        for (int i = 0, n = pose2D_traj.size(); i < n; i++)
        {
            Pose2D pose = pose2D_traj[i];
            if (map_ptr->getIndex(grid_map::Position(pose.x, pose.y), index))
            {
                index_traj_a_check_point.push_back(index);
            }
        }
        traj.insert(traj.end(), index_traj_a_check_point.begin(), index_traj_a_check_point.end());
    }
    static double res_temp = 0.0;
    double res = map_ptr->getResolution();
    if (res_temp != res)
    {
        generate_grid_index_inside(map_ptr);
        res_temp = res;
    }

    traj.insert(traj.end(), index_inside_.begin(), index_inside_.end());
    return true;
}

void TrajectorySamplerRectangularRobot::generate_check_points()
{
    double left_y = 0.5 * robot_y_;
    double right_y = -left_y;

    double front_x = 0.5 * robot_x_;
    double rear_x = -front_x;

    for (double x = rear_x; x <= front_x; x = x + map_resolution_ * 0.7)
    {
        check_points_positive_vx_.emplace_back(x, left_y, 0);
        check_points_positive_vx_.emplace_back(x, right_y, 0);
    }

    for (double y = right_y; y <= left_y; y = y + map_resolution_ * 0.7)
    {
        check_points_positive_vx_.emplace_back(front_x, y, 0);
    }   

    check_points_negtive_vx_ = check_points_positive_vx_;

    for (int i = 0, n = check_points_negtive_vx_.size(); i < n; i++)
    {
        check_points_negtive_vx_[i].x = -check_points_negtive_vx_[i].x;
    }

    check_points_zero_vx_ = check_points_positive_vx_;
    for (double y = right_y; y <= left_y; y = y + map_resolution_ * 0.7)
    {
        check_points_zero_vx_.emplace_back(rear_x, y, 0);
    }  
}

void TrajectorySamplerRectangularRobot::choose_check_points(const Velocity &velocity, std::vector<Pose2D> &check_points)
{
    if (velocity.v_x > 0.02)
    {
        check_points = check_points_positive_vx_;
    }
    else if (velocity.v_x < -0.02)
    {
        check_points = check_points_negtive_vx_;
    }
    else
    {
        check_points = check_points_zero_vx_;
    }
    
}

void TrajectorySamplerRectangularRobot::generate_grid_index_inside(const std::shared_ptr<const grid_map::GridMap>& map_ptr)
{
    index_inside_.resize(0);
    grid_map::Position leftup_position(robot_x_/2.0, robot_y_/2.0);
    grid_map::Index center_index;
    map_ptr->getIndex(leftup_position, center_index);
    grid_map::Index submapStartIndex(center_index);
    grid_map::Index submapBufferSize(robot_x_ / map_ptr->getResolution() +1, robot_y_ / map_ptr->getResolution() + 1);

    for (grid_map::SubmapIterator iterator(*map_ptr, submapStartIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator)
    {
        index_inside_.push_back(*iterator);
    }
}


// CollisionDetector
CollisionDetector::CollisionDetector(ros::NodeHandle nh) : nh_(nh)
{
    ros::NodeHandle pn("~");
    pn.param<std::string>("map_topic_name", map_topic_name_, "/collision_detect_local_map");
    pn.param<std::string>("map_out_put_topic_name", map_out_put_topic_name_, "/detect_area_collision_map");

    double robot_x;
    double robot_y;
    pn.param<double>("robot_x", robot_x, 0.790);
    pn.param<double>("robot_y", robot_y, 0.582);
    pn.param<double>("map_resolution", map_resolution_, 0.03);
    pn.param<double>("local_map_not_update_tolerance_duaration", local_map_not_update_tolerance_duration_, 0.15);
    ROS_INFO_STREAM("local_map_not_update_tolerance_duaration: " << local_map_not_update_tolerance_duration_);

    XmlRpc::XmlRpcValue speed_level_xml, deceleration_dis_level_xml;
    pn.getParam("speed_level", speed_level_xml);
    pn.getParam("deceleration_dis_level", deceleration_dis_level_xml);
    ROS_ASSERT(speed_level_xml.size() == deceleration_dis_level_xml.size());

    int speed_level_num = speed_level_xml.size();
    speed_level_.resize(speed_level_num);
    deceleration_dis_level_.resize(speed_level_num);

    for (int i = 0; i < speed_level_num; i++)
    {
        speed_level_[i] = double(speed_level_xml[i]);
        deceleration_dis_level_[i] = double(deceleration_dis_level_xml[i]);
    }

    local_map_latest_update_time_ = 0;
    collision_check_grid_map_ptr_ = std::make_shared<GridMap>();
    local_map_sub_ = nh_.subscribe(map_topic_name_, 10, &CollisionDetector::local_map_sub_call_back, this);
    trajectory_sampler_ptr_ = std::make_shared<TrajectorySamplerRectangularRobot>(robot_x, robot_y, map_resolution_);
    map_publisher_ = nh_.advertise<grid_map_msgs::GridMap>(map_out_put_topic_name_, 10, true);
}


void CollisionDetector::local_map_sub_call_back(const grid_map_msgs::GridMap& grid_map_msg)
{
    std::lock_guard<std::mutex> lg(local_map_mtx_);
    GridMapRosConverter::fromMessage(grid_map_msg, *collision_check_grid_map_ptr_);
    local_map_latest_update_time_ = ros::Time::now().toSec();
    static bool initialized = false;
    if (!initialized)
    {
        map_layers_ = collision_check_grid_map_ptr_->getLayers();
        sensor_layers_.clear();
        for (auto layer : map_layers_)
        {
            if (layer == detect_area_layer_name_ || layer == all_sensor_layer_name_)
                continue;
            sensor_layers_.push_back(layer);
        }
        initialized = true;
    }
}

CollisionDetector::CollisionCheckResult CollisionDetector::collision_check(const Velocity &vel_in, double deceleration_distance, grid_map::Position& collision_position,  std::string& collision_layer)
{ 
    std::lock_guard<std::mutex> lg(local_map_mtx_);
    if (ros::Time::now().toSec() - local_map_latest_update_time_ > local_map_not_update_tolerance_duration_)
    {
        return NO_MAP;
    }

    std::vector<grid_map::Index> trajectory_index_;
    trajectory_sampler_ptr_->generate_trajectory(vel_in, collision_check_grid_map_ptr_, deceleration_distance, map_resolution_*0.6, trajectory_index_);

    (*collision_check_grid_map_ptr_)[detect_area_layer_name_].setConstant(0.0);
    for (auto& index : trajectory_index_)
    {
        collision_check_grid_map_ptr_->at(detect_area_layer_name_, index) = 100;
    }
    publish_map();

    for (auto& index : trajectory_index_)
    {
        if (collision_check_grid_map_ptr_->at(all_sensor_layer_name_, index) > 99)
        {
            collision_check_grid_map_ptr_->getPosition(index, collision_position);
            for (auto& layer : sensor_layers_)
            {
                if (collision_check_grid_map_ptr_->at(layer, index) > 99)
                {
                    collision_layer = layer;
                    return COllISION;
                }
            }
        }
    }
    
    return NO_COLLISION;
}

void CollisionDetector::collision_avoid(const Velocity &vel_in, Velocity &vel_out)
{
    grid_map::Position collision_position;
    string collision_layer;
    vel_out = vel_in;
    if (fabs(vel_out.v_x) < 0.02 && fabs(vel_out.v_theta) < 0.01)
    {
        ROS_INFO_THROTTLE(2.0, "speed too small, skip collision check");
        return;
    }
    speed_rank_ = std::upper_bound(speed_level_.begin(), speed_level_.end(), fabs(vel_out.v_x)) - speed_level_.begin();
    double decelerate_dis =
        (fabs(vel_out.v_x) - speed_level_[speed_rank_ - 1]) / (speed_level_[speed_rank_] - speed_level_[speed_rank_ - 1]) * deceleration_dis_level_[speed_rank_] +
        (speed_level_[speed_rank_] - fabs(vel_out.v_x)) / (speed_level_[speed_rank_] - speed_level_[speed_rank_ - 1]) * deceleration_dis_level_[speed_rank_ - 1];

    do
    {
        switch (collision_check(vel_out, decelerate_dis, collision_position, collision_layer))
        {
        case NO_MAP:
        {
            vel_out.set(0.0, 0.0, 0.0);
            ROS_INFO_THROTTLE(2.0, "no map or map too old, output zero speed");
            return;
        }
        case COllISION:
        {
            speed_rank_--;
            vel_out.set(sgn(vel_in.v_x)*speed_level_[speed_rank_], 0, vel_in.v_theta * fabs(speed_level_[speed_rank_] / vel_in.v_x));
            decelerate_dis = deceleration_dis_level_[speed_rank_];
            static string temp_collision_layer;
            static double temp_last_print = ros::Time::now().toSec();
            if (temp_collision_layer != collision_layer || ros::Time::now().toSec() - temp_last_print > 1.0)
            {
                ROS_INFO_STREAM("collision detected in layer " << collision_layer << "(position: ["<< collision_position(0) <<  ", " << collision_position(1) << "]), slow down.");
                temp_collision_layer = collision_layer;
                temp_last_print = ros::Time::now().toSec();
            } 
            break;
        }
        case NO_COLLISION:
        {
            return;
        }

        default:
            ROS_ASSERT(false);
        }
    } while (speed_rank_>=1);

    return;
}

void CollisionDetector::collision_avoid(const geometry_msgs::Twist& twist_in, geometry_msgs::Twist& twist_out)
{
    Velocity vel_in, vel_out;
    vel_in.from_twist(twist_in);
    collision_avoid(vel_in, vel_out);
    twist_out = vel_out.to_twist();
}


void CollisionDetector::publish_map()
{
    if (collision_check_grid_map_ptr_->getLayers().size() >= 2)
    {
        ros::Time time = ros::Time::now();
        collision_check_grid_map_ptr_->setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(*collision_check_grid_map_ptr_, message);
        map_publisher_.publish(message);
    }
}

} // namespace collision_detection
