#ifndef __CHASSIS_MOVE_LOCAL_MAP_BUILDER__
#define __CHASSIS_MOVE_LOCAL_MAP_BUILDER__

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <functional>
#include <mutex>


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <grid_map_msgs/GridMap.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


namespace collision_detection
{
class SinCosLookUpTableFor2DScans
{
public:
    static SinCosLookUpTableFor2DScans* get_instance()
    {
        static SinCosLookUpTableFor2DScans look_up_table;
        return &look_up_table;
    }

    std::pair<std::vector<double>, std::vector<double>>& look_up_sincos(const sensor_msgs::LaserScan::ConstPtr scan_ptr);

private:
    SinCosLookUpTableFor2DScans() {}

    std::map<int, std::pair<std::vector<double>, std::vector<double>>> sin_cos_data_;
};

struct Transform2D
{
    double cos_theta;
    double sin_theta;
    double trans_x;
    double trans_y;

    void from_tf_transform(const tf::Transform &transform);
};


// 2D坐标点
struct Point2D
{
    double x_;
    double y_;

    Point2D(double x = 0.0, double y = 0.0);
    Point2D(const tf::Vector3 &vec3);

    tf::Vector3 to_vector3() const;
    Point2D transform(const tf::Transform &transform) const;
    void transform_in_place(const tf::Transform &transform);
    void transform_in_place(const Transform2D &transform);
};

// 一帧雷达数据生成的点集合
class LaserScanPointData
{
public:
    typedef std::shared_ptr<collision_detection::LaserScanPointData> Ptr;
    LaserScanPointData(){};
    LaserScanPointData(std::vector<Point2D> &scan_point_data) { point_data_ = scan_point_data; }
    LaserScanPointData(const sensor_msgs::LaserScan::ConstPtr &scan_ptr);
    void fill_data(const sensor_msgs::LaserScan::ConstPtr &scan_ptr);
    LaserScanPointData transform(const tf::Transform &trans) const;
    void transform_in_place(const tf::Transform &trans);
    void transform_in_place(const Transform2D &transform);

    std::vector<Point2D> point_data_;    
};

// 传感器类型
enum SensorType
{
    LASER = 0,
    LIDAR,
    ULTRA_SONIC,
    ULTRA_SONIC_SET
};

// 传感器信息
struct SensorInfo
{
    SensorInfo() {}
    SensorInfo(SensorType _sensor_type, std::string _sensor_name, std::string _topic_name, std::string _map_layer, std::string _sensor_frame_id, double _allowable_failure_time, bool _necessity)
        : type(_sensor_type), name(_sensor_name), topic_name(_topic_name), map_layer(_map_layer), frame_id(_sensor_frame_id),
          allowable_failure_time(_allowable_failure_time), necessity(_necessity) {}
    SensorType type;
    std::string name;
    std::string topic_name;
    std::string map_layer;
    std::string frame_id;
    tf::StampedTransform base_link_to_sensor_transform; //订阅tf后查询得到
    double allowable_failure_time; // 允许失效时间
    bool necessity; // 必要意思是，若缺少该传感器的数据，则认为地图更新失败
};

// 传感器数据处理
class SensorSubscriber
{
public:
    SensorSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr);
    bool time_out(double current_time);
    virtual bool update_map(double t) = 0;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    SensorInfo sensor_info_;
    std::shared_ptr<grid_map::GridMap> map_ptr_;
    std::mutex sensor_data_process_mutex_;
    bool has_new_data_;
    double time_last_update_;
    std::string layer_;
};

// 激光雷达数据处理
class LaserSubscriber : public SensorSubscriber
{
public:
    LaserSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr);
    bool update_map(double t);

private:
    void laser_call_back(const sensor_msgs::LaserScanConstPtr &msg_ptr);
    LaserScanPointData laser_scan_point_data_;
    sensor_msgs::LaserScanConstPtr scan_ptr_;
    Transform2D transform_baselink_to_sensor_;
    double x_max_;
    double x_min_;
    double y_max_;
    double y_min_;
};


// 超声数据处理 
class UltraSonicSubscriber : public SensorSubscriber
{
public:    
    UltraSonicSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr);
    bool update_map(double t);

private:
    void ultrasonic_call_back(const sensor_msgs::RangeConstPtr &msg_ptr);
    sensor_msgs::RangeConstPtr range_ptr_;
};

// 工厂类
class SensorSubscriberFactory
{
public:
    static SensorSubscriberFactory *get_instance()
    {
        static SensorSubscriberFactory sensor_subscriber_factory;
        return &sensor_subscriber_factory;
    }

    std::shared_ptr<SensorSubscriber> create_subscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
    {
        auto iter = class_map_.find(_sensor_info.type);
        if (iter == class_map_.end())
            return (std::shared_ptr<SensorSubscriber> ());
        else
            return iter->second(_nh, _sensor_info, _map_ptr);
      
    }
    
private:
    SensorSubscriberFactory()
    {
        class_map_.insert(std::make_pair(LASER,
                                         [](ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
                                             -> std::shared_ptr<SensorSubscriber> { return std::make_shared<LaserSubscriber>(_nh, _sensor_info, _map_ptr); }));                                     
        class_map_.insert(std::make_pair(ULTRA_SONIC,
                                         [](ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
                                             -> std::shared_ptr<SensorSubscriber> { return std::make_shared<UltraSonicSubscriber>(_nh, _sensor_info, _map_ptr); }));                                                                               
    }

    std::map<SensorType, std::function<std::shared_ptr<SensorSubscriber> (ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)>> class_map_;
};

// 地图构建类
class LocalMapBuilder
{
public:
    LocalMapBuilder(ros::NodeHandle nh_);

    // 增加传感器，sensor_info中只需包含sensor_type，sensor_name， topic_name, frame_id.
    // tf 信息会自动查询得到
    bool add_sensor(SensorInfo &sensor_info);
    void publish_map();
    void update(const ros::TimerEvent &e);
    void start();

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    ros::Timer map_publish_timer_;
    ros::Publisher map_publisher_;

    std::shared_ptr<grid_map::GridMap> map_ptr_;
    std::vector<SensorInfo> sensor_info_vec_;
    std::vector<std::shared_ptr<SensorSubscriber>> subscriber_shared_ptrs;

    std::string map_frame_id_;
    std::string map_topic_name_;

    std::string all_sensor_layer_name_;

    // 更新失败打印标志
    std::vector<bool> update_fail_print_flags_;

    ros::Timer refresh_sensor_switch_status_timer_;
    void refresh_sensor_switch_status(const ros::TimerEvent &e);
    bool ignore_sensor_all_;
};
}





#endif