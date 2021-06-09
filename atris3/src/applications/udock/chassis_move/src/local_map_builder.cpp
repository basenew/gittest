#include "local_map_builder.h"

using namespace ros;
using namespace grid_map;
using namespace std;

namespace collision_detection
{
// std::mutex g_map_update_mtx;

// SinCosLookUpTableFor2DScans
std::pair <std::vector<double>, std::vector<double>>& SinCosLookUpTableFor2DScans::look_up_sincos(const sensor_msgs::LaserScan::ConstPtr scan_ptr)
{
    // here do a simple hash operation.
    int key = scan_ptr->angle_min * 102.0 + scan_ptr->angle_increment * 97848.0 + scan_ptr->angle_max * 89.0;
    // std::cout << "SinCosLookUpTableFor2DScans :: look_up_sincos(): the key is " << key << std::endl;

    auto it = sin_cos_data_.find(key);
    if (it != sin_cos_data_.end())
    {
        return it->second;
    }
    else
    {
        ROS_INFO_STREAM("SinCosLookUpTableFor2DScans :: look_up_sincos(): generate new table..." );
        std::vector<double> sins(scan_ptr->ranges.size(), 0.0);
        std::vector<double> coss(scan_ptr->ranges.size(), 0.0);
        float angle_start = scan_ptr->angle_min;
        for (int i = 0, n = scan_ptr->ranges.size(); i < n; i++)
        {
            sins[i] = sin(angle_start);
            coss[i] = cos(angle_start);
            angle_start = angle_start + scan_ptr->angle_increment;
        }

        std::pair<std::vector<double>, std::vector<double>> res = std::make_pair(sins, coss);
        sin_cos_data_.insert({key, res});
        return sin_cos_data_[key];
    }
}


void Transform2D::from_tf_transform(const tf::Transform &transform)
{
    cos_theta = transform.getBasis()[0][0];
    sin_theta = transform.getBasis()[1][0];

    trans_x = transform.getOrigin().x();
    trans_y = transform.getOrigin().y();

    // ROS_INFO_STREAM("cos_theta " << cos_theta << " sin_theta" << sin_theta << " trans_x" << trans_x << " trans_y" << trans_y);
}

// Point2D
Point2D::Point2D(double x, double y)
{
    x_ = x;
    y_ = y;
}

Point2D::Point2D(const tf::Vector3 &vec3)
{
    x_ = vec3.x();
    y_ = vec3.y();
}

tf::Vector3 Point2D::to_vector3() const
{
    tf::Vector3 res;
    res.setX(x_);
    res.setY(y_);
    return res;
}

Point2D Point2D::transform(const tf::Transform &transform) const
{
    return Point2D(transform(this->to_vector3()));
}

void Point2D::transform_in_place(const tf::Transform &transform)
{
    *this = transform(this->to_vector3());
}

void Point2D::transform_in_place(const Transform2D &transform)
{
    // 本应该按如下写法
    // double x_temp = transform.cos_theta * x_ - transform.sin_theta * y_ + transform.trans_x;
    // double y_temp = transform.sin_theta * x_ + transform.cos_theta * y_ + transform.trans_y;


    // 为了加快处理速度，考虑到uv上激光雷达没有y 偏移和 转角，做了简化
    x_ += transform.trans_x;
}

// LaserScanPointData
LaserScanPointData::LaserScanPointData(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
    fill_data(scan_ptr);
}

void LaserScanPointData::fill_data(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
    point_data_.clear();
    point_data_.reserve(scan_ptr->ranges.size());
    auto& sin_cos_s = SinCosLookUpTableFor2DScans::get_instance()->look_up_sincos(scan_ptr);
    for (int i = 0, n = scan_ptr->ranges.size(); i < n; i++)
    {
        if (scan_ptr->ranges[i] > 1.5) continue;
        point_data_.emplace_back(sin_cos_s.second[i] * scan_ptr->ranges[i], sin_cos_s.first[i] * scan_ptr->ranges[i]);
    }
}

LaserScanPointData LaserScanPointData::transform(const tf::Transform &trans) const
{
    LaserScanPointData res;
    res.point_data_.resize(this->point_data_.size());
    int i = 0;
    for (auto const& point : this->point_data_)
    {
        res.point_data_[i] = point.transform(trans);
        i++;
    }
    return res;
}

void LaserScanPointData::transform_in_place(const tf::Transform &trans)
{
    for (auto &point : this->point_data_)
    {
        point.transform_in_place(trans);
    }
}

void LaserScanPointData::transform_in_place(const Transform2D &transform)
{
    for (auto &point : this->point_data_)
    {
        point.transform_in_place(transform);
    }    
}


// SensorSubscriber
SensorSubscriber::SensorSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
    : nh_(_nh), sensor_info_(_sensor_info), map_ptr_(_map_ptr),has_new_data_(false), time_last_update_(0.0), layer_(_sensor_info.map_layer)
{
}


bool SensorSubscriber::time_out(double current_time)
{
    return ((current_time - time_last_update_) > sensor_info_.allowable_failure_time);
}

////////////////////////////////////
// LaserSubscriber
///////////////////////////////////
LaserSubscriber::LaserSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
    : SensorSubscriber(_nh, _sensor_info, _map_ptr)
{
    sub_ = nh_.subscribe(sensor_info_.topic_name, 10, &LaserSubscriber::laser_call_back, this);
    transform_baselink_to_sensor_.from_tf_transform(sensor_info_.base_link_to_sensor_transform);
    x_max_ = _map_ptr->getSize().x() / 2.0 - transform_baselink_to_sensor_.trans_x - 0.03;
    x_min_ = -_map_ptr->getSize().x() / 2.0 - transform_baselink_to_sensor_.trans_x + 0.03;
    y_max_ = _map_ptr->getSize().y() / 2.0 - transform_baselink_to_sensor_.trans_y - 0.03;
    y_min_ = -_map_ptr->getSize().y() / 2.0 - transform_baselink_to_sensor_.trans_y + 0.03;
}

void LaserSubscriber::laser_call_back(const sensor_msgs::LaserScanConstPtr &msg_ptr)
{
    std::lock_guard<std::mutex> l_g(sensor_data_process_mutex_);
    scan_ptr_ = msg_ptr;
    has_new_data_ = true;
    time_last_update_ = msg_ptr->header.stamp.toSec();
}
bool LaserSubscriber::update_map(double t)
{ 
    sensor_data_process_mutex_.lock();
    if (has_new_data_ == false)
    {
        sensor_data_process_mutex_.unlock();
        if (time_out(t) == false)
        {
            return true;
        }
        else
        {
            // std::lock_guard<std::mutex> l_g(g_map_update_mtx);
            (*map_ptr_)[layer_].setConstant(0.0);
            return false;
        }
    }
    sensor_msgs::LaserScanConstPtr scan_ptr_temp(new sensor_msgs::LaserScan(*scan_ptr_));
    has_new_data_ = false;
    sensor_data_process_mutex_.unlock();

    laser_scan_point_data_.fill_data(scan_ptr_temp);
    laser_scan_point_data_.transform_in_place(transform_baselink_to_sensor_);
    int n = laser_scan_point_data_.point_data_.size();
    Position position;
    // std::lock_guard<std::mutex> l_g(g_map_update_mtx);
    (*map_ptr_)[layer_].setConstant(0.0);
    for (int i = 0; i < n; i++)
    {
        if (laser_scan_point_data_.point_data_[i].x_ < x_min_ ||
            laser_scan_point_data_.point_data_[i].x_ > x_max_ ||
            laser_scan_point_data_.point_data_[i].y_ < y_min_ ||
            laser_scan_point_data_.point_data_[i].y_ > y_max_)
            continue;
        position << laser_scan_point_data_.point_data_[i].x_, laser_scan_point_data_.point_data_[i].y_;
        if (map_ptr_->isInside(position))
        {
            map_ptr_->atPosition(layer_, position) = 100.0;
        }
    }
    return true;
}


/////////////////////////////////////
// UltraSonicSubscriber
/////////////////////////////////////
UltraSonicSubscriber::UltraSonicSubscriber(ros::NodeHandle _nh, SensorInfo &_sensor_info, std::shared_ptr<grid_map::GridMap> _map_ptr)
    : SensorSubscriber(_nh, _sensor_info, _map_ptr)
{
    sub_ = nh_.subscribe(sensor_info_.topic_name, 10, &UltraSonicSubscriber::ultrasonic_call_back, this);
}

void UltraSonicSubscriber::ultrasonic_call_back(const sensor_msgs::RangeConstPtr &msg_ptr)
{
    std::lock_guard<std::mutex> l_g(sensor_data_process_mutex_);
    range_ptr_ = msg_ptr;
    has_new_data_ = true;
    time_last_update_ = msg_ptr->header.stamp.toSec();
}

bool UltraSonicSubscriber::update_map(double t)
{
    static Position old_position(0.0, 0.0);
    sensor_data_process_mutex_.lock();
    if (has_new_data_ == false)
    {
        sensor_data_process_mutex_.unlock();
        if (time_out(t) == false)
        {
            return true;
        }
        else
        {
            // std::lock_guard<std::mutex> l_g(g_map_update_mtx);
            map_ptr_ -> atPosition(layer_, old_position) = 0.0;
            old_position << 0.0, 0.0;
            return false;
        }
    }
    
    has_new_data_ = false;
    sensor_msgs::RangeConstPtr range_ptr_temp(new sensor_msgs::Range(*range_ptr_));
    sensor_data_process_mutex_.unlock();

    Point2D end_point(range_ptr_temp->range, 0);
    end_point.transform_in_place(sensor_info_.base_link_to_sensor_transform);
    // std::lock_guard<std::mutex> l_g(g_map_update_mtx);
    map_ptr_ -> atPosition(layer_, old_position) = 0.0;
    old_position << 0.0, 0.0;
    Position position;
    position << end_point.x_, end_point.y_;
    if (map_ptr_->isInside(position))
    {
        map_ptr_->atPosition(layer_, position) = 100.0;
        old_position = position;
    }

    return true;
}


///////////////////////////////////////////
// LocalMapBuilder
///////////////////////////////////////////
LocalMapBuilder::LocalMapBuilder(ros::NodeHandle _nh): nh_(_nh)
{
    ros::NodeHandle pn("~");
    all_sensor_layer_name_ = "all_sensor";
    double map_length, map_width, map_resolution, map_update_period;
    pn.param<double>("map_length", map_length, 2.0);
    pn.param<double>("map_width", map_width, 1.0);
    pn.param<double>("map_resolution", map_resolution, 0.04);
    pn.param<double>("map_update_period", map_update_period, 0.08);
    pn.param<std::string>("map_frame_id", map_frame_id_, "base_link");
    pn.param<std::string>("map_topic_name", map_topic_name_, "/collision_detect_local_map");
    ROS_INFO_STREAM("map_update_period: " << map_update_period);
    map_ptr_ = std::make_shared<GridMap>(std::vector<std::string>{"detect_area", all_sensor_layer_name_});
    map_ptr_->setFrameId(map_frame_id_);
    map_ptr_->setGeometry(Length(map_length, map_width), map_resolution);
    ROS_INFO("created map , frame_id : %s , size %f x %f m (%i x %i cells).", map_frame_id_.c_str(),
             map_ptr_->getLength().x(), map_ptr_->getLength().y(),
             map_ptr_->getSize()(0), map_ptr_->getSize()(1));

    // publish map
    map_publisher_ = nh_.advertise<grid_map_msgs::GridMap>(map_topic_name_, 10, true);
    map_publish_timer_ = nh_.createTimer(ros::Duration(map_update_period), &LocalMapBuilder::update, this, false, false);
    refresh_sensor_switch_status_timer_ = nh_.createTimer(ros::Duration(1.0), &LocalMapBuilder::refresh_sensor_switch_status, this, false, true);
}

bool LocalMapBuilder::add_sensor(SensorInfo &sensor_info)
{
    // 获取 传感器与base_link之间的坐标变换
    // 多查询几次才能确保查询到，don't know why
    int query_time = 0;
    while (ros::ok() && !sensor_info.frame_id.empty())
    {
        query_time++;
        try
        {
            tf_listener_.lookupTransform(map_frame_id_, sensor_info.frame_id,
                                    ros::Time(0), sensor_info.base_link_to_sensor_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            if (query_time > 10)
            {
                ROS_ERROR_STREAM("sensor frame lookup failed, sensor frame id: " << sensor_info.frame_id);
                return false;
            }
            ros::Duration(0.5).sleep();
            continue;
        }
        break;
    }

    // 生成 subscriber,并处理传感器数据
    auto sensor_sub_ = SensorSubscriberFactory::get_instance()->create_subscriber(nh_, sensor_info, map_ptr_);
    if (sensor_sub_)
    {
        sensor_info_vec_.push_back(sensor_info);
        subscriber_shared_ptrs.push_back(sensor_sub_);
        std::vector<std::string> layers = map_ptr_->getLayers();
        if (std::find(layers.cbegin(), layers.cend(), sensor_info.map_layer) == layers.end())
        {
            map_ptr_->add(sensor_info.map_layer);
        }
        ROS_INFO_STREAM("a sensor subscriber created, sensor name: " << sensor_info.name);
        update_fail_print_flags_.resize(sensor_info_vec_.size(), true);
        return true;
    }
    ROS_ERROR_STREAM("SensorSubscriberFactory create subscriber failed, sensor type: " << sensor_info.type);
    return false;
}
void LocalMapBuilder::start()
{
    map_publish_timer_.start();
}


void LocalMapBuilder::update(const ros::TimerEvent &e)
{
    (*map_ptr_)[all_sensor_layer_name_].setConstant(0.0);
    if (!ignore_sensor_all_)
    {
        for (int i = 0, n = subscriber_shared_ptrs.size(); i < n; i++)
        {
            auto &sensor_subscriber_ptr = subscriber_shared_ptrs[i];
            if (!sensor_subscriber_ptr->update_map(ros::Time::now().toSec()) && sensor_subscriber_ptr->sensor_info_.necessity)
            {
                ROS_WARN_STREAM_COND(update_fail_print_flags_[i], "local collision map update failed: no " << sensor_subscriber_ptr->sensor_info_.name << " data received...");
                update_fail_print_flags_[i] = false;
                return;
            }
            ROS_WARN_STREAM_COND(!update_fail_print_flags_[i], "local collision map recover updating.");
            update_fail_print_flags_[i] = true;
            (*map_ptr_)[all_sensor_layer_name_] += (*map_ptr_)[sensor_subscriber_ptr->sensor_info_.map_layer];
        }
    }
    publish_map();
}



void LocalMapBuilder::publish_map()
{
    // std::lock_guard<std::mutex> l_g(g_map_update_mtx);
    if (map_ptr_->getLayers().size() >= 2)
    {
        ros::Time time = ros::Time::now();
        map_ptr_->setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(*map_ptr_, message);
        map_publisher_.publish(message);
    }
}

void LocalMapBuilder::refresh_sensor_switch_status(const ros::TimerEvent &e)
{
    if (!ros::param::get("/ignore_sensor_all", ignore_sensor_all_))
    {
        ignore_sensor_all_ = false;
        ROS_WARN_STREAM("get parameter \"ignore_sensor_all\" failed, default value is " << std::boolalpha << ignore_sensor_all_);
    }
    static bool ignore_sensor_all_last = !ignore_sensor_all_;
    if (ignore_sensor_all_last != ignore_sensor_all_)
    {
        ROS_INFO_STREAM("parameter ignore_sensor_all set to: " << std::boolalpha << ignore_sensor_all_);
        ignore_sensor_all_last = ignore_sensor_all_;
    }
}


}

