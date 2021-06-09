#include "ros/ros.h"
#include "utility.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"


#define MIN_HEIGHT  -5
#define MAX_HEIGHT   5

class MultiToSingle
{
public:
    ros::NodeHandle nh;
    ros::Subscriber subLidarCloud;
    ros::Publisher pubLaserScan_1;
    ros::Publisher pubLaserScan_3;
    ros::Publisher pubLaserScan_r1;
    ros::Publisher pubLaserScan_r3;

private:
    double angle_min_, angle_max_, angle_increment_, range_min_, range_max_, scan_time_;
    std::string frame_id_;
    bool use_inf_;
    double inf_epsilon_;

public:
    MultiToSingle():nh("~")
    {
        nh.param<double>("angle_min", angle_min_, -3.14);
        nh.param<double>("angle_max", angle_max_, 3.14);
        nh.param<double>("angle_increment", angle_increment_, 0.007);
        nh.param<double>("scan_time", scan_time_, 0.0);
        nh.param<double>("range_min", range_min_, 0.1);
        nh.param<double>("range_max", range_max_, 8.0);
        nh.param<std::string>("frame_id", frame_id_, "laser");

        ROS_INFO("angle_min: %f", angle_min_);
        ROS_INFO("angle_max: %f", angle_max_);
        ROS_INFO("angle_increment: %f", angle_increment_);
        ROS_INFO("scan_time: %f", scan_time_);
        ROS_INFO("range_min: %f", range_min_);
        ROS_INFO("range_max: %f", range_max_);
        ROS_INFO("frame_id: %s", frame_id_.c_str());

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_1.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_3.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_r1.reset(new pcl::PointCloud<PointType>());
        laserCloudOut_r3.reset(new pcl::PointCloud<PointType>());
        subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &MultiToSingle::cloudHandler, this);

        pubLaserScan_1 = nh.advertise<sensor_msgs::LaserScan> ("/scan_1", 1);
        pubLaserScan_3 = nh.advertise<sensor_msgs::LaserScan> ("/scan_3", 1);
        pubLaserScan_r1 = nh.advertise<sensor_msgs::LaserScan> ("/scan_r1", 1);
        pubLaserScan_r3 = nh.advertise<sensor_msgs::LaserScan> ("/scan_r3", 1);
    }

    void cloud_to_laser_scan(pcl::PointCloud<PointType>::Ptr cloud_msg, sensor_msgs::LaserScan *scan_msg)
    {
        static uint32_t seq = 0;
        scan_msg->header.stamp = ros::Time::now();
        scan_msg->header.seq = seq++;
        scan_msg->header.frame_id = frame_id_;

        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->scan_time = 0.1;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;

        // determine amount of rays to create
        uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.resize(ranges_size);

        scan_msg->time_increment = scan_msg->scan_time/ranges_size;

        // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
        // if (use_inf_) {
        //   scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        // } else {
        //   scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
        // }

        // Transform cloud if necessary
        // if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
        //   try {
        //     auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        //     tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
        //     cloud_msg = cloud;
        //   } catch (tf2::TransformException & ex) {
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
        //     return;
        //   }
        // }

        // Iterate through pointcloud
        size_t cloudSize = cloud_msg->points.size();
        PointType thisPoint;

        for (int i=0; i<cloudSize; i++)
        {
            thisPoint.x = cloud_msg->points[i].x;
            thisPoint.y = cloud_msg->points[i].y;
            thisPoint.z = cloud_msg->points[i].z;

          if (std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) {
            continue;
          }

          double range = hypot(thisPoint.x, thisPoint.y);
          if (range < range_min_) {
            continue;
          }
          if (range > range_max_) {
            continue;
          }

          double angle = atan2(thisPoint.y, thisPoint.x);
          if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
            continue;
          }
          // overwrite range at laserscan ray if new range is smaller
          int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
          if(index >= ranges_size){
            continue;
          }

          scan_msg->ranges[index] = range;
          

        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        std_msgs::Header cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        laserCloudIn->clear();
        laserCloudOut_1->clear();
        laserCloudOut_3->clear();
        laserCloudOut_r1->clear();
        laserCloudOut_r3->clear();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // float verticalAngle, horizonAngle, range;
        // size_t rowIdn, columnIdn, index = 0, cloudSize; 
        // PointType thisPoint;

        // cloudSize = laserCloudIn->points.size();
        // uint16_t *p_ring = (uint16_t*) (&laserCloudMsg->data[16]);

        // for (int i = 0; i < cloudSize; i++)
        // {
        //     uint16_t *p_ring = (uint16_t*) (&laserCloudMsg->data[22* i + 16]);
        //     uint16_t ring = *p_ring;
        //     ROS_INFO_STREAM(ring);
        //     if (ring == 7)
        //     {
        //         laserCloudOut_1->points.push_back(laserCloudIn->points[i]);
        //     }
        // }
        // sensor_msgs::PointCloud2 laserCloudTemp;
        // pcl::toROSMsg(*laserCloudOut_1, laserCloudTemp);
        // laserCloudTemp.header.stamp = cloudHeader.stamp;
        // laserCloudTemp.header.frame_id = cloudHeader.frame_id;
        // pubLaserScan_1.publish(laserCloudTemp);

        size_t cloudSize = laserCloudIn->points.size();
        PointType thisPoint;
        
        for (size_t i = 0; i < cloudSize; ++i)
        {
          if(MIN_HEIGHT <= laserCloudIn->points[i].z && laserCloudIn->points[i].z <= MAX_HEIGHT){
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            float verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            int rowIdn;
            if(verticalAngle >0){
              rowIdn = (verticalAngle + 0.1f);
            }
            else{
              rowIdn = (verticalAngle - 0.1f);
            }
            
            if (rowIdn == -1)// degree
            {
              laserCloudOut_r1->points.push_back(thisPoint);
            }
            else if(rowIdn == -3){
              laserCloudOut_r3->points.push_back(thisPoint);
            }
            else if(rowIdn == 1){
              laserCloudOut_1->points.push_back(thisPoint);
            }
            else if(rowIdn == 3){
              laserCloudOut_3->points.push_back(thisPoint);
            }
            //
          }
        }
        sensor_msgs::LaserScan scan_msg;
        cloud_to_laser_scan(laserCloudOut_r1, &scan_msg);
        pubLaserScan_r1.publish(scan_msg);

        cloud_to_laser_scan(laserCloudOut_r3, &scan_msg);
        pubLaserScan_r3.publish(scan_msg);

        cloud_to_laser_scan(laserCloudOut_1, &scan_msg);
        pubLaserScan_1.publish(scan_msg);

        cloud_to_laser_scan(laserCloudOut_3, &scan_msg);
        pubLaserScan_3.publish(scan_msg);


        // sensor_msgs::PointCloud2 laserCloudTemp;
        // pcl::toROSMsg(*laserCloudOut_1, laserCloudTemp);
        // laserCloudTemp.header.stamp = cloudHeader.stamp;
        // laserCloudTemp.header.frame_id = cloudHeader.frame_id;
        
    }

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr laserCloudOut_1;
    pcl::PointCloud<PointType>::Ptr laserCloudOut_3;
    pcl::PointCloud<PointType>::Ptr laserCloudOut_r1;
    pcl::PointCloud<PointType>::Ptr laserCloudOut_r3;
    string pointCloudTopic = "/velodyne_points";
};