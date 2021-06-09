#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "local_map_builder.h"

using namespace collision_detection;

void test2()
{

    ros::NodeHandle nh;

    // SensorInfo(SensorType _sensor_type, std::string _sensor_name, std::string _topic_name, std::string _map_layer, std::string _sensor_frame_id, double _allowable_failure_time, bool _necessity)
    SensorInfo bottom_laser_info(LASER, "bottom_laser", "/scan_bottom", "bottom_laser", "bottom_laser_link", 0.3, true); // 底部雷达
    SensorInfo top_laser_info(LASER, "top_laser", "/scan_top", "top_laser", "top_laser_link", 0.3, true);                // 顶部雷达
    SensorInfo ultrasonic_set_info(ULTRA_SONIC_SET, "ultrasonic_set", "/sensor_ultrasonic", "ultrasonic", "", 1, true);  //所有超声传感器， frame_id 置空，不去查询tf
    SensorInfo waist_rgbd_info(RGBD, "waist_rgbd", "/camera/depth/image_raw", "waist_rgbd", "camera_depth_optical_frame", 1, true);   //腰部rgbd
    SensorInfo top_rgbd_info(RGBD_TOP, "top_rgbd", "/top_camera/depth/image_rect_raw", "top_rgbd", "top_camera_depth_optical_frame", 1, true);   //顶部rgbd
    LocalMapBuilder local_map_builder(nh);

    if (local_map_builder.add_sensor(bottom_laser_info) == false)
        return ;
    if (local_map_builder.add_sensor(top_laser_info) == false)
        return ;
    if (local_map_builder.add_sensor(ultrasonic_set_info) == false)
        return ;
    if (local_map_builder.add_sensor(waist_rgbd_info) == false) return;
    if (local_map_builder.add_sensor(top_rgbd_info) == false) return;

    local_map_builder.start();
    ROS_INFO_STREAM("local_map_builder start running...");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}

void depth_image_callback(const sensor_msgs::ImageConstPtr ptr)
{
    // sensor_msgs::Image image = *ptr;
}

void depth_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr ptr)
{
    // sensor_msgs::PointCloud2 point_cloud = *ptr;
}

void test1()
{
    ros::NodeHandle nh;

    // ros::Subscriber sub1 = nh.subscribe("/canglong2/image_raw", 10, &depth_image_callback);
    // ros::Subscriber sub2 = nh.subscribe("/camera/depth/image", 10, &depth_image_callback);
    ros::Subscriber sub3 = nh.subscribe("/camera/depth/points", 10, &depth_pointcloud_callback);

    // ros::Subscriber sub1 = nh.subscribe("/top_camera/color/image_raw", 10, &depth_image_callback);
    // ros::Subscriber sub2 = nh.subscribe("/top_camera/depth/image_rect_raw", 10, &depth_image_callback);
    // ros::Subscriber sub3 = nh.subscribe("/top_camera/depth/color/points", 10, &depth_pointcloud_callback);

    ros::Rate rate(10);
    ROS_INFO_STREAM("rgbg test");
    while (ros::ok())
    {
        // v_theta = v_theta + 0.01;
        // vel_in.v_theta = fmod(v_theta, 2.4) - 1.2;

        //collision_detector.collision_check(vel_in, 0.6, collision_index);
        // ROS_INFO_STREAM("vel_in: [" << vel_in.v_x << ", " << vel_in.v_theta << "]");
        // ROS_INFO_STREAM("vel_out: [" << vel_out.v_x << ", " << vel_out.v_theta << "]");
        // collision_detector.collision_check(vel_in, 0.6, collision_index);

        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rgbd_test");

    test2();
    return 0;
}