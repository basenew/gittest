#include "local_map_builder.h"

using namespace collision_detection;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_map_builder_node");
    ros::NodeHandle nh;

    //SensorInfo(SensorType _sensor_type, std::string _sensor_name, std::string _topic_name, std::string _map_layer, std::string _sensor_frame_id, double _allowable_failure_time, bool _necessity)
    //SensorInfo bottom_laser_info(LASER, "bottom_laser", "/scan_bottom", "bottom_laser", "bottom_laser_link", 0.3, true); // 底部雷达
    //SensorInfo top_laser_info(LASER, "top_laser", "/scan_top", "top_laser", "top_laser_link", 0.3, true);  // 顶部雷达
    //SensorInfo waist_rgbd_info(RGBD, "waist_rgbd", "/camera/depth/image_raw", "waist_rgbd", "camera_depth_optical_frame", 1, true);   //腰部rgbd
    //SensorInfo top_rgbd_info(RGBD_TOP, "top_rgbd", "/top_camera/depth/image_rect_raw", "top_rgbd", "top_camera_depth_optical_frame", 1, true);   //顶部rgbd
    LocalMapBuilder local_map_builder(nh);

    // if (local_map_builder.add_sensor(bottom_laser_info) == false) return 0;

    local_map_builder.start();
    ROS_INFO_STREAM("local_map_builder start running...");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}