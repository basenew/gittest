#include "udock2/dock_unit_test_localization.h"

namespace dock
{

DockUnit::DockUnit(ros::NodeHandle nh):nh_(nh)
{
    laser_scan_sub_ = nh_.subscribe("/scan_bottom", 10, &DockUnit::laser_scan_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_.ns = "basic_shapes";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.scale.x = 0.13;
    marker_.scale.y = 0.13;
    marker_.scale.z = 0.13;
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration(0.5);
}

void DockUnit::laser_scan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
    dock::Pose dock_pose;
    dock::Pose robot_pose;
    dock::Point start_point, end_point;
    if (dock_recognizer_.localize_in_dock_frame(msg_ptr, robot_pose, start_point, end_point))
    {
        // ROS_WARN_STREAM("dock found, the pose is  [" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << "] .");
        ROS_WARN_STREAM("localize in dock frame successfully, the pose is  [" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << "] .");
        marker_.header.stamp = ros::Time::now();
        marker_.header.frame_id = msg_ptr->header.frame_id;
        // marker_.id = 0;
        // marker_.pose.position.x = dock_pose.x;
        // marker_.pose.position.y = dock_pose.y;
        // marker_pub_.publish(marker_);

        marker_.id = 1;
        marker_.pose.position.x = start_point.x;
        marker_.pose.position.y = start_point.y;
        marker_pub_.publish(marker_);

        marker_.id = 2;
        marker_.pose.position.x = end_point.x;
        marker_.pose.position.y = end_point.y;
        marker_pub_.publish(marker_);

        

    };
}

}