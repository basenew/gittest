#include "collision_detector.h"
#include "geometry_msgs/Twist.h"

using namespace collision_detection;

Velocity vel_in, vel_out;
void vel_cmd_callback(const geometry_msgs::Twist::ConstPtr& twist_ptr)
{

    vel_in.from_twist(*twist_ptr);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "collision_avoid_test_node");
    ros::NodeHandle nh;

    collision_detection::CollisionDetector collision_detector(nh);

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_chassis", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_test", 10, &vel_cmd_callback);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate rate(10);
    grid_map::Index index;
    grid_map::Index collision_index;


    while (ros::ok())
    {
        // v_theta = v_theta + 0.01;
        // vel_in.v_theta = fmod(v_theta, 2.4) - 1.2;

        //collision_detector.collision_check(vel_in, 0.6, collision_index);
        // ROS_INFO_STREAM("vel_in: [" << vel_in.v_x << ", " << vel_in.v_theta << "]");
        // ROS_INFO_STREAM("vel_out: [" << vel_out.v_x << ", " << vel_out.v_theta << "]");
        // collision_detector.collision_check(vel_in, 0.6, collision_index);
        collision_detector.collision_avoid(vel_in, vel_out);
        geometry_msgs::Twist twist_out = vel_out.to_twist();
        twist_pub.publish(twist_out);
        ROS_INFO_STREAM("vel_in: [" << vel_in.v_x << ", " << vel_in.v_theta << "]");
        ROS_INFO_STREAM("vel_out: [" << vel_out.v_x << ", " << vel_out.v_theta << "]");
        rate.sleep();
    }

    return 0;
}

