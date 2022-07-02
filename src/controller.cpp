#include "ros/ros.h"
#include "std_msgs/Float64.h"

ros::Publisher throttle_publisher;
double positive_factor = 1.0;
double negative_factor = 1.0;
double break_threshold = 0.1;
double odom = 0.0;
double set_point = 0.0;

void apply(double set_point, double odom);

void odom_callback(const std_msgs::Float64 msg)
{
    odom = msg.data;
    apply(set_point, odom);
}

void set_point_callback(const std_msgs::Float64 msg)
{
    set_point = msg.data;
    apply(set_point, odom);
}

void apply(double set_point, double odom)
{
    std_msgs::Float64 drive;

    if (odom > 1) odom = 1;
    if (set_point > 1) set_point = 1;

    double delta = set_point - odom;

    if (delta >= 0)
        drive.data = positive_factor * delta;
    else
        drive.data = negative_factor * delta;

    // break well
    if ((set_point < break_threshold) && (odom > 0.0) {
        drive.data = -1.0;
    }

    throttle_publisher.publish(drive);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::NodeHandle pH("~");

    pH.param("positive_factor", positive_factor, 1);
    pH.param("negative_factor", negative_factor, 1);
    ph.param("break_threshold", break_threshold, 0.1);

    throttle_publisher = n.advertise<std_msgs::Float64>("/actuator/throttle", 1);

    // Subscribe to /actuator/drive
    ros::Subscriber set_point = n.subscribe("/controller/throttle", 1, set_point_callback);   
    ros::Subscriber odom = n.subscribe("/odom", 1, odom_callback);   

    // Handle ROS communication events
    ros::spin();

    return 0;
}