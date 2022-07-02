#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include "car_speed_controller/SpeedControllerConfig.h"

ros::Publisher throttle_publisher;
double positive_factor = 1.0;
double negative_factor = 1.0;
double break_threshold = 0.1;
double odom = 0.0;
double set_point = 0.0;
bool first_reconfig = true;

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

    if (drive.data > 1.0) drive.data = 1.0;
    if (drive.data < -1.0) drive.data = -1.0;
    // break well
    if ((set_point < break_threshold) && (odom > 0.0)) {
        drive.data = -1.0;
    }

    throttle_publisher.publish(drive);
}

void reconfigure_callback(car_speed_controller::SpeedControllerConfig &config, uint32_t level)
{
    if (first_reconfig)
    {
        config.positive_factor = positive_factor;
        config.negative_factor = negative_factor;
    
        first_reconfig = false;
    } 
    else 
    {
        positive_factor = config.positive_factor;
        negative_factor = config.negative_factor;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::NodeHandle pH("~");

    pH.param("positive_factor", positive_factor, 1.0);
    pH.param("negative_factor", negative_factor, 1.0);
    pH.param("break_threshold", break_threshold, 0.1);

    throttle_publisher = n.advertise<std_msgs::Float64>("/actuator/throttle", 1);

    // Subscribe to /actuator/drive
    ros::Subscriber set_point = n.subscribe("/controller/throttle", 1, set_point_callback);   
    ros::Subscriber odom = n.subscribe("/odom", 1, odom_callback);   

    dynamic_reconfigure::Server<car_speed_controller::SpeedControllerConfig> dyn_conf_srv(pH);
    dynamic_reconfigure::Server<car_speed_controller::SpeedControllerConfig>::CallbackType f;
    f = boost::bind(&reconfigure_callback, _1, _2);
    dyn_conf_srv.setCallback(f);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
