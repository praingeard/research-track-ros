#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <vector>
#include <string>
#include "std_msgs/Float64.h"

ros::Publisher pub;
double scan_l;
double scan_r;
double scan_fl;
double scan_fr;
double lin_speed = 1.0;

void scanLeftCallback(const sensor_msgs::LaserScan &msg)
{
    scan_l = msg.range_min;
}

void scanRightCallback(const sensor_msgs::LaserScan &msg)
{
    scan_r = msg.range_min;
}
void scanFrontLeftCallback(const sensor_msgs::LaserScan &msg)
{
    scan_fl = msg.range_min;
}
void scanFrontRightCallback(const sensor_msgs::LaserScan &msg)
{
    scan_fr = msg.range_min;
}
void speedCallback(const std_msgs::Float64 &msg){
    lin_speed = msg.data;
}

void timerCallback(const ros::TimerEvent& event){
    geometry_msgs::Twist speed;
    double min_scan = std::min(scan_fr,scan_fl);
    if (min_scan < 0.6){
        speed.linear.x = 0.1;
        if (scan_l > scan_r){
            speed.angular.z = -30.0*abs(scan_r - scan_l);
        }
        else{
            speed.angular.z = 30.0*abs(scan_r - scan_l);
        }
    }
    else{
        speed.linear.x = lin_speed*min_scan;
        if (scan_fl > scan_fr){
            speed.angular.z = -2.5*lin_speed*abs(scan_fr - scan_fl);
        }
        else{
            speed.angular.z = 2.5*lin_speed*abs(scan_fr - scan_fl);
        }
    }
    pub.publish(speed);
}
int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub5 = nh.subscribe("current_speed", 1, speedCallback);
    ros::Subscriber sub1 = nh.subscribe("scan_left", 1, scanLeftCallback);
    ros::Subscriber sub2 = nh.subscribe("scan_right", 1, scanRightCallback);
    ros::Subscriber sub3 = nh.subscribe("scan_left_front", 1, scanFrontRightCallback);
    ros::Subscriber sub4 = nh.subscribe("scan_right_front", 1, scanFrontLeftCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    ros::spin();
    return 0;
}