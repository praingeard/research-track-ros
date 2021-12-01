#include "ros/ros.h"
#include <string>
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"

//setup publisher and speed
ros::Publisher pub;
double speed = 1.0;

//service to add speed
bool add(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res)
   {
    //add 20% of current speed
    speed = speed*1.2;
    std::cout << "speed increased to " << speed << std::endl;
    return true;
   }

//service to reduce speed
bool reduce(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res)
   {
    //reduce 20% of current speed
    speed = speed*0.8;
    std::cout << "speed reduced to " << speed << std::endl;
    return true;
   }

//timer to publish speed
void timerCallback(const ros::TimerEvent& event){
    std_msgs::Float64 msg;
    msg.data = speed;
    //publish current speed
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    //initialize timer and services
    pub = nh.advertise<std_msgs::Float64>("current_speed", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
    ros::ServiceServer service_add = nh.advertiseService("increase_speed", add);
    ros::ServiceServer service_reduce = nh.advertiseService("reduce_speed", reduce);
    ros::spin();
    return 0;
}