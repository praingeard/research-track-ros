#include "ros/ros.h"
#include <string>
#include <signal.h>
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    //initialize service clients
    ros::ServiceClient client_increase = nh.serviceClient<std_srvs::Empty>("increase_speed");
    ros::ServiceClient client_reduce = nh.serviceClient<std_srvs::Empty>("reduce_speed");
    ros::ServiceClient client_reset = nh.serviceClient<std_srvs::Empty>("reset_positions");
    std_srvs::Empty srv;
    int input = 0;
    while (ros::ok)
    {
        //getting input
        ROS_INFO("Enter 1 to increase_speed, 2 to reduce speed and 3 to reset the robot's position");
        std::string str;
        std::cin >> str;

        //errors
        try
        {
            input = std::stoi(str);
        }
        catch (const std::invalid_argument &error)
        {
            ROS_ERROR("Invalid input");
        }
        if (input < 1 || input > 3)
        {
            std::cout << "input must be between 1 and 3" << std::endl;
        }

        //calling services
        if (input == 1)
        {
            if (client_increase.call(srv))
            {
                ROS_INFO("increased speed");
            }
            else
            {
                ROS_ERROR("Failed to call service increase_speed");
                return 1;
            }
        }
        else if (input == 2)
        {
            if (client_reduce.call(srv))
            {
                ROS_INFO("reduced speed");
            }
            else
            {
                ROS_ERROR("Failed to call service reduce_speed");
                return 1;
            }
        }
        else if (input == 3)
        {
            if (client_reset.call(srv))
            {
                ROS_INFO("reset ok");
            }
            else
            {
                ROS_ERROR("Failed to call service reset_positions");
                return 1;
            }
        }
    }
    return 0;
}