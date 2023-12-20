#include "ros/ros.h"

#include <string>
#include <vector>

#include "ros_queue_tests/distribution_sample_server.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "distribution_service_nodes.cpp");

    ros::NodeHandle nh("~");
    
    float publisher_rate = 5.0f;

    if (!nh.getParam("publisher_rate", publisher_rate))
    {
        ROS_WARN("CONFIG distribution service node: publihser_rate is not set. Default value of 5.0f will be used.");
    }

    DistributionSampleServer server(nh, publisher_rate);
    
    ros::spin();
}