#include "ros/ros.h"

#include <string>
#include <vector>

#include "ros_queue_tests/distribution_sample_server.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "distribution_service_nodes.cpp");

    ros::NodeHandle nh("~");
    DistributionSampleServer server(nh);

    ros::spin();
}