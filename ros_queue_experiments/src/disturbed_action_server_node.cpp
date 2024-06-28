#include "ros_queue_experiments/disturbed_action_server.hpp"

#include "ros/ros.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "disturbed_action_server");
    ros::NodeHandle nh("~");

    DisturbedActionServer disturbed_action_server(nh);

    ros::spin();

    return 0;
}