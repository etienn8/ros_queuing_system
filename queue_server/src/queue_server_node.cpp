#include "queue_server/queue_server.hpp"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_queue");
    ros::NodeHandle nh("~");

    // Initialize a queue server
    QueueServer queue_server(nh);

    ros::spin();
}