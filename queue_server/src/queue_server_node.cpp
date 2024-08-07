#include "queue_server/queue_server.hpp"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_server");
    ros::NodeHandle nhp("~");

    // Initialize a queue server

    float server_spin_rate = 10.0f;

    if(!nhp.getParam("server_spin_rate", server_spin_rate))
    {
        ROS_WARN_STREAM("Server spin rate wasn't specified, the server will spin at " << server_spin_rate << " hz.");
    }

    QueueServer queue_server(nhp, server_spin_rate);

    ros::spin();
}