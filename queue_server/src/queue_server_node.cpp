#include "queue_server/queue_server.hpp"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_server");
    ros::NodeHandle nh("~");

    // Initialize a queue server
    QueueServer queue_server(nh);

    float server_spin_rate = 10.0f;

    if(!nh.getParam("server_spin_rate", server_spin_rate))
    {
        ROS_WARN_STREAM("Server spin rate wasn't specified, the server will spin at " << server_spin_rate << " hz.");
    }

    ros::Rate rate(server_spin_rate);

    while(ros::ok())
    {
        queue_server.serverSpin();

        ros::spinOnce();
        rate.sleep();
    }
}