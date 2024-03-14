#include "ros/ros.h"

#include "ros_queue_experiments/auv_system.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_node");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);

    AUVSystem auv_system(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}