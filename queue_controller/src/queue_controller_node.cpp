#include "ros/ros.h"

#include "queue_controller/queue_controller.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_controller");
    ros::NodeHandle nh("~");
    
    QueueController queue_controller(nh);

    ros::spin();
}