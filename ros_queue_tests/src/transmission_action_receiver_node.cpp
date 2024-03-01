#include "ros/ros.h"

#include "ros_queue_tests/transmission_action_receiver.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "transmission_action_receiver_node");

    ros::NodeHandle nh("~");

    TransmissionActionReceiver action_receiver(nh);

    ros::spin();
}
