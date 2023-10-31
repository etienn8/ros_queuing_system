#include "ros/ros.h"
#include "ros_queue/ReturnSentValue.h"

bool returnValueServicePlusTwo(ros_queue::ReturnSentValue::Request  &req,
                   ros_queue::ReturnSentValue::Response &res)
{
    res.prediction = req.value_to_return+2;
    return true;
}

bool returnValueServicePlusPlusThree(ros_queue::ReturnSentValue::Request  &req,
                   ros_queue::ReturnSentValue::Response &res)
{
    res.prediction = req.value_to_return+3;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_service_node");

    ros::NodeHandle nh;

    ROS_INFO("Advertise services");
    ros::ServiceServer service1 = nh.advertiseService("return_sent_value_plus_two", returnValueServicePlusTwo);
    ros::ServiceServer service2 = nh.advertiseService("return_sent_value_plus_three", returnValueServicePlusPlusThree);

    ros::spin();
    return 0;
}