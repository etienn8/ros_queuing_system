#include "ros/ros.h"

#include <utility>

#include "ros_queue/ReturnSentValue.h"
#include "ros_queue_msgs/ConversionTemplateService.h"
#include "ros_queue_msgs/FloatRequest.h"

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

bool conversionToBytesService(ros_queue_msgs::ConversionTemplateService::Request  &req,
                   ros_queue_msgs::ConversionTemplateService::Response &res)
{
    for(auto it = req.queue_to_convert.begin(); it !=req.queue_to_convert.end(); ++it)
    {
        res.converted_costs.push_back(std::move(sizeof(*it)));
    }
    return true;
}

bool return1point2(ros_queue_msgs::FloatRequest::Request &req,
                    ros_queue_msgs::FloatRequest::Response &res)
{
    res.value = 1.2f;
    return true;
}

bool return3f(ros_queue_msgs::FloatRequest::Request &req,
                    ros_queue_msgs::FloatRequest::Response &res)
{
    res.value = 3.0f;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_service_node");

    ros::NodeHandle nh;

    ROS_INFO("Advertise services");
    ros::ServiceServer service1 = nh.advertiseService("return_sent_value_plus_two",   returnValueServicePlusTwo);
    ros::ServiceServer service2 = nh.advertiseService("return_sent_value_plus_three", returnValueServicePlusPlusThree);
    ros::ServiceServer service3 = nh.advertiseService("conversion_to_bytes_service",  conversionToBytesService);
    ros::ServiceServer service4 = nh.advertiseService("/return_1point2", return1point2);
    ros::ServiceServer service5 = nh.advertiseService("/return_3f", return3f);
    

    ros::spin();
    return 0;
}