#include "ros_queue_tests/prediction_server.hpp"

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "prediction_test_server");

    ros::NodeHandle nh("~");

    PredictionServer prediction_server(nh);

    ros::spin();
}