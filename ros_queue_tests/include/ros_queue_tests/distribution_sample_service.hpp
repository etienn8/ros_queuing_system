#pragma once

#include <memory>
#include <string>

#include "ros/ros.h"

#include "inversed_cumulative_distribution.hpp"

#include "ros_queue_msgs/FloatRequest.h"

class DistributionSampleService
{
    public:
        /**
         * @brief Construct the server and checks if the service_name is non-empty.
         * @param inversed_distribution Ramdom distribution object that gives a ramdom sample from its distribution. It's used as the return value of the service.
         * @param service_name Name of the ROS service with a ros_queue_msgs::FloatRequest message type to advertise.
         * @param nh ROS handle to keep the node alive and to advertise the service.
        */
        DistributionSampleService(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                                  std::string service_name,
                                  ros::NodeHandle& nh);

    private:
        ros::NodeHandle nh_;

        ros::ServiceServer service_;

        /**
         * @brief Ramdom distribution object that provides a ramdom sample from its distribution. 
         * It's used as the return value of the service.
        */
        std::unique_ptr<InversedCumulativeDistribution> inversed_distribution_;

        /**
         * @brief Callback of the ROS service that returns a ramdom sample from the distribution stored in DistributionSampleService::inversed_distribution.
        */
        bool generateSampleServiceCallback(ros_queue_msgs::FloatRequest::Request& req,
                                           ros_queue_msgs::FloatRequest::Response& res);

        /**
         * @brief Name of the ROS service with a ros_queue_msgs::FloatRequest message type to advertise.
        */
        std::string service_name_;
};