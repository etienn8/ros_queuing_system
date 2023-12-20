#pragma once

#include <string>

#include "ros/ros.h"

#include "inversed_cumulative_distribution.hpp"

using std::string;

class DistributionSampleTopicSize
{
    public:
        /**
         * @brief Checks if the topic_name is non-empty and create the topic publisher if it's not the case.
         * @param inversed_distribution Ramdom distribution object that gives a ramdom sample from its distribution. 
         * It's used to output a message with a random size. Since the size of a message is in bytes, float value will be rounded down.
         * @param service_name Name of the ROS topic with a ros_queue_msgs::QueueTransmitTemplate message type to publish.
         * @param nh ROS handle to keep the node alive and to create the publisher.
        */
        DistributionSampleTopicSize(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                                    std::string topic_name,
                                    ros::NodeHandle& nh);
        /**
         * @brief Computes the ramdom sample from its internal distribution and add that much intergers (4 bytes)
         * in a message before publishing it.
        */
        void publishFromRamdomSample();

    private:
        ros::NodeHandle nh_;

        ros::Publisher publisher_;

        /**
         * @brief Ramdom distribution object that provides a ramdom sample from its distribution. 
         * It's used to compute the size of the sent messages. Since the size of a message is in bytes, 
         * float value will be rounded down.
        */
        std::unique_ptr<InversedCumulativeDistribution> inversed_distribution_;

        /**
         * @brief Name of the ROS service with a ros_queue_msgs::FloatRequest message type to advertise.
        */
        std::string service_name_;
};