#include "ros_queue_tests/distribution_sample_topic_size.hpp"

#include "ros_queue_msgs/QueueTransmitTemplate.h"

DistributionSampleTopicSize::DistributionSampleTopicSize(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                                                         std::string topic_name,
                                                         ros::NodeHandle& nh): inversed_distribution_(std::move(inversed_distribution)), nh_(nh)
{
    if (inversed_distribution_)
    {
        if(!topic_name.empty())
        {
            publisher_ = nh_.advertise<ros_queue_msgs::QueueTransmitTemplate>(topic_name, 1000);

            ROS_INFO_STREAM("Created a ramdom distribution sample message publisher on " << topic_name);
        }
        else
        {
            ROS_ERROR("No topic name name was specified for a ramdom sample publisher.");
        }
    }
}

void DistributionSampleTopicSize::publishFromRamdomSample()
{
    if (!publisher_.getTopic().empty() && inversed_distribution_)
    {
        int random_sample = inversed_distribution_->generateRandomSample();

        ros_queue_msgs::QueueTransmitTemplate new_msg;

        for (int index=0; index < random_sample; ++index)
        {
            ros_queue_msgs::QueueIntElement new_int_msg;
            new_int_msg.value = index;

            new_msg.queue_elements.push_back(new_int_msg);
        }

        publisher_.publish(new_msg);
    }
}