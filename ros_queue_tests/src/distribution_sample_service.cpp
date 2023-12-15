#include "ros_queue_tests/distribution_sample_service.hpp"

#include <utility>

#include "ros/ros.h"
#include "ros_queue_msgs/FloatRequest.h"

DistributionSampleService::DistributionSampleService(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                            std::string service_name,
                            ros::NodeHandle& nh):inversed_distribution_(std::move(inversed_distribution)), nh_(nh), service_name_(service_name)
{
    if (inversed_distribution_)
    {
        if(!service_name_.empty())
        {
            service_ = nh_.advertiseService(service_name_, &DistributionSampleService::generateSampleServiceCallback, this);
            
            ROS_INFO_STREAM("Created a ramdom distribution sample service named " << service_name_);
        }
        else
        {
            ROS_ERROR("No service name was specified for a ramdom sample service.");
        }
    }
}

bool DistributionSampleService::generateSampleServiceCallback(ros_queue_msgs::FloatRequest::Request& req,
                                                              ros_queue_msgs::FloatRequest::Response& res)
{
    if(inversed_distribution_)
    {
        res.value = inversed_distribution_->generateRandomSample();
    }
    return true;
}