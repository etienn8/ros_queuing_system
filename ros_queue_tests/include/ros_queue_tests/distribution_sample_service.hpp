#pragma once

#include <memory>
#include <string>

#include "ros/ros.h"

#include "distributions/inversed_cumulative_distribution.hpp"

#include "ros_queue_msgs/ByteSizeRequest.h"
#include "ros_queue_msgs/FloatRequest.h"

template <typename TROSServiceType>
class DistributionSampleService {};

template<>
class DistributionSampleService<ros_queue_msgs::FloatRequest>
{
    public:
        /**
         * @brief Construct the server and checks if the service_name is non-empty.
         * @param inversed_distribution Ramdom distribution object that gives a ramdom sample from its distribution. It's used as the return value of the service.
         * @param service_name Name of the ROS service with a TROSServiceType message type to advertise.
         * @param nh ROS handle to keep the node alive and to advertise the service.
        */
        DistributionSampleService(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                                  std::string service_name,
                                  ros::NodeHandle& nh):inversed_distribution_(std::move(inversed_distribution)), nh_(nh), service_name_(service_name)
        {
            if (inversed_distribution_)
            {
                if(!service_name_.empty())
                {
                    service_ = nh_.advertiseService(service_name_, &DistributionSampleService<ros_queue_msgs::FloatRequest>::generateSampleServiceCallback, this);
                    
                    ROS_INFO_STREAM("Created a ramdom distribution sample service named " << service_name_);
                }
                else
                {
                    ROS_ERROR("No service name was specified for a ramdom sample service.");
                }
            }
        }

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
                                           ros_queue_msgs::FloatRequest::Response& res)
        {
            if(inversed_distribution_)
            {
                res.value = inversed_distribution_->generateRandomSample();
            }
            return true;
        }

        /**
         * @brief Name of the ROS service with a TROSServiceType message type to advertise.
        */
        std::string service_name_;
};

template<>
class DistributionSampleService<ros_queue_msgs::ByteSizeRequest>
{
    public:
        /**
         * @brief Construct the server and checks if the service_name is non-empty.
         * @param inversed_distribution Ramdom distribution object that gives a ramdom sample from its distribution. It's used as the return value of the service.
         * @param service_name Name of the ROS service with a TROSServiceType message type to advertise.
         * @param nh ROS handle to keep the node alive and to advertise the service.
        */
        DistributionSampleService(std::unique_ptr<InversedCumulativeDistribution>&& inversed_distribution,
                                  std::string service_name,
                                  ros::NodeHandle& nh):inversed_distribution_(std::move(inversed_distribution)), nh_(nh), service_name_(service_name)
        {
            if (inversed_distribution_)
            {
                if(!service_name_.empty())
                {
                    service_ = nh_.advertiseService(service_name_, &DistributionSampleService<ros_queue_msgs::ByteSizeRequest>::generateSampleServiceCallback, this);
                    
                    ROS_INFO_STREAM("Created a ramdom distribution sample service named " << service_name_);
                }
                else
                {
                    ROS_ERROR("No service name was specified for a ramdom sample service.");
                }
            }
        }

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
        bool generateSampleServiceCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                           ros_queue_msgs::ByteSizeRequest::Response& res)
        {
            if(inversed_distribution_)
            {
                res.nb_of_bytes = inversed_distribution_->generateRandomSample();
            }
            return true;
        }

        /**
         * @brief Name of the ROS service with a TROSServiceType message type to advertise.
        */
        std::string service_name_;
};