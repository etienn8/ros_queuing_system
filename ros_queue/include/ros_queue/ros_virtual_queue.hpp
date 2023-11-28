#pragma once

#include <mutex>
#include <string>
#include <stdexcept>

#include "ros/ros.h"
#include "ros/duration.h"

#include "ros_queue_info.hpp"

#include "lib_queue/virtual_queue.hpp"

#include "ros_queue/ComputeVirtualQueueMetric.h"

using std::string;
using std::invalid_argument;

/**
 * @brief Virtual queue wrapped with some ROS interface to interact with the queue with services and/or pointer functions. The queue increases by a metric_target and reduces by a metric that is evaluated at each update.
 * @tparam TDynamicVirtualQueueType Type of the virtual queue used. Queue that could be used: InConVirtualQueue or EqConVirtualQueue
 */
template <typename TDynamicVirtualQueueType>
class ROSVirtualQueue: public TDynamicVirtualQueueType
{
    public:
        using TDynamicVirtualQueueType::update;

        /**
         * @brief Member that contains meta data for queues.
        */
        ROSQueueInfo info_;

        /**
         * @brief Struct that contains all the options related to using pointer functions, or ROS Topics/Services for prediction, transmission and conversion. 
         * @param metric_computation_fptr Pointer to a user-defined function that is called at the update to compute the size of the departure of the queue. If defined, metric_computation_service_name won't be used.
         * @param metricservice_name String of the service name called to compute the size of the departure of the queue. 
         */ 
        struct InterfacesArgs
        {
            float (*metric_computation_fptr)() = nullptr;
            string metric_computation_service_name = "";
        };

        /**
         * @brief Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param interfaces Struct that contains all the options for the change interfaces. See ROSVirtualQueue::InterfacesArgs.
         * @param metric_target Time average target of the measure metric of the queue. It indicates how much the queue grows at each update if the queue metric is null.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, const float metric_target, InterfacesArgs interfaces)
                        :TDynamicVirtualQueueType(max_queue_size), info_(info), nh_(nh), target_metric_(metric_target)
                        {
                            // Init the metric calculator
                            if (interfaces.metric_computation_fptr)
                            {
                                metric_computation_fptr_ = interfaces.metric_computation_fptr;

                                if (!interfaces.metric_computation_service_name.empty())
                                {
                                    ROS_WARN("A metric calculator function pointer and a service name has been provided. The function pointer will be used.");
                                }
                            }
                            else if (!interfaces.metric_computation_service_name.empty())
                            {
                                metric_computation_service_client_ = nh_.serviceClient<ros_queue::ComputeVirtualQueueMetric>(interfaces.metric_computation_service_name);
                            }
                            else
                            {
                                throw invalid_argument("No metric calculator function pointer or service name provided.");
                            }
                        }

        void setTargetMetric(const float new_target)
        {
            std::lock_guard<std::mutex> lock(metric_manipulation_mutex_);
            target_metric_ = new_target;
        }

        /**
         * @brief Method that updates the virtual queue. It changes by the target_metric_ minus the returned value of the metric calculator (chose at the constructor).
         */
        void update()
        {
            std::lock_guard<std::mutex> lock(metric_manipulation_mutex_);

            float change = 0.0f;
            bool change_computed = false;

            if (metric_computation_fptr_)
            {
                change = target_metric_ - metric_computation_fptr_();
                change_computed = true;
            }
            else
            {
                ros_queue::ComputeVirtualQueueMetric local_service; 

                // Service ROS call
                if (metric_computation_service_client_.waitForExistence(WAIT_DURATION_FOR_SERVICE_EXISTENCE))
                {
                    if (metric_computation_service_client_.call(local_service))
                    {
                        change =  target_metric_ - local_service.response.value;
                        change_computed = true;
                    }
                }
                else
                {
                    ROS_DEBUG_STREAM_THROTTLE(2, "Metric calculator service " <<  metric_computation_service_client_.getService() <<" is not available.");
                }
            }
            
            if (change_computed);
            {
                update(change);
            }
        }

    private:
        /**
         * @brief ROS Node handle used for the service call and make sure that a node handle exist for the life time of the ROSQueue.
        */
        ros::NodeHandle nh_;
        /**
         * @brief Service client used to compute the change in the virtual queue.
         */
        ros::ServiceClient metric_computation_service_client_;
        /**
         * @brief Function pointer for the virtual queue change service calls.
         */
        float (*metric_computation_fptr_)() = nullptr;

        /**
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /** 
         * @brief Time average constraint value.
        */
       float target_metric_ = 0.0f;

        /**
         * @brief Mutex to protect the metric target change.
        */
       std::mutex metric_manipulation_mutex_;
};



