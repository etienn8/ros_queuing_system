#pragma once

#include <mutex>
#include <string>
#include <stdexcept>

#include "ros/ros.h"
#include "ros/duration.h"

#include "ros_queue_info.hpp"

#include "lib_queue/dynamic_virtual_queue.hpp"

#include "ros_queue_msgs/ComputeVirtualQueueMetric.h"

using std::string;
using std::invalid_argument;

/**
 * @brief Virtual queue wrapped with some ROS interface to interact with the queue with services and/or pointer functions. The queue increases by an arrival evaluation made by a user-defined function or a ROS service. It also decreases by a similar departure evaluation..
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
         * @param arrival_evaluation_fptr Function pointer used to compute the increase of the virtual queue.If defined, arrival_evaluation_service_name won't be used.
         * @param arrival_evaluation_service_name tring of the service name called to compute the increase of the queue size.
         * @param departure_evaluation_fptr Function pointer used to compute the decrease of the virtual queue.If defined, departure_evaluation_service_name won't be used.
         * @param departure_evaluation_service_name String of the service name called to compute the decrease of the queue size.
         */ 
        struct InterfacesArgs
        {
            float (*arrival_evaluation_fptr)() = nullptr;
            string arrival_evaluation_service_name = "";

            float (*departure_evaluation_fptr)() = nullptr;
            string departure_evaluation_service_name = "";
        };

        /**
         * @brief Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param interfaces Struct that contains all the options for the change interfaces. See ROSVirtualQueue::InterfacesArgs.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, InterfacesArgs interfaces)
                        :TDynamicVirtualQueueType(max_queue_size), info_(info), nh_(nh)
            {
                // Init the arrival evaluator
                if (interfaces.arrival_evaluation_fptr)
                {
                    arrival_evaluation_fptr_ = interfaces.arrival_evaluation_fptr;

                    if (!interfaces.arrival_evaluation_service_name.empty())
                    {
                        ROS_WARN("An arrival evaluator function pointer and a service name has been provided. The function pointer will be used.");
                    }
                }
                else if (!interfaces.arrival_evaluation_service_name.empty())
                {
                    arrival_evaluation_service_client_ = nh_.serviceClient<ros_queue_msgs::ComputeVirtualQueueMetric>(interfaces.arrival_evaluation_service_name);
                }
                else
                {
                    throw invalid_argument("No arrival evaluator function pointer or service name provided.");
                }

                // Init the departure evaluator
                if (interfaces.departure_evaluation_fptr)
                {
                    departure_evaluation_fptr_ = interfaces.departure_evaluation_fptr;

                    if (!interfaces.departure_evaluation_service_name.empty())
                    {
                        ROS_WARN("A departure evaluator function pointer and a service name has been provided. The function pointer will be used.");
                    }
                }
                else if (!interfaces.departure_evaluation_service_name.empty())
                {
                    departure_evaluation_service_client_ = nh_.serviceClient<ros_queue_msgs::ComputeVirtualQueueMetric>(interfaces.departure_evaluation_service_name);
                }
                else
                {
                    throw invalid_argument("No departure evaluator function pointer or service name provided.");
                }
                            

        }

        /**
         * @brief Method that updates the virtual queue. It changes by the target_metric_ minus the returned value of the metric calculator (chose at the constructor).
         */
        void update()
        {
            bool was_arrival_evaluated = false;
            bool was_departure_evaluated = false;

            float arrival = 0.0f;
            float departure = 0.0f;

            if (arrival_evaluation_fptr_)
            {
                arrival = arrival_evaluation_fptr_();
                was_arrival_evaluated = true;
            }
            else
            {
                ros_queue_msgs::ComputeVirtualQueueMetric local_service; 

                // Service ROS call
                if (arrival_evaluation_service_client_.waitForExistence(WAIT_DURATION_FOR_SERVICE_EXISTENCE))
                {
                    if (arrival_evaluation_service_client_.call(local_service))
                    {
                        arrival = local_service.response.value;
                        was_arrival_evaluated = true;
                    }
                }
                else
                {
                    ROS_WARN_STREAM_THROTTLE(2, "Arrival evaluator service " <<  arrival_evaluation_service_client_.getService() <<" is not available.");
                }
            }

            if (departure_evaluation_fptr_)
            {
                departure = departure_evaluation_fptr_();
                was_departure_evaluated = true;
            }
            else
            {
                ros_queue_msgs::ComputeVirtualQueueMetric local_service; 

                // Service ROS call
                if (departure_evaluation_service_client_.waitForExistence(WAIT_DURATION_FOR_SERVICE_EXISTENCE))
                {
                    if (departure_evaluation_service_client_.call(local_service))
                    {
                        departure = local_service.response.value;
                        was_departure_evaluated = true;
                    }
                }
                else
                {
                    ROS_WARN_STREAM_THROTTLE(2, "Departure evaluator service " <<  arrival_evaluation_service_client_.getService() <<" is not available.");
                }
            }
            
            if (was_arrival_evaluated && was_departure_evaluated)
            {
                update(arrival-departure);
            }
            else 
            {
                ROS_WARN("The arrival or the departure evaluation was not evaluated. No queue update will occur.");
            }
        }

    private:
        /**
         * @brief ROS Node handle used for the service call and make sure that a node handle exist for the life time of the ROSQueue.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Service client used to compute the increase of the virtual queue.
         */
        ros::ServiceClient arrival_evaluation_service_client_;
        /**
         * @brief Function pointer used to compute the increase of the virtual queue.
         */
        float (*arrival_evaluation_fptr_)() = nullptr;

        /**
         * @brief Service client used to compute the decrease of the virtual queue.
         */
        ros::ServiceClient departure_evaluation_service_client_;
        /**
         * @brief Function pointer used to compute the decrease of the virtual queue.
         */
        float (*departure_evaluation_fptr_)() = nullptr;

        /**
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /**
         * @brief Mutex to protect the metric target change.
        */
       std::mutex metric_manipulation_mutex_;
};

typedef ROSVirtualQueue<InConVirtualQueue> ROSInConVirtualQueue;
typedef ROSVirtualQueue<EqConVirtualQueue> ROSEqConVirtualQueue; 