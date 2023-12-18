#pragma once

#include <stdexcept>

#include "ros/ros.h"
#include "ros_queue_msgs/FloatRequest.h"

#include "ros_queue_msgs/QueueInfo.h"

using std::string;
using std::invalid_argument;

class ROSQueueCommonInterfaces
{
    public:
        /**
         * @brief Initialize the common services used by the ROS queues and stores the ROSQueueInfo.
         * @param info ros_queue_msgs::QueueInfo rvalue that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
        */
        ROSQueueCommonInterfaces(ros::NodeHandle& nh, ros_queue_msgs::QueueInfo&& info): info_(std::move(info)), nh_(nh)
        {
            if (!info_.queue_name.empty())
            {
                string queue_size_service_name = info_.queue_name + "/getQueueSize";
                queue_size_service_ = nh_.advertiseService(queue_size_service_name, &ROSQueueCommonInterfaces::getSizeServiceCallback, this);
            }
            else 
            {
                throw invalid_argument("No queue name was provided.");
            }
        }

        /**
         * @brief Member that contains meta data for queues.
        */
        ros_queue_msgs::QueueInfo info_;
    
    protected:
        /**
         * @brief ROS Node handle used for the service call and make sure that a node handle exist for the life time of the ROSQueue.
        */
        ros::NodeHandle nh_;

        /**
         * @brief ROS service server to provide the state of the queue.
        */
        ros::ServiceServer queue_size_service_;

        /**
         * @brief Callback method that returns the size of the queue.
         * @param req Request of the FloatRequest service definition. Which is empty.
         * @param res Response of th
        */
        bool getSizeServiceCallback(ros_queue_msgs::FloatRequest::Request & req,
                                    ros_queue_msgs::FloatRequest::Response& res)
        {
            res.value = (float)getSizeForService();
            return true;            
        }

        /**
         * @brief Internal call for the queue size service that needs 
         * to be overriden and that returns the size of the queue.
         * @return Size of the queue.
        */
        virtual float getSizeForService()=0;


};