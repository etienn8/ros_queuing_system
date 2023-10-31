#pragma once

#include <string>
#include <stdexcept>

#include "ros/ros.h"

#include "ros_queue_info.hpp"
#include "lib_queue/dynamic_queue.hpp"
#include "lib_queue/virtual_queue.hpp"


using std::string;
using std::invalid_argument;


template <typename DynamicQueueType, typename TServiceClass>
class ROSQueue: public DynamicQueueType
{
    public:
        ROSQueueInfo info_;
        
        ros::NodeHandle nh_;

        // Constructor
        ROSQueue(int max_queue_size, ROSQueueInfo& info, int (*arrival_prediction_fptr)(const TServiceClass&),
         int (*transmission_prediction_fptr)(const TServiceClass&)):DynamicQueueType(max_queue_size), info_(info)
        {
            if (arrival_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate arrival_prediction with null function pointer.");
            }
            arrival_prediction_fptr_ = arrival_prediction_fptr;

            if (transmission_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission_prediction with null function pointer.");
            }
            transmission_prediction_fptr_ = transmission_prediction_fptr;
        }

        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, int (*arrival_prediction_fptr)(const TServiceClass&),
         string transmission_prediction_service_name):DynamicQueueType(max_queue_size), info_(info), nh_(nh)
        {
            if (arrival_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate arrival_prediction with null function pointer.");
            }
            arrival_prediction_fptr_ = arrival_prediction_fptr;

            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);
        }

        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        int (*transmission_prediction_fptr)(const TServiceClass&)):DynamicQueueType(max_queue_size), info_(info), nh_(nh)
        {
            if (transmission_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission_prediction with null function pointer.");
            }
            transmission_prediction_fptr_ = transmission_prediction_fptr;
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
        }

        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        string transmission_prediction_service_name):DynamicQueueType(max_queue_size), info_(info), nh_(nh)
        {
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);
        }

    protected:

        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the converted size of the estimated arrival queue.
         */
        virtual int arrival_prediction(const TServiceClass& states) override 
        {
            if (arrival_prediction_fptr_)
            {
                return arrival_prediction_fptr_(states);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing service call
                TServiceClass local_service= states; 

                // Service ROS call
                if (arrival_service_client_.call(local_service))
                {
                    return local_service.response.prediction;
                }
            }
        }

        virtual int transmission_prediction(const TServiceClass& states) override
        {
            if (transmission_prediction_fptr_)
            {
                return transmission_prediction_fptr_(states);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing service call
                TServiceClass local_service = states; 

                // Service ROS call
                if (transmission_service_client_.call(local_service))
                {
                    return local_service.response.prediction;
                }
            }

            return 0;
        }

    private:
        ros::ServiceClient arrival_service_client_;
        int (*arrival_prediction_fptr_)(const TServiceClass&) = nullptr;

        ros::ServiceClient transmission_service_client_;
        int (*transmission_prediction_fptr_)(const TServiceClass&) = nullptr;

};



