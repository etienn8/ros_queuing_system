#pragma once

#include <string>
#include <stdexcept>

#include "ros/ros.h"
#include "ros/duration.h"

#include "ros_queue_info.hpp"
#include "lib_queue/virtual_queue.hpp"

static const double WAIT_DELAY_FOR_SERVICE_EXISTENCE = 0.5;

using std::string;
using std::invalid_argument;

/**
 * @brief Virtual queue wrapped with some ROS interface to interact with the queue with services and/or pointer functions.
 * @tparam TDynamicVirtualQueueType Type of the virtual queue used. Queue that could be used: InConVirtualQueue<TServiceClass> or EqConVirtualQueue<TServiceClass> 
 * @tparam TServiceClass Type of the service used for the evaluation and prediction step. The response must have an int32 named "prediction".
 */
template <template <typename> class TDynamicVirtualQueueType, typename TServiceClass>
class ROSVirtualQueue: public TDynamicVirtualQueueType<TServiceClass>
{
    public:
        /**
         * @brief Member that contains meta data for queues.
        */
        ROSQueueInfo info_;

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, int (*arrival_prediction_fptr)(const TServiceClass&),
         int (*transmission_prediction_fptr)(const TServiceClass&)):TDynamicVirtualQueueType<TServiceClass>(max_queue_size), info_(info)
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

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @throw Throws an std::invalid_argument if the arrival prediction function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, int (*arrival_prediction_fptr)(const TServiceClass&),
         string transmission_prediction_service_name):TDynamicVirtualQueueType<TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (arrival_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate arrival_prediction with null function pointer.");
            }
            arrival_prediction_fptr_ = arrival_prediction_fptr;

            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @throw Throws an std::invalid_argument if the transmission prediction function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        int (*transmission_prediction_fptr)(const TServiceClass&)):TDynamicVirtualQueueType<TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (transmission_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission_prediction with null function pointer.");
            }
            transmission_prediction_fptr_ = transmission_prediction_fptr;
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @throw Throws an std::invalid_argument if the transmission prediction function pointers is null. 
        */
        ROSVirtualQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        string transmission_prediction_service_name):TDynamicVirtualQueueType<TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);
        }

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. It uses a user-defined callback given by the user in the constructor if defined, otherwise it will make a service call.
         * @param service Is a service class definition that is used for the service call and for the user-defined prediction function as data structure input and output.
         * @return Returns the converted size of the estimated arrival queue.
         */
        virtual int arrival_prediction(const TServiceClass& service) override 
        {
            if (arrival_prediction_fptr_)
            {
                return arrival_prediction_fptr_(service);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing the service call
                TServiceClass local_service= service; 

                // Service ROS call
                if (arrival_service_client_.waitForExistence(wait_for_service_delay_))
                {
                    if (arrival_service_client_.call(local_service))
                    {
                        return local_service.response.prediction;
                    }
                }
                else
                {
                    ROS_DEBUG_STREAM_THROTTLE(2, "Arrival prediction service " <<  arrival_service_client_.getService() <<" is not available. Prediction will be bad and return 0.");
                }
            }

            return 0;
        }

        /**
         * @brief Method used in the evaluation process to predict what will be the number of departing element. It uses a user-defined callback given by the user in the constructor if defined, otherwise it will make a service call.
         * @param service Is a service class definition that is used for the service call and for the user-defined prediction function as data structure input and output.
         * @return Returns the converted size of the estimated transmission queue.
         */
        virtual int transmission_prediction(const TServiceClass& service) override
        {
            if (transmission_prediction_fptr_)
            {
                return transmission_prediction_fptr_(service);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing the service call
                TServiceClass local_service = service; 

                // Service ROS call
                if (transmission_service_client_.waitForExistence(wait_for_service_delay_))
                {
                    if (transmission_service_client_.call(local_service))
                    {
                        return local_service.response.prediction;
                    }
                }
                else
                {
                    ROS_DEBUG_STREAM_THROTTLE(2, "Transmission prediction service " <<  transmission_service_client_.getService() <<" is not available. Prediction will be bad and return 0.");
                }
            }

            return 0;
        }

    private:
        /**
         * @brief ROS Node handle used for the service call and make sure that a node handle exist for the life time of the ROSQueue.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Service client used for the arrival prediction service calls.
         */
        ros::ServiceClient arrival_service_client_;
        /**
         * @brief Function pointer for the arrival prediction service calls.
         */
        int (*arrival_prediction_fptr_)(const TServiceClass&) = nullptr;

        /**
         * @brief Service client used for the transmission prediction service calls.
         */
        ros::ServiceClient transmission_service_client_;
        /**
         * @brief Function pointer for the transmission prediction service calls.
         */
        int (*transmission_prediction_fptr_)(const TServiceClass&) = nullptr;

        ros::Duration wait_for_service_delay_ = ros::Duration(WAIT_DELAY_FOR_SERVICE_EXISTENCE);
};



