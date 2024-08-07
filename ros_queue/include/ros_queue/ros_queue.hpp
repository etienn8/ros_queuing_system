#pragma once

#include <string>
#include <stdexcept>
#include <deque>

#include "ros/ros.h"

#include "ros_queue_common_interfaces.hpp"
#include "ros_queue_utils.hpp"

#include "lib_queue/dynamic_queue.hpp"

#include "ros_queue_msgs/QueueInfo.h"

#include "ros_boosted_utilities/persistent_service_client.hpp"

using std::string;
using std::invalid_argument;
using std::deque;

/**
 * @brief Queue of a rosmsg type with some ROS interface to interact with the queue via services and topics, and/or pointer functions.
 * @tparam TROSMsgType ROS msg used to publish the queue and that contains a vector named "queue_elements" where its type defines the type of the internal dequeu.
 * @tparam TPredictionServiceClass Type of the service used for the evaluation and prediction step. The response must have an int32 named "prediction".
 */
template <typename TROSMsgType, typename TPredictionServiceClass>
class ROSQueue: public DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TPredictionServiceClass>, public ROSQueueCommonInterfaces
{
    public:
        /**
         * @brief Struct that contains all the options related to using pointer functions, or ROS Topics/Services for prediction, transmission and conversion. 
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals. If defined, arrival_prediction_service_name won't be used.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data. 
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure. If defined, transmission_prediction_service_name won't be used.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update. If defined, transmission_topic_name won't be used.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
         */ 
        struct InterfacesArgs
        {
            int (*arrival_prediction_fptr)(const TPredictionServiceClass&) = nullptr;
            string arrival_prediction_service_name = "";

            int (*transmission_prediction_fptr)(const TPredictionServiceClass&) = nullptr;
            string transmission_prediction_service_name = "";

            bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&&) = nullptr;
            string transmission_topic_name = "";
        };

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ros_queue_msgs::QueueInfo reference that contains meta data about the queue.
         * @param interfaces Struct that contains all the options for the prediction and transmission interfaces. See ROSQueue::InterfacesArgs.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSQueue(int max_queue_size, ros_queue_msgs::QueueInfo&& info, ros::NodeHandle& nh, InterfacesArgs interfaces)
                :DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TPredictionServiceClass>(max_queue_size),
                 ROSQueueCommonInterfaces(nh, std::move(info))
        {
            // Init the arrival prediction
            if (interfaces.arrival_prediction_fptr)
            {
                arrival_prediction_fptr_ = interfaces.arrival_prediction_fptr;

                if (!interfaces.arrival_prediction_service_name.empty())
                {
                    ROS_WARN("An arrival prediction function pointer and a service name has been provided. The function pointer will be used.");
                }
            }
            else if (!interfaces.arrival_prediction_service_name.empty())
            {
                arrival_service_client_ = PersistentServiceClient<TPredictionServiceClass>(nh_, interfaces.arrival_prediction_service_name);
            }
            else
            {
                throw invalid_argument("No arrival prediction function pointer or service name provided.");
            }

            // Init the transmission prediction
            if (interfaces.transmission_prediction_fptr)
            {
                transmission_prediction_fptr_ = interfaces.transmission_prediction_fptr;

                if (!interfaces.transmission_prediction_service_name.empty())
                {
                    ROS_WARN("A transmission prediction function pointer and a service name has been provided. The function pointer will be used.");
                }
            }
            else if (!interfaces.transmission_prediction_service_name.empty())
            {
                transmission_service_client_ = PersistentServiceClient<TPredictionServiceClass>(nh_, interfaces.transmission_prediction_service_name);
            }
            else
            {
                throw invalid_argument("No transmission prediction function pointer or service name provided.");
            }

            if (interfaces.transmission_fptr)
            {
                transmission_fptr_ = interfaces.transmission_fptr;

                if (!interfaces.transmission_topic_name.empty())
                {
                    ROS_WARN("A transmission function pointer and a topic name has been provided. The function pointer will be used.");
                }
            }
            else if (!interfaces.transmission_topic_name.empty())
            {
                transmission_pub_ = nh_.advertise<TROSMsgType>(interfaces.transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
            }
            else
            {
                throw invalid_argument("No transmission function pointer or topic name provided.");
            }
        }

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. It uses a user-defined callback given by the user in the constructor if defined, otherwise it will make a service call.
         * @param service Is a service class definition that is used for the service call and for the user-defined prediction function as data structure input and output.
         * @return Returns the converted size of the estimated arrival queue.
         */
        virtual int arrival_prediction(const TPredictionServiceClass& service) override 
        {
            if (arrival_prediction_fptr_)
            {
                return arrival_prediction_fptr_(service);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing the service call
                TPredictionServiceClass local_service= service;

                // Service ROS call
                if(!arrival_service_waited_)
                {
                    arrival_service_client_.waitForExistence();
                    arrival_service_waited_ = true;
                }
                if (arrival_service_client_.call(local_service))
                {
                    return local_service.response.prediction;
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
        virtual int transmission_prediction(const TPredictionServiceClass& service) override
        {
            if (transmission_prediction_fptr_)
            {
                return transmission_prediction_fptr_(service);
            }
            else
            {
                //Make a local copy to respect the const arg but still allowing the service call
                TPredictionServiceClass local_service = service; 

                // Service ROS call
                if(!transmission_service_waited_)
                {
                    transmission_service_client_.waitForExistence();
                    transmission_service_waited_ = true;
                }
                if (transmission_service_client_.call(local_service))
                {
                    return local_service.response.prediction;
                }
                else
                {
                    ROS_DEBUG_STREAM_THROTTLE(2, "Transmission prediction service " <<  transmission_service_client_.getService() <<" is not available. Prediction will be bad and return 0.");
                }
            }

            return 0;
        }

        /**
         * @brief Method used internaly to transmit data.It uses the user-defined ROSQueue::transmission_fptr_if it's defined, otherwise, it publishes on a ROS topic based on a name given by the constructor.
         * @param queue_to_transmit Rvalue of a queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&& queue_to_transmit) override
        {
            if (transmission_fptr_)
            {
                return transmission_fptr_(std::move(queue_to_transmit));
            }
            else
            {
                TROSMsgType new_msg;

                for(typename deque<typename QueueElementTrait<TROSMsgType>::ElementType>::const_iterator it = queue_to_transmit.begin(); it != queue_to_transmit.end(); ++it)
                {
                    new_msg.queue_elements.push_back(std::move(*it));
                }

                transmission_pub_.publish(new_msg);
            }

            return true;
        }

        /**
         * @brief Internal call for the queue size service that needs 
         * to be overriden and that returns the size of the queue.
         * @return Size of the queue.
        */
        virtual float getSizeForService() override
        {
            return this->getSize();
        }

    private:
        /**
         * @brief Service client used for the arrival prediction service calls.
         */
        PersistentServiceClient<TPredictionServiceClass> arrival_service_client_;

        /**
         * @brief Function pointer for the arrival prediction service calls.
         * @param TPredictionServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*arrival_prediction_fptr_)(const TPredictionServiceClass&) = nullptr;

        /**
         * @brief Flag to know if the arrival service was waited for.
         */ 
        bool arrival_service_waited_ = false;

        /**
         * @brief Service client used for the transmission prediction service calls.
         */
        PersistentServiceClient<TPredictionServiceClass> transmission_service_client_;

        /**
         * @brief Function pointer for the transmission prediction service calls.
         * @param TPredictionServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*transmission_prediction_fptr_)(const TPredictionServiceClass&) = nullptr;

        /**
         * @brief Flag to know if the transmission service was waited for.
         */
        bool transmission_service_waited_ = false;

        /**
         * @brief Publisher used for the transmission publication calls.
         */
        ros::Publisher transmission_pub_;
        /**
         * @brief Function pointer for the transmission of data queue from the updates.
         * @param deque<typename QueueElementTrait<TROSMsgType>::ElementType>&& Rvalue to a queue to transmit.
         */
        bool (*transmission_fptr_)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&&) = nullptr;

        /**
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /**
         * @brief Number of message that the publisher could hold in its queue (not related to ROSQueue, but a ROS primitive).
        */
        const int MAX_TRANSMIT_TOPIC_QUEUE_SIZE = 50;
};
