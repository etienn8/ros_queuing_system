#pragma once

#include <string>
#include <stdexcept>
#include <deque>

#include "ros/ros.h"

#include "ros_queue_info.hpp"
#include "ros_queue_utils.hpp"

#include "lib_queue/dynamic_converted_queue.hpp"


using std::string;
using std::invalid_argument;
using std::deque;

/**
 * @brief Queue of a rosmsg type with some ROS interface to interact with the queue via services and topics, and/or pointer functions but where the size of the queue is given by a user-defined conversion.
 * @tparam TROSMsgType ROS msg used to publish the queue and that contains a vector named "queue_elements" where its type defines the type of the internal dequeu.
 * @tparam TPredictionServiceClass Type of the service used for the evaluation and prediction step. The response must have an int32 named "prediction".
 * @tparam TConversionServiceClass Type of the service used for the computed the converted size of a queue element. 
 * The request must take an array name "queue_to_convert" with the type of the "queue_elements" type of the TROSMsgType used and it must return an int32 array name "converted costs" with the converted cost of each elements.
 */
template <typename TROSMsgType, typename TPredictionServiceClass, typename TConversionServiceClass>
class ROSConvertedQueue: public DynamicConvertedQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TPredictionServiceClass>
{
    public:
        /**
         * @brief Member that contains meta data for queues.
        */
        ROSQueueInfo info_;

        /**
         * @brief Constructors that initialized all the prediction, transmission and conversion functions.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals. If defined, arrival_prediction_service_name won't be used.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure. If defined, transmission_prediction_service_name won't be used.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data. 
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update. If defined, transmission_topic_name won't be used.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
         * @param conversion_fptr Pointer to a user-defined function that create a deque of the queue elements with an additionnal converted size that will be used for the size of the queue. If defined, conversion_service_name won't be used.
         * @param conversion_service_name String of the service name called to create an array of converted cost for each element in the sent queue.
         * @throw Throws an std::invalid_argument if one of the function pointers is null or if no prediction, transmission or conversion function, topic or service name is given. 
        */
        ROSConvertedQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh,
                            int (*arrival_prediction_fptr)(const TPredictionServiceClass) = nullptr,
                            int (*transmission_prediction_fptr)(const TPredictionServiceClass) = nullptr,
                            string arrival_prediction_service_name = "",
                            string transmission_prediction_service_name = "",
                            bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&) = nullptr,
                            string transmission_topic_name = "",
                            void (*conversion_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&, deque<ElementWithConvertedSize<typename QueueElementTrait<TROSMsgType>::ElementType>>&) = nullptr,
                            string conversion_service_name = "")
                            : DynamicConvertedQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TPredictionServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            // Init the arrival prediction
            if (arrival_prediction_fptr)
            {
                arrival_prediction_fptr_ = arrival_prediction_fptr;

                if (!arrival_prediction_service_name.empty())
                {
                    ROS_WARN("An arrival prediction function pointer and a service name has been provided. The function pointer will be used.");
                }
            }
            else if (!arrival_prediction_service_name.empty())
            {
                arrival_service_client_ = nh.serviceClient<TPredictionServiceClass>(arrival_prediction_service_name);
            }
            else
            {
                throw invalid_argument("No arrival prediction function pointer or service name provided.");
            }

            // Init the transmission prediction
            if (transmission_prediction_fptr)
            {
                transmission_prediction_fptr_ = transmission_prediction_fptr;

                if (!transmission_prediction_service_name.empty())
                {
                    ROS_WARN("An transmission prediction function pointer and a service name has been provided. The function pointer will be used.");
                }
            }
            else if (!transmission_prediction_service_name.empty())
            {
                transmission_service_client_ = nh.serviceClient<TPredictionServiceClass>(transmission_prediction_service_name);
            }
            else
            {
                throw invalid_argument("No transmission prediction function pointer or service name provided.");
            }

            if (transmission_fptr)
            {
                transmission_fptr_ = transmission_fptr;

                if (!transmission_topic_name.empty())
                {
                    ROS_WARN("An transmission function pointer and a topic name has been provided. The function pointer will be used.");
                }
            }
            else if (!transmission_topic_name.empty())
            {
                transmission_pub_ = nh_.advertise<TROSMsgType>(transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
            }
            else
            {
                throw invalid_argument("No transmission function pointer or topic name provided.");
            }

            if (conversion_fptr)
            {
                conversion_fptr_ = conversion_fptr;
            }
            else if (!conversion_service_name.empty())
            {
                conversion_service_client_ = nh.serviceClient<TConversionServiceClass>(conversion_service_name);
            }
            else
            {
                throw invalid_argument("No conversion function pointer or service name provided.");
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
                if (arrival_service_client_.waitForExistence(WAIT_DURATION_FOR_SERVICE_EXISTENCE))
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
                if (transmission_service_client_.waitForExistence(WAIT_DURATION_FOR_SERVICE_EXISTENCE))
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

        /**
         * @brief Method used internaly to transmit data.It uses the user-defined ROSQueue::transmission_fptr_if it's defined, otherwise, it publishes on a ROS topic based on a name given by the constructor.
         * @param queue_to_transmit Queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<typename QueueElementTrait<TROSMsgType>::ElementType> &queue_to_transmit) override
        {
            if (transmission_fptr_)
            {
                return transmission_fptr_(queue_to_transmit);
            }
            else
            {
                TROSMsgType new_msg;

                for(typename deque<typename QueueElementTrait<TROSMsgType>::ElementType>::const_iterator it = queue_to_transmit.begin(); it != queue_to_transmit.end(); ++it)
                {
                    new_msg.queue_elements.push_back(*it);
                }

                transmission_pub_.publish(new_msg);
            }

            return true;
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
         * @param TPredictionServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*arrival_prediction_fptr_)(const TPredictionServiceClass&) = nullptr;

        /**
         * @brief Service client used for the transmission prediction service calls.
         */
        ros::ServiceClient transmission_service_client_;
        /**
         * @brief Function pointer for the transmission prediction service calls.
         * @param TPredictionServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*transmission_prediction_fptr_)(const TPredictionServiceClass&) = nullptr;

        /**
         * @brief Publisher used for the transmission publication calls.
         */
        ros::Publisher transmission_pub_;
        /**
         * @brief Function pointer for the transmission of data queue from the updates.
         * @param deque<typename QueueElementTrait<TROSMsgType>::ElementType>& Type of dequeue to transmit.
         */
        bool (*transmission_fptr_)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&) = nullptr;

        /**
         * @brief Service client used to evaluate the converted size of a queue element through ROS service calls.
         */
        ros::ServiceClient conversion_service_client_;
        /**
         * @brief Function pointer to a function that creates a new queue that stores the original queue elements in addition to a converted size.
         * @param deque<QueueElementTrait<TROSMsgType>::ElementType>& Reference to a queue to convert.
         * @param deque<ElementWithConvertedSize<QueueElementTrait<TROSMsgType>::ElementType>>& Queue in which to put the wrapped queue. Serves as an output to the function.
         * @details To work properly and prevent BadConversionException during runtime, follow those requirements: The wrapped queue should be the 
         * same size as the input queue size and elements should have non-zero positive converted size.
         * @throw Might not throw any exception directly, but the update step might throw BadConversionException during runtime because of a bad behavior of this user-defined function.
         */
        void (*conversion_fptr_)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&,
                                deque<ElementWithConvertedSize<typename QueueElementTrait<TROSMsgType>::ElementType>>&) = nullptr;

        /**
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /**
         * @brief Number of message that the publisher could hold in its queue (not related to ROSQueue, but a ROS primitive).
        */
        const int MAX_TRANSMIT_TOPIC_QUEUE_SIZE = 50;
};
