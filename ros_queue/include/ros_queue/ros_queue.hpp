#pragma once

#include <string>
#include <stdexcept>
#include <deque>

#include "ros/ros.h"

#include "ros_queue_info.hpp"
#include "ros_queue_utils.hpp"

#include "lib_queue/dynamic_queue.hpp"


using std::string;
using std::invalid_argument;
using std::deque;

/**
 * @brief Queue of a rosmsg type with some ROS interface to interact with the queue via services and topics, and/or pointer functions.
 * @tparam TROSMsgType ROS msg used to publish the queue and that contains a vector named "queue_elements" where its type defines the type of the internal dequeu.
 * @tparam TServiceClass Type of the service used for the evaluation and prediction step. The response must have an int32 named "prediction".
 */
template <typename TROSMsgType, typename TServiceClass>
class ROSQueue: public DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>
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
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, int (*arrival_prediction_fptr)(const TServiceClass&),
         int (*transmission_prediction_fptr)(const TServiceClass&), bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&)):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info)
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

            if (transmission_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission function with null function pointer.");
            }
            transmission_fptr_ = transmission_fptr;
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update.
         * @throw Throws an std::invalid_argument if the arrival prediction function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, int (*arrival_prediction_fptr)(const TServiceClass&),
         string transmission_prediction_service_name, bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&)):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (arrival_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate arrival_prediction with null function pointer.");
            }
            arrival_prediction_fptr_ = arrival_prediction_fptr;

            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);

            if (transmission_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission function with null function pointer.");
            }
            transmission_fptr_ = transmission_fptr;
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update.
         * @throw Throws an std::invalid_argument if the transmission prediction function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        int (*transmission_prediction_fptr)(const TServiceClass&), bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&)):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (transmission_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission_prediction with null function pointer.");
            }
            transmission_prediction_fptr_ = transmission_prediction_fptr;
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);

            if (transmission_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission function with null function pointer.");
            }
            transmission_fptr_ = transmission_fptr;
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @param transmission_fptr Pointer to a user-defined function that is called whenever data should be transmitted from the update.
         * @throw Throws an std::invalid_argument if the transmission prediction function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        string transmission_prediction_service_name, bool (*transmission_fptr)(deque<typename QueueElementTrait<TROSMsgType>::ElementType>&)):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);

            if (transmission_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission function with null function pointer.");
            }
            transmission_fptr_ = transmission_fptr;
        }


        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
         * @throw Throws an std::invalid_argument if one of the function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, int (*arrival_prediction_fptr)(const TServiceClass&),
         int (*transmission_prediction_fptr)(const TServiceClass&), string transmission_topic_name):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info)
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

            transmission_pub_ = nh_.advertise<TROSMsgType>(transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_fptr Pointer to a user-defined function that is called to predict the number of arrivals.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
         * @throw Throws an std::invalid_argument if the arrival prediction function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, int (*arrival_prediction_fptr)(const TServiceClass&),
         string transmission_prediction_service_name, string transmission_topic_name):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (arrival_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate arrival_prediction with null function pointer.");
            }
            arrival_prediction_fptr_ = arrival_prediction_fptr;

            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);

            transmission_pub_ = nh_.advertise<TROSMsgType>(transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_fptr Pointer to a user-defined function that is called to predict the number of departure.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
         * @throw Throws an std::invalid_argument if the transmission prediction function pointers is null. 
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        int (*transmission_prediction_fptr)(const TServiceClass&), string transmission_topic_name):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            if (transmission_prediction_fptr == nullptr)
            {
                throw invalid_argument("Tried to initiate transmission_prediction with null function pointer.");
            }
            transmission_prediction_fptr_ = transmission_prediction_fptr;
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);

            transmission_pub_ = nh_.advertise<TROSMsgType>(transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
        }

        /**
         * @brief  Constructor that initialize the max queue size, the ROSQueueINfo and the different predictions methods.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ROSQueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param arrival_prediction_service_name String of the service name called to predict the number of arriving data.
         * @param transmission_prediction_service_name String of the service name called to predict the number of transmitted data.
         * @param transmission_topic_name String of the topic name to publish a TROSMsgType message of the queue elements to transmit.
        */
        ROSQueue(int max_queue_size, ROSQueueInfo& info, ros::NodeHandle& nh, string arrival_prediction_service_name,
        string transmission_prediction_service_name, string transmission_topic_name):DynamicQueue<typename QueueElementTrait<TROSMsgType>::ElementType, TServiceClass>(max_queue_size), info_(info), nh_(nh)
        {
            arrival_service_client_ = nh.serviceClient<TServiceClass>(arrival_prediction_service_name);
            transmission_service_client_ = nh.serviceClient<TServiceClass>(transmission_prediction_service_name);

            transmission_pub_ = nh_.advertise<TROSMsgType>(transmission_topic_name, MAX_TRANSMIT_TOPIC_QUEUE_SIZE);
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
         * @param TServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*arrival_prediction_fptr_)(const TServiceClass&) = nullptr;

        /**
         * @brief Service client used for the transmission prediction service calls.
         */
        ros::ServiceClient transmission_service_client_;
        /**
         * @brief Function pointer for the transmission prediction service calls.
         * @param TServiceClass& Service class used as a data structure to pass input to predictions.
         */
        int (*transmission_prediction_fptr_)(const TServiceClass&) = nullptr;

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
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /**
         * @brief Number of message that the publisher could hold in its queue (not related to ROSQueue, but a ROS primitive).
        */
        const int MAX_TRANSMIT_TOPIC_QUEUE_SIZE = 50;
};
