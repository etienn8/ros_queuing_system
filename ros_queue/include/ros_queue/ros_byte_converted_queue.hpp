#pragma once

#include <string>
#include <stdexcept>
#include <deque>
#include <vector>
#include <utility>

#include "ros/ros.h"

#include "ros_queue_utils.hpp"

#include "lib_queue/dynamic_converted_queue.hpp"
#include "topic_tools/shape_shifter.h"

#include "ros_queue_msgs/QueueInfo.h"


using std::string;
using std::invalid_argument;
using std::deque;

/**
 * @brief Queue of a generic ROS messages with some ROS interface to interact with the queue via services and topics. 
 * The queue stores ROS messages alongside its size in bytes. The size of the queue is the sum of all the bytes in the queue. 
 */
class ROSByteConvertedQueue: public DynamicConvertedQueue<topic_tools::ShapeShifter::ConstPtr>
{
    public:
        /**
         * @brief Type definition of the shapeshifter pointer for clarity.
        */
        typedef topic_tools::ShapeShifter::ConstPtr ShapeShifterPtr;

        /**
         * @brief Member that contains meta data for queues.
        */
        ros_queue_msgs::QueueInfo info_;

        /**
         * @brief Struct that contains all the options related to using pointer functions, or ROS Topics/Services for prediction, transmission and conversion. 
         * @param arrival_topic_name String of the topic from which to add messages to the data queue. 
         * @param transmission_topic_name String of the topic name to publish the stored ROS messages in the queuue.
         */
        struct InterfacesArgs
        {
            string arrival_topic_name="";
            string transmission_topic_name = "";
        };

        /**
         * @brief Constructor that initialized all the prediction, transmission and conversion functions.
         * @param max_queue_size Maximum size the queue can take and over which, data will be discarded.
         * @param info ros_queue_msgs::QueueInfo reference that contains meta data about the queue.
         * @param nh Its ros::NodeHandle used to create the services and make sure that a node handle exists during the life time of the ROSQueue.
         * @param interfaces Struct that contains all the options for the prediction, transmission and conversion interfaces. See ROSConvertedQueue::InterfacesArgs.
         * @throw Throws an std::invalid_argument if one of the topic name is empty.
        */
        ROSByteConvertedQueue(int max_queue_size, ros_queue_msgs::QueueInfo& info, ros::NodeHandle& nh, InterfacesArgs interfaces)
                            : DynamicConvertedQueue<ShapeShifterPtr>(max_queue_size), info_(info), nh_(nh),
                             transmission_topic_name_(interfaces.transmission_topic_name)       
        {
            if (!interfaces.arrival_topic_name.empty())
            {
                ROS_WARN_STREAM("Created subscriber with name: "<< interfaces.arrival_topic_name);
                arrival_sub_ = nh_.subscribe(interfaces.arrival_topic_name, MAX_TOPIC_QUEUE_SIZE, &ROSByteConvertedQueue::arrivalCallback, this);
            }
            else
            {
                throw invalid_argument("No arrival topic name provided.");
            }

            if (interfaces.transmission_topic_name.empty())
            {
                throw invalid_argument("No transmission topic name provided.");
            }
        }

    protected:
        /**
         * @brief Method used internaly to transmit the stored queue messages. Won't send data if no data as been added to the queue.
         * @param queue_to_transmit Rvalue of a queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<ShapeShifterPtr>&& queue_to_transmit) override
        {
            if(is_publisher_initialized)
            {
                for(typename deque<ShapeShifterPtr>::const_iterator it = queue_to_transmit.begin(); it != queue_to_transmit.end(); ++it)
                {
                    transmission_pub_.publish(*it);
                }
                return true;
            }

            ROS_WARN("Tried to transmit data but publisher is not initialized. Data is discarded.");
            return false;
        }


        /**
         * @brief Method to convert a queue in a another queue where each element is stored with its size in bytes.
         * @param arriving_queue Rvalue of a deque of elements that needs to be converted.
         * @param converted_dequeue Reference that serves as an output of the wrapped queue with a size affiliated to each element.
         */
        virtual void generateConvertedQueue(deque<ShapeShifterPtr>&& arriving_queue,
                                             deque<ElementWithConvertedSize<ShapeShifterPtr>>& converted_queue) override
        {
            if (!arriving_queue.empty())
            {
                const int arriving_queue_size = arriving_queue.size();

                for(auto it = arriving_queue.begin(); it != arriving_queue.end(); ++it)
                {
                    // Get the size of the serialized message
                    const int size_of_msg = (*it)->size();

                    ElementWithConvertedSize<ShapeShifterPtr> convertedElement(std::move(*it), size_of_msg);
                    converted_queue.push_back(std::move(convertedElement));
                }
            }
        }

    private:
        /**
         * @brief Callback called when a message is received on the arrival_topic_name and it adds it in the queue. 
         * At the first message, it creates a publisher for the transmission based on the received msg type.
         * @param msg topic_tool::ShapeShifter pointer of a generic ROS message to add in the queue.
        */
        void arrivalCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
        {
            ROS_WARN("Callback called");
            if(!is_publisher_initialized)
            {
                /** Only could start publishing if we know the type of the msg,
                 * so we wait for the first message to decalare the publisher.
                 * */ 

                transmission_pub_ = msg->advertise(nh_, transmission_topic_name_, MAX_TOPIC_QUEUE_SIZE);
                is_publisher_initialized = true;
                ROS_WARN("Publisher initialized");
            }

            deque<ShapeShifterPtr> single_msg_deque;
            single_msg_deque.push_back(msg);

            // Insert an element in the queue and transmit nothing.
            ROS_WARN("Added new element to the queue");
            update(std::move(single_msg_deque), 0);
        }

        /**
         * @brief ROS Node handle used for the service call and make sure that a node handle exist for the life time of the ROSQueue.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Name of the topic to transmit the queue elements.
        */
        string transmission_topic_name_ = "";

        /**
         * @brief Publisher used for the transmission publication calls.
         */
        ros::Publisher transmission_pub_;

        /**
         * @brief Flag that indicated if the publisher has been initialized
        */
        bool is_publisher_initialized = false;

        /**
         * @brief Subsciber used to received data in the queue.
        */
        ros::Subscriber arrival_sub_;

        /**
         * @brief Duration to wait for the existence of services at each call.
        */
        const ros::Duration WAIT_DURATION_FOR_SERVICE_EXISTENCE = ros::Duration(0.5);

        /**
         * @brief Number of message that the publisher could hold in its queue (not related to ROSQueue, but a ROS primitive).
        */
        const int MAX_TOPIC_QUEUE_SIZE = 100;
};
