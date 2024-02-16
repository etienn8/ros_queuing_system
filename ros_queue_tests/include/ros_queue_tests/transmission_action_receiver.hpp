#pragma once

#include <mutex>

#include "ros/ros.h"

#include "ros_queue_msgs/TransmissionVector.h"
#include "ros_queue_msgs/ByteSizeRequest.h"

/**
 * @brief Class that listen to the output of a queue controller that uses transmission vector as its action and indicates
 * the bytes that can be send via a service interface that the queue server can connect to.
*/
class TransmissionActionReceiver
{
    public:
        /**
         * @brief Constructor that starts the subscriber and the service server.
        */
        TransmissionActionReceiver(ros::NodeHandle& nh);

    private:
        ros::NodeHandle nh_;
        
        /**
         * @brief Subscriber that connects to the output of a queue controller that uses transmission vector as its action.
        */
        ros::Subscriber action_receiver_subscriber_;
        
        /**
         * @brief Service server that returns a number of bytes that can be transmitted based on the last received 
         * action message.
        */
        ros::ServiceServer byte_size_request_service_server_;

        /**
         * @brief Service client to know the transmission rate of the real queue based on if the real queue can transmit.
        */
        ros::ServiceClient departure_service_;
        
        /**
         * @brief Number of bytes that can be transmitted at the next service call.
        */
        int byte_to_send_ = 0;

        /**
         * @brief Mutex used to protect the access to byte_to_send_ since the service and the subscriber callbacks
         * manipulate it.
        */
        std::mutex byte_to_send_mutex_;

        /**
         * @brief Callback used whenever a best action is outputed from the queue controller. It sets the byte_to_send_
         * based on the value of the transmission rate of the real queue (index 0 of the transmission vector).
         * @param msg Optimal transmission vector message sent by the queue_controller. The first entry of the 
         * transmission vector should be the transmission of the real queue.
        */
        void receivedActionCallback(const ros_queue_msgs::TransmissionVector::ConstPtr& msg);

        /**
         * @brief Callback used whenever a real queue connected to the service wants to know how many bytes it could 
         * transmit. It sets the response to the current value of byte_to_send_ and then before setting it to zero.
        */
        bool byteSizeRequestCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                     ros_queue_msgs::ByteSizeRequest::Response& res);
};