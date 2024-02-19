#pragma once

#include <memory>
#include <mutex>

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#include "ros_queue_msgs/TransmissionVector.h"
#include "ros_queue_msgs/ByteSizeRequest.h"
#include "ros_queue_msgs/TransmissionVectorAction.h"

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
         * @brief Action server that listens to the output of a queue controller that uses transmission vector as its action
         * and returns if the action was successful or not.
        */
        std::shared_ptr<actionlib::SimpleActionServer<ros_queue_msgs::TransmissionVectorAction>> action_server_;
        
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
         * @brief Callback used whenever the action server receives a new action from the queue controller. It
         * cancels the old acctive action and sets the byte_to_send based on the received goal. It also aborts the 
         * goal if the departure service fails to return a prediction. 
        */
        void receivedActionCallback();

        /**
         * @brief Callback used whenever the action server is preempted. It sets the byte_to_send_ to zero.
        */
        void receivedPreemptCallback();

        /**
         * @brief Callback used whenever a real queue connected to the service wants to know how many bytes it could 
         * transmit. It sets the response to the current value of byte_to_send_ and then before setting it to zero.
        */
        bool byteSizeRequestCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                     ros_queue_msgs::ByteSizeRequest::Response& res);
};