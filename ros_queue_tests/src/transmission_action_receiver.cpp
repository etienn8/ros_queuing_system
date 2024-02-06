#include "ros_queue_tests/transmission_action_receiver.hpp"

#include "ros_queue_msgs/MetricControlPredictions.h"

#include <string>

using std::string;

TransmissionActionReceiver::TransmissionActionReceiver(ros::NodeHandle& nh): nh_(nh)
{
    string action_subscriber_name;
    if(nh_.getParam("action_subscriber_name", action_subscriber_name) && !action_subscriber_name.empty())
    {
        action_receiver_subscriber_ = nh_.subscribe(action_subscriber_name, 10, &TransmissionActionReceiver::receivedActionCallback, this);
    }

    string departure_service_name;
    if(nh_.getParam("departure_evaluation_service", departure_service_name) && !departure_service_name.empty())
    {
        departure_service_ = nh_.serviceClient<ros_queue_msgs::MetricControlPredictions>(departure_service_name);
    }

    // vector_transmission_service_ = nh_
    byte_size_request_service_server_ = nh_.advertiseService("transmission_request_evaluation", &TransmissionActionReceiver::byteSizeRequestCallback, this);
}


void TransmissionActionReceiver::receivedActionCallback(const ros_queue_msgs::PotentialAction::ConstPtr& msg)
{
    ros_queue_msgs::MetricControlPredictions departure_msg;
    departure_msg.request.action_set.action_set.push_back(*msg);

    if(departure_service_.call(departure_msg) && !departure_msg.response.predictions.empty())
    {
        std::lock_guard<std::mutex> lock(byte_to_send_mutex_);
        // Hardcoded value for tests. The index 0 matches the RealQueue0 index in the action set.
        byte_to_send_ = departure_msg.response.predictions[0];
    }
    else
    {
        ROS_WARN_STREAM("Failed to call the departure evaluation service in the "<< ros::this_node::getName());
    }
}

bool TransmissionActionReceiver::byteSizeRequestCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                ros_queue_msgs::ByteSizeRequest::Response& res)
{
    std::lock_guard<std::mutex> lock(byte_to_send_mutex_);
    res.nb_of_bytes = byte_to_send_;

    // Reset the value to simulate if it was a one time request
    byte_to_send_ = 0;
    
    return true;
}