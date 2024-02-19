#include "ros_queue_tests/transmission_action_receiver.hpp"

#include "ros_queue_msgs/MetricTransmissionVectorPredictions.h"

#include <string>

using std::string;

TransmissionActionReceiver::TransmissionActionReceiver(ros::NodeHandle& nh): nh_(nh)
{
    string action_server_name;
    if(nh_.getParam("action_server_name", action_server_name) && !action_server_name.empty())
    {
        action_server_ = std::make_shared<actionlib::SimpleActionServer<ros_queue_msgs::TransmissionVectorAction>>(nh_, action_server_name, false);
        action_server_->registerGoalCallback(boost::bind(&TransmissionActionReceiver::receivedActionCallback, this));
        action_server_->registerPreemptCallback(boost::bind(&TransmissionActionReceiver::receivedPreemptCallback, this));
        action_server_->start();
    }

    string departure_service_name;
    if(nh_.getParam("departure_evaluation_service", departure_service_name) && !departure_service_name.empty())
    {
        departure_service_ = nh_.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>(departure_service_name);
    }

    // vector_transmission_service_ = nh_
    byte_size_request_service_server_ = nh_.advertiseService("transmission_request_evaluation", &TransmissionActionReceiver::byteSizeRequestCallback, this);
}


void TransmissionActionReceiver::receivedActionCallback()
{
    std::lock_guard<std::mutex> lock(byte_to_send_mutex_);

    // Cancel old goal if new one is received
    if (action_server_->isActive())
    {
        action_server_->setAborted();
    }

    // Accept new goal
    auto goal = action_server_->acceptNewGoal();

    ros_queue_msgs::TransmissionVector transmission_vector = goal->action_goal;;
    
    ros_queue_msgs::MetricTransmissionVectorPredictions departure_msg;
    departure_msg.request.action_set.action_set.push_back(transmission_vector);

    if(departure_service_.call(departure_msg) && !departure_msg.response.predictions.empty())
    {
        // Hardcoded value for tests. The index 0 matches the RealQueue0 index in the action set.
        byte_to_send_ = departure_msg.response.predictions[0];
    }
    else
    {
        ROS_WARN_STREAM("Failed to call the departure evaluation service in the "<< ros::this_node::getName());
        // Indicate that the goal failed since the departure evaluation service failed.
        action_server_->setAborted();
    }
}

void TransmissionActionReceiver::receivedPreemptCallback()
{
    std::lock_guard<std::mutex> lock(byte_to_send_mutex_);
    action_server_->setPreempted();

}

bool TransmissionActionReceiver::byteSizeRequestCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                ros_queue_msgs::ByteSizeRequest::Response& res)
{
    std::lock_guard<std::mutex> lock(byte_to_send_mutex_);

    if (!action_server_->isActive())
    {
        return false;
    }
    else
    {
        res.nb_of_bytes = byte_to_send_;
        // Reset the value to simulate if it was a one time request
        byte_to_send_ = 0;
        ros_queue_msgs::TransmissionVectorResult result;
        result.success = true;

        action_server_->setSucceeded();
    }
    
    return true;
}