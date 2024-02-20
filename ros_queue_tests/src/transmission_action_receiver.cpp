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

    if(nh_.getParam("delay_action_execution", delay_action_execution_))
    {
        if(!nh_.getParam("delay_action_execution_time", delay_action_execution_time_))
        {
            ROS_WARN("The delay_action_execution_time parameter was not set and was expected since the delay_action_execution param is set to true.");
        }
    }

    // vector_transmission_service_ = nh_
    byte_size_request_service_server_ = nh_.advertiseService("transmission_request_evaluation", &TransmissionActionReceiver::byteSizeRequestCallback, this);
}


void TransmissionActionReceiver::receivedActionCallback()
{

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
        std::lock_guard<std::mutex> lock(byte_to_send_mutex_);
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
    action_server_->setPreempted();
}

bool TransmissionActionReceiver::byteSizeRequestCallback(ros_queue_msgs::ByteSizeRequest::Request& req,
                                ros_queue_msgs::ByteSizeRequest::Response& res)
{
    bool success = false;
    if (action_server_->isActive())
    {
        std::lock_guard<std::mutex> lock(byte_to_send_mutex_);

        res.nb_of_bytes = byte_to_send_;
        // Reset the value to simulate if it was a one time request
        byte_to_send_ = 0;
        ros_queue_msgs::TransmissionVectorResult result;

        if(delay_action_execution_)
        {
            ros::Duration(delay_action_execution_time_).sleep();
        }

        if (action_server_->isActive())
        {
            success = true;
            result.success = true;
            action_server_->setSucceeded(result);
        }
        else
        {
            // Goal got preempted during the wait time. Don't set the action to sucess 
            // and return a fialed service call.
            success = false;
        }
    }
    return success;
}