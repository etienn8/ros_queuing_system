#include "ros_queue_experiments/disturbed_action_server.hpp"

#include "ros_queue_experiments/ActionPerformance.h"
#include "ros_queue_experiments/auv_states.hpp"

#include "std_msgs/UInt8.h"

#include <string>
#include <time.h>
#include <stdlib.h>

DisturbedActionServer::DisturbedActionServer(ros::NodeHandle& nh): nhp_(nh), nh_(ros::NodeHandle()), action_server_(nhp_, "disturbed_action_server", false)
{
    if(nhp_.getParam("pertubation_at_each_x_control_steps", perturbation_at_each_x_control_steps_))
    {
        if (perturbation_at_each_x_control_steps_ > 0)
        {
            ROS_INFO_STREAM("Perturbation set at each " << perturbation_at_each_x_control_steps_ << " control steps.");
        }
        else if(perturbation_at_each_x_control_steps_ == 0)
        {
            ROS_INFO_STREAM("perturbation_at_each_x_control_steps_ set to 0 so no perturbations will be applied.");
        }
        else
        {
            ROS_ERROR("Pertubation at each x control steps cannot be negative. No perturbations will be applied.");
        }
    }
    else
    {
        ROS_ERROR("Pertubation at each x control steps not set. Not perturbations will be applied.");
    }

    std::string perturbation_type_param;
    if(nhp_.getParam("pertubation_type", perturbation_type_param))
    {
        if (perturbation_type_param == "not_moving")
        {
            perturbation_type_ = PerturbationType::NotMoving;
        }
        else if (perturbation_type_param == "offset_next_step")
        {
            perturbation_type_ = PerturbationType::OffsetPlusOne;
        }
        else if (perturbation_type_param == "other_random")
        {
            perturbation_type_ = PerturbationType::OtherRandom;
            /* Initialize the random seed for the rand() function globaly*/
            srand(time(NULL));
        }
        else
        {
            ROS_ERROR("Pertubation type not recognized. No perturbations will be applied. Supported types: not_moving, offset_next_step.");
        }
    }

    nhp_.getParam("is_dummy", is_dummy_);
    
    if(is_dummy_)
    {
        ROS_INFO_STREAM("The disturbed action server "<< ros::this_node::getName() << " is dummy and will not any action to the system. No perturbations are applied.");
    }
    else 
    {
        auv_state_client_ = PersistentServiceClient<ros_queue_experiments::GetRealAUVStates>(nh_, "auv_system_node/get_auv_state");
        ROS_INFO("Waiting for the AUV state service to be ready...");
        auv_state_client_.waitForExistence();
        ROS_INFO("AUV state service is ready.");

        auv_action_client_ = PersistentServiceClient<ros_queue_experiments::SendNewAUVCommand>(nh_, "auv_system_node/new_command");
        ROS_INFO("Waiting for the AUV action service to be ready...");
        auv_action_client_.waitForExistence();
        ROS_INFO("AUV action service is ready.");
        
        send_sucess_timer_ = nhp_.createTimer(ros::Duration(1.0), &DisturbedActionServer::sendSuccessCallback, this, true);
        send_sucess_timer_.stop();
    }

    action_server_.registerGoalCallback(boost::bind(&DisturbedActionServer::commandReceivedCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&DisturbedActionServer::preemptActionCallback, this));
    action_server_.start();

    queue_stats_client_ = PersistentServiceClient<ros_queue_msgs::QueueServerStatsFetch>(nh_, "queue_server/get_server_stats");
    queue_stats_client_.waitForExistence();
    action_performance_publisher_ = nhp_.advertise<ros_queue_experiments::ActionPerformance>("action_performance", 10);
}

void DisturbedActionServer::commandReceivedCallback()
{
    ros::Time start_time = ros::Time::now();

    // Cancel old action if its still active and stop it's success callback from being called
    if (action_server_.isActive())
    {
        action_server_.setAborted();
        send_sucess_timer_.stop();
    }

    auto goal = action_server_.acceptNewGoal();
    ros_queue_msgs::TransmissionVector target_action = goal->action_goal;
    ros_queue_msgs::TransmissionVector disturbed_action = goal->action_goal;

    // Add perturbations to the action
    if (!is_dummy_)
    {
        ros_queue_experiments::GetRealAUVStates states_srv;
        if(auv_state_client_.call(states_srv))
        {
            if ((perturbation_at_each_x_control_steps_ != 0) &&
                (steps_since_last_perturbation_ == perturbation_at_each_x_control_steps_-1))
            {
                if (perturbation_type_ == PerturbationType::NotMoving)
                {
                    disturbed_action = states_srv.response.states.current_zone;
                }
                else if (perturbation_type_ == PerturbationType::OffsetPlusOne)
                {
                    AUVStates::Zones predisturbed_zone = AUVStates::getZoneFromTransmissionVector(disturbed_action);

                    if (predisturbed_zone == AUVStates::Zones::TaskZone)
                    {
                        disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::HighLocZone);
                    }
                    else if (predisturbed_zone == AUVStates::Zones::HighLocZone)
                    {
                        disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::ColdZone);
                    }
                    else if (predisturbed_zone == AUVStates::Zones::ColdZone)
                    {
                        disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
                    }
                }
                else if (perturbation_type_ == PerturbationType::OtherRandom)
                {
                    AUVStates::Zones predisturbed_zone = AUVStates::getZoneFromTransmissionVector(disturbed_action);
                    // Add random perturbation
                    int offset = static_cast<int>(rand()) % 2;

                    if (predisturbed_zone == AUVStates::Zones::TaskZone)
                    {
                        if (offset ==1)
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::HighLocZone);
                        }
                        else
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::ColdZone);
                        }
                    }
                    else if (predisturbed_zone == AUVStates::Zones::HighLocZone)
                    {
                        if (offset ==1)
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
                        }
                        else
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::ColdZone);
                        }
                    }
                    else if (predisturbed_zone == AUVStates::Zones::ColdZone)
                    {
                        if (offset ==1)
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
                        }
                        else
                        {
                            disturbed_action = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::HighLocZone);
                        }
                    }
                }
            }
            ++steps_since_last_perturbation_;
            if (steps_since_last_perturbation_ == perturbation_at_each_x_control_steps_)
            {
                steps_since_last_perturbation_ = 0;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service to get the current AUV states. No perturbations will be applied.");
        }

        // Send the disturbed action to the AUV system
        ros_queue_experiments::SendNewAUVCommand action_srv;
        action_srv.request.command = disturbed_action;
        action_srv.request.command_of_controller = target_action;
        
        float time_to_execute = 0.0;
        if(auv_action_client_.call(action_srv))
        {
            time_to_execute = action_srv.response.time_to_execute;
        }
        else
        {
            ROS_ERROR("Failed to call service to send the disturbed action to the AUV system.");
        }

        // Reset the time of the success callback and start it.
        float elapsed_time = (ros::Time::now() - start_time).toSec();
        send_sucess_timer_.setPeriod(ros::Duration(time_to_execute-elapsed_time), true);
        send_sucess_timer_.start();
    }

    // Send action performance monitoring
    ros_queue_experiments::ActionPerformance action_performance_msg;
    ros_queue_msgs::QueueServerStatsFetch queue_server_stats_fetch;
    if(queue_stats_client_.call(queue_server_stats_fetch))
    {
        action_performance_msg.header.stamp = ros::Time::now();

        action_performance_msg.target_action = target_action;
        action_performance_msg.applied_action = disturbed_action;

        action_performance_msg.controller_action_index = AUVStates::getZoneFromTransmissionVector(action_performance_msg.target_action);
        action_performance_msg.applied_action_index = AUVStates::getZoneFromTransmissionVector(action_performance_msg.applied_action);
        action_performance_msg.queue_server_stats = queue_server_stats_fetch.response.queue_stats;

        action_performance_publisher_.publish(action_performance_msg);
    }
    else
    {
        ROS_ERROR("Perturbation node: Failed to call service to get the queue server stats.");
    }

}

void DisturbedActionServer::preemptActionCallback()
{
    ROS_INFO("Preempting the action.");
    action_server_.setPreempted();
    send_sucess_timer_.stop();
}

void DisturbedActionServer::sendSuccessCallback(const ros::TimerEvent& event)
{
    ros_queue_msgs::TransmissionVectorResult result;
    result.success = true;
    action_server_.setSucceeded(result);
}