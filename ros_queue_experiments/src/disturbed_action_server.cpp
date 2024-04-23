#include "ros_queue_experiments/disturbed_action_server.hpp"

#include "ros_queue_experiments/GetRealAUVStates.h"
#include "ros_queue_experiments/SendNewAUVCommand.h"
#include "ros_queue_experiments/ActionPerformance.h"

#include "ros_queue_experiments/auv_states.hpp"

#include "std_msgs/UInt8.h"

#include <string>

DisturbedActionServer::DisturbedActionServer(ros::NodeHandle& nh): nh_(nh), action_server_(nh_, "disturbed_action_server", false)
{
    if(nh_.getParam("pertubation_at_each_x_control_steps", perturbation_at_each_x_control_steps_))
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
    if(nh_.getParam("pertubation_type", perturbation_type_param))
    {
        if (perturbation_type_param == "not_moving")
        {
            perturbation_type_ = PerturbationType::NotMoving;
        }
        else if (perturbation_type_param == "offset_next_step")
        {
            perturbation_type_ = PerturbationType::OffsetPlusOne;
        }
        else
        {
            ROS_ERROR("Pertubation type not recognized. No perturbations will be applied. Supported types: not_moving, offset_next_step.");
        }
    }

    auv_state_client_ = nh_.serviceClient<ros_queue_experiments::GetRealAUVStates>("/auv_system_node/get_auv_state");
    ROS_INFO("Waiting for the AUV state service to be ready...");
    auv_state_client_.waitForExistence();
    ROS_INFO("AUV state service is ready.");

    auv_action_client_ = nh_.serviceClient<ros_queue_experiments::SendNewAUVCommand>("/auv_system_node/new_command");
    ROS_INFO("Waiting for the AUV action service to be ready...");
    auv_action_client_.waitForExistence();
    ROS_INFO("AUV action service is ready.");

    action_server_.registerGoalCallback(boost::bind(&DisturbedActionServer::commandReceivedCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&DisturbedActionServer::preemptActionCallback, this));
    action_server_.start();
    send_sucess_timer_ = nh_.createTimer(ros::Duration(1.0), &DisturbedActionServer::sendSuccessCallback, this, true);
    send_sucess_timer_.stop();

    action_performance_publisher_ = nh_.advertise<ros_queue_experiments::ActionPerformance>("action_performance", 10);
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
                for(int zone_index =0 ; zone_index < disturbed_action.transmission_vector.size(); zone_index++)
                {
                    if (disturbed_action.transmission_vector[zone_index] == 1)
                    {
                        // If last zone, set it to the first zone
                        if ((zone_index + 1) == disturbed_action.transmission_vector.size())
                        {
                            disturbed_action.transmission_vector[0] = 1;
                        }
                        else
                        {
                            disturbed_action.transmission_vector[zone_index + 1] = 1;
                        }   

                        disturbed_action.transmission_vector[zone_index] = 0;
                        break;
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

    // Send action performance monitoring
    ros_queue_experiments::ActionPerformance action_performance_msg;
    action_performance_msg.target_action = target_action;
    action_performance_msg.applied_action = disturbed_action;
    action_performance_msg.action_index_difference = AUVStates::getZoneFromTransmissionVector(target_action) - AUVStates::getZoneFromTransmissionVector(disturbed_action); 

    action_performance_publisher_.publish(action_performance_msg);
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