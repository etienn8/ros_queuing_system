#include "ros_queue_experiments/disturbed_action_server.hpp"

#include "ros_queue_experiments/GetRealAUVStates.h"
#include "ros_queue_experiments/SendNewAUVCommand.h"

#include "ros_queue_experiments/auv_states.hpp"

#include <string>

DisturbedActionServer::DisturbedActionServer(ros::NodeHandle& nh): nh_(nh), action_server_(nh_, "Disturbed_action_server", boost::bind<void>(&DisturbedActionServer::executeActionCallback, this, _1), false)
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
    auv_state_client_ = nh_.serviceClient<ros_queue_experiments::GetRealAUVStates>("get_auv_state");
    auv_action_client_ = nh_.serviceClient<ros_queue_experiments::SendNewAUVCommand>("new_command");
    
    action_server_.start();
}

void DisturbedActionServer::executeActionCallback(const ros_queue_msgs::TransmissionVectorGoalConstPtr& goal)
{
    ros::Time start_time = ros::Time::now();

    ros_queue_msgs::TransmissionVectorResult result;
    result.success = true;

    ros_queue_msgs::TransmissionVector disturbed_action = goal->action_goal;
    // Add perturbations to the action
    ros_queue_experiments::GetRealAUVStates states_srv;
    if(auv_state_client_.call(states_srv))
    {
        if (steps_since_last_perturbation_ == perturbation_at_each_x_control_steps_)
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
                        // If last zone, put set it the first zone
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
    }
    else
    {
        ROS_ERROR("Failed to call service to get the current AUV states. No perturbations will be applied.");
    }
    
    ++steps_since_last_perturbation_;
    
    if (steps_since_last_perturbation_ > perturbation_at_each_x_control_steps_)
    {
        steps_since_last_perturbation_ = 0;
    }

    if(action_server_.isPreemptRequested())
    {
        result.success = false;
        action_server_.setPreempted();
        return;
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

    // Wait for the amount of time that the AUV system needs to execute the action.
    float elapsed_time = (ros::Time::now() - start_time).toSec();
    ros::Duration(time_to_execute - elapsed_time).sleep();

    if(action_server_.isPreemptRequested())
    {
        result.success = false;
        action_server_.setPreempted();
        return;
    }

    if (result.success)
    {
        action_server_.setSucceeded(result);
    }
}