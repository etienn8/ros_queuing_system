#include "ros_queue_experiments/auv_system.hpp"
#include "ros_queue_experiments/auv_state_manager.hpp"

#include "ros_queue_experiments/metrics/low_temperature_services.hpp"
#include "ros_queue_experiments/metrics/metric_services.hpp"
#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"
#include "ros_queue_experiments/metrics/localization_services.hpp"
#include "ros_queue_experiments/metrics/task_publisher.hpp"
#include "ros_queue_experiments/metrics/penalty_services.hpp"

#include "ros_queue_experiments/AuvStates.h"

AUVSystem::AUVSystem(ros::NodeHandle& nh): nh_(nh), ns_nh_(ros::NodeHandle())
{
    // Initialize the state manager
    auv_state_manager_ = std::make_shared<AUVStateManager>(this);
    
    expected_time_services_ = std::make_shared<RenewalTimeServices>(nh_, "renewal_time", auv_state_manager_);
    temperature_services_ = std::make_shared<TemperatureServices>(nh_, "temperature", auv_state_manager_, expected_time_services_);
    low_temperature_services_ = std::make_shared<LowTemperatureServices>(nh_, "low_temperature", auv_state_manager_, expected_time_services_);
    task_services_ = std::make_shared<TaskPublisher>(nh_, "task", auv_state_manager_, expected_time_services_);
    localization_services_ = std::make_shared<LocalizationServices>(nh_, "localization", auv_state_manager_, expected_time_services_);
    penalty_metric_services_ = std::make_shared<PenaltyServices>(nh_, "penalty", auv_state_manager_);

    auv_state_service_ = nh_.advertiseService("get_auv_state", &AUVSystem::auvStateCallback, this); 
    
    command_service_ = nh_.advertiseService("new_command", &AUVSystem::commandCallback, this);

    // Initialize the action set service
    action_set_service_ = nh_.advertiseService("potential_action_set", &AUVSystem::potentialActionSetCallback, this);

    // Initialize the publishers
    state_pub_ = nh_.advertise<ros_queue_experiments::AuvStates>("auv_state", 1);

    control_started_sub_ = ns_nh_.subscribe("optimal_controller/control_loop_started", 1, &AUVSystem::stateMonitoringCallback, this);
}

bool AUVSystem::potentialActionSetCallback(ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Request& req,
                                        ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Response& res)
{
    if (auv_state_manager_)
    {
        res.action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone));
        res.action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::HighLocZone));
        res.action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::ColdZone));
    }
    else
    {
        ROS_ERROR("AUV state manager not initialized");
        return false;
    }
    return true;
}

bool AUVSystem::auvStateCallback(ros_queue_experiments::GetRealAUVStates::Request& req,
                                ros_queue_experiments::GetRealAUVStates::Response& res)
{
    if(auv_state_manager_)
    {
        res.states = auv_state_manager_->getCurrentStates();
    }
    else
    {
        ROS_ERROR("AUV state manager not initialized");
        return false;
    }
    return true;
}


bool AUVSystem::commandCallback(ros_queue_experiments::SendNewAUVCommand::Request& req,
                                ros_queue_experiments::SendNewAUVCommand::Response& res)
{
    if(auv_state_manager_)
    {
        // Send current state before changing the zone to the new one so the monitoring can see changes 
        // that the queue server should have been updated with.
        //state_pub_.publish(auv_state_manager_->getCurrentStates());

        AUVStates::Zones new_zone = AUVStates::getZoneFromTransmissionVector(req.command);
        res.time_to_execute = expected_time_services_->getRealRenewalTimeWithTransitionFromCurrentState(new_zone);
        auv_state_manager_->commandToNextZone(new_zone, req.command_of_controller);
        ROS_INFO("New command received. Going to zone %d", new_zone);
    }
    else
    {
        ROS_ERROR("AUV state manager not initialized");
        return false;
    }
    return true;
}

void AUVSystem::stateMonitoringCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if(auv_state_manager_)
    {
        state_pub_.publish(auv_state_manager_->getCurrentStates());
    }
    else
    {
        ROS_ERROR("AUV state manager not initialized");
    }
}