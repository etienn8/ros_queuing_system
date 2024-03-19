#include "ros_queue_experiments/auv_system.hpp"
#include "ros_queue_experiments/auv_state_manager.hpp"

AUVSystem::AUVSystem(ros::NodeHandle& nh): nh_(nh)
{
    // Initialize the state manager
    auv_state_manager_ = std::make_shared<AUVStateManager>(this);
    
    expected_time_services_ = std::make_shared<RenewalTimeServices>(nh_, "renewal_time", auv_state_manager_);
    temperature_services_ = std::make_shared<TemperatureServices>(nh_, "temperature", auv_state_manager_, expected_time_services_);
    task_services_ = std::make_shared<TaskPublisher>(nh_, "task", auv_state_manager_, expected_time_services_);
    localization_services_ = std::make_shared<LocalizationServices>(nh_, "localization", auv_state_manager_);
    penalty_metric_services_ = std::make_shared<PenaltyServices>(nh_, "penalty", auv_state_manager_);

    // Initialize the action set service
    action_set_service_ = nh_.advertiseService("potential_action_set", &AUVSystem::potentialActionSetCallback, this);
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


