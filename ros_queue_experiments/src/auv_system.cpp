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
}

