#include "ros_queue_experiments/auv_state_manager.hpp"
#include "ros_queue_experiments/auv_system.hpp"
#include "ros_queue_experiments/auv_states.hpp"

AUVStateManager::AUVStateManager(AUVSystem* auv_system): auv_system_(auv_system)
{
    // Initialize to cold zone 
    states_at_last_transition_.current_zone = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
    states_at_last_transition_.last_zone = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
    states_at_last_transition_.localization  = 1.0;
    states_at_last_transition_.temperature  = 20.0;
    
    states_at_last_transition_.penalty = 0.0;
    states_at_last_transition_.transition_completion = 0.0;

    time_spent_in_zones_.addState();
    time_spent_in_zones_.addState();
    time_spent_in_zones_.addState();

    last_transition_time_ = ros::Time::now();
}

ros_queue_experiments::AuvStates AUVStateManager::getCurrentStates()
{
    std::lock_guard<std::mutex> lock(state_access_mutex_);
    
    // Get the rates of change
    ros_queue_experiments::AuvStates current_states;
    current_states = states_at_last_transition_;

    if (auv_system_)
    {
        current_states.header.stamp = ros::Time::now();
        
        const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
        const AUVStates::Zones last_zone = AUVStates::getZoneFromTransmissionVector(current_states.last_zone);
        
        const double elapsed_time_since_last_transition = (ros::Time::now() - last_transition_time_).toSec();

        float temperature_rate = auv_system_->temperature_services_->getRealArrival(current_zone) - auv_system_->temperature_services_->getRealDeparture(current_zone);
        current_states.temperature_integral += current_states.temperature*elapsed_time_since_last_transition +
                                               0.5*temperature_rate*elapsed_time_since_last_transition*elapsed_time_since_last_transition;

        current_states.temperature += temperature_rate * elapsed_time_since_last_transition;

        double renewal_time =  auv_system_->expected_time_services_->getRealRenewalTimeWithStateTransition(last_zone, current_zone);
        current_states.transition_completion = (elapsed_time_since_last_transition / renewal_time);
        if(current_states.transition_completion > 1.0)
        {
            current_states.transition_completion = 1.0;
        }
        
        // The penalty is the total accumulated energy spent during transitions since the beginning.
        current_states.penalty += auv_system_->penalty_metric_services_->getRealPenaltyTransition(last_zone, current_zone);

        float current_localization = auv_system_->localization_services_->getRealLocalizationUncertainty(current_zone);
        current_states.localization_integral += current_localization*elapsed_time_since_last_transition;
        current_states.localization = current_localization;
        
        // Get the proportion of time spent in each zone
        current_states.proportion_time_spent_in_zones = time_spent_in_zones_.getProportionTimeSpent();

        return current_states;
    }

    return current_states;
}

void AUVStateManager::commandToNextZone(AUVStates::Zones new_zone)
{
    ros_queue_experiments::AuvStates current_state = getCurrentStates();

    std::lock_guard<std::mutex> lock(state_access_mutex_);
    
    // Save the time spent in the last zone
    float elapsed_time = (ros::Time::now() - last_transition_time_).toSec();
    time_spent_in_zones_.addTimeSpentInState(elapsed_time, AUVStates::getZoneFromTransmissionVector(current_state.current_zone));
    
    // Save the current states to serve as the new point to evaluate the future states from.
    states_at_last_transition_ = current_state;

    // Set the new zone and the timing of the transition.
    states_at_last_transition_.last_zone = states_at_last_transition_.current_zone;
    states_at_last_transition_.current_zone = AUVStates::getTransmissionVectorFromZone(new_zone);
    states_at_last_transition_.transition_completion = 0.0;

    last_transition_time_ = ros::Time::now();
}

