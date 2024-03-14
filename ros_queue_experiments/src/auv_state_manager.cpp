#include "ros_queue_experiments/auv_state_manager.hpp"
#include "ros_queue_experiments/auv_states.hpp"

AUVStateManager::AUVStateManager()
{
    // Initialize to cold zone 
    ros_queue_experiments::AuvStates initial_states;
    initial_states.current_zone = AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone);
    initial_states.localization  = 1.0;
    initial_states.temperature  = 20.0;
    // TODO: Compute the penalty given the starting state.
    initial_states.penalty = 0.0;

    setCurrentStates(std::move(initial_states));
}

ros_queue_experiments::AuvStates AUVStateManager::getCurrentStates()
{
    std::lock_guard<std::mutex> lock(state_access_mutex_);
    return current_states_;
}

void AUVStateManager::setCurrentStates(ros_queue_experiments::AuvStates&& states)
{
    std::lock_guard<std::mutex> lock(state_access_mutex_);
    current_states_ = std::move(states);
}