#include "ros_queue_experiments/auv_state_manager.hpp"

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