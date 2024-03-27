#pragma once

#include <mutex>

#include "auv_forward_declarations.hpp"

#include "proportion_time_spent.hpp"

#include "auv_states.hpp"
#include "ros_queue_experiments/AuvStates.h"

class AUVStateManager
{
    public:

        /**
         * @brief Initialization of the states.
        */
        AUVStateManager(AUVSystem* auv_system);
        
        /**
         * @brief Get the current states of the AUV. 
         * It will the state at the start of the last transition et extrapolate the current states.
         * 
         * @return AUVStates::Zones 
         */
        ros_queue_experiments::AuvStates getCurrentStates();

        /**
         * @brief Set the auv to a new zone and take a snapshot of the current states.
         * 
         * @param states 
         */
        void commandToNextZone(AUVStates::Zones new_zone);

    private:

        /**
         * @brief Time of when the last action was performed.
        */
        ros::Time last_transition_time_;

        /**
         * @brief Current zone that the AUV is in currently and states at the moment of transitions.
         */
        ros_queue_experiments::AuvStates states_at_last_transition_;

        /**
         * @brief Mutex to protect the current zone.
         */
        std::mutex state_access_mutex_;

        /**
         * @brief Pointer to the AUVSystem class to have access to the metrics.
        */
        AUVSystem* auv_system_;

        /**
         * @brief Object that keeps track of the time spent in each zone.
        */
        ProportionTimeSpent time_spent_in_zones_;
};