#pragma once

#include <mutex>

#include "auv_forward_declarations.hpp"

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
         * @brief Get the current states of the AUV
         * 
         * @return AUVStates::Zones 
         */
        ros_queue_experiments::AuvStates getCurrentStates();

        /**
         * @brief Set the current states of the AUV
         * 
         * @param states 
         */
        void setCurrentStates(ros_queue_experiments::AuvStates&& states);

    private:

        /**
         * @brief Current zone that the AUV is in currently.
         */
        ros_queue_experiments::AuvStates current_states_;

        /**
         * @brief Mutex to protect the current zone.
         */
        std::mutex state_access_mutex_;

        /**
         * @brief Pointer to the AUVSystem class to have access to the metrics.
        */
        AUVSystem* auv_system_;
};