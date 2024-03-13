#pragma once

#include <mutex>

#include "auv_states.hpp"

#include "ros_queue_experiments/AuvStates.h"

class AUVStateManager
{
    public:
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
};