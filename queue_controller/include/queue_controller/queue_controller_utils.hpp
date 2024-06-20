#pragma once

#include "ros/ros.h"

#include <chrono>
#include <string>

using std::string;

namespace queue_controller_utils
{
    /**
     * @brief Get the time since the time 0 in milliseconds and updates the time point.
     * @param time_0 The time 0 from which the time difference is calculated.
     * @param time_diff The time difference in milliseconds.
     * @return The current time point. It could be used to chain the function with the next time point
    */
    std::chrono::time_point<std::chrono::system_clock> updateTimePointAndGetTimeDifferenceMS(std::chrono::time_point<std::chrono::system_clock> time_0, double& time_diff);

    /**
     * @brief Indicate if the controller type is a known renewal controller (it's time step is not constant).
     * @param controller_type The controller type.
     * @return Returns true if the controller is a renewal type.
    */
    bool isRenewalController(const string& controller_type);
}