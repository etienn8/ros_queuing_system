#pragma once

#include "ros/ros.h"

#include <chrono>
#include <string>

using std::string;

namespace queue_controller_utils
{
    template <typename TServiceType>
    void check_persistent_service_connection(ros::NodeHandle& nh, ros::ServiceClient& client)
    {
        if (!client.isValid())
        {
            const string service_name = client.getService();

            ROS_WARN_STREAM("Lost connection to service :" << service_name<<". Trying to reconnect and waiting until available.");
            client = nh.serviceClient<TServiceType>(service_name, true);
            client.waitForExistence();
            ROS_WARN_STREAM("Restored connection to "<<service_name);
        }
    }

    /**
     * @brief Get the time since the time 0 in milliseconds and updates the time point.
     * @param time_0 The time 0 from which the time difference is calculated.
     * @param time_diff The time difference in milliseconds.
     * @return The current time point. It could be used to chain the function with the next time point
    */
    std::chrono::time_point<std::chrono::system_clock> updateTimePointAndGetTimeDifferenceMS(std::chrono::time_point<std::chrono::system_clock> time_0, double& time_diff)
   {
        std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::high_resolution_clock::now();
        time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(new_time_point - time_0).count();
        return new_time_point;
    }
}