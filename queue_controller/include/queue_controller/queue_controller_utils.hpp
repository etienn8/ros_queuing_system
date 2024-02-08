#pragma once

#include "ros/ros.h"

#include <string>

using std::string;

namespace queue_controller_utils
{
    template <typename TServiceType>
    void check_persistent_service_connection(ros::NodeHandle nh, ros::ServiceClient& client)
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
}