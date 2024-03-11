#include "ros_queue_experiments/auv_states.hpp"

#include "ros/ros.h"

AUVStates::Zones AUVStates::getZoneFromTransmissionVector(const ros_queue_msgs::TransmissionVector& transmission_vector)
{
    bool transition_found = false;
    AUVStates::Zones zone = AUVStates::Zones::TaskZone;

    if (transmission_vector.transmission_vector[0])
    {
        zone = AUVStates::Zones::TaskZone;
        transition_found = true;
    }
    if (transmission_vector.transmission_vector[1])
    {
        if(!transition_found)
        {
            zone = AUVStates::Zones::HighLocZone;
            transition_found = true;
        }
        else if
        {
            ROS_ERROR("More than one transition found in the transmission vector. This is not allowed.");
        }
    }
    if (transmission_vector.transmission_vector[2])
    {
        if(!transition_found)
        {
            zone = AUVStates::Zones::ColdZone;
            transition_found = true;
        }
        else if
        {
            ROS_ERROR("More than one transition found in the transmission vector. This is not allowed.");
        }
    }

    if (!transition_found)
    {
        ROS_ERROR("No transition found in the transmission vector. This is not allowed.");
    }
    return zone;
}

ros_queue_msgs::TransmissionVector AUVStates::getTransmissionVectorFromZone(AUVStates::Zones zone)
{
    ros_queue_msgs::TransmissionVector transmission_vector;
    transmission_vector.transmission_vector[0] = false;
    transmission_vector.transmission_vector[1] = false;
    transmission_vector.transmission_vector[2] = false;

    switch (zone)
    {
        case AUVStates::Zones::TaskZone:
            transmission_vector.transmission_vector[0] = true;
            break;
        case AUVStates::Zones::HighLocZone:
            transmission_vector.transmission_vector[1] = true;
            break;
        case AUVStates::Zones::ColdZone:
            transmission_vector.transmission_vector[2] = true;
            break;
        default:
            ROS_ERROR("Zone not recognized. Transmission vector will be set to all false.");
            break;
    }

    return transmission_vector;
}