#pragma once
#include <ros_queue_msgs/TransmissionVector.h>

namespace AUVStates
{
    /**
     * @brief Possible zones (states) that the AUV can be in. The count enumerator is not a zone
     * and is only used to know the number of states there is.
    */
    enum Zones
    {
        TaskZone =0,
        HighLocZone,
        ColdZone,
        count
    };

    /**
     * @brief Decode the corresponding zone (state) from the transmission vector. 
     * @param transmission_vector Transmission vector to decode the zone from.
     * @return Corresponding zone (state) from the transmission vector.
    */
    AUVStates::Zones getZoneFromTransmissionVector(const ros_queue_msgs::TransmissionVector& transmission_vector);

    /**
     * @brief Get the equivalent transmission vector for a given zone.
     * @param zone Zone (state) to get the equivalent transmission vector for.
     * @return Equivalent transmission vector for the given zone.
    */
    ros_queue_msgs::TransmissionVector getTransmissionVectorFromZone(AUVStates::Zones zone);
}