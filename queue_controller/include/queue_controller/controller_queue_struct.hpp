#pragma once

#include <string>
#include <memory>

#include "ros/ros.h"

using std::string;

/**
 * @brief Structure that contains all the necessary parameters about a queue that the controller 
 * needs to evaluate its changes and compute the best action.
*/
struct ControllerQueueStruct
{
    /**
     * @brief Indicate the name of a queue from the queue server.
    */
    string queue_name_;

    /**
     * @brief Indicate if the queue is virtual. It mainly affects if the queue_controller will send manual
     * changes to this queue since only virtual queues are manually updated.
    */
    bool is_virtual_;

    /**
     * @brief The weight (factor) applied to the queue's term in the Lyapunov equation. Its value
     * compared to the other queues' weight and the v_parameter of the controller indicate how much
     * this queue will be prioritize in the controller equation.
    */
    float weight_;

    /**
     * @brief Flag to indicate if the expected_arrival_service_ or the arrival_independent_from_action_service_ should be used.
    */
    bool is_arrival_action_dependent;

    /**
     * @brief Flag to indicate if the expected_departure_service_ or the departure_independent_from_action_service_ should be used.
    */
    bool is_departure_action_dependent;

    /**
     * @brief ROS Service used to compute the expected arrivals of a queue based on a the current state
     * of the system and a given potential action. Should not be defined if arrival_based_on_queue_size_service_
     * is used. If both are defined, expected_arrival_service_ will be used.
    */
    ros::ServiceClient expected_arrival_service_;

    /**
     * @brief ROS Service used to compute the expected departures of a queue based on a the current state
     * of the system and a given potential action. Should not be defined if departure_based_on_queue_size_service_
     * is used. If both are defined, expected_arrival_service_ will be used.
    */
    ros::ServiceClient expected_departure_service_;

    /**
     * @brief ROS Service used to get a a metric thas is independant from the action. Should not be defined 
     * if expected_arrival_service_ is used. If both are defined, expected_arrival_service_ will be used.
     * @details Since it uses a different service definition compared to expected_arrival_service_ because
     * the latter uses a potential action as an input, arrival_independant_from_action_service_ need to be created.
    */
    ros::ServiceClient arrival_independent_from_action_service_;

    /**
     * @brief ROS Service used to get a current queue size if it's used as the departure metric. Should not be defined 
     * if expected_departure_service_ is used. If both are defined, expected_departure_service_ will be used.
     * @details Since it uses a different service definition  compared to expected_departure_service_ because
     * the latter uses a potential action as an input, departure_independant_from_action_service_ need to be created.
    */
    ros::ServiceClient departure_independent_from_action_service_;
};