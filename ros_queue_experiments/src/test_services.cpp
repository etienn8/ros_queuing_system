#include "ros/ros.h"

#include <vector>

#include "ros_queue_msgs/MetricTransmissionVectorPredictions.h"
#include "ros_queue_experiments/auv_states.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "service_callers");

    ros::NodeHandle nh;

    ros::ServiceClient predicted_temperature_service = nh.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>("auv_system_node/temperature/arrival/expected_metric");
    predicted_temperature_service.waitForExistence();

    ros::ServiceClient predicted_renewal_service = nh.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>("auv_system_node/renewal_time/expected_metric");
    predicted_renewal_service.waitForExistence();

    ros::ServiceClient predicted_penalty_service = nh.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>("auv_system_node/penalty/expected_metric");
    predicted_penalty_service.waitForExistence();

    ros::ServiceClient localization_temperature_service = nh.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>("auv_system_node/localization/arrival/expected_metric");
    localization_temperature_service.waitForExistence();

    ros::ServiceClient task_departure_service = nh.serviceClient<ros_queue_msgs::MetricTransmissionVectorPredictions>("auv_system_node/task/departure/expected_metric");
    task_departure_service.waitForExistence();

    ros_queue_msgs::PotentialTransmissionVectorSet action_set;
    action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::TaskZone));
    action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::HighLocZone));
    action_set.action_set.push_back(AUVStates::getTransmissionVectorFromZone(AUVStates::Zones::ColdZone));

    ROS_INFO_STREAM("Renewal time service response:");
    ros_queue_msgs::MetricTransmissionVectorPredictions renewal_time_predictions;
    renewal_time_predictions.request.action_set = action_set;
    if(!predicted_renewal_service.call(renewal_time_predictions))
    {
        ROS_ERROR("Failed to call renewal time service.");
    }
    else
    {
        for(int action_index = 0; action_index < action_set.action_set.size(); ++action_index)
        {
            ROS_INFO_STREAM(renewal_time_predictions.response.predictions[action_index]);
        }
    }

    ROS_INFO_STREAM("Penalty service response:");
    ros_queue_msgs::MetricTransmissionVectorPredictions penalty_predictions;
    penalty_predictions.request.action_set = action_set;
    if(!predicted_penalty_service.call(penalty_predictions))
    {
        ROS_ERROR("Failed to call penalty service.");
    }
    else
    {
        for(int action_index = 0; action_index < action_set.action_set.size(); ++action_index)
        {
            ROS_INFO_STREAM(penalty_predictions.response.predictions[action_index]);
        }
    }

    ROS_INFO_STREAM("Temperature service response:");
    ros_queue_msgs::MetricTransmissionVectorPredictions temperature_time_predictions;
    temperature_time_predictions.request.action_set = action_set;
    if(!predicted_temperature_service.call(temperature_time_predictions))
    {
        ROS_ERROR("Failed to call temperature service.");
    }
    else
    {
        for(int action_index = 0; action_index < action_set.action_set.size(); ++action_index)
        {
            ROS_INFO_STREAM(temperature_time_predictions.response.predictions[action_index]);
        }
    }

    ROS_INFO_STREAM("Localization uncertainty service response:");
    ros_queue_msgs::MetricTransmissionVectorPredictions localization_predictions;
    localization_predictions.request.action_set = action_set;
    if(!localization_temperature_service.call(localization_predictions))
    {
        ROS_ERROR("Failed to call localization service.");
    }
    else
    {
        for(int action_index = 0; action_index < action_set.action_set.size(); ++action_index)
        {
            ROS_INFO_STREAM(localization_predictions.response.predictions[action_index]);
        }
    }

    ROS_INFO_STREAM("Task departure service response:");
    ros_queue_msgs::MetricTransmissionVectorPredictions task_departure_predictions;
    task_departure_predictions.request.action_set = action_set;
    if(!task_departure_service.call(task_departure_predictions))
    {
        ROS_ERROR("Failed to call task departure service.");
    }
    else
    {
        for(int action_index = 0; action_index < action_set.action_set.size(); ++action_index)
        {
            ROS_INFO_STREAM(task_departure_predictions.response.predictions[action_index]);
        }
    }

    ros::spin();
}


