#pragma once

#include "ros/ros.h"

#include "ros_queue_msgs/MetricTransmissionVectorPredictions.h"
#include "ros_queue_msgs/FloatRequest.h"

#include "ros_queue_tests/distributions/inversed_cumulative_distribution.hpp"

#include <string>
#include <vector>


using std::string;


class PredictionService
{
    public:
        struct ParameterOptions
        {
            string service_name = "";
            string control_action_type = "";
            
            string distribution_type = "";
            
            bool is_transmission_evaluation = false;
            int transmission_vector_id = 0;
            float transmission_value = 0.0f;
        };
        
        PredictionService(ros::NodeHandle nh, ParameterOptions& options);
        
        ros::ServiceServer service_server_;
    private:
        ros::NodeHandle nh_;

        ParameterOptions options_;

        bool transmissionVectorCb(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req,
                             ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);

        bool actionIndependentCallback(ros_queue_msgs::FloatRequest::Request& req,
                                       ros_queue_msgs::FloatRequest::Response& res);
};