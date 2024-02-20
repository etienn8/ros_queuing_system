#include "ros_queue_tests/prediction_service.hpp"

#include <memory>

#include "ros_queue_msgs/FloatRequest.h"

#include "ros_queue_tests/distributions/inverted_poisson.hpp"

#include "ros/ros.h"

PredictionService::PredictionService(ros::NodeHandle nh, ParameterOptions& options): nh_(nh), options_(options)
{
    if (options_.service_name.empty())
    {
        ROS_WARN_STREAM("Prediction service has no service name. Would not be created");
    }
    else
    {
        // Control action type
        if(options_.control_action_type.empty())
        {
            ROS_WARN_STREAM("Prediction service has no control_action_type. Would not be created");
        }
        else if (options_.control_action_type == "none")
        {
            service_server_ = nh_.advertiseService(options_.service_name, &PredictionService::actionIndependentCallback, this);
        }
        else if (options_.control_action_type == "transmission_vector")
        {
            service_server_ = nh_.advertiseService(options_.service_name, &PredictionService::transmissionVectorCb , this);
        }
        else
        {
            ROS_WARN_STREAM("Prediction service has no supporterd control_action_type. Would not be created");
        }
    } 
}

bool PredictionService::transmissionVectorCb(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req,
                                             ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if (options_.is_transmission_evaluation)
    {
        // Check if the id of the transmission vector is bigger than the number of expected queues
        if (options_.transmission_vector_id > (req.action_set.action_set.size() - 1))
        {
            ROS_WARN_STREAM("The id in the transmission vector is bigger than the action. In the service: "<< service_server_.getService());
        }
        else
        {
            for (int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
            {
                res.predictions.push_back(options_.transmission_value * req.action_set.action_set[action_index].transmission_vector[options_.transmission_vector_id]);
            }    
        }

        return true;
    }
    else if(options_.distribution_type == "static")
    {
        for (int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            res.predictions.push_back(options_.transmission_value);
        }
        return true;
    }

    return false;
}

bool PredictionService::actionIndependentCallback(ros_queue_msgs::FloatRequest::Request& req,
                                       ros_queue_msgs::FloatRequest::Response& res)
{
    if (options_.distribution_type == "static")
    {
        res.value = options_.transmission_value;
    }
    else 
    {
        ROS_WARN_STREAM("Unrecognized distribution type for the service " << service_server_.getService());
    }
    return true;
}
