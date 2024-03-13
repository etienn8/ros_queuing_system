#pragma once

#include "ros/ros.h"

#include <string>
#include <memory>

#include "ros_queue_experiments/auv_state_manager.hpp"

#include <ros_queue_msgs/FloatRequest.h>
#include <ros_queue_msgs/MetricTransmissionVectorPredictions.h>
#include <ros_queue_experiments/AuvStates.h>


class MetricServices
{
    public:
        MetricServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);

    protected:
        std::string metric_name_;
        
        ros::NodeHandle nh_;

        ros::ServiceServer real_metric_service_;
        ros::ServiceServer expected_metric_service_;

        std::shared_ptr<AUVStateManager> auv_state_manager_;

        virtual bool realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) = 0;
        
        virtual bool expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) =0;

        ros_queue_experiments::AuvStates getCurrentStates();
    private:
        bool realServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                       ros_queue_msgs::FloatRequest::Response& res);

        bool expectedServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                           ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);
};