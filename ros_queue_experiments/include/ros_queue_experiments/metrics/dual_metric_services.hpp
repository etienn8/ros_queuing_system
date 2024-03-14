#pragma once

#include "ros/ros.h"

#include <string>
#include <memory>

#include "ros_queue_experiments/auv_state_manager.hpp"

#include <ros_queue_msgs/FloatRequest.h>
#include <ros_queue_msgs/MetricTransmissionVectorPredictions.h>
#include <ros_queue_experiments/AuvStates.h>


class DualMetricServices
{
    public:
        DualMetricServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);

    protected:
        std::string metric_name_;
        
        ros::NodeHandle nh_;

        ros::ServiceServer real_arrival_metric_service_;
        ros::ServiceServer expected_arrival_metric_service_;

        ros::ServiceServer real_departure_metric_service_;
        ros::ServiceServer expected_departure_metric_service_;

        std::shared_ptr<AUVStateManager> auv_state_manager_;

        virtual bool realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) = 0;
        
        virtual bool expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) =0;

        virtual bool realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) = 0;
        
        virtual bool expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) =0;

        ros_queue_experiments::AuvStates getCurrentStates();
    private:
        bool realArrivalServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                       ros_queue_msgs::FloatRequest::Response& res);

        bool expectedArrivalServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                           ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);
        
        bool realDepartureServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                       ros_queue_msgs::FloatRequest::Response& res);

        bool expectedDepartureServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                           ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);
};