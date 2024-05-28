#pragma once

#include "ros/ros.h"
#include "temperature_services.hpp"

class LowTemperatureServices: public TemperatureServices
{
    public:
        LowTemperatureServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services);

    protected:
        virtual bool realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                               ros_queue_msgs::FloatRequest::Response& res) override;

        virtual bool realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
        
        virtual bool expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                 ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
        
        // Rate callbacks
        virtual bool realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
};