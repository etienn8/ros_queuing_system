#pragma once

#include <map>

#include "ros/ros.h"

#include "metric_services.hpp"

#include "ros_queue_experiments/auv_states.hpp"
#include "ros_queue_experiments/auv_state_manager.hpp"

class PenaltyServices: public MetricServices
{
    public:
        PenaltyServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);
        
        float getRealPenaltyTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone);
        
        float getPredictedPenaltyTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone);
    protected:

        virtual bool realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool realExpectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;


    private:
        float real_penalty_transitions_[AUVStates::Zones::count][AUVStates::Zones::count];
        float predicted_penalty_transitions_[AUVStates::Zones::count][AUVStates::Zones::count];
};