#pragma once

#include <map>
#include <memory>

#include "ros_queue_experiments/auv_states.hpp"

#include "dual_metric_services.hpp"

using std::string;

class LocalizationServices: public DualMetricServices
{
    public:
        LocalizationServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);

    protected:
        virtual bool realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;

        virtual bool expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;

        virtual bool expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
        
    private:
        std::map<AUVStates::Zones, float> predicted_localization_uncertainties_;
        std::map<AUVStates::Zones, float> real_localization_uncertainties_;

        float localization_target_ = 0.0f;

        ros::NodeHandle nh_;
};