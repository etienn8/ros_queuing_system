#pragma once

#include <map>
#include <memory>

#include "ros_queue_experiments/auv_states.hpp"

#include "metric_services.hpp"

using std::string;

class LocalizationServices: public MetricServices
{
    public:
        LocalizationServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);

    protected:
        virtual bool realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
    private:
        std::map<AUVStates::Zones, float> predicted_localization_uncertainties_;
        std::map<AUVStates::Zones, float> real_localization_uncertainties_;

        ros::NodeHandle nh_;
};