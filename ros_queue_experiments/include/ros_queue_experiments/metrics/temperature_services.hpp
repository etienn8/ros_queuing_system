#pragma once

#include <map>
#include <memory>

#include "ros_queue_experiments/auv_states.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"

#include "metric_services.hpp"

using std::string;

class TemperatureServices: public MetricServices
{
    public:
        TemperatureServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services);

    protected:
        virtual bool realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

    private:
        std::map<AUVStates::Zones, float> arrival_predictions_;
        std::map<AUVStates::Zones, float> departure_predictions_;

        std::shared_ptr<RenewalTimeServices> renewal_time_services_;

        ros::NodeHandle nh_;
};