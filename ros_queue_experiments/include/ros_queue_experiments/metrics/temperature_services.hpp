#pragma once

#include <map>
#include <memory>

#include "ros_queue_experiments/auv_states.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"

#include "dual_metric_services.hpp"

using std::string;

class TemperatureServices: public DualMetricServices
{
    public:
        TemperatureServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services);

        float getRealArrival(AUVStates::Zones zone);
        float getRealDeparture(AUVStates::Zones zone);

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
        std::map<AUVStates::Zones, float> arrival_predictions_;
        std::map<AUVStates::Zones, float> departure_predictions_;

        std::shared_ptr<RenewalTimeServices> renewal_time_services_;

        ros::NodeHandle nh_;

        float temp_target_ = 0.0f;
};