#pragma once

#include <map>

#include "ros_queue_experiments/auv_states.hpp"

#include "metric_services.hpp"

using std::string;

class RenewalTimeServices: public MetricServices
{
    public:

        RenewalTimeServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager);
        
        float getRealRenewalTimeWithStateTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone);
        float getPredictedRenewalTimeWithStateTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone);
        
        float getRealRenewalTimeWithTransitionFromCurrentState(AUVStates::Zones to_zone);
        float getPredictedRenewalTimeWithTransitionFromCurrentState(AUVStates::Zones to_zone);

    protected:
        virtual bool realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

    private:
        float real_renewal_time_transitions_[AUVStates::Zones::count][AUVStates::Zones::count];
        float predicted_renewal_time_transitions_[AUVStates::Zones::count][AUVStates::Zones::count];

        ros::NodeHandle nh_;
};