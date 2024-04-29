#pragma once

#include <map>
#include <memory>

#include "ros/ros.h"

#include "dual_metric_services.hpp"
#include "renewal_time_services.hpp"
#include "ros_queue_msgs/ByteSizeRequest.h"

class TaskPublisher: public DualMetricServices
{
    public:
        TaskPublisher(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services);

    protected:
// Change callbacks
        virtual bool realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                               ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                 ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

         virtual bool realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);
        
        // Rate callbacks
        virtual bool realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
    private:
        ros::NodeHandle nh_;

        ros::Publisher publisher_;
        ros::Timer timer_;

        std::map<AUVStates::Zones, float> predicted_task_departure_rates_;
        std::map<AUVStates::Zones, float> real_task_departure_rates_;

        void publishTask(const ros::TimerEvent& event);

        float arrival_task_per_second_ = 0.1f;
        int last_task_id_ = 0;

        ros::ServiceServer qos_transmission_service_;
        ros::Time last_transmission_time_;
        double accumulated_untransmitted_time_ = 0.0;
        bool qosTransmissionCallback(ros_queue_msgs::ByteSizeRequest::Request& req, 
                                     ros_queue_msgs::ByteSizeRequest::Response& res);
};
