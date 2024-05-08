#include "ros_queue_experiments/metrics/dual_metric_services.hpp"

#include "ros_queue_msgs/FloatRequest.h"

DualMetricServices::DualMetricServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager,  std::shared_ptr<RenewalTimeServices> renewal_time_services): nh_(nh), ns_nh_(ros::NodeHandle()), metric_name_(metric_name), auv_state_manager_(auv_state_manager), renewal_time_services_(renewal_time_services)
{
    real_renewal_service_ = ns_nh_.serviceClient<ros_queue_msgs::FloatRequest>("queue_controller/get_last_renewal_time");

    real_arrival_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/change/real_metric", &DualMetricServices::realArrivalServiceMetricCallback, this);
    real_arrival_prediction_service_ = nh_.advertiseService(metric_name_ + "/arrival/change/real_expected_metric", &DualMetricServices::realArrivalPredictionServiceMetricCallback, this);
    expected_arrival_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/change/expected_metric", &DualMetricServices::expectedArrivalServiceMetricCallback, this);
    real_departure_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/change/real_metric", &DualMetricServices::realDepartureServiceMetricCallback, this);
    real_departure_prediction_service_ = nh_.advertiseService(metric_name_ + "/departure/change/real_expected_metric", &DualMetricServices::realDeparturePredictionServiceMetricCallback, this);
    expected_departure_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/change/expected_metric", &DualMetricServices::expectedDepartureServiceMetricCallback, this);

    real_arrival_rate_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/rate/real_metric", &DualMetricServices::realArrivalRateServiceMetricCallback, this);
    expected_arrival_rate_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/rate/expected_metric", &DualMetricServices::expectedArrivalRateServiceMetricCallback, this);
    real_departure_rate_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/rate/real_metric", &DualMetricServices::realDepartureRateServiceMetricCallback, this);
    expected_departure_rate_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/rate/expected_metric", &DualMetricServices::expectedDepartureRateServiceMetricCallback, this);
}

// Change services
bool DualMetricServices::realArrivalServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                          ros_queue_msgs::FloatRequest::Response& res)
{
    return realArrivalMetricCallback(req, res);
}

bool DualMetricServices::realArrivalPredictionServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return realArrivalPredictionMetricCallback(req, res);
}

bool DualMetricServices::expectedArrivalServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                   ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedArrivalMetricCallback(req, res);
}

bool DualMetricServices::realDepartureServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                            ros_queue_msgs::FloatRequest::Response& res)
{
    return realDepartureMetricCallback(req, res);
}

bool DualMetricServices::realDeparturePredictionServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return realDeparturePredictionMetricCallback(req, res);
}

bool DualMetricServices::expectedDepartureServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                   ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedDepartureMetricCallback(req, res);
}

// Rate services
bool DualMetricServices::realArrivalRateServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                              ros_queue_msgs::FloatRequest::Response& res)
{
    return realArrivalRateMetricCallback(req, res);
}

bool DualMetricServices::expectedArrivalRateServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                                  ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedArrivalRateMetricCallback(req, res);
}

bool DualMetricServices::realDepartureRateServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                                ros_queue_msgs::FloatRequest::Response& res)
{
    return realDepartureRateMetricCallback(req, res);
}

bool DualMetricServices::expectedDepartureRateServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedDepartureRateMetricCallback(req, res);
}

ros_queue_experiments::AuvStates DualMetricServices::getCurrentStates()
{
    if(auv_state_manager_ == nullptr)
    {
        ROS_ERROR("AUVStateManager is not set");
        return ros_queue_experiments::AuvStates();
    }
    return auv_state_manager_->getCurrentStates();
}
