#include "ros_queue_experiments/metrics/dual_metric_services.hpp"

DualMetricServices::DualMetricServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager): nh_(nh), metric_name_(metric_name), auv_state_manager_(auv_state_manager)
{
    real_arrival_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/real_metric", &DualMetricServices::realArrivalServiceMetricCallback, this);
    expected_arrival_metric_service_ = nh_.advertiseService(metric_name_ + "/arrival/expected_metric", &DualMetricServices::expectedArrivalServiceMetricCallback, this);
    real_departure_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/real_metric", &DualMetricServices::realDepartureServiceMetricCallback, this);
    expected_departure_metric_service_ = nh_.advertiseService(metric_name_ + "/departure/expected_metric", &DualMetricServices::expectedDepartureServiceMetricCallback, this);
}


bool DualMetricServices::realArrivalServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                              ros_queue_msgs::FloatRequest::Response& res)
{
    return realArrivalMetricCallback(req, res);
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

bool DualMetricServices::expectedDepartureServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                   ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedDepartureMetricCallback(req, res);
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
