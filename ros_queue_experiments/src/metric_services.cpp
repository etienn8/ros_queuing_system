#include "ros_queue_experiments/metric_services.hpp"

MetricServices::MetricServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager): nh_(nh), metric_name_(metric_name), auv_state_manager_(auv_state_manager)
{
    real_metric_service_ = nh_.advertiseService(metric_name_ + "/real_metric", &MetricServices::realServiceMetricCallback, this);
    expected_metric_service_ = nh_.advertiseService(metric_name_ + "/expected_metric", &MetricServices::expectedServiceMetricCallback, this);
}


bool MetricServices::realServiceMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                              ros_queue_msgs::FloatRequest::Response& res)
{
    return realMetricCallback(req, res);
}

bool MetricServices::expectedServiceMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                   ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return expectedMetricCallback(req, res);
}

ros_queue_experiments::AuvStates MetricServices::getCurrentStates()
{
    if(auv_state_manager_ == nullptr)
    {
        ROS_ERROR("AUVStateManager is not set");
        return ros_queue_experiments::AuvStates();
    }
    return auv_state_manager_->getCurrentStates();
}
