#include "ros_queue_experiments/metrics/low_temperature_services.hpp"

LowTemperatureServices::LowTemperatureServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services):
    TemperatureServices(nh, metric_name, auv_state_manager, renewal_time_services)
{
    if(!nh_.getParam("low_temp_target", temp_target_))
    {
        ROS_ERROR("Missing low_temp_target parameter.");
    }
}

// Overriden methods
bool LowTemperatureServices::realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealDepartureMetricCallback(req, res);
}

bool LowTemperatureServices::realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempRealDeparturePredictionMetricCallback(req, res);
}

bool LowTemperatureServices::expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedDepartureMetricCallback(req, res);
}

bool LowTemperatureServices::realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                            ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealArrivalMetricCallback(req, res);
}

bool LowTemperatureServices::realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempRealArrivalPredictionMetricCallback(req, res);
}

bool LowTemperatureServices::expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedArrivalMetricCallback(req, res);
}

// Rate callbacks
bool LowTemperatureServices::realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealDepartureRateMetricCallback(req, res);
}

bool LowTemperatureServices::expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedDepartureRateMetricCallback(req, res);
}

bool LowTemperatureServices::realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealArrivalRateMetricCallback(req, res);
}

bool LowTemperatureServices::expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedArrivalRateMetricCallback(req, res);
}