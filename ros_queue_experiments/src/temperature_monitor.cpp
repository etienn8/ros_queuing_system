#include "ros_queue_experiments/performance_monitor/temperature_monitor.hpp"

#include "ros_queue_msgs/FloatRequest.h"

#include "ros_queue_experiments/MetricPerformance.h"

TemperatureMonitor::TemperatureMonitor(ros::NodeHandle& nh): MetricMonitor(nh)
{
    metric_name_ = "Temperature";
    // There could be a race condition since this publisher might not be ready when the first callback of the auv state publisher is called.
    performance_metric_pub_ = nh_.advertise<ros_queue_experiments::MetricPerformance>("temperature", 1);
    
    metric_target_client_ = ns_nh_.serviceClient<ros_queue_msgs::FloatRequest>("auv_system_node/temperature/departure/rate/real_metric");
    ROS_INFO_STREAM("Waiting for existence of the "<< metric_target_client_.getService() << " service");
    metric_target_client_.waitForExistence();
    ROS_INFO_STREAM("Service "<< metric_target_client_.getService() << " exists");
}

double TemperatureMonitor::getMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    return msg->temperature;
}

double TemperatureMonitor::getQueueServerTimeAverageMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg)
{
    for (auto queue_stats : msg.queue_stats.queue_stats)
    {
        if(queue_stats.queue_name == "TemperatureQueue")
        {
            return queue_stats.arrival_time_average;
        }
    }
    ROS_WARN_THROTTLE(1, "Queue server does not have a queue named %s", "TemperatureQueue");
    return 0.0;
}

double TemperatureMonitor::getMetricTarget()
{
    ros_queue_msgs::FloatRequest metric_target_srv;
    if(metric_target_client_.call(metric_target_srv))
    {
        return metric_target_srv.response.value;
    }
    else
    {
        ROS_ERROR("Failed to call service auv_system_node/temperature/departure/real_metric");
        return 0.0;
    }
}

double TemperatureMonitor::getContinuousIntegralOfMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    return msg->temperature_integral;
}

double TemperatureMonitor::getQueueServerArrivalMeanMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg)
{
    for (auto queue_stats : msg.queue_stats.queue_stats)
    {
        if(queue_stats.queue_name == "TemperatureQueue")
        {
            return queue_stats.arrival_mean;
        }
    }
    ROS_WARN_THROTTLE(1, "Queue server does not have a queue named %s", "TemperatureQueue");
    return 0.0;
}